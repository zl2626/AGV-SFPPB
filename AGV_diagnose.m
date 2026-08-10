function report = AGV_diagnose(stop_time, make_plots)
%AGV_DIAGNOSE Reconstruct Identifier and Critic diagnostics offline.
%   report = AGV_diagnose() runs the complete 20 s benchmark and reports:
%       |F_true - F_hat|, ||omega_c||, two Bellman residuals, and both
%       critic rates.
%   The model configuration is changed only in memory and is not saved.
%
%   F_true is reconstructed from the logged z2 trajectory:
%       F_true = d(z2-O2)/dt - C*delta - O2.
%   This is a diagnostic estimate, not a replacement for a plant-side
%   Identifier state model.

if nargin < 1
    stop_time = 20;
end
if nargin < 2
    make_plots = true;
end

model = 'AGV_simulate';
load_system(model);
cleanup_model = onCleanup(@() close_system(model, 0));

set_param(model, ...
    'StopTime', num2str(stop_time, 16), ...
    'SaveState', 'on', ...
    'StateSaveName', 'xout', ...
    'SaveOutput', 'on', ...
    'OutputSaveName', 'yout');
simulation = sim(model, 'ReturnWorkspaceOutputs', 'on');

ctrl_timeseries = simulation.xout{3}.Values;
plant_states = simulation.xout{2}.Values.Data;
assist_states = simulation.xout{4}.Values.Data;
transform_states = simulation.xout{5}.Values.Data;
t = ctrl_timeseries.Time(:);
ctrl_states = ctrl_timeseries.Data;

z2 = [simulation.z2y(:), simulation.z2phi(:)];
s1 = [simulation.s1y(:), simulation.s1phi(:)];
delta = simulation.delta(:);
delta_applied = simulation.delta1(:);
O2 = ctrl_states(:, 37:38);
n = numel(t);

F_hat = zeros(n, 2);
F_true = zeros(n, 2);
sigma = zeros(n, 2);
omega = zeros(n, 1);
epsilon_hat = zeros(n, 1);
critic_rate_squared = zeros(n, 1);
critic_rate_single = zeros(n, 1);
critic_rate_actual = zeros(n, 1);
s1_dot_history = zeros(n, 2);
value_gradient = zeros(n, 4);
instant_cost_history = zeros(n, 1);

for k = 1:n
    WF = reshape(ctrl_states(k, 1:18).', 9, 2);
    Wc = ctrl_states(k, 19:27).';
    Z_F = plant_states(k, 1:4).';
    phi_F = AGV_RBF(Z_F, 'F');
    F_hat(k, :) = (WF'*phi_F).';

    z2_bar = z2(k, :).'-O2(k, :).';
    transform_input = [plant_states(k, 1); plant_states(k, 3); ...
        assist_states(k, 1); plant_states(k, 2); plant_states(k, 4); ...
        assist_states(k, 1)];
    transform_output = AGV_transfor(...
        t(k), transform_states(k, :).', transform_input, 3);
    sigma(k, :) = [transform_output(5), transform_output(8)];

    Z_J = [s1(k, :).'; z2_bar];
    [~, dphi_J] = AGV_RBF(Z_J, 'J');
    cf = 80000*(1 + 0.1*sin(0.01*t(k)));
    C = [cf/1832; 1.18*cf/2488];
    s1_dot = [-2*s1(k, 1) + 2*sigma(k, 1)*(z2_bar(1) + O2(k, 1)); ...
        -22*s1(k, 2) + 1.5*sigma(k, 2)*(z2_bar(2) + O2(k, 2))];
    s1_dot_history(k, :) = s1_dot.';
    z2_bar_dot_hat = F_hat(k, :).'+C*delta(k)+O2(k, :).';
    X_H_dot = [s1_dot; z2_bar_dot_hat];

    s1_now = s1(k, :).';
    k0 = 0.04;
    seed_weight = 1 + s1_now.^2;
    grad_s1_seed = k0*s1_now.*(z2_bar.^2);
    grad_z2_seed = k0*seed_weight.*z2_bar;
    grad_J_critic = [grad_s1_seed; grad_z2_seed] + dphi_J'*Wc;
    value_gradient(k, :) = grad_J_critic.';
    instant_cost = s1(k, :)*s1(k, :).'+z2_bar'*z2_bar+2*delta(k)^2;
    instant_cost_history(k) = instant_cost;
    epsilon_hat(k) = instant_cost + grad_J_critic'*X_H_dot;
    critic_regressor = dphi_J*X_H_dot;
    normalizer = 1 + critic_regressor'*critic_regressor;
    omega(k) = norm(critic_regressor);
    critic_rate_squared(k) = norm(-0.75*critic_regressor*epsilon_hat(k) ...
        /normalizer^2);
    critic_rate_single(k) = norm(-0.75*critic_regressor*epsilon_hat(k) ...
        /normalizer);
    critic_rate_actual(k) = norm(-0.005*critic_regressor*epsilon_hat(k) ...
        /normalizer - 0.0005*Wc);
end

z2_bar = z2-O2;
dz2_bar = [gradient(z2_bar(:, 1), t), gradient(z2_bar(:, 2), t)];
for k = 1:n
    cf = 80000*(1 + 0.1*sin(0.01*t(k)));
    C = [cf/1832; 1.18*cf/2488];
    F_true(k, :) = (dz2_bar(k, :).'-C*delta(k)-O2(k, :).').';
end

identifier_error = F_true-F_hat;
identifier_error_norm = vecnorm(identifier_error, 2, 2);
X_H_dot_data = [s1_dot_history, dz2_bar];
epsilon_data = instant_cost_history + sum(value_gradient.*X_H_dot_data, 2);
epsilon_gap = epsilon_hat-epsilon_data;

% Exclude one-sided/end-transient numerical derivatives from summary
% statistics. The full trajectories remain available in the report.
valid = t > 0.02 & t < t(end)-0.01;
if ~any(valid)
    valid = true(size(t));
end

report.t = t;
report.F_true = F_true;
report.F_hat = F_hat;
report.identifier_error = identifier_error;
report.identifier_error_norm = identifier_error_norm;
report.valid = valid;
report.identifier_error_rms = sqrt(mean(identifier_error_norm(valid).^2));
report.identifier_error_max = max(identifier_error_norm(valid));
report.omega_c = omega;
report.bellman_error = epsilon_hat;
report.epsilon_hat = epsilon_hat;
report.epsilon_data = epsilon_data;
report.epsilon_gap = epsilon_gap;
report.epsilon_hat_rms = sqrt(mean(epsilon_hat(valid).^2));
report.epsilon_data_rms = sqrt(mean(epsilon_data(valid).^2));
report.epsilon_gap_rms = sqrt(mean(epsilon_gap(valid).^2));
report.epsilon_gap_relative = report.epsilon_gap_rms ...
    /max(report.epsilon_hat_rms, eps);
report.critic_rate_squared = critic_rate_squared;
report.critic_rate_single = critic_rate_single;
report.critic_rate_actual = critic_rate_actual;
report.sigma = sigma;
report.min_margin_y = min([simulation.e_y(:)-simulation.eyl(:); ...
    simulation.eyu(:)-simulation.e_y(:)]);
report.min_margin_phi = min([simulation.e_phi(:)-simulation.ephil(:); ...
    simulation.ephiu(:)-simulation.e_phi(:)]);
report.max_delta = max(abs(delta));
report.max_delta_applied = max(abs(delta_applied));
report.max_saturation_gap = max(abs(delta-delta_applied));
report.max_O2 = max(vecnorm(O2,2,2));
report.Wc_end_norm = norm(ctrl_states(end,19:27));
Wa0 = [-14.9071198*0.03, -102.062194*0.005, ...
    -1.56893982*0.5, -0.718999721*0.2, zeros(1,5)];
report.Wa_move = norm(ctrl_states(end,28:36)-Wa0);
report.final_error = [simulation.e_y(end),simulation.e_phi(end)];

fprintf('AGV diagnostic stop time: %.9f s\n', t(end));
fprintf('minimum SFPPB margins [y, phi] = [%.9g, %.9g]\n', ...
    report.min_margin_y,report.min_margin_phi);
fprintf('max |delta| / |applied| / saturation gap = %.9g / %.9g / %.9g\n', ...
    report.max_delta,report.max_delta_applied,report.max_saturation_gap);
fprintf('max ||O2|| = %.9g, end ||Wc|| = %.9g, Actor move = %.9g\n', ...
    report.max_O2,report.Wc_end_norm,report.Wa_move);
fprintf('F_hat(end)  = [% .9g, % .9g]\n', F_hat(end, 1), F_hat(end, 2));
fprintf('F_true(end, one-sided raw) = [% .9g, % .9g]\n', ...
    F_true(end, 1), F_true(end, 2));
fprintf('|F_true-F_hat|(end, one-sided raw) = [% .9g, % .9g], norm = %.9g\n', ...
    abs(identifier_error(end, 1)), abs(identifier_error(end, 2)), ...
    identifier_error_norm(end));
fprintf('valid-window rms ||F_true-F_hat|| = %.9g\n', ...
    report.identifier_error_rms);
fprintf('valid-window max ||F_true-F_hat|| = %.9g\n', ...
    report.identifier_error_max);
fprintf('max ||omega_c|| = %.9g\n', max(omega));
fprintf('valid-window rms |epsilon_hat| = %.9g\n', report.epsilon_hat_rms);
fprintf('valid-window rms |epsilon_data| = %.9g\n', report.epsilon_data_rms);
fprintf('valid-window rms |epsilon_hat-epsilon_data| = %.9g\n', ...
    report.epsilon_gap_rms);
fprintf('residual-gap ratio = %.6g%%\n', 100*report.epsilon_gap_relative);
fprintf('valid-window max |epsilon_hat-epsilon_data| = %.9g\n', ...
    max(abs(epsilon_gap(valid))));
fprintf('max ||dWc||, squared normalization = %.9g\n', ...
    max(critic_rate_squared));
fprintf('max ||dWc||, single normalization = %.9g\n', ...
    max(critic_rate_single));
fprintf('max ||dWc||, active safe rate = %.9g\n', ...
    max(critic_rate_actual));
fprintf('end ||dWc||, squared normalization = %.9g\n', ...
    critic_rate_squared(end));
fprintf('end ||dWc||, single normalization = %.9g\n', ...
    critic_rate_single(end));
fprintf('end ||dWc||, active safe rate = %.9g\n', ...
    critic_rate_actual(end));

if make_plots
    figure('Name', 'AGV Bellman residual diagnostics', 'Color', 'w');
    tiledlayout(2, 1);
    nexttile;
    plot(t, epsilon_hat, 'b-', t, epsilon_data, 'r--', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('Bellman residual');
    legend('epsilon hat', 'epsilon data', 'Location', 'best');
    nexttile;
    plot(t, epsilon_gap, 'k-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('epsilon hat - epsilon data');
end
end

