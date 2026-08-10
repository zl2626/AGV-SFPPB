function report = AGV_diagnose(stop_time)
%AGV_DIAGNOSE Reconstruct Identifier and Critic diagnostics offline.
%   report = AGV_diagnose() runs the model only up to the last safe point
%   before the current SFPPB singularity and reports:
%       |F_true - F_hat|, ||omega_c||, |epsilon_H|, and both critic rates.
%   The model configuration is changed only in memory and is not saved.
%
%   F_true is reconstructed from the logged z2 trajectory:
%       F_true = d(z2-O2)/dt - C*delta_applied - O2.
%   This is a diagnostic estimate, not a replacement for a plant-side
%   Identifier state model.

if nargin < 1
    stop_time = 1.8945;
end

model = 'AGV_simulate';
load_system(model);
cleanup_model = onCleanup(@() close_system(model, 0)); %#ok<NASGU>

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
bellman_error = zeros(n, 1);
critic_rate_squared = zeros(n, 1);
critic_rate_single = zeros(n, 1);

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
    z2_bar_dot_hat = F_hat(k, :).'+C*delta(k)+O2(k, :).';
    X_H_dot = [s1_dot; z2_bar_dot_hat];

    grad_J_critic = [0; 0; 0.04*z2_bar] + dphi_J'*Wc;
    instant_cost = s1(k, :)*s1(k, :).'+z2_bar'*z2_bar+2*delta(k)^2;
    bellman_error(k) = instant_cost + grad_J_critic'*X_H_dot;
    critic_regressor = dphi_J*X_H_dot;
    normalizer = 1 + critic_regressor'*critic_regressor;
    omega(k) = norm(critic_regressor);
    critic_rate_squared(k) = norm(-0.75*critic_regressor*bellman_error(k) ...
        /normalizer^2);
    critic_rate_single(k) = norm(-0.75*critic_regressor*bellman_error(k) ...
        /normalizer);
end

z2_bar = z2-O2;
dz2_bar = [gradient(z2_bar(:, 1), t), gradient(z2_bar(:, 2), t)];
for k = 1:n
    cf = 80000*(1 + 0.1*sin(0.01*t(k)));
    C = [cf/1832; 1.18*cf/2488];
    F_true(k, :) = (dz2_bar(k, :).'-C*delta_applied(k)-O2(k, :).').';
end

identifier_error = F_true-F_hat;
identifier_error_norm = vecnorm(identifier_error, 2, 2);

report.t = t;
report.F_true = F_true;
report.F_hat = F_hat;
report.identifier_error = identifier_error;
report.identifier_error_norm = identifier_error_norm;
report.identifier_error_rms = sqrt(mean(identifier_error_norm.^2));
report.omega_c = omega;
report.bellman_error = bellman_error;
report.critic_rate_squared = critic_rate_squared;
report.critic_rate_single = critic_rate_single;
report.sigma = sigma;

fprintf('AGV diagnostic stop time: %.9f s\n', t(end));
fprintf('F_hat(end)  = [% .9g, % .9g]\n', F_hat(end, 1), F_hat(end, 2));
fprintf('F_true(end) = [% .9g, % .9g]\n', F_true(end, 1), F_true(end, 2));
fprintf('|F_true-F_hat|(end) = [% .9g, % .9g], norm = %.9g\n', ...
    abs(identifier_error(end, 1)), abs(identifier_error(end, 2)), ...
    identifier_error_norm(end));
fprintf('max ||F_true-F_hat|| = %.9g\n', max(identifier_error_norm));
fprintf('rms ||F_true-F_hat|| = %.9g\n', report.identifier_error_rms);
fprintf('max ||omega_c|| = %.9g\n', max(omega));
fprintf('max |epsilon_H| = %.9g\n', max(abs(bellman_error)));
fprintf('max ||dWc||, squared normalization = %.9g\n', ...
    max(critic_rate_squared));
fprintf('max ||dWc||, single normalization = %.9g\n', ...
    max(critic_rate_single));
fprintf('end ||dWc||, squared normalization = %.9g\n', ...
    critic_rate_squared(end));
fprintf('end ||dWc||, single normalization = %.9g\n', ...
    critic_rate_single(end));
end

