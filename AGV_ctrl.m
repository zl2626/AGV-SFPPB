function [sys,x0,str,ts] = AGV_ctrl( ...
    t,x,u,flag,safety_lambda,learning_enabled,filter_gated_actor, ...
    control_weight,actuator_limit)
% The final five inputs are Level-1 S-Function parameters.
if nargin < 5 || isempty(safety_lambda)
    safety_lambda = 100;
end
if nargin < 6 || isempty(learning_enabled)
    learning_enabled = true;
end
if nargin < 7 || isempty(filter_gated_actor)
    filter_gated_actor = true;
end
if nargin < 8 || isempty(control_weight)
    control_weight = 0.25;
end
if nargin < 9 || isempty(actuator_limit)
    actuator_limit = 0.5;
end
switch flag
case 0
    [sys,x0,str,ts] = mdlInitializeSizes;
case 1
    sys = mdlDerivatives( ...
        t,x,u,safety_lambda,learning_enabled,filter_gated_actor, ...
        control_weight,actuator_limit);
case 3
    sys = mdlOutputs( ...
        t,x,u,safety_lambda,control_weight,actuator_limit);
case {2,4,9}
    sys = [];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 38;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 11;
sizes.NumInputs      = 13;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

% [WF(9x2); Wc(9); Wa(9); O2(2)]
WF0 = zeros(9,2);
Wc0 = zeros(9,1);
Wa0 = zeros(9,1);
O20 = zeros(2,1);
x0 = [WF0(:); Wc0; Wa0; O20];
str = [];
ts = [0 0];

function sys = mdlDerivatives( ...
    t,x,u,safety_lambda,learning_enabled,filter_gated_actor, ...
    control_weight,actuator_limit)
[WF,Wc,Wa,O2] = unpackStates(x);
[Z_F,Z_J,z2_bar,C] = controllerInputs(t,u,O2);

% Identifier: estimate the two components of the unknown z2 dynamics.
phi_F = AGV_RBF(Z_F,'F');
Gamma_F = 0.2;
sigma_F = 2.0;
dWF = Gamma_F*(phi_F*z2_bar' - sigma_F*WF);
F_hat = WF'*phi_F;

% Critic and Actor share the centered value-function basis and its
% analytical gradient with respect to X_H = [s1; z2_bar].
[~,dphi_J] = AGV_RBF(Z_J,'J');
s1 = Z_J(1:2);
r = control_weight;
u_d = actuator_limit;
delta_scale = 0.5;
s1_scale = [2; 1.5];
z2_scale = [0.5; 0.2];
Q1 = diag(1./s1_scale.^2);
Q2 = diag(1./z2_scale.^2);
R_delta = r/delta_scale^2;

delta_admissible = admissiblePolicy(Z_F);
grad_J_admissible = admissibleValueGradient( ...
    Z_J,C,delta_admissible,R_delta);
grad_J_critic = grad_J_admissible + dphi_J'*Wc;
grad_J_actor = grad_J_admissible + dphi_J'*Wa;
psi_actor = -(dphi_J(:,3:4)*C)/(2*R_delta);
delta_nominal = -(C'*grad_J_actor(3:4))/(2*R_delta);
[delta,~] = sfppbSafetyFilter(t,Z_F,delta_nominal,C,safety_lambda);

% The vector auxiliary state exactly uses the steering signal applied to
% the vehicle, so z2_bar = z2 - O2 removes the saturation mismatch.
delta_applied = saturateSteering(delta,u_d);
dO2 = -O2 + C*(delta_applied - delta);

% Complete four-dimensional Bellman residual for X_H = [s1; z2_bar].
Sigma = diag([u(1), u(4)]);
D1 = diag([2, 1.5]);
C1 = diag([2, 22]);
s1_dot = -C1*s1 + D1*Sigma*(z2_bar + O2);
z2_bar_dot_hat = F_hat + C*delta + O2;
X_H_dot = [s1_dot; z2_bar_dot_hat];
instant_cost = s1'*Q1*s1 + z2_bar'*Q2*z2_bar ...
    + R_delta*delta^2;
bellman_error = instant_cost + grad_J_critic'*X_H_dot;
critic_regressor = dphi_J*X_H_dot;
critic_normalizer = 1 + critic_regressor'*critic_regressor;
gamma_c = 0.005;
sigma_c = 0.0005;
dWc = -gamma_c*critic_regressor*bellman_error/critic_normalizer ...
    - sigma_c*Wc;

% Value-gradient Actor. Wa learns a residual value function over the same
% basis as the Critic, and the policy follows directly from the HJB
% stationary condition delta = -(1/(2r))*C'*grad_z2(Ja).
hamiltonian_gradient = 2*R_delta*delta_nominal ...
    + C'*grad_J_critic(3:4);
gamma_a = 0.001;
sigma_a = 0.02;
dWa_hamiltonian = -gamma_a*psi_actor*hamiltonian_gradient ...
    /(1 + psi_actor'*psi_actor) ...
    - gamma_a*(Wa-Wc);
filter_active = abs(delta-delta_nominal) > 1e-8;
if filter_gated_actor && filter_active
    dWa_hamiltonian = zeros(size(Wa));
end
dWa = dWa_hamiltonian - sigma_a*Wa;

if ~learning_enabled
    dWF = zeros(size(dWF));
    dWc = zeros(size(dWc));
    dWa = zeros(size(dWa));
end

sys = [dWF(:); dWc; dWa; dO2];

function sys = mdlOutputs( ...
    t,x,u,safety_lambda,control_weight,actuator_limit)
[WF,Wc,Wa,O2] = unpackStates(x);
[Z_F,Z_J,~,C] = controllerInputs(t,u,O2);

[~,dphi_J] = AGV_RBF(Z_J,'J');
delta_admissible = admissiblePolicy(Z_F);
u_d = actuator_limit;
delta_scale = 0.5;
R_delta = control_weight/delta_scale^2;
grad_J_admissible = admissibleValueGradient( ...
    Z_J,C,delta_admissible,R_delta);
grad_J_actor = grad_J_admissible + dphi_J'*Wa;
delta_nominal = -(C'*grad_J_actor(3:4))/(2*R_delta);
[delta,safety_conflict] = sfppbSafetyFilter( ...
    t,Z_F,delta_nominal,C,safety_lambda);

delta_applied = saturateSteering(delta,u_d);
weight_norm = norm([WF(:); Wc; Wa]);
WF_norm = norm(WF(:));
Wc_norm = norm(Wc);
Wa_move = norm(Wa);
delta_RL = delta_nominal-delta_admissible;
delta_safety_correction = delta-delta_nominal;
% First three outputs preserve the original Simulink signal order. The
% remaining outputs are diagnostic logs only. The final three expose the
% admissible, learned, and safety-filter steering contributions.
sys = [delta; delta_applied; weight_norm; delta_nominal; ...
    WF_norm; Wc_norm; Wa_move; double(safety_conflict); ...
    delta_admissible; delta_RL; delta_safety_correction];

function [WF,Wc,Wa,O2] = unpackStates(x)
WF = reshape(x(1:18),9,2);
Wc = x(19:27);
Wa = x(28:36);
O2 = x(37:38);

function [Z_F,Z_J,z2_bar,C] = controllerInputs(t,u,O2)
% Existing 13-input Simulink interface:
% [sigma_y,s2y,s1y,sigma_phi,s2phi,s1phi,e_y,e_phi,...
%  de_y,de_phi,z2_y,z2_phi,w].
s1 = [u(3); u(6)];
e_y = u(7);
e_phi = u(8);
de_y = u(9);
de_phi = u(10);
z2 = [u(11); u(12)];

Z_F = [e_y; e_phi; de_y; de_phi];
z2_bar = z2 - O2;
Z_J = [s1; z2_bar];

m = 1832;
Iz = 2488;
lf = 1.18;
cf = 80000*(1 + 0.1*sin(0.01*t));
C = [cf/m; lf*cf/Iz];

function delta = admissiblePolicy(Z_F)
% The CARE/LQR policy supplies the admissible control-direction gradient.
K0 = [14.9071198; 102.062194; 1.56893982; 0.718999721];
delta = -K0'*Z_F;

function gradient = admissibleValueGradient( ...
    Z_J,C,delta_admissible,R_delta)
% Its z2 component is the minimum-norm value gradient that recovers the
% admissible policy through delta = -(1/(2r))*C'*grad_z2(J).
s1 = Z_J(1:2);
z2_bar = Z_J(3:4);
k0 = 0.04;
grad_s1 = k0*s1.*(z2_bar.^2);
grad_z2 = -(2*R_delta*delta_admissible/(C'*C))*C;
gradient = [grad_s1; grad_z2];

function delta_applied = saturateSteering(delta,u_d)
if abs(delta) > u_d
    delta_applied = u_d*sign(delta);
else
    delta_applied = delta;
end

function [delta,interval_conflict] = sfppbSafetyFilter( ...
    t,Z_F,delta_nominal,C,lambda)
delta = delta_nominal;
interval_conflict = false;
if t < 5
    return;
end

e = Z_F(1:2);
de = Z_F(3:4);
m = 1832;
Iz = 2488;
lf = 1.18;
lr = 1.77;
vx = 20;
cf = 80000*(1 + 0.1*sin(0.01*t));
cr = 120000*(1 + 0.1*sin(0.01*t));
A11 = -(cf + cr)/(m*vx);
A12 = (-lf*cf + lr*cr)/(m*vx) - vx;
A21 = (-lf*cf + lr*cr)/(Iz*vx);
A22 = -(lf^2*cf + lr^2*cr)/(Iz*vx);
f = [(cf + cr)*e(2)/m + A11*de(1) + A12*de(2); ...
    (-lf*cf - lr*cr)*e(2)/Iz + A21*de(1) + A22*de(2)];

terminal_bound = [0.03; 0.005];
disturbance_bound = [25; 25];
h_lower = e + terminal_bound;
h_upper = terminal_bound - e;
lower_each = (-f + disturbance_bound - 2*lambda*de ...
    - lambda^2*h_lower)./C;
upper_each = (-f - disturbance_bound - 2*lambda*de ...
    + lambda^2*h_upper)./C;
lower = max([-0.5; lower_each]);
upper = min([0.5; upper_each]);

if lower <= upper
    delta = min(max(delta_nominal,lower),upper);
else
    interval_conflict = true;
    risks = [h_lower./terminal_bound; h_upper./terminal_bound];
    [~,critical] = min(risks);
    if critical <= 2
        delta = max(-0.5,min(0.5,lower_each(critical)));
    else
        channel = critical - 2;
        delta = max(-0.5,min(0.5,upper_each(channel)));
    end
end
