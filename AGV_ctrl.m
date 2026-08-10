function [sys,x0,str,ts] = AGV_ctrl( ...
    t,x,u,flag,safety_lambda,learning_enabled)
% safety_lambda and learning_enabled are Level-1 S-Function parameters.
if nargin < 5 || isempty(safety_lambda)
    safety_lambda = 100;
end
if nargin < 6 || isempty(learning_enabled)
    learning_enabled = true;
end
switch flag
case 0
    [sys,x0,str,ts] = mdlInitializeSizes;
case 1
    sys = mdlDerivatives(t,x,u,safety_lambda,learning_enabled);
case 3
    sys = mdlOutputs(t,x,u,safety_lambda);
case {2,4,9}
    sys = [];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 38;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 8;
sizes.NumInputs      = 13;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

% [WF(9x2); Wc(9); Wa(9); O2(2)]
WF0 = zeros(9,2);
Wc0 = zeros(9,1);
Wa0 = admissibleActorWeights;
O20 = zeros(2,1);
x0 = [WF0(:); Wc0; Wa0; O20];
str = [];
ts = [0 0];

function sys = mdlDerivatives(t,x,u,safety_lambda,learning_enabled)
[WF,Wc,Wa,O2] = unpackStates(x);
[Z_F,Z_J,z2_bar,C] = controllerInputs(t,u,O2);

% Identifier: estimate the two components of the unknown z2 dynamics.
phi_F = AGV_RBF(Z_F,'F');
Gamma_F = 0.2;
sigma_F = 2.0;
dWF = Gamma_F*(phi_F*z2_bar' - sigma_F*WF);
F_hat = WF'*phi_F;

% Critic: approximate one scalar value function and obtain its
% two-dimensional z2 gradient analytically.
[~,dphi_J] = AGV_RBF(Z_J,'J');
% SFPPB barrier-conditioned positive seed value:
% J0 = 0.5*k0*sum((1+s1_i^2)*z2_bar_i^2).
% A channel approaching its performance boundary therefore dominates the
% single steering allocation without a fixed projection matrix.
s1 = Z_J(1:2);
k0 = 0.04;
seed_weight = 1 + s1.^2;
grad_s1_seed = k0*s1.*(z2_bar.^2);
grad_z2_seed = k0*seed_weight.*z2_bar;
Q1 = eye(2);
Q2 = eye(2);
r = 2;

grad_J_critic = [grad_s1_seed; grad_z2_seed] + dphi_J'*Wc;
psi_actor = actorFeatures(Z_F,Z_J);
delta_nominal = Wa'*psi_actor;
delta = sfppbSafetyFilter(t,Z_F,delta_nominal,C,safety_lambda);

% The vector auxiliary state exactly uses the steering signal applied to
% the vehicle, so z2_bar = z2 - O2 removes the saturation mismatch.
u_d = 0.5;
delta_applied = saturateSteering(delta,u_d);
dO2 = -O2 + C*(delta_applied - delta);

% Complete four-dimensional Bellman residual for X_H = [s1; z2_bar].
Sigma = diag([u(1), u(4)]);
D1 = diag([2, 1.5]);
C1 = diag([2, 22]);
s1_dot = -C1*s1 + D1*Sigma*(z2_bar + O2);
z2_bar_dot_hat = F_hat + C*delta + O2;
X_H_dot = [s1_dot; z2_bar_dot_hat];
instant_cost = s1'*Q1*s1 + z2_bar'*Q2*z2_bar + r*delta^2;
bellman_error = instant_cost + grad_J_critic'*X_H_dot;
critic_regressor = dphi_J*X_H_dot;
critic_normalizer = 1 + critic_regressor'*critic_regressor;
gamma_c = 0.005;
sigma_c = 0.0005;
dWc = -gamma_c*critic_regressor*bellman_error/critic_normalizer ...
    - sigma_c*Wc;

% Direct policy Actor. The initial admissible policy is encoded in Wa0;
% there is no separately summed baseline/residual control law.
hamiltonian_gradient = 2*r*delta_nominal ...
    + C'*grad_J_critic(3:4);
gamma_a = 0.001;
sigma_a = 0.02;
dWa = -gamma_a*psi_actor*hamiltonian_gradient ...
    /(1 + psi_actor'*psi_actor) ...
    - sigma_a*(Wa - admissibleActorWeights);

if ~learning_enabled
    dWF = zeros(size(dWF));
    dWc = zeros(size(dWc));
    dWa = zeros(size(dWa));
end

sys = [dWF(:); dWc; dWa; dO2];

function sys = mdlOutputs(t,x,u,safety_lambda)
[WF,Wc,Wa,O2] = unpackStates(x);
[Z_F,Z_J,~,C] = controllerInputs(t,u,O2);

psi_actor = actorFeatures(Z_F,Z_J);
delta_nominal = Wa'*psi_actor;
[delta,safety_conflict] = sfppbSafetyFilter( ...
    t,Z_F,delta_nominal,C,safety_lambda);

u_d = 0.5;
delta_applied = saturateSteering(delta,u_d);
weight_norm = norm([WF(:); Wc; Wa]);
WF_norm = norm(WF(:));
Wc_norm = norm(Wc);
Wa_move = norm(Wa - admissibleActorWeights);
% First three outputs preserve the original Simulink signal order. The
% remaining outputs are diagnostic logs only.
sys = [delta; delta_applied; weight_norm; delta_nominal; ...
    WF_norm; Wc_norm; Wa_move; double(safety_conflict)];

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

function Wa0 = admissibleActorWeights
actor_scales = [0.03; 0.005; 0.5; 0.2];
K0 = [14.9071198; 102.062194; 1.56893982; 0.718999721];
Wa0 = [-K0.*actor_scales; zeros(5,1)];

function psi = actorFeatures(Z_F,Z_J)
psi = [Z_F(1)/0.03; Z_F(2)/0.005; Z_F(3)/0.5; Z_F(4)/0.2; ...
    tanh(Z_J(1)/5); tanh(Z_J(2)/5); ...
    tanh(Z_J(3)/0.25); tanh(Z_J(4)/0.25); 1];

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
