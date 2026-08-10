function [sys,x0,str,ts] = AGV_ctrl(t,x,u,flag)
switch flag
case 0
    [sys,x0,str,ts] = mdlInitializeSizes;
case 1
    sys = mdlDerivatives(t,x,u);
case 3
    sys = mdlOutputs(t,x,u);
case {2,4,9}
    sys = [];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 38;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
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

function sys = mdlDerivatives(t,x,u)
[WF,Wc,Wa,O2] = unpackStates(x);
[Z_F,Z_J,z2_bar,C] = controllerInputs(t,u,O2);

% Identifier: estimate the two components of the unknown z2 dynamics.
phi_F = AGV_RBF(Z_F,'F');
Gamma_F = 0.2;
sigma_F = 2.0;
dWF = Gamma_F*(phi_F*z2_bar' - sigma_F*WF);
F_hat = WF'*phi_F;

% Critic/Actor: approximate one scalar value function and obtain its
% two-dimensional gradient analytically. No scalar error projection is
% introduced in this interface.
[~,dphi_J] = AGV_RBF(Z_J,'J');
dphi_dz2 = dphi_J(:,3:4);
% Equal-channel quadratic term supplies a bounded initial value gradient
% while the RBF actor learns the state-dependent correction. Its ability
% to stabilize the full AGV/SFPPB dynamics must be verified separately.
grad_J_seed = 0.04*z2_bar;
Q1 = eye(2);
Q2 = eye(2);
r = 2;

grad_J_critic = [zeros(2,1); grad_J_seed] + dphi_J'*Wc;
grad_J_actor = grad_J_seed + dphi_dz2'*Wa;
delta = -(C'*grad_J_actor)/(2*r);

% The vector auxiliary state exactly uses the steering signal applied to
% the vehicle, so z2_bar = z2 - O2 removes the saturation mismatch.
u_d = 0.5;
delta_applied = saturateSteering(delta,u_d);
dO2 = -O2 + C*(delta_applied - delta);

% Complete four-dimensional Bellman residual for X_H = [s1; z2_bar].
s1 = Z_J(1:2);
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
gamma_c = 0.75;
dWc = -gamma_c*critic_regressor*bellman_error/critic_normalizer^2;

% The actor represents the same value function and tracks the critic.
gamma_a = 1.0;
dWa = -gamma_a*(Wa - Wc);

sys = [dWF(:); dWc; dWa; dO2];

function sys = mdlOutputs(t,x,u)
[WF,Wc,Wa,O2] = unpackStates(x);
[~,Z_J,z2_bar,C] = controllerInputs(t,u,O2);

[~,dphi_J] = AGV_RBF(Z_J,'J');
dphi_dz2 = dphi_J(:,3:4);
grad_J_seed = 0.04*z2_bar;
r = 2;
grad_J_actor = grad_J_seed + dphi_dz2'*Wa;
delta = -(C'*grad_J_actor)/(2*r);

u_d = 0.5;
delta_applied = saturateSteering(delta,u_d);
weight_norm = norm([WF(:); Wc; Wa]);
sys = [delta; delta_applied; weight_norm];

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

function delta_applied = saturateSteering(delta,u_d)
if abs(delta) > u_d
    delta_applied = u_d*sign(delta);
else
    delta_applied = delta;
end
