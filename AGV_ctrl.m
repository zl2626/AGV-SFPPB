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
sizes.NumContStates  = 22;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 13;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

% [WF(1:7); Wc(1:7); Wa(1:7); O]
WF0 = zeros(7,1);
Wc0 = 0.1*ones(7,1);
Wa0 = 0.1*ones(7,1);
O0 = 0;
x0 = [WF0; Wc0; Wa0; O0];
str = [];
ts = [0 0];

function sys = mdlDerivatives(t,x,u)
% =========================================================
% 1. NN weights and input-saturation auxiliary state
% =========================================================
WF = x(1:7);
Wc = x(8:14);
Wa = x(15:21);
O = x(22);

% =========================================================
% 2. Signals from AGV_transfor.m
% =========================================================
% The 13-input interface is unchanged from the original Simulink model.
s2y = u(2);
s2phi = u(5);
e_y = u(7);
e_phi = u(8);
de_y = u(9);
de_phi = u(10);
z2 = [u(11); u(12)];

X1 = [e_y; e_phi];
X2 = [de_y; de_phi];
Z2 = [X1; X2];

% =========================================================
% 3. Actual steering-input direction of the AGV
% =========================================================
m = 1832;
Iz = 2488;
lf = 1.18;
cf = 80000*(1 + 0.1*sin(0.01*t));

c_y = cf/m;
c_phi = lf*cf/Iz;
C = [c_y; c_phi];
C_norm = norm(C);

% =========================================================
% 4. Seven-node RBF feature vector
% =========================================================
phi = AGV_RBF(Z2);

% =========================================================
% 5. Combined controllable error and Identifier
% =========================================================
% The two virtual errors are projected onto the single physical
% steering direction; O compensates the input saturation mismatch.
z2_control = C'*z2/C_norm - O;

Gamma_F = 0.2;
sigma_F = 2.0;
dWF = Gamma_F*(z2_control*phi - sigma_F*WF);

% =========================================================
% 6. Critic
% =========================================================
gamma_c = 0.75;
dWc = -gamma_c*phi*(phi'*Wc);

% =========================================================
% 7. Actor
% =========================================================
gamma_a = 1.0;
actor_error = gamma_a*(Wa - Wc) + gamma_c*Wc;
dWa = -phi*(phi'*actor_error);

% =========================================================
% 8. SFPPB-RL steering controller and O dynamics
% =========================================================
c2 = 30;
F_hat = WF'*phi;
actor_term = Wa'*phi;

delta = (-c2*z2_control - F_hat - 0.5*actor_term)/C_norm;

u_d = 0.5;
k_delta = u_d*tanh(delta/u_d);
dO = -O + (k_delta - delta);

sys = [dWF; dWc; dWa; dO];

function sys = mdlOutputs(t,x,u)
% Keep the output calculation explicit and in the same order as the
% derivative calculation.  The Simulink output interface stays 3-wide.

% =========================================================
% 1. NN weights and input-saturation auxiliary state
% =========================================================
Wc = x(8:14); %#ok<NASGU>
Wa = x(15:21);
O = x(22);

% =========================================================
% 2. Signals from AGV_transfor.m
% =========================================================
e_y = u(7);
e_phi = u(8);
de_y = u(9);
de_phi = u(10);
z2 = [u(11); u(12)];

X1 = [e_y; e_phi];
X2 = [de_y; de_phi];
Z2 = [X1; X2];

% =========================================================
% 3. Actual steering-input direction of the AGV
% =========================================================
m = 1832;
Iz = 2488;
lf = 1.18;
cf = 80000*(1 + 0.1*sin(0.01*t));

c_y = cf/m;
c_phi = lf*cf/Iz;
C = [c_y; c_phi];
C_norm = norm(C);

% =========================================================
% 4. RBF, combined error, and SFPPB-RL steering law
% =========================================================
phi = AGV_RBF(Z2);
z2_control = C'*z2/C_norm - O;

WF = x(1:7);
F_hat = WF'*phi;
actor_term = Wa'*phi;

c2 = 30;
delta = (-c2*z2_control - F_hat - 0.5*actor_term)/C_norm;

% Keep the hard actuator limit used by the vehicle model.
u_d = 0.5;
if abs(delta) > u_d
    delta_sat = u_d*sign(delta);
else
    delta_sat = delta;
end

% Third output is the Actor norm for observing weight growth.
sys = [delta; delta_sat; norm(Wa)];
