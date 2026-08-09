function [sys,x0,str,ts] = AGV_transfor(t,x,u,flag)
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
sizes.NumContStates  = 5;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 21;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

% [z1y_int; z1phi_int; z2y_int; z2phi_int; w]
x0 = [0; 0; 0; 0; 1];
str = [];
ts = [0 0];

function sys = mdlDerivatives(t,x,u)
% =========================================================
% 1. SFPPB parameters
% =========================================================
% The first version uses l_kappa = l_s = 1.  The initial-error
% shifting term is kept explicit so that the boundary is readable.
kappa0_y = 0.10;
kappaT_y = 0.03;
T_y = 5;
lambda1_y = 0.15;
lambda2_y = 0.15;
eta_shape_y = 1.0;
e0_y = -0.10;

kappa0_phi = 0.02;
kappaT_phi = 0.005;
T_phi = 5;
lambda1_phi = 0.03;
lambda2_phi = 0.03;
eta_shape_phi = 1.0;
e0_phi = 0.01;

% =========================================================
% 2. Existing backstepping signals and states
% =========================================================
c1y = 2;
c1phi = 22;
k1y = 0.1;
k1phi = 0.2;
k2y = 0.01;
k2phi = 0.01;

z1y_int = x(1);
z1phi_int = x(2);
z2y_int = x(3);
z2phi_int = x(4);

e_y = u(1);
e_phi = u(4);
X2 = [u(2); u(5)];
eta_y = u(3);
eta_phi = u(6);
w = x(5);

% =========================================================
% 3. Sliding flexible prescribed performance boundaries
% =========================================================
if t < T_y
    angle_y = pi*t/(2*T_y);
    kappa_y = kappa0_y + (kappaT_y - kappa0_y)*sin(angle_y);
    shift_y = 1 - sin(angle_y);
else
    kappa_y = kappaT_y;
    shift_y = 0;
end

if t < T_phi
    angle_phi = pi*t/(2*T_phi);
    kappa_phi = kappa0_phi + (kappaT_phi - kappa0_phi)*sin(angle_phi);
    shift_phi = 1 - sin(angle_phi);
else
    kappa_phi = kappaT_phi;
    shift_phi = 0;
end

% Horizontal error: e0_y < 0.
if e0_y < 0
    B_lower_y = -kappa_y + e0_y*shift_y ...
        - lambda1_y*tanh(eta_y);
    B_upper_y = eta_shape_y*kappa_y + e0_y*shift_y ...
        + lambda2_y*tanh(eta_y);
else
    B_lower_y = -eta_shape_y*kappa_y + e0_y*shift_y ...
        - lambda1_y*tanh(eta_y);
    B_upper_y = kappa_y + e0_y*shift_y ...
        + lambda2_y*tanh(eta_y);
end

% Heading error: e0_phi > 0.
if e0_phi < 0
    B_lower_phi = -kappa_phi + e0_phi*shift_phi ...
        - lambda1_phi*tanh(eta_phi);
    B_upper_phi = eta_shape_phi*kappa_phi + e0_phi*shift_phi ...
        + lambda2_phi*tanh(eta_phi);
else
    B_lower_phi = -eta_shape_phi*kappa_phi + e0_phi*shift_phi ...
        - lambda1_phi*tanh(eta_phi);
    B_upper_phi = kappa_phi + e0_phi*shift_phi ...
        + lambda2_phi*tanh(eta_phi);
end

% Nominal SPPB without the flexible relaxation, for diagnostics only.
nominal_lower_y = B_lower_y + lambda1_y*tanh(eta_y);
nominal_upper_y = B_upper_y - lambda2_y*tanh(eta_y);
nominal_lower_phi = B_lower_phi + lambda1_phi*tanh(eta_phi);
nominal_upper_phi = B_upper_phi - lambda2_phi*tanh(eta_phi);

% =========================================================
% 4. NMT and the existing first backstepping layer
% =========================================================
z1_y = log((e_y - B_lower_y)/(B_upper_y - e_y));
z1_phi = log((e_phi - B_lower_phi)/(B_upper_phi - e_phi));

sigma_y = (B_upper_y - B_lower_y) ...
    / ((B_upper_y - e_y)*(e_y - B_lower_y));
sigma_phi = (B_upper_phi - B_lower_phi) ...
    / ((B_upper_phi - e_phi)*(e_phi - B_lower_phi));
sigma_y_ni = ((B_upper_y - e_y)*(e_y - B_lower_y)) ...
    / (B_upper_y - B_lower_y);
sigma_phi_ni = ((B_upper_phi - e_phi)*(e_phi - B_lower_phi)) ...
    / (B_upper_phi - B_lower_phi);

s1y = 2*z1_y + k1y*z1y_int;
s1phi = 1.5*z1_phi + k1phi*z1phi_int;
s1 = [s1y; s1phi];

sigma_ni = diag([sigma_y_ni, sigma_phi_ni]);
TT_y = B_upper_y/(B_upper_y - e_y) ...
    + B_lower_y/(e_y - B_lower_y);
TT_phi = B_upper_phi/(B_upper_phi - e_phi) ...
    + B_lower_phi/(e_phi - B_lower_phi);
T = [0.01*TT_y; 0.01*TT_phi];
z1 = [z1_y; z1_phi];
K1 = diag([k1y, k1phi]);
C1 = diag([c1y, c1phi]);

theta_1 = diag([k2y*sigma_y, k2phi*sigma_phi]);
rho_1 = 10*exp(-0.02);
denom_1 = sqrt(s1'*s1*norm(theta_1,2)^2 + rho_1^2);
eta_1 = (s1*norm(theta_1,2)^2)/denom_1;

% Keep the original virtual-control/backstepping layer in version 1.
alpha_1 = sigma_ni*(-C1*s1 + T - K1*z1) + w*eta_1;
z2 = X2 - alpha_1;
z2_y = z2(1);
z2_phi = z2(2);

s2y = 2*z2_y + k2y*z2y_int;
s2phi = 1.5*z2_phi + k2phi*z2phi_int;
s2 = [s2y; s2phi];

% Existing auxiliary state w dynamics.
c1 = 10;
rho_val = 10*exp(-0.02*t);
k2 = diag([k2y, k2phi]);
theta_2 = [1; k2*z2];
denom_2 = sqrt(s2'*s2*(theta_2'*theta_2) + rho_val^2);
eta_2 = (s2*(theta_2'*theta_2))/denom_2;
tau_1 = s1'*eta_1 - c1*w;
dw_dt = tau_1 + s2'*eta_2;

sys = [z1_y; z1_phi; z2_y; z2_phi; dw_dt];

function sys = mdlOutputs(t,x,u)
% Keep this calculation in the same order as mdlDerivatives so that
% the Level-1 S-Function preserves the original 21-output interface.

% =========================================================
% 1. SFPPB parameters
% =========================================================
kappa0_y = 0.10;
kappaT_y = 0.03;
T_y = 5;
lambda1_y = 0.15;
lambda2_y = 0.15;
eta_shape_y = 1.0;
e0_y = -0.10;

kappa0_phi = 0.02;
kappaT_phi = 0.005;
T_phi = 5;
lambda1_phi = 0.03;
lambda2_phi = 0.03;
eta_shape_phi = 1.0;
e0_phi = 0.01;

% =========================================================
% 2. Existing backstepping signals and states
% =========================================================
c1y = 2;
c1phi = 22;
k1y = 0.1;
k1phi = 0.2;
k2y = 0.01;
k2phi = 0.01;

z1y_int = x(1);
z1phi_int = x(2);
z2y_int = x(3);
z2phi_int = x(4);

e_y = u(1);
e_phi = u(4);
X2 = [u(2); u(5)];
eta_y = u(3);
eta_phi = u(6);
w = x(5);

% =========================================================
% 3. Sliding flexible prescribed performance boundaries
% =========================================================
if t < T_y
    angle_y = pi*t/(2*T_y);
    kappa_y = kappa0_y + (kappaT_y - kappa0_y)*sin(angle_y);
    shift_y = 1 - sin(angle_y);
else
    kappa_y = kappaT_y;
    shift_y = 0;
end

if t < T_phi
    angle_phi = pi*t/(2*T_phi);
    kappa_phi = kappa0_phi + (kappaT_phi - kappa0_phi)*sin(angle_phi);
    shift_phi = 1 - sin(angle_phi);
else
    kappa_phi = kappaT_phi;
    shift_phi = 0;
end

if e0_y < 0
    B_lower_y = -kappa_y + e0_y*shift_y ...
        - lambda1_y*tanh(eta_y);
    B_upper_y = eta_shape_y*kappa_y + e0_y*shift_y ...
        + lambda2_y*tanh(eta_y);
else
    B_lower_y = -eta_shape_y*kappa_y + e0_y*shift_y ...
        - lambda1_y*tanh(eta_y);
    B_upper_y = kappa_y + e0_y*shift_y ...
        + lambda2_y*tanh(eta_y);
end

if e0_phi < 0
    B_lower_phi = -kappa_phi + e0_phi*shift_phi ...
        - lambda1_phi*tanh(eta_phi);
    B_upper_phi = eta_shape_phi*kappa_phi + e0_phi*shift_phi ...
        + lambda2_phi*tanh(eta_phi);
else
    B_lower_phi = -eta_shape_phi*kappa_phi + e0_phi*shift_phi ...
        - lambda1_phi*tanh(eta_phi);
    B_upper_phi = kappa_phi + e0_phi*shift_phi ...
        + lambda2_phi*tanh(eta_phi);
end

nominal_lower_y = B_lower_y + lambda1_y*tanh(eta_y);
nominal_upper_y = B_upper_y - lambda2_y*tanh(eta_y);
nominal_lower_phi = B_lower_phi + lambda1_phi*tanh(eta_phi);
nominal_upper_phi = B_upper_phi - lambda2_phi*tanh(eta_phi);

% =========================================================
% 4. NMT and the existing first backstepping layer
% =========================================================
z1_y = log((e_y - B_lower_y)/(B_upper_y - e_y));
z1_phi = log((e_phi - B_lower_phi)/(B_upper_phi - e_phi));

sigma_y = (B_upper_y - B_lower_y) ...
    / ((B_upper_y - e_y)*(e_y - B_lower_y));
sigma_phi = (B_upper_phi - B_lower_phi) ...
    / ((B_upper_phi - e_phi)*(e_phi - B_lower_phi));
sigma_y_ni = ((B_upper_y - e_y)*(e_y - B_lower_y)) ...
    / (B_upper_y - B_lower_y);
sigma_phi_ni = ((B_upper_phi - e_phi)*(e_phi - B_lower_phi)) ...
    / (B_upper_phi - B_lower_phi);

s1y = 2*z1_y + k1y*z1y_int;
s1phi = 1.5*z1_phi + k1phi*z1phi_int;
s1 = [s1y; s1phi];

sigma_ni = diag([sigma_y_ni, sigma_phi_ni]);
TT_y = B_upper_y/(B_upper_y - e_y) ...
    + B_lower_y/(e_y - B_lower_y);
TT_phi = B_upper_phi/(B_upper_phi - e_phi) ...
    + B_lower_phi/(e_phi - B_lower_phi);
T = [0.01*TT_y; 0.01*TT_phi];
z1 = [z1_y; z1_phi];
K1 = diag([k1y, k1phi]);
C1 = diag([c1y, c1phi]);

theta_1 = diag([k2y*sigma_y, k2phi*sigma_phi]);
rho_1 = 10*exp(-0.02);
denom_1 = sqrt(s1'*s1*norm(theta_1,2)^2 + rho_1^2);
eta_1 = (s1*norm(theta_1,2)^2)/denom_1;

alpha_1 = sigma_ni*(-C1*s1 + T - K1*z1) + w*eta_1;
z2 = X2 - alpha_1;
z2_y = z2(1);
z2_phi = z2(2);

s2y = 2*z2_y + k2y*z2y_int;
s2phi = 1.5*z2_phi + k2phi*z2phi_int;

% Preserve the original output ordering used by the Simulink Demux blocks.
sys = [B_upper_y; B_lower_y; B_upper_phi; B_lower_phi; ...
       sigma_y; s2y; s1y; sigma_phi; s2phi; s1phi; ...
       z2_y; z2_phi; w; z1_y; z1_phi; z2_y; z2_phi; ...
       nominal_lower_y; nominal_upper_y; ...
       nominal_lower_phi; nominal_upper_phi];
