function [sys,x0,str,ts] = AGV_transfor(t,x,u,flag)
switch flag,
case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
case 1,
    sys=mdlDerivatives(t,x,u);
case 3,
    sys=mdlOutputs(t,x,u);
case {2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts]=mdlInitializeSizes
global rho_0_y rho_T_y delta_y e_0_y  T_y a_y lambda1_y lambda2_y;
global rho_0_phi rho_T_phi delta_phi e_0_phi  T_phi a_phi lambda1_phi lambda2_phi;
global rho0_y rhoTs_y mu_bar_y mu_under_y;
global rho0_phi rhoTs_phi mu_bar_phi mu_under_phi;

sizes = simsizes;
sizes.NumContStates  = 5; 
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 21;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = [0; 0; 0; 0; 1];
str = [];
ts = [0 0];

rho_0_y = 0.402;       % 初始边界
rho_T_y = 0.304;       % 稳态精度
delta_y = 0.329;        % 边界宽度
e_0_y = -0.1;          
T_y = 1;              % 收敛时间
a_y = 1;            % 收敛速度
lambda1_y = 0.4;
lambda2_y = 0.4;

rho0_y = 0.5; 
rhoTs_y = 0.3; 
mu_bar_y = 0.28; % 0.28
mu_under_y = 0.35;% 0.35

rho_0_phi = 0.207;     % 初始边界
rho_T_phi = 0.196;     % 稳态精度
delta_phi = 0.09;      % 边界宽度
e_0_phi = 0.01;
T_phi = 1;            % 收敛时间
a_phi = 1;          % 收敛速度
lambda1_phi = 0.4;
lambda2_phi = 0.4;

rho0_phi = 0.3; 
rhoTs_phi = 0.25;
mu_bar_phi = 0.05; %0.05
mu_under_phi = 0.09;%0.09

% rho_0_y = 0.402;       % 初始边界
% rho_T_y = 0.304;       % 稳态精度
% delta_y = 0.15;        % 边界宽度
% e_0_y = -0.1;          
% T_y = 5;              % 收敛时间
% a_y = 1;            % 收敛速度
% lambda1_y = 0.02;
% lambda2_y = 0.02;
% 
% rho0_y = 0.5; 
% rhoTs_y = 0.3; 
% mu_bar_y = 0.28; % 0.28
% mu_under_y = 0.35;% 0.35
% 
% rho_0_phi = 0.207;     % 初始边界
% rho_T_phi = 0.196;     % 稳态精度
% delta_phi = 0.08;      % 边界宽度
% e_0_phi = 0.01;
% T_phi = 1;            % 收敛时间
% a_phi = 1;          % 收敛速度
% lambda1_phi = 0.02;
% lambda2_phi = 0.02;
% 
% rho0_phi = 0.3; 
% rhoTs_phi = 0.25;
% mu_bar_phi = 0.05; %0.05
% mu_under_phi = 0.09;%0.09

% rho_0_y = 0.4;       % 初始边界
% rho_T_y = 0.1;       % 稳态精度
% delta_y = 0.329;        % 边界宽度
% e_0_y = -1.0;          
% T_y = 1;              % 收敛时间
% a_y = 1;            % 收敛速度
% lambda1_y = 0;
% lambda2_y = 0;
% 
% rho0_y = 0.7;
% rhoTs_y = 0.1;
% mu_bar_y = 0.35;
% mu_under_y = 0.85;
% 
% rho_0_phi = 0.05;     % 初始边界
% rho_T_phi = 0.02;     % 稳态精度
% delta_phi = 0.1;      % 边界宽度
% e_0_phi = 0.1;
% T_phi = 1;            % 收敛时间
% a_phi = 1;          % 收敛速度
% lambda1_phi = 0;
% lambda2_phi = 0;
% 
% rho0_phi = 0.35; 
% rhoTs_phi = 0.08;
% mu_bar_phi = 0.3; %0.05
% mu_under_phi = 0.3;%0.09

function sys=mdlDerivatives(t,x,u)
global rho_0_y rho_T_y delta_y e_0_y  T_y a_y lambda1_y lambda2_y;
global rho_0_phi rho_T_phi delta_phi e_0_phi  T_phi a_phi lambda1_phi lambda2_phi;
global rho0_y rhoTs_y mu_bar_y mu_under_y;
global rho0_phi rhoTs_phi mu_bar_phi mu_under_phi;

c1y = 2; c1phi = 22;
k1y = 0.1; k1phi = 0.2; k2y = 0.01; k2phi = 0.01;
% k1y = 0.1; k1phi = 0.2; k2y = 0.1; k2phi = 0.5;
z1y_int = x(1); z1phi_int = x(2); z2y_int = x(3); z2phi_int = x(4); 
e_y = u(1); e_phi = u(4); X2 = [u(2); u(5)];
eta_y=u(3); eta_phi=u(6);
w = x(5);

% 性能函数计算
if   t<T_phi
    rho_phi = (rho_0_phi - rho_T_phi) .* ((a_phi .* (T_phi - t) ./ T_phi) .^ 3) + rho_T_phi;
    % rho_t_phi = (rho0_phi - t/T_phi) * exp(1 - T_phi/(T_phi - t)) + 0.04;
else
    rho_phi = rho_T_phi;
    % rho_t_phi = 0.04;
end
if t < T_y
    rho_y = (rho_0_y - rho_T_y) .* ((a_y .* (T_y - t) ./ T_y) .^ 3) + rho_T_y;
    % rho_t_y = (rho0_y - t/T_y) * exp(1 - T_y/(T_y - t)) + 0.06;
else
    rho_y = rho_T_y;
    % rho_t_y = 0.06;
end

% 边界函数计算
E_under_y = (sign(e_0_y) * (rho_y - rho_T_y) - delta_y * rho_y) - lambda1_y * tanh(eta_y);
E_bar_y = sign(e_0_y) * (rho_y - rho_T_y) + delta_y * rho_y + lambda2_y * tanh(eta_phi);
% E_bar_y = (sign(e_0_y) + mu_bar_y) * rho_t_y * (rho0_y - rhoTs_y) + mu_bar_y * rhoTs_y + lambda2_y * tanh(eta_y);
% E_under_y = (sign(e_0_y) - mu_under_y) * rho_t_y * (rho0_y - rhoTs_y) - mu_under_y * rhoTs_y - lambda1_y * tanh(eta_y);
% E_under_y = rho_y;
% E_bar_y = -rho_y;

E_under_phi = (sign(e_0_phi) * (rho_phi - rho_T_phi) - delta_phi * rho_phi) - lambda1_phi * tanh(eta_phi);
E_bar_phi = sign(e_0_phi) * (rho_phi - rho_T_phi) + delta_phi * rho_phi + lambda2_phi * tanh(eta_phi);
% E_bar_phi = (sign(e_0_phi) + mu_bar_phi) * rho_t_phi * (rho0_phi - rhoTs_phi) + mu_bar_phi * rhoTs_phi + lambda2_phi * tanh(eta_phi);
% E_under_phi = (sign(e_0_phi) - mu_under_phi) * rho_t_phi * (rho0_phi - rhoTs_phi) - mu_under_phi * rhoTs_phi - lambda1_phi * tanh(eta_phi);
% E_under_phi = rho_phi;
% E_bar_phi = -rho_phi;

% 误差变换
z1_y = log((-E_under_y + e_y) / (E_bar_y - e_y));
z1_phi = log((-E_under_phi + e_phi)/(E_bar_phi-e_phi));

% 变换参数
sigma_y = (E_bar_y - E_under_y) / ((E_bar_y - e_y) * (e_y - E_under_y));
sigma_phi = (E_bar_phi - E_under_phi) / ((E_bar_phi - e_phi) * (e_phi - E_under_phi));
sigma_y_ni = ((E_bar_y - e_y) * (e_y - E_under_y)) / (E_bar_y - E_under_y);
sigma_phi_ni = ((E_bar_phi - e_phi) * (e_phi - E_under_phi)) / (E_bar_phi - E_under_phi);

% PI补偿信号
s1y = 2*z1_y + k1y * z1y_int;
s1phi = 1.5*z1_phi + k1phi * z1phi_int;
s1 = [s1y; s1phi];

% 虚拟控制律
sigma_ni = diag([sigma_y_ni, sigma_phi_ni]);
TT_y = E_bar_y/(E_bar_y - e_y) + E_under_y/(e_y - E_under_y); 
TT_phi = E_bar_phi/(E_bar_phi - e_phi) + E_under_phi/(e_phi - E_under_phi);
T = [0.01*TT_y; 0.01*TT_phi];
z1 = [z1_y; z1_phi];
K1 = diag([k1y, k1phi]);
C1 = diag([c1y, c1phi]);

theta_1 = diag([k2y*sigma_y, k2phi*sigma_phi]);
rho = 10 * exp(-0.02);
denom = sqrt(s1' * s1 * norm(theta_1, 2)^2 + rho^2);
eta_1 = (s1 * norm(theta_1, 2)^2) / denom;
% fprintf('eta_1 = [%.6f, %.6f]\n', eta_1(1), eta_1(2));

alpha_1 = sigma_ni * (-C1 * s1 + T - K1 * z1) + w * eta_1;

% 中间误差
z2 = X2 - alpha_1;

z2_y = z2(1); z2_phi = z2(2);

% PI补偿信号
s2y = 2*z2_y + k2y * z2y_int;
s2phi = 1.5*z2_phi + k2phi * z2phi_int;
s2 = [s2y; s2phi];

c1 = 10;
rho_val = 10 * exp(-0.02*t);
k2 = diag([k2y, k2phi]);
theta_2 = [1; k2*z2];
denom2 = sqrt(s2' * s2 * (theta_2' * theta_2) + rho_val^2);
eta_2 = (s2 * (theta_2' * theta_2)) / denom2;
tau_1 = s1' * eta_1 - c1 * w;
dw_dt = tau_1 + s2' * eta_2;

% fprintf('s2 * eta_2 = %.6f\n', s2' * eta_2);

% w_max = 1000;
% if w > w_max && dw_dt > 0
%     dw_dt = 0;
% elseif w < -w_max && dw_dt < 0  
%     dw_dt = 0;
% end

% 积分项更新
dz1y_int = z1_y;
dz1phi_int = z1_phi;
dz2y_int = z2_y;
dz2phi_int = z2_phi;

sys = [dz1y_int; dz1phi_int; dz2y_int; dz2phi_int; dw_dt];

function sys=mdlOutputs(t,x,u)
global rho_0_y rho_T_y delta_y e_0_y  T_y a_y lambda1_y lambda2_y;
global rho_0_phi rho_T_phi delta_phi e_0_phi  T_phi a_phi lambda1_phi lambda2_phi;
global rho0_y rhoTs_y mu_bar_y mu_under_y;
global rho0_phi rhoTs_phi mu_bar_phi mu_under_phi;

c1y = 2; c1phi = 22;
k1y = 0.1; k1phi = 0.2; k2y = 0.01; k2phi = 0.01;
% k1y = 0.1; k1phi = 0.2; k2y = 0.1; k2phi = 0.5;
z1y_int = x(1); z1phi_int = x(2); z2y_int = x(3); z2phi_int = x(4);
e_y = u(1); e_phi = u(4); X2 = [u(2); u(5)];
eta_y=u(3); eta_phi=u(6);
w = x(5);

% fprintf('z1_int = [%.6f, %.6f]\n', z1y_int, z1phi_int);
% fprintf('z2_int = [%.6f, %.6f]\n', z2y_int, z2phi_int);

% 性能函数计算
if   t<T_phi
    rho_phi = (rho_0_phi - rho_T_phi) .* ((a_phi .* (T_phi - t) ./ T_phi) .^ 3) + rho_T_phi;
    % rho_t_phi = (rho0_phi - t/T_phi) * exp(1 - T_phi/(T_phi - t)) + 0.04;
else
    rho_phi = rho_T_phi;
    % rho_t_phi = 0.04;
end
if t < T_y
    rho_y = (rho_0_y - rho_T_y) .* ((a_y .* (T_y - t) ./ T_y) .^ 3) + rho_T_y;
    % rho_t_y = (rho0_y - t/T_y) * exp(1 - T_y/(T_y - t)) + 0.06;
else
    rho_y = rho_T_y;
    % rho_t_y = 0.06;
end

% 边界函数计算
e_under_y = (sign(e_0_y) * (rho_y - rho_T_y) - delta_y * rho_y);
e_bar_y = sign(e_0_y) * (rho_y - rho_T_y) + delta_y * rho_y;
E_under_y = (sign(e_0_y) * (rho_y - rho_T_y) - delta_y * rho_y) - lambda1_y * tanh(eta_y);
E_bar_y = sign(e_0_y) * (rho_y - rho_T_y) + delta_y * rho_y + lambda2_y * tanh(eta_phi);
% E_bar_y = (sign(e_0_y) + mu_bar_y) * rho_t_y * (rho0_y - rhoTs_y) + mu_bar_y * rhoTs_y + lambda2_y * tanh(eta_y);
% E_under_y = (sign(e_0_y) - mu_under_y) * rho_t_y * (rho0_y - rhoTs_y) - mu_under_y * rhoTs_y - lambda1_y * tanh(eta_y);
% E_under_y = rho_y;
% E_bar_y = -rho_y;

e_under_phi = (sign(e_0_phi) * (rho_phi - rho_T_phi) - delta_phi * rho_phi);
e_bar_phi = sign(e_0_phi) * (rho_phi - rho_T_phi) + delta_phi * rho_phi;
E_under_phi = (sign(e_0_phi) * (rho_phi - rho_T_phi) - delta_phi * rho_phi) - lambda1_phi * tanh(eta_phi);
E_bar_phi = sign(e_0_phi) * (rho_phi - rho_T_phi) + delta_phi * rho_phi + lambda2_phi * tanh(eta_phi);
% E_bar_phi = (sign(e_0_phi) + mu_bar_phi) * rho_t_phi * (rho0_phi - rhoTs_phi) + mu_bar_phi * rhoTs_phi + lambda2_phi * tanh(eta_phi);
% E_under_phi = (sign(e_0_phi) - mu_under_phi) * rho_t_phi * (rho0_phi - rhoTs_phi) - mu_under_phi * rhoTs_phi - lambda1_phi * tanh(eta_phi);
% E_under_phi = rho_phi;
% E_bar_phi = -rho_phi;

% fprintf('e_l_y=%.4f, e_u_y=%.4f, e_l_phi=%.4f, e_u_phi=%.4f\n', E_under_y, E_bar_y, E_under_phi, E_bar_phi);
% fprintf('rho_t_y=%.4f, rho_t_phi=%.4f\n', rho_t_y, rho_t_phi);

% 误差变换
z1_y = log((-E_under_y + e_y) / (E_bar_y - e_y));
z1_phi = log((-E_under_phi + e_phi)/(E_bar_phi-e_phi));

% 变换参数
sigma_y = (E_bar_y - E_under_y) / ((E_bar_y - e_y) * (e_y - E_under_y));
sigma_phi = (E_bar_phi - E_under_phi) / ((E_bar_phi - e_phi) * (e_phi - E_under_phi));
sigma_y_ni = ((E_bar_y - e_y) * (e_y - E_under_y)) / (E_bar_y - E_under_y);
sigma_phi_ni = ((E_bar_phi - e_phi) * (e_phi - E_under_phi)) / (E_bar_phi - E_under_phi);

% PI补偿信号
s1y = 2*z1_y + k1y * z1y_int;
s1phi = 1.5*z1_phi + k1phi * z1phi_int;
s1 = [s1y; s1phi];
% fprintf('s1 = [%.6f, %.6f]\n', s1(1), s1(2));

% 虚拟控制律
sigma_ni = diag([sigma_y_ni, sigma_phi_ni]);
TT_y = E_bar_y/(E_bar_y - e_y) + E_under_y/(e_y - E_under_y); 
TT_phi = E_bar_phi/(E_bar_phi - e_phi) + E_under_phi/(e_phi - E_under_phi);
T = [0.01*TT_y; 0.01*TT_phi];
z1 = [z1_y; z1_phi];
K1 = diag([k1y, k1phi]);
C1 = diag([c1y, c1phi]);

theta_1 = diag([k2y*sigma_y, k2phi*sigma_phi]);
rho = 10 * exp(-0.02);
denom = sqrt(s1' * s1 * norm(theta_1, 2)^2 + rho^2);
eta_1 = (s1 * norm(theta_1, 2)^2) / denom;
% fprintf('eta_1 = [%.6f, %.6f]\n', eta_1(1), eta_1(2));
% fprintf('w = %.6f\n', w);
% fprintf('theta_1 = [%.6f, %.6f]\n', 0.01*sigma_y, 0.01*sigma_phi);
% fprintf('sigma = [%.6f, %.6f]\n', sigma_y_ni, sigma_phi_ni);
% alpha_1 = sigma \ (-C1 * s1 + T - K1 * z1) + w * eta_1;
alpha_1 = sigma_ni * (-C1 * s1 + T - K1 * z1) + w * eta_1;
% fprintf('alpha_1 = [%.6f, %.6f]\n', alpha_1(1), alpha_1(2));
% fprintf('alpha_1(1),e_y = [%.6f, %.6f]\n', alpha_1(1), e_y);
% fprintf('alpha_1 = [%.6f, %.6f]\n', sigma_ni * (-C1 * s1 + T - K1 * z1));

% 中间误差
z2 = X2 - alpha_1;
z2_y = z2(1); z2_phi = z2(2);
% fprintf('z2 = [%.6f, %.6f]\n', z2(1), z2(2));
% fprintf('X2 = [%.6f, %.6f]\n', X2(1), X2(2));

% PI补偿信号
s2y = 2*z2_y + k2y * z2y_int;
s2phi = 1.5*z2_phi + k2phi * z2phi_int;
s2 = [s2y; s2phi];

sys = [E_bar_y; E_under_y; E_bar_phi; E_under_phi; sigma_y; s2y; s1y; sigma_phi; s2phi; s1phi; z2_y; z2_phi; w; z1_y; z1_phi; z2_y; z2_phi;e_under_y;e_bar_y;e_under_phi;e_bar_phi];                                