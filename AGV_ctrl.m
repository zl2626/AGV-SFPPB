function [sys,x0,str,ts] = AGV_ctrl(t,x,u,flag)
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
sizes = simsizes;
sizes.NumContStates  = 1;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 13;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = 1;
str = [];
ts = [0 0];

function sys=mdlDerivatives(t,x,u)
% 解包状态
W = x(1);

sigma_y = u(1); sigma_phi = u(4);
s2y = u(2); s2phi = u(5);
s1y = u(3); s1phi = u(6); 
e_y = u(7); e_phi = u(8);
X1 = [e_y; e_phi]; X2 = [u(9); u(10)];

% 完整的s1和s2
s1 = [s1y; s1phi];
s2 = [s2y; s2phi];

% RBFNN权重更新
k = 1; r = 2;
Z2 = [X1; X2];
phi_y = AGV_RBF(Z2); 
phi_phi = AGV_RBF(Z2);
% fprintf('phi = [%.6f, %.6f]\n', phi_y, phi_phi);

dW_dt = (1/(2*k^2)) * (s2'*s2) * (phi_y'*phi_y + phi_phi'*phi_phi) - r * W;

% W_max = 100;
% if W > W_max && dW_dt > 0
%     dW_dt = -r * W;
% elseif W < -W_max && dW_dt < 0
%     dW_dt = -r * W;
% end

sys = dW_dt;

function sys=mdlOutputs(t,x,u)
% 解包状态
W = x(1);
% fprintf('W = %.6f\n', W);
% 输入解包
w = u(13);
sigma_y = u(1); sigma_phi = u(4);
s2y = u(2); s2phi = u(5);
s1y = u(3); s1phi = u(6); 
e_y = u(7); e_phi = u(8);
X1 = [e_y; e_phi]; X2 = [u(9); u(10)];
z2 = [u(11); u(12)];
k2y = 0.1; k2phi = 0.5;
k2 = diag([k2y, k2phi]);

m = 1832; Iz = 2488; lf = 1.18;
cf = 80000 * (1 + 0.1*sin(0.01*t));

% 完整的信号
s1 = [s1y; s1phi];
s2 = [s2y; s2phi];

sigma = diag([sigma_y, sigma_phi]);

% 控制器参数
% c2y = 10; c2phi = 20;
c2y = 2; c2phi = 5;
k_nn = 0.1;
rho_val = 10 * exp(-0.02*t);

C2 = diag([c2y, c2phi]);

% 计算theta_2和eta_2
u_d = 0.5;
c_y = cf/m;
c_phi = lf*cf/Iz;
c_min = [c_y* tanh(c_y/u_d); c_phi* tanh(c_phi/u_d)];
theta_2 = diag([1/c_min, 1/c_min]);
% fprintf('theta_2 = [%.6f, %.6f]\n', theta_2(1), theta_2(2));

denom2 = sqrt(s2' * s2 * norm(theta_2, 2)^2 + rho_val^2);
eta_2 = (s2 * norm(theta_2, 2)^2) / denom2;

% fprintf('eta_2 = [%.6f, %.6f]\n', eta_2(1), eta_2(2));

% RBFNN计算
Z2 = [X1; X2];
phi_y = AGV_RBF(Z2); 
phi_phi = AGV_RBF(Z2);

% 计算a2_hat
nn_term = (1/(2*k_nn^2)) * s2 * W * (phi_y'*phi_y + phi_phi'*phi_phi);
a2_hat = C2 * s2 + 0.5 * s2 + sigma * s1 + w * eta_2 + nn_term;

% 最终控制律
I = [1; 1];
numerator = -I' * s2 * (a2_hat' * a2_hat);
denominator = sqrt(s2' * s2 * (a2_hat' * a2_hat) + rho_val^2);
delta = numerator /denominator;

% numerator = -s2' * a2_hat;
% denominator = sqrt(s2' * s2 * (a2_hat' * a2_hat) + rho_val^2);
% delta = 1 *numerator /(50 * denominator);

% numerator = -sqrt(s2' * s2) * (a2_hat' * a2_hat);
% denominator = sqrt(s2' * s2 * (a2_hat' * a2_hat) + rho_val^2);
% delta = numerator /denominator;

% fprintf('=== 详细调试信息 t=%.3f ===\n', t);
% fprintf('s1 = [%.6f, %.6f]\n', s1(1), s1(2));
% fprintf('s2 = [%.6f, %.6f]\n', s2(1), s2(2));
% fprintf('a2_hat = [%.6f, %.6f]\n', a2_hat(1), a2_hat(2));
% fprintf('s2''*s2 = %.6f\n', s2'*s2);
% fprintf('a2_hat''*a2_hat = %.6f\n', a2_hat'*a2_hat);
% fprintf('s2''*s2 * a2_hat''*a2_hat = %.6f\n', s2'*s2 * (a2_hat'*a2_hat));
% fprintf('rho_val = %.6f\n', rho_val);
% fprintf('denominator = %.6f\n', denominator);
% fprintf('numerator = %.6f\n', numerator);
% fprintf('c_min = %.6f\n', c_min);
% fprintf('delta计算: %.6f / (%.6f * %.6f) = ', numerator, c_min, denominator);
% fprintf('delta = %.6f\n', delta);

% fprintf('e_y=%.4f, e_phi=%.4f, delta=%.6f\n', e_y, e_phi, delta);
u_d = 0.5;
if abs(delta) > u_d
    delta_sat = u_d * sign(delta);
else
    delta_sat = delta;
end
sys(1) = delta;
sys(2) = delta_sat;
sys(3)=W;

