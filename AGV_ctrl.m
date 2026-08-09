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
% 神经网络权重
WF = x(1:7);
Wc = x(8:14);
Wa = x(15:21);
O = x(22);

% 输入信号
% 保持原有 13 输入接口不变。
e_y = u(7);
e_phi = u(8);
de_y = u(9);
de_phi = u(10);
z2 = [u(11); u(12)];

X1 = [e_y; e_phi];
X2 = [de_y; de_phi];
Z2 = [X1; X2];

% 车辆参数
m = 1832;
Iz = 2488;
lf = 1.18;
cf = 80000*(1 + 0.1*sin(0.01*t));

c_y = cf/m;
c_phi = lf*cf/Iz;
C = [c_y; c_phi];
C_norm = norm(C);

% RBFNN
phi = AGV_RBF(Z2);

% Identifier
% 将两个虚拟误差投影到实际转向方向，O 补偿输入饱和误差。
z2_control = C'*z2/C_norm - O;

Gamma_F = 0.2;
sigma_F = 2.0;
dWF = Gamma_F*(z2_control*phi - sigma_F*WF);

% Critic
gamma_c = 0.75;
dWc = -gamma_c*phi*(phi'*Wc);

% Actor
gamma_a = 1.0;
actor_error = gamma_a*(Wa - Wc) + gamma_c*Wc;
dWa = -phi*(phi'*actor_error);

% 最终控制律
c2 = 30;
F_hat = WF'*phi;
actor_term = Wa'*phi;

delta = (-c2*z2_control - F_hat - 0.5*actor_term)/C_norm;

u_d = 0.5;
k_delta = u_d*tanh(delta/u_d);
dO = -O + C_norm*(k_delta - delta);

sys = [dWF; dWc; dWa; dO];

function sys = mdlOutputs(t,x,u)
% 输出计算顺序与导数计算保持一致，Simulink 输出接口保持 3 路。

% 神经网络权重
WF = x(1:7);
Wc = x(8:14);
Wa = x(15:21);
O = x(22);

% 输入信号
e_y = u(7);
e_phi = u(8);
de_y = u(9);
de_phi = u(10);
z2 = [u(11); u(12)];

X1 = [e_y; e_phi];
X2 = [de_y; de_phi];
Z2 = [X1; X2];

% 车辆参数
m = 1832;
Iz = 2488;
lf = 1.18;
cf = 80000*(1 + 0.1*sin(0.01*t));

c_y = cf/m;
c_phi = lf*cf/Iz;
C = [c_y; c_phi];
C_norm = norm(C);

% RBFNN 与最终控制律
phi = AGV_RBF(Z2);
z2_control = C'*z2/C_norm - O;

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

% 输出全部神经网络权重范数，用于观察整体权重变化。
weight_norm = norm([WF; Wc; Wa]);
sys = [delta; delta_sat; weight_norm];
