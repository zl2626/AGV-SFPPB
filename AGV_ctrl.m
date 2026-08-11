function [sys,x0,str,ts] = AGV_ctrl(t,x,u,flag)
% AGV_CTRL  SFPPB-PI-ICAS-RL 控制器
% 第一层：z1 -> s1 -> alpha1
% 第二层：z2 -> s2 -> delta
% O 是输入饱和补偿状态，不是 Safety Filter 或残差控制器。
% 最终只有一个实际方向盘输入 delta。

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
        error('AGV_ctrl:UnhandledFlag','Unhandled flag = %d.',flag);
end
end

function [sys,x0,str,ts] = mdlInitializeSizes
% ========================== PI 参数 ==========================
global k1y k1phi k2y k2phi

k1y = 0.10;                      % 第一层横向误差积分系数
k1phi = 0.20;                    % 第一层航向误差积分系数
k2y = 0.01;                      % 第二层横向误差积分系数
k2phi = 0.01;                    % 第二层航向误差积分系数

% ======================== ICAS-RL 参数 =======================
global N c1 c2 Upsilon1 Upsilon2 sigma1 sigma2
global gamma_c1 gamma_c2 gamma_a1 gamma_a2

N = 7;                           % RBF 节点数
c1 = [2;22];                     % 第一层稳定项 [c1y,c1phi]
c2 = [2;5];                      % 第二层稳定项 [c2y,c2phi]
Upsilon1 = 0.04;                 % 第一层 Identifier 增益
Upsilon2 = 0.04;                 % 第二层 Identifier 增益
sigma1 = 0.08;                   % 第一层 Identifier sigma
sigma2 = 0.08;                   % 第二层 Identifier sigma
gamma_c1 = 0.004;                % 第一层 Critic 增益
gamma_c2 = 0.004;                % 第二层 Critic 增益
gamma_a1 = 0.012;                % 第一层 Actor 增益
gamma_a2 = 0.012;                % 第二层 Actor 增益

% ======================== 输入和车辆参数 ======================
global learning_enabled u_d m Iz lf cf0 cf_rate
global C_direction_normalization

u_d = 0.5;                        % 方向盘饱和上限
m = 1832;                         % 车辆质量
Iz = 2488;                        % 横摆转动惯量
lf = 1.18;                        % 前轴到质心距离
cf0 = 80000;                      % 初始前轮侧偏刚度
cf_rate = 0.10;                   % 侧偏刚度缓慢变化幅度

% 明确声明输入方向处理：Cbar=Cphysical/||Cphysical||。
% 改为 0 时直接使用车辆模型中的物理输入增益 Cphysical。
C_direction_normalization = 1;

% 设为 0 可冻结六组权重；默认在线学习。
if isempty(learning_enabled)
    learning_enabled = 1;
end

sizes = simsizes;
sizes.NumContStates  = 12*N+6;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 9;
sizes.NumInputs      = 13;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

% 状态顺序：[WF1;WC1;WA1;WF2;WC2;WA2;O;I1;I2]
% 六组权重都是 N×2，O、I1、I2 都是 2×1。
W0 = 0.4;                        % 论文参考幅值
w0 = W0*[-1;-1;-1;0;1;1;1];      % RBF 节点的确定性对称初值
WF10 = repmat(w0,1,2);
WC10 = repmat(w0,1,2);
WA10 = repmat(w0,1,2);
WF20 = repmat(w0,1,2);
WC20 = repmat(w0,1,2);
WA20 = repmat(w0,1,2);
O0 = zeros(2,1);
I10 = zeros(2,1);
I20 = zeros(2,1);
x0 = [WF10(:);WC10(:);WA10(:);WF20(:);WC20(:);WA20(:);O0;I10;I20];
str = [];
ts = [0 0];
end

function sys = mdlDerivatives(t,x,u)
global N c1 c2 Upsilon1 Upsilon2 sigma1 sigma2
global k1y k1phi k2y k2phi
global gamma_c1 gamma_c2 gamma_a1 gamma_a2 learning_enabled
global u_d m Iz lf cf0 cf_rate C_direction_normalization

[WF1,WC1,WA1,WF2,WC2,WA2,O,I1,I2] = unpackStates(x,N);
[varsigma,z1,chi2,Gamma,Z_F] = readInputs(u);

K1 = [k1y;k1phi];
K2 = [k2y;k2phi];

% ------------------------- 第一层 PI-RL -----------------------
% I1_dot=z1，s1=z1+K1*I1。
s1 = z1+K1.*I1;
S_F1 = AGV_RBF(Z_F,'F');
S_J1 = AGV_RBF([Z_F;s1],'J');
F1_hat = WF1'*S_F1;

% alpha1=varsigma^(-1)[-C1*s1+Gamma-K1*z1-Fhat1-(1/2)Wa1'*SJ1]
alpha1 = varsigma\(-c1.*s1+Gamma-K1.*z1-F1_hat-0.5*WA1'*S_J1);

% ------------------------- 第二层 PI-RL -----------------------
% z2=chi2-alpha1-O，I2_dot=z2，s2=z2+K2*I2。
z2 = chi2-alpha1-O;
s2 = z2+K2.*I2;
S_F2 = AGV_RBF(Z_F,'F');
S_J2 = AGV_RBF([Z_F;s2],'J');
F2_hat = WF2'*S_F2;

% 车辆输入方向：保留物理方向，并把是否归一化写成显式参数。
cf = cf0*(1+cf_rate*sin(0.01*t));
C_physical = [cf/m;lf*cf/Iz];
if C_direction_normalization
    C = C_physical/max(norm(C_physical),eps);
else
    C = C_physical;
end

% 第二层控制律只产生一个方向盘请求 delta。
p_a2 = 2*c2.*s2+2*F2_hat+WA2'*S_J2;
delta = -0.5*C'*p_a2;

% 输入饱和补偿：O_dot=-O+C[k(delta)-delta]。
delta_smooth = u_d*tanh(delta/u_d);
dO = -O+C*(delta_smooth-delta);

if learning_enabled
    % ----------------------- Identifier -----------------------
    dWF1 = Upsilon1*(S_F1*s1'-sigma1*WF1);
    dWF2 = Upsilon2*(S_F2*s2'-sigma2*WF2);

    % -------------------------- Critic -------------------------
    dWC1 = -gamma_c1*(S_J1*S_J1')*WC1;
    dWC2 = -gamma_c2*(S_J2*S_J2')*WC2;

    % --------------------------- Actor -------------------------
    dWA1 = -(S_J1*S_J1')* ...
        (gamma_a1*(WA1-WC1)+gamma_c1*WC1);
    dWA2 = -(S_J2*S_J2')* ...
        (gamma_a2*(WA2-WC2)+gamma_c2*WC2);
else
    dWF1 = zeros(size(WF1));
    dWC1 = zeros(size(WC1));
    dWA1 = zeros(size(WA1));
    dWF2 = zeros(size(WF2));
    dWC2 = zeros(size(WC2));
    dWA2 = zeros(size(WA2));
end

% PI 积分器始终运行；learning_enabled 只冻结六组自适应权重。
dI1 = z1;
dI2 = z2;
sys = [dWF1(:);dWC1(:);dWA1(:);dWF2(:);dWC2(:);dWA2(:);dO;dI1;dI2];
end

function sys = mdlOutputs(t,x,u)
global N c1 c2 u_d m Iz lf cf0 cf_rate
global k1y k1phi k2y k2phi C_direction_normalization

[WF1,WC1,WA1,WF2,WC2,WA2,O,I1,I2] = unpackStates(x,N);
[varsigma,z1,chi2,Gamma,Z_F] = readInputs(u);

K1 = [k1y;k1phi];
K2 = [k2y;k2phi];
s1 = z1+K1.*I1;

% 第一层 Actor 和 Identifier。
S_F1 = AGV_RBF(Z_F,'F');
S_J1 = AGV_RBF([Z_F;s1],'J');
F1_hat = WF1'*S_F1;
alpha1 = varsigma\(-c1.*s1+Gamma-K1.*z1-F1_hat-0.5*WA1'*S_J1);

% 第二层只有一个方向盘控制通道。
z2 = chi2-alpha1-O;
s2 = z2+K2.*I2;
S_F2 = AGV_RBF(Z_F,'F');
S_J2 = AGV_RBF([Z_F;s2],'J');
F2_hat = WF2'*S_F2;
cf = cf0*(1+cf_rate*sin(0.01*t));
C_physical = [cf/m;lf*cf/Iz];
if C_direction_normalization
    C = C_physical/max(norm(C_physical),eps);
else
    C = C_physical;
end
p_a2 = 2*c2.*s2+2*F2_hat+WA2'*S_J2;
delta = -0.5*C'*p_a2;
delta1 = min(max(delta,-u_d),u_d);

W = norm([WF1(:);WC1(:);WA1(:);WF2(:);WC2(:);WA2(:)]);
% 额外六个输出只用于原模型的结果记录，不增加控制方向。
sys = [delta;delta1;W;s1;s2;z2];
end

function [WF1,WC1,WA1,WF2,WC2,WA2,O,I1,I2] = unpackStates(x,N)
i = 0;
WF1 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
WC1 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
WA1 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
WF2 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
WC2 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
WA2 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
O = x(i+1:i+2); i = i+2;
I1 = x(i+1:i+2); i = i+2;
I2 = x(i+1:i+2);
end

function [varsigma,z1,chi2,Gamma,Z_F] = readInputs(u)
% Mux1 输入：[varsigma_y,z1_y,z1_phi,varsigma_phi,0,0,
%           e_y,e_phi,de_y,de_phi,Gamma_y,Gamma_phi,rho]
varsigma = diag([max(u(1),eps),max(u(4),eps)]);
z1 = [u(2);u(3)];
chi2 = [u(9);u(10)];
Gamma = [u(11);u(12)];
Z_F = [u(7);u(8);u(9);u(10)];
end
