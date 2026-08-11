function [sys,x0,str,ts] = AGV_ctrl(t,x,u,flag)
% AGV_CTRL  SFPPB-PI-RL 控制器
% 按照原 AGV-TFS 的写法组织：初始化、状态导数、输出。
% 参数都在下面的初始化函数里，调参时直接修改数值即可。

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
end

function [sys,x0,str,ts] = mdlInitializeSizes
% ========================== 参数区 ===========================
% RBF 节点数
global N
N = 7;

% PI 参数：s1=z1+K1*I1，s2=z2+K2*I2
global k1y k1phi k2y k2phi
k1y = 0.10;                         % 第一层横向误差积分系数
k1phi = 0.20;                       % 第一层航向误差积分系数
k2y = 0.01;                         % 第二层横向误差积分系数
k2phi = 0.01;                       % 第二层航向误差积分系数

% 两层控制器参数
global c1y c1phi c2y c2phi
c1y = 2;                            % 第一层横向稳定系数
c1phi = 22;                         % 第一层航向稳定系数
c2y = 2;                            % 第二层横向稳定系数
c2phi = 5;                          % 第二层航向稳定系数

% RBF 自适应参数
global Upsilon1 Upsilon2 sigma1 sigma2
global gamma_c1 gamma_c2 gamma_a1 gamma_a2
Upsilon1 = 0.04;                    % 第一层辨识增益
Upsilon2 = 0.04;                    % 第二层辨识增益
sigma1 = 0.08;                      % 第一层泄漏系数
sigma2 = 0.08;                      % 第二层泄漏系数
gamma_c1 = 0.004;                   % 第一层 Critic 增益
gamma_c2 = 0.004;                   % 第二层 Critic 增益
gamma_a1 = 0.012;                   % 第一层 Actor 增益
gamma_a2 = 0.012;                   % 第二层 Actor 增益

% 车辆和方向盘参数
global u_d m Iz lf cf0 cf_rate
u_d = 0.5;                          % 方向盘最大输入
m = 1832;                           % 车辆质量
Iz = 2488;                          % 横摆转动惯量
lf = 1.18;                          % 前轴到质心距离
cf0 = 80000;                        % 初始前轮侧偏刚度
cf_rate = 0.10;                     % 侧偏刚度变化幅度

% S-function 接口
sizes = simsizes;
sizes.NumContStates  = 12*N+6;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 9;
sizes.NumInputs      = 13;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

% 状态顺序：[WF1;WC1;WA1;WF2;WC2;WA2;O;I1;I2]
W0 = 0.4;
w0 = W0*[-1;-1;-1;0;1;1;1];
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
% 这里按顺序完成：解包状态、计算控制量、更新权重和 PI 积分器。
global N c1y c1phi c2y c2phi
global k1y k1phi k2y k2phi
global Upsilon1 Upsilon2 sigma1 sigma2
global gamma_c1 gamma_c2 gamma_a1 gamma_a2
global u_d m Iz lf cf0 cf_rate

% ------------------------- 解包状态 --------------------------
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

% ------------------------- 解包输入 --------------------------
varsigma = diag([max(u(1),eps),max(u(4),eps)]);
z1 = [u(2);u(3)];
chi2 = [u(9);u(10)];
Gamma = [u(11);u(12)];
Z_F = [u(7);u(8);u(9);u(10)];

C1 = [c1y;c1phi];
C2 = [c2y;c2phi];
K1 = [k1y;k1phi];
K2 = [k2y;k2phi];

% ------------------------- 第一层 ----------------------------
s1 = z1+K1.*I1;
S_F1 = AGV_RBF(Z_F,'F');
S_J1 = AGV_RBF([Z_F;s1],'J');
F1_hat = WF1'*S_F1;
alpha1 = varsigma\(-C1.*s1+Gamma-K1.*z1-F1_hat-0.5*WA1'*S_J1);

% ------------------------- 第二层 ----------------------------
z2 = chi2-alpha1-O;
s2 = z2+K2.*I2;
S_F2 = AGV_RBF(Z_F,'F');
S_J2 = AGV_RBF([Z_F;s2],'J');
F2_hat = WF2'*S_F2;

% 第二层 PI 的积分项求导会产生 K2*z2，这一项不能漏掉。
F2_PI = F2_hat+K2.*z2;

% 车辆输入方向，先归一化方向，再由控制增益调节幅值。
cf = cf0*(1+cf_rate*sin(0.01*t));
C_physical = [cf/m;lf*cf/Iz];
C = C_physical/max(norm(C_physical),eps);

% 方向盘控制量和输入饱和补偿状态
p_a2 = 2*C2.*s2+2*F2_PI+WA2'*S_J2;
delta = -0.5*C'*p_a2;
delta_smooth = u_d*tanh(delta/u_d);
dO = -O+C*(delta_smooth-delta);

% ------------------------- 权重更新 --------------------------
dWF1 = Upsilon1*(S_F1*s1'-sigma1*WF1);
dWF2 = Upsilon2*(S_F2*s2'-sigma2*WF2);
dWC1 = -gamma_c1*(S_J1*S_J1')*WC1;
dWC2 = -gamma_c2*(S_J2*S_J2')*WC2;
dWA1 = -(S_J1*S_J1')*(gamma_a1*(WA1-WC1)+gamma_c1*WC1);
dWA2 = -(S_J2*S_J2')*(gamma_a2*(WA2-WC2)+gamma_c2*WC2);

% PI 积分器
dI1 = z1;
dI2 = z2;

sys = [dWF1(:);dWC1(:);dWA1(:);dWF2(:);dWC2(:);dWA2(:);dO;dI1;dI2];
end

function sys = mdlOutputs(t,x,u)
% 输出实际方向盘请求、饱和后的请求、权重范数以及调试信号。
global N c1y c1phi c2y c2phi
global k1y k1phi k2y k2phi
global u_d m Iz lf cf0 cf_rate

% ------------------------- 解包状态 --------------------------
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

% ------------------------- 解包输入 --------------------------
varsigma = diag([max(u(1),eps),max(u(4),eps)]);
z1 = [u(2);u(3)];
chi2 = [u(9);u(10)];
Gamma = [u(11);u(12)];
Z_F = [u(7);u(8);u(9);u(10)];

C1 = [c1y;c1phi];
C2 = [c2y;c2phi];
K1 = [k1y;k1phi];
K2 = [k2y;k2phi];

% 第一层 PI 和 RBF
s1 = z1+K1.*I1;
S_F1 = AGV_RBF(Z_F,'F');
S_J1 = AGV_RBF([Z_F;s1],'J');
F1_hat = WF1'*S_F1;
alpha1 = varsigma\(-C1.*s1+Gamma-K1.*z1-F1_hat-0.5*WA1'*S_J1);

% 第二层 PI 和 RBF
z2 = chi2-alpha1-O;
s2 = z2+K2.*I2;
S_F2 = AGV_RBF(Z_F,'F');
S_J2 = AGV_RBF([Z_F;s2],'J');
F2_hat = WF2'*S_F2;
F2_PI = F2_hat+K2.*z2;

% 车辆输入方向和最终控制量
cf = cf0*(1+cf_rate*sin(0.01*t));
C_physical = [cf/m;lf*cf/Iz];
C = C_physical/max(norm(C_physical),eps);
p_a2 = 2*C2.*s2+2*F2_PI+WA2'*S_J2;
delta = -0.5*C'*p_a2;
delta_sat = min(max(delta,-u_d),u_d);

W = norm([WF1(:);WC1(:);WA1(:);WF2(:);WC2(:);WA2(:)]);

% 前三个是控制器主输出，后六个供 plot 记录误差变量。
sys = [delta;delta_sat;W;s1;s2;z2];
end
