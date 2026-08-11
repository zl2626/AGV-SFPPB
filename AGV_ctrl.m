function [sys,x0,str,ts] = AGV_ctrl(t,x,u,flag)
% AGV_CTRL  SFPPB-RL方向盘控制器
% 结构与原AGV_TFS一致：初始化、状态更新、输出三个部分。
% 第一层：z1 -> Actor1 -> alpha1
% 第二层：z2=chi2-alpha1 -> Actor2 -> delta
% 只有一个实际执行器delta，最后经过物理饱和。

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
% =========================== 参数区 ===================================
% 这些参数都可以直接在这里修改，不再调用单独的参数文件。
global N Q1 R1 Q2 r2
global gamma_F1 gamma_F2 sigma_F1 sigma_F2
global gamma_C1 gamma_C2 gamma_A1 gamma_A2
global gamma_pi1 gamma_pi2 sigma_C sigma_A W_max
global actor_initial1 actor_initial2 learning_enabled
global m Iz lf cf0 cf_rate u_d

% 车辆和执行器参数（与AGV_plant.m保持一致）
m = 1832;
Iz = 2488;
lf = 1.18;
cf0 = 80000;
cf_rate = 0.10;
u_d = 0.5;

% 两层HJB代价函数
N = 7;                         % RBF节点数
Q1 = diag([1,1]);              % 第一层误差代价
R1 = diag([1,10]);             % 第一层虚拟控制代价
Q2 = diag([1,1]);              % 第二层误差代价
r2 = 1;                        % 方向盘代价

% 自适应律参数
gamma_F1 = 0.04;
gamma_F2 = 0.04;
sigma_F1 = 0.08;
sigma_F2 = 0.08;
gamma_C1 = 0.004;
gamma_C2 = 0.004;
gamma_A1 = 0.012;
gamma_A2 = 0.012;
gamma_pi1 = 0.002;
gamma_pi2 = 0.002;
sigma_C = 0.001;
sigma_A = 0.001;
W_max = 25;

% Actor初值：给出有界的初始值梯度，便于从t=0开始产生控制
actor_initial1 = 2.0;
actor_initial2 = 0.5;
if isempty(learning_enabled)
    learning_enabled = 1;      % AGV_run_rebuild可切换为0做冻结对比
end

sizes = simsizes;
sizes.NumContStates  = 8*N;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 13;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

% 状态顺序：[WF1; WC1; WA1; WF2; WC2; WA2]
WF10 = zeros(2*N,1);
WC10 = zeros(N,1);
WA10 = actor_initial1*[1;-ones(N-1,1)];
WF20 = zeros(2*N,1);
WC20 = zeros(N,1);
WA20 = actor_initial2*[1;-ones(N-1,1)];
x0 = [WF10;WC10;WA10;WF20;WC20;WA20];
str = [];
ts = [0 0];
end

function sys = mdlDerivatives(t,x,u)
% ======================== 解包自适应状态 ===============================
global N Q1 R1 Q2 r2
global gamma_F1 gamma_F2 sigma_F1 sigma_F2
global gamma_C1 gamma_C2 gamma_A1 gamma_A2
global gamma_pi1 gamma_pi2 sigma_C sigma_A W_max
global learning_enabled m Iz lf cf0 cf_rate

WF1 = reshape(x(1:2*N),N,2);
WC1 = x(2*N+1:3*N);
WA1 = x(3*N+1:4*N);
WF2 = reshape(x(4*N+1:6*N),N,2);
WC2 = x(6*N+1:7*N);
WA2 = x(7*N+1:8*N);

% Mux1输入：[varsigma_y,z1_y,z1_phi,varsigma_phi,0,0,
%           e_y,e_phi,de_y,de_phi,0,0,rho]
varsigma = diag([max(u(1),eps),max(u(4),eps)]);
z1 = [u(2);u(3)];
chi2 = [u(9);u(10)];
Z_F = [u(7);u(8);u(9);u(10)];

% ========================== 第一层RL =================================
[S_F1,~] = AGV_RBF(Z_F,'F');
[~,dS_J1] = AGV_RBF([Z_F;z1],'J');
dS1 = dS_J1(:,5:6);          % 对z1求偏导
F1_hat = WF1'*S_F1;
grad_JA1 = dS1'*WA1;
alpha1 = -0.5*(R1\(varsigma'*grad_JA1));

% ========================== 第二层RL =================================
z2 = chi2-alpha1;
[S_F2,~] = AGV_RBF(Z_F,'F');
[~,dS_J2] = AGV_RBF([Z_F;z2],'J');
dS2 = dS_J2(:,5:6);          % 对z2求偏导
F2_hat = WF2'*S_F2;
cf = cf0*(1+cf_rate*sin(0.01*t));
C = [cf/m; lf*cf/Iz];        % 2维状态到1维方向盘的输入矩阵
grad_JA2 = dS2'*WA2;
delta = -(C'*grad_JA2)/(2*r2);

% 估计的两层状态动态，用于Critic更新
z1_dot_hat = F1_hat+varsigma*alpha1;
z2_dot_hat = F2_hat+C*delta;

if ~learning_enabled
    sys = zeros(size(x));
    return
end

% =========================== Identifier ===============================
dWF1 = gamma_F1*(S_F1*z1')/(1+S_F1'*S_F1)-sigma_F1*WF1;
dWF2 = gamma_F2*(S_F2*z2')/(1+S_F2'*S_F2)-sigma_F2*WF2;

% ============================= Critic =================================
grad_JC1 = dS1'*WC1;
HJB1 = z1'*Q1*z1+alpha1'*R1*alpha1+grad_JC1'*z1_dot_hat;
E_C1 = dS1*z1_dot_hat;
dWC1 = -gamma_C1*E_C1*HJB1/(1+E_C1'*E_C1)^2-sigma_C*WC1;

grad_JC2 = dS2'*WC2;
HJB2 = z2'*Q2*z2+r2*delta^2+grad_JC2'*z2_dot_hat;
E_C2 = dS2*z2_dot_hat;
dWC2 = -gamma_C2*E_C2*HJB2/(1+E_C2'*E_C2)^2-sigma_C*WC2;

% ============================== Actor =================================
E_A1 = dS1*varsigma;
stationarity1 = R1*alpha1+0.5*varsigma'*grad_JC1;
dWA1 = -gamma_A1*(WA1-WC1) ...
    +gamma_pi1*E_A1*stationarity1/(1+E_A1(:)'*E_A1(:))-sigma_A*WA1;

E_A2 = dS2*C;
stationarity2 = r2*delta+0.5*C'*grad_JC2;
dWA2 = -gamma_A2*(WA2-WC2) ...
    +gamma_pi2*E_A2*stationarity2/(1+E_A2'*E_A2)-sigma_A*WA2;

% 投影算子：只限制权重范数，不增加任何控制器
dWF1 = project(WF1,dWF1,W_max);
dWC1 = project(WC1,dWC1,W_max);
dWA1 = project(WA1,dWA1,W_max);
dWF2 = project(WF2,dWF2,W_max);
dWC2 = project(WC2,dWC2,W_max);
dWA2 = project(WA2,dWA2,W_max);

sys = [dWF1(:);dWC1;dWA1;dWF2(:);dWC2;dWA2];
end

function sys = mdlOutputs(t,x,u)
% 输出只保留：请求方向盘、实际方向盘、权重范数
global N m Iz lf cf0 cf_rate r2 u_d R1

WF1 = reshape(x(1:2*N),N,2);
WA1 = x(3*N+1:4*N);
WF2 = reshape(x(4*N+1:6*N),N,2);
WA2 = x(7*N+1:8*N);

varsigma = diag([max(u(1),eps),max(u(4),eps)]);
z1 = [u(2);u(3)];
chi2 = [u(9);u(10)];
Z_F = [u(7);u(8);u(9);u(10)];

% 第一层Actor产生alpha1
[~,dS_J1] = AGV_RBF([Z_F;z1],'J');
grad_JA1 = dS_J1(:,5:6)'*WA1;
alpha1 = -0.5*(R1\(varsigma'*grad_JA1));

% 第二层Actor产生唯一标量delta
z2 = chi2-alpha1;
[~,dS_J2] = AGV_RBF([Z_F;z2],'J');
cf = cf0*(1+cf_rate*sin(0.01*t));
C = [cf/m;lf*cf/Iz];
delta = -(C'*(dS_J2(:,5:6)'*WA2))/(2*r2);
delta1 = min(max(delta,-u_d),u_d);

W = norm([WF1(:);WA1;WF2(:);WA2]);
sys = [delta;delta1;W];
end

function dW = project(W,dW,W_max)
% 简单投影：达到边界后不再向外增长
nw = norm(W);
if nw >= W_max && W(:)'*dW(:) > 0
    dW = dW-W*(W(:)'*dW(:))/(nw^2);
end
end

