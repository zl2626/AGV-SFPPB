function [sys,x0,str,ts] = AGV_ctrl(t,x,u,flag)
% AGV_CTRL  SFPPB-ICAS-RL控制器
% 第一层：z1 -> alpha1
% 第二层：z2=chi2-alpha1-O -> delta
% O是论文中的输入饱和补偿状态，不是Safety Filter或残差控制器。
% 只有一个实际方向盘delta。

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
% ========================== ICAS-RL参数 ==========================
global N c1 c2 Upsilon1 Upsilon2 sigma1 sigma2
global gamma_c1 gamma_c2 gamma_a1 gamma_a2
global learning_enabled u_d m Iz lf cf0 cf_rate

% 论文控制参数
N = 7;                           % RBF节点数
c1 = 2;                          % 第一层稳定项
c2 = 2;                          % 第二层稳定项
Upsilon1 = 0.04;                 % 第一层Identifier增益
Upsilon2 = 0.04;                 % 第二层Identifier增益
sigma1 = 0.08;                   % 第一层Identifier sigma
sigma2 = 0.08;                   % 第二层Identifier sigma
gamma_c1 = 0.004;                % 第一层Critic增益
gamma_c2 = 0.004;                % 第二层Critic增益
gamma_a1 = 0.012;                % 第一层Actor增益
gamma_a2 = 0.012;                % 第二层Actor增益

% AGV输入矩阵参数
u_d = 0.5;
m = 1832;
Iz = 2488;
lf = 1.18;
cf0 = 80000;
cf_rate = 0.10;

% 设为0可做冻结权重对比；默认是在线学习。
if isempty(learning_enabled)
    learning_enabled = 1;
end

sizes = simsizes;
sizes.NumContStates  = 12*N+2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 13;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

% 状态顺序：[WF1;WC1;WA1;WF2;WC2;WA2;O]
% 六组权重都是N×2，O=[O_y,O_phi]。
x0 = zeros(12*N+2,1);
str = [];
ts = [0 0];
end

function sys = mdlDerivatives(t,x,u)
global N c1 c2 Upsilon1 Upsilon2 sigma1 sigma2
global gamma_c1 gamma_c2 gamma_a1 gamma_a2 learning_enabled
global u_d m Iz lf cf0 cf_rate

[WF1,WC1,WA1,WF2,WC2,WA2,O] = unpackStates(x,N);
[varsigma,z1,chi2,Z_F] = readInputs(u);

% ------------------------- 第一层RL -------------------------
S_F1 = AGV_RBF(Z_F,'F');
S_J1 = AGV_RBF([Z_F;z1],'J');
F1_hat = WF1'*S_F1;

% alpha1 = A^(-1)[-c1*z1-F1_hat-(1/2)Wa1'*S_J1]
alpha1 = varsigma\(-c1*z1-F1_hat-0.5*WA1'*S_J1);

% ------------------------- 第二层RL -------------------------
z2 = chi2-alpha1-O;
S_F2 = AGV_RBF(Z_F,'F');
S_J2 = AGV_RBF([Z_F;z2],'J');
F2_hat = WF2'*S_F2;

cf = cf0*(1+cf_rate*sin(0.01*t));
C = [cf/m;lf*cf/Iz];

% 向量化AGV方向盘公式：
% p_a2=2*c2*z2+2*F2_hat+Wa2'*S_J2
% delta=-(1/2)C'*p_a2
p_a2 = 2*c2*z2+2*F2_hat+WA2'*S_J2;
delta = -0.5*C'*p_a2;
delta1 = min(max(delta,-u_d),u_d);

% 论文输入饱和补偿：O_dot=-O+C[k(delta)-delta]
dO = -O+C*(delta1-delta);

if learning_enabled
    % ----------------------- Identifier -----------------------
    dWF1 = Upsilon1*(S_F1*z1'-sigma1*WF1);
    dWF2 = Upsilon2*(S_F2*z2'-sigma2*WF2);

    % -------------------------- Critic -------------------------
    dWC1 = -gamma_c1*(S_J1*S_J1')*WC1;
    dWC2 = -gamma_c2*(S_J2*S_J2')*WC2;

    % --------------------------- Actor -------------------------
    dWA1 = (S_J1*S_J1')*(gamma_a1*(WA1-WC1)+gamma_c1*WC1);
    dWA2 = (S_J2*S_J2')*(gamma_a2*(WA2-WC2)+gamma_c2*WC2);
else
    dWF1 = zeros(size(WF1));
    dWC1 = zeros(size(WC1));
    dWA1 = zeros(size(WA1));
    dWF2 = zeros(size(WF2));
    dWC2 = zeros(size(WC2));
    dWA2 = zeros(size(WA2));
end

sys = [dWF1(:);dWC1(:);dWA1(:);dWF2(:);dWC2(:);dWA2(:);dO];
end

function sys = mdlOutputs(t,x,u)
global N c1 c2 u_d m Iz lf cf0 cf_rate

[WF1,WC1,WA1,WF2,WC2,WA2,O] = unpackStates(x,N);
[varsigma,z1,chi2,Z_F] = readInputs(u);

% 第一层Actor和Identifier
S_F1 = AGV_RBF(Z_F,'F');
S_J1 = AGV_RBF([Z_F;z1],'J');
F1_hat = WF1'*S_F1;
alpha1 = varsigma\(-c1*z1-F1_hat-0.5*WA1'*S_J1);

% 第二层唯一方向盘控制
z2 = chi2-alpha1-O;
S_F2 = AGV_RBF(Z_F,'F');
S_J2 = AGV_RBF([Z_F;z2],'J');
F2_hat = WF2'*S_F2;
cf = cf0*(1+cf_rate*sin(0.01*t));
C = [cf/m;lf*cf/Iz];
p_a2 = 2*c2*z2+2*F2_hat+WA2'*S_J2;
delta = -0.5*C'*p_a2;
delta1 = min(max(delta,-u_d),u_d);

W = norm([WF1(:);WC1(:);WA1(:);WF2(:);WC2(:);WA2(:)]);
sys = [delta;delta1;W];
end

function [WF1,WC1,WA1,WF2,WC2,WA2,O] = unpackStates(x,N)
i = 0;
WF1 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
WC1 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
WA1 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
WF2 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
WC2 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
WA2 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
O = x(i+1:i+2);
end

function [varsigma,z1,chi2,Z_F] = readInputs(u)
% Mux1输入：[varsigma_y,z1_y,z1_phi,varsigma_phi,0,0,
%           e_y,e_phi,de_y,de_phi,0,0,rho]
varsigma = diag([max(u(1),eps),max(u(4),eps)]);
z1 = [u(2);u(3)];
chi2 = [u(9);u(10)];
Z_F = [u(7);u(8);u(9);u(10)];
end
