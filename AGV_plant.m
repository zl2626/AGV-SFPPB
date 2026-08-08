function [sys,x0,str,ts] = AGV_plant(t,x,u,flag)
% Level-1 MATLAB S-Function for AGV plant model
% 修复版，保持与一级S-Function模块的兼容性

switch flag
    case 0  % 初始化
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 1  % 连续状态导数
        sys = mdlDerivatives(t,x,u);
    case 3  % 输出
        sys = mdlOutputs(t,x,u);
    case 9  % 终止
        sys = [];
    otherwise
        sys = [];
end

%% 初始化函数
function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 6;   % 6个连续状态
sizes.NumDiscStates  = 0;   % 0个离散状态
sizes.NumOutputs     = 6;   % 6个输出
sizes.NumInputs      = 2;   % 2个输入 (delta, rho_0)
sizes.DirFeedthrough = 1;   % 有直接馈通（输出依赖输入）
sizes.NumSampleTimes = 1;   % 1个采样时间

sys = simsizes(sizes);
x0  = [-0.1; 0.01; 0; 0; 0; 0];  % 初始状态
str = [];
ts  = [0 0];  % 连续系统

%% 导数计算函数
function sys = mdlDerivatives(t,x,u)
% 状态解包
e_y = x(1);    e_phi = x(2);
de_y = x(3);   de_phi = x(4);
vy = x(5);     omega_z = x(6);

% 输入解包
delta = u(1);
rho_0 = u(2);

% 车辆物理参数
m = 1832;
Iz = 2488;
lf = 1.18;
lr = 1.77;
vx = 20;
u_d = 0.5;

% 时变轮胎侧偏刚度
cf_nom = 80000;
cr_nom = 120000;
cf = cf_nom * (1 + 0.1*sin(0.01*t));
cr = cr_nom * (1 + 0.1*sin(0.01*t));

% 转向角饱和限制
if abs(delta) > u_d
    delta_sat = u_d * sign(delta);
else
    delta_sat = delta;
end

% 动力学模型参数
A11 = -(cf + cr)/(m*vx);
A12 = (-lf*cf + lr*cr)/(m*vx) - vx;
A21 = (-lf*cf + lr*cr)/(Iz*vx);
A22 = -(lf^2*cf + lr^2*cr)/(Iz*vx);
B1 = cf/m;
B2 = lf*cf/Iz;

% 矩阵构建
A = [0, 0, 1, 0;
     0, 0, 0, 1;
     0, (cf + cr)/m, A11, A12;
     0, (-lf*cf - lr*cr)/Iz, A21, A22];
B = [0; 0; B1; B2];

% 扰动项
disturbance_amplitude = 18;
disturbance_period = 8;
current_phase = mod(t, disturbance_period);
disturbance = 0;
if current_phase > (disturbance_period - 1)
    disturbance = disturbance_amplitude * sin(2*pi*2*t);
end
if t >= 0 && t <= 1
    disturbance = disturbance + 0 * sin(2*pi*1*t);  % 这个扰动项为0，可以删除
end

% 扰动向量 - 修正了公式中的错误（原代码中使用了lf*cf和lf*cr，应该是lr*cr）
D = [0;
     0;
     -((lf*cf - lr*cr)/m + vx^2)*rho_0 + disturbance;  % 修正：lr*cr 而不是 lf*cr
     -((lf^2*cf + lr^2*cr)/Iz)*rho_0 + disturbance];   % 修正：lr^2*cr 而不是 lf^2*cr

% 状态导数计算
X = [e_y; e_phi; de_y; de_phi];
dX_dt = A*X + B*delta_sat + D;

% 计算vy和omega_z的导数
dvy_dt = A11*vy + A12*omega_z + B1*delta_sat;
domega_z_dt = A21*vy + A22*omega_z + B2*delta_sat;

% 返回状态导数
sys = [dX_dt(1); dX_dt(2); dX_dt(3); dX_dt(4); dvy_dt; domega_z_dt];

%% 输出函数
function sys = mdlOutputs(t,x,u)
% 输出所有状态
sys = x;