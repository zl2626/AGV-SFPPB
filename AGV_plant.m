function [sys,x0,str,ts] = AGV_plant(t,x,u,flag)
% AGV_PLANT  车辆横向误差模型。
% 输入顺序：[delta_sat,rho_0]。

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
sizes.NumContStates  = 4;   % [e_y,e_phi,de_y,de_phi]
sizes.NumDiscStates  = 0;   % 0个离散状态
sizes.NumOutputs     = 6;   % 6个输出
sizes.NumInputs      = 2;   % 2个输入 (delta_sat, rho_0)
sizes.DirFeedthrough = 0;   % 输出只返回状态x，不依赖当前输入u
sizes.NumSampleTimes = 1;   % 1个采样时间

sys = simsizes(sizes);
x0  = [-0.1; 0.01; 0; 0];       % 初始误差和误差导数
str = [];
ts  = [0 0];  % 连续系统

%% 导数计算函数
function sys = mdlDerivatives(t,x,u)
% 状态解包
e_y = x(1);    e_phi = x(2);
de_y = x(3);   de_phi = x(4);

% 输入解包：上游控制器已经完成唯一一次饱和。
delta_sat = u(1);
rho_0 = u(2);

% 车辆物理参数
m = 1832;
Iz = 2488;
lf = 1.18;
lr = 1.77;
vx = 20;

% 时变轮胎侧偏刚度
cf_nom = 80000;
cr_nom = 120000;
cf = cf_nom * (1 + 0.1*sin(0.01*t));
cr = cr_nom * (1 + 0.1*sin(0.01*t));

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
     0, (-lf*cf + lr*cr)/Iz, A21, A22];
B = [0; 0; B1; B2];

% 扰动项。横向加速度和横摆角加速度使用各自的量纲参数。
% 数值暂沿用原压力测试工况，后续论文实验应单独说明物理标定。
disturbance_y_amplitude = 18;       % m/s^2
disturbance_phi_amplitude = 18;     % rad/s^2
disturbance_period = 8;
current_phase = mod(t, disturbance_period);
disturbance_y = 0;
disturbance_phi = 0;
if current_phase > (disturbance_period - 1)
    disturbance_y = disturbance_y_amplitude*sin(2*pi*2*t);
    disturbance_phi = disturbance_phi_amplitude*sin(2*pi*2*t);
end

% 扰动向量 - 修正了公式中的错误（原代码中使用了lf*cf和lf*cr，应该是lr*cr）
D = [0;
     0;
     -((lf*cf - lr*cr)/m + vx^2)*rho_0 + disturbance_y;
     -((lf^2*cf + lr^2*cr)/Iz)*rho_0 + disturbance_phi];

% 状态导数计算
X = [e_y; e_phi; de_y; de_phi];
dX_dt = A*X + B*delta_sat + D;

% 只积分同一套误差状态，避免再建立一套并行的vy、omega_z动力学。
sys = dX_dt;

%% 输出函数
function sys = mdlOutputs(~,x,~)
% 根据同一套误差状态给出车辆可视化量。
vx = 20;
v_y = x(3)-vx*x(2);
omega_z = x(4);
sys = [x;v_y;omega_z];
