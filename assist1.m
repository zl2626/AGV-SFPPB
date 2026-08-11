function [sys,x0,str,ts] = assist1(t,x,u,flag)
% ASSIST1  输入饱和补偿状态 rho。
% 输出的是两个状态：[rho; rho_dot]。
% rho_dot 先经过一个连续一阶滤波状态，避免 rho_dot 直接把
% delta -> SFPPB -> delta 连成代数环。

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 1
        sys = mdlDerivatives(t,x,u);
    case 3
        sys = mdlOutputs(x);
    case {2,4,9}
        sys = [];
    otherwise
        error('assist1:UnhandledFlag','Unhandled flag = %d.',flag);
end
end

function [sys,x0,str,ts] = mdlInitializeSizes
global u_d p1 p2 rho_filter_tau

% u_d 由 AGV_ctrl.m 统一设置；直接运行 assist1 时才使用默认值。
if isempty(u_d)
    u_d = 0.5;
end
if isempty(p1)
    p1 = 2;                         % rho 衰减系数
end
if isempty(p2)
    p2 = 5;                         % 饱和超限增益
end
if isempty(rho_filter_tau)
    rho_filter_tau = 0.02;          % rho_dot 滤波时间常数(s)
end

sizes = simsizes;
sizes.NumContStates  = 2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = [0;0];
str = [];
ts = [0 0];
end

function sys = mdlDerivatives(~,x,u)
global u_d p1 p2 rho_filter_tau

rho = max(x(1),0);
delta = u(1);

% 论文中的两个饱和超限项。
varpi1 = (sign(delta-u_d)+1)*(delta-u_d);
varpi2 = (sign(delta+u_d)-1)*(delta+u_d);

d_rho = -p1*rho+p2*(varpi1+varpi2);
if x(1) <= 0 && d_rho < 0
    d_rho = 0;
end

rho_dot = x(2);
d_rho_dot = (d_rho-rho_dot)/rho_filter_tau;

sys = [d_rho;d_rho_dot];
end

function sys = mdlOutputs(x)

rho = max(x(1),0);
rho_dot = x(2);
if x(1) <= 0 && rho_dot < 0
    rho_dot = 0;
end

% 输出只依赖连续状态，因此本模块没有直接馈通。
sys = [rho;rho_dot];
end
