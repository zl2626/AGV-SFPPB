function [sys,x0,str,ts] = assist1(t,x,u,flag)
% ASSIST1  输入饱和补偿状态 rho。
% rho_dot = -p1*rho + p2*(varpi1+varpi2)

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
global u_d p1 p2 rho_dot rho_dot_time

% u_d 由 AGV_ctrl.m 统一设置；直接运行 assist1 时才使用默认值。
if isempty(u_d)
    u_d = 0.5;
end
p1 = 5;                            % rho 衰减系数
p2 = 0.5;                          % 饱和超限增益
rho_dot = 0;                       % 提供给 SFPPB 边界导数
rho_dot_time = -inf;               % 同一求解时刻只更新一次

sizes = simsizes;
sizes.NumContStates  = 1;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = 0;
str = [];
ts = [0 0];
end

function sys = mdlDerivatives(t,x,u)
global u_d p1 p2 rho_dot rho_dot_time

rho = max(x(1),0);
delta = u(1);

% 论文中的两个饱和超限项。
varpi1 = (sign(delta-u_d)+1)*(delta-u_d);
varpi2 = (sign(delta+u_d)-1)*(delta+u_d);

d_rho = -p1*rho+p2*(varpi1+varpi2);
if x(1) <= 0 && d_rho < 0
    d_rho = 0;
end

% 避免代数环：同一求解时刻只把一次导数传给 SFPPB。
if t > rho_dot_time+1e-10
    rho_dot = d_rho;
    rho_dot_time = t;
end
sys = d_rho;
end

function sys = mdlOutputs(x)
sys = max(x(1),0);
end
