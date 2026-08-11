function [sys,x0,str,ts] = assist1(t,x,u,flag)
% ASSIST1  输入饱和补偿状态 rho。
% rho_dot = -p1*rho + p2*(varpi1+varpi2)

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 1
        sys = mdlDerivatives(t,x,u);
    case 3
        sys = mdlOutputs(x,u);
    case {2,4,9}
        sys = [];
    otherwise
        error('assist1:UnhandledFlag','Unhandled flag = %d.',flag);
end
end

function [sys,x0,str,ts] = mdlInitializeSizes
global u_d p1 p2

% u_d 由 AGV_ctrl.m 统一设置；直接运行 assist1 时才使用默认值。
if isempty(u_d)
    u_d = 0.5;
end
p1 = 5;                            % rho 衰减系数
p2 = 0.5;                          % 饱和超限增益

sizes = simsizes;
sizes.NumContStates  = 1;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = 0;
str = [];
ts = [0 0];
end

function sys = mdlDerivatives(~,x,u)
global u_d p1 p2

rho = max(x(1),0);
delta = u(1);

% 论文中的两个饱和超限项。
varpi1 = (sign(delta-u_d)+1)*(delta-u_d);
varpi2 = (sign(delta+u_d)-1)*(delta+u_d);

d_rho = -p1*rho+p2*(varpi1+varpi2);
if x(1) <= 0 && d_rho < 0
    d_rho = 0;
end

sys = d_rho;
end

function sys = mdlOutputs(x,u)
global u_d p1 p2

rho = max(x(1),0);
delta = u(1);
varpi1 = (sign(delta-u_d)+1)*(delta-u_d);
varpi2 = (sign(delta+u_d)-1)*(delta+u_d);
d_rho = -p1*rho+p2*(varpi1+varpi2);
if x(1) <= 0 && d_rho < 0
    d_rho = 0;
end

% 输出 rho 和当前 rho_dot，供 SFPPB 直接计算柔性边界导数。
sys = [rho;d_rho];
end
