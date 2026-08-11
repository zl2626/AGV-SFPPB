function [sys,x0,str,ts] = assist1(~,x,u,flag)
% ASSIST1  饱和辅助状态rho
% 请求方向盘delta和实际饱和方向盘delta1的差值驱动rho，
% rho再同时送给y和phi两条SFPPB边界。

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 1
        sys = mdlDerivatives(x,u);
    case 3
        sys = mdlOutputs(x);
    case {2,4,9}
        sys = [];
    otherwise
        error('assist1:UnhandledFlag','Unhandled flag = %d.',flag);
end
end
function [sys,x0,str,ts] = mdlInitializeSizes
% =========================== 参数区 ===================================
global u_d m1 m2
u_d = 0.5;                     % 方向盘物理饱和上限
m1 = 5;                        % rho恢复系数
m2 = 0.5;                     % 饱和超限输入系数

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

function sys = mdlDerivatives(x,u)
global u_d m1 m2

rho = max(x(1),0);
delta = u(1);
delta1 = min(max(delta,-u_d),u_d);

% 对称饱和下，omega1+omega2=2|delta-delta1|。
omega = 2*abs(delta-delta1);
d_rho = -m1*rho+m2*omega;

% 保证rho不因数值误差变成负数。
if x(1) <= 0 && d_rho < 0
    d_rho = 0;
end
sys = d_rho;
end

function sys = mdlOutputs(x)
sys = max(x(1),0);
end
