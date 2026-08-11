function [sys,x0,str,ts] = assist1(~,x,u,flag)
% ASSIST1  SFPPB输入饱和辅助状态
% rho_dot = -p1*rho + p2*(varpi1+varpi2)
% rho同时送给横向和航向两个SFPPB边界。

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
% ======================= SFPPB参数 =======================
global u_d p1 p2
u_d = 0.5;                         % 执行器饱和上限
p1 = 5;                            % rho衰减系数
p2 = 0.5;                          % 饱和超限增益

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
global u_d p1 p2

rho = max(x(1),0);
delta = u(1);

% 论文中的两个饱和超限项，不使用等价的abs简写。
varpi1 = (sign(delta-u_d)+1)*(delta-u_d);
varpi2 = (sign(delta+u_d)-1)*(delta+u_d);

d_rho = -p1*rho+p2*(varpi1+varpi2);
if x(1) <= 0 && d_rho < 0
    d_rho = 0;
end
sys = d_rho;
end

function sys = mdlOutputs(x)
sys = max(x(1),0);
end
