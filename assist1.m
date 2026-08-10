function [sys,x0,str,ts] = assist1(t,x,u,flag,actuator_limit)
if nargin < 5 || isempty(actuator_limit)
    actuator_limit = 0.5;
end
switch flag
case 0
    [sys,x0,str,ts]=mdlInitializeSizes;
case 1
    sys=mdlDerivatives(t,x,u,actuator_limit);
case 3
    sys=mdlOutputs(t,x,u);
case {2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 1;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = 0;
str = [];
ts  = [0 0];

function sys=mdlDerivatives(~,x,u,u_d)
m1=5;m2=0.5;
v=u(1);
delta1=(sign(v-u_d)+1)*(v-u_d);
delta2=(sign(v+u_d)-1)*(v+u_d);
relative_excess=(delta1+delta2)/max(u_d,eps);
sys=-m1*x(1)+m2*relative_excess;


function sys=mdlOutputs(~,x,~)
sys(1)=x(1);
