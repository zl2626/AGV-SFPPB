function [sys,x0,str,ts] = AGV_transfor(t,~,u,flag)
% AGV_TRANSFOR  SFPPB边界和NMT变换
% 代码结构沿用原AGV_TFS，只把性能边界改成SFPPB。
% Mux4输入：[e_y,de_y,rho_sat,e_phi,de_phi,rho_sat]
% 输出前4个是柔性边界，5到8个是varsigma和z1。

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 1
        sys = [];
    case 3
        sys = mdlOutputs(t,u);
    case {2,4,9}
        sys = [];
    otherwise
        error('AGV_transfor:UnhandledFlag','Unhandled flag = %d.',flag);
end
end
function [sys,x0,str,ts] = mdlInitializeSizes
% =========================== 参数区 ===================================
% y和phi各使用一组论文中的SFPPB参数，直接在这里调节。
global rho_0_y rho_T_y delta_y e_0_y T_y a_y
global rho_0_phi rho_T_phi delta_phi e_0_phi T_phi a_phi
global lambda1_y lambda2_y lambda1_phi lambda2_phi nmt_margin

rho_0_y = 0.28;                % y初始名义边界半宽
rho_T_y = 0.24;                % y稳态名义边界半宽
delta_y = 1;                   % y边界尺度
e_0_y = -0.10;
T_y = 5;
a_y = 1;

rho_0_phi = 0.10;              % phi初始名义边界半宽
rho_T_phi = 0.08;
delta_phi = 1;                 % phi边界尺度
e_0_phi = 0.01;
T_phi = 5;
a_phi = 1;

lambda1_y = 0.50;              % 饱和时下边界放宽量
lambda2_y = 0.50;              % 饱和时上边界放宽量
lambda1_phi = 0.40;
lambda2_phi = 0.40;
nmt_margin = 1e-9;             % 只用于数值试探点，不改记录的边界

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 21;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = [];
str = [];
ts = [0 0];
end

function sys = mdlOutputs(t,u)
global rho_0_y rho_T_y delta_y e_0_y T_y a_y
global rho_0_phi rho_T_phi delta_phi e_0_phi T_phi a_phi
global lambda1_y lambda2_y lambda1_phi lambda2_phi nmt_margin

% 从Mux4中取出两个物理误差；第三和第六个信号是同一个rho_sat。
e_y = u(1);
e_phi = u(4);
rho_sat = max(0,0.5*(u(3)+u(6)));

% -------------------------- SFPPB边界 -------------------------------
if t < T_y
    rho_y = (rho_0_y-rho_T_y)*(a_y*(T_y-t)/T_y)^3+rho_T_y;
    shift_y = (1-t/T_y)^1;
else
    rho_y = rho_T_y;
    shift_y = 0;
end
if t < T_phi
    rho_phi = (rho_0_phi-rho_T_phi)*(a_phi*(T_phi-t)/T_phi)^3+rho_T_phi;
    shift_phi = (1-t/T_phi)^1;
else
    rho_phi = rho_T_phi;
    shift_phi = 0;
end

% 通过e_0把名义边界平移到给定初始误差附近。
e_under_y = -delta_y*rho_y+e_0_y*shift_y;
e_bar_y = delta_y*rho_y+e_0_y*shift_y;
e_under_phi = -delta_phi*rho_phi+e_0_phi*shift_phi;
e_bar_phi = delta_phi*rho_phi+e_0_phi*shift_phi;

% 饱和状态同时作用于y和phi，只有一个公共rho_sat。
e_under_y_flex = e_under_y-lambda1_y*tanh(rho_sat);
e_bar_y_flex = e_bar_y+lambda2_y*tanh(rho_sat);
e_under_phi_flex = e_under_phi-lambda1_phi*tanh(rho_sat);
e_bar_phi_flex = e_bar_phi+lambda2_phi*tanh(rho_sat);

% ---------------------------- NMT ------------------------------------
% 求解器试探点可能恰好落在边界上，只在计算NMT时做极小截断。
margin_y = max(nmt_margin,1e-8*(e_bar_y_flex-e_under_y_flex));
margin_phi = max(nmt_margin,1e-8*(e_bar_phi_flex-e_under_phi_flex));
e_y_nmt = min(max(e_y,e_under_y_flex+margin_y),e_bar_y_flex-margin_y);
e_phi_nmt = min(max(e_phi,e_under_phi_flex+margin_phi),e_bar_phi_flex-margin_phi);

z1_y = log((e_y_nmt-e_under_y_flex)/(e_bar_y_flex-e_y_nmt));
z1_phi = log((e_phi_nmt-e_under_phi_flex)/(e_bar_phi_flex-e_phi_nmt));
varsigma_y = (e_bar_y_flex-e_under_y_flex) / ...
    ((e_bar_y_flex-e_y_nmt)*(e_y_nmt-e_under_y_flex));
varsigma_phi = (e_bar_phi_flex-e_under_phi_flex) / ...
    ((e_bar_phi_flex-e_phi_nmt)*(e_phi_nmt-e_under_phi_flex));

% 输出顺序必须和原Simulink Demux5一致。
zero = 0;
sys = [e_bar_y_flex;e_under_y_flex;e_bar_phi_flex;e_under_phi_flex; ...
       varsigma_y;z1_y;z1_phi;varsigma_phi; ...
       zero;zero;zero;zero;rho_sat; ...
       z1_y;z1_phi;zero;zero; ...
       e_bar_y;e_under_y;e_bar_phi;e_under_phi];
end
