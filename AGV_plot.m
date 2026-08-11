close all;

% 从工作区读取原模型的记录信号。
t = out.t(:);
e_y = out.e_y;
eyu = out.eyu;
eyl = out.eyl;
e_phi = out.e_phi;
ephiu = out.ephiu;
ephil = out.ephil;
v_y = out.v_y;
omega_z = out.omega_z;
s1y = out.s1y;
z1y = out.z1y;
s1phi = out.s1phi;
z1phi = out.z1phi;
s2y = out.s2y;
z2y = out.z2y;
s2phi = out.s2phi;
z2phi = out.z2phi;
delta = out.delta;
delta1 = out.delta1;
W = out.W;
w = out.w;
rho_0_data = out.rho_0(:);
rho = out.rho(:);
rho_dot = out.rho_dot(:);

% 方向盘饱和上限，与 AGV_ctrl.m 保持一致。
% 做 u_d=0.3 饱和试验时，在运行本脚本前设置 u_d_plot=0.3。
if ~exist('u_d_plot','var')
    u_d_plot = 0.5;
end
u_d = u_d_plot;

% 每次运行都记录方向盘平滑性、饱和时间和边界余量。
d_delta = gradient(delta,t);
J_delta = sqrt(mean(d_delta.^2));
T_sat = trapz(t,double(abs(delta)>u_d+1e-8));
gap_y_low = min(e_y-eyl);
gap_y_high = min(eyu-e_y);
gap_phi_low = min(e_phi-ephil);
gap_phi_high = min(ephiu-e_phi);
fprintf(['J_delta=%.6g, T_sat=%.6g s, ' ...
    'min_gap_y=[%.6g, %.6g], min_gap_phi=[%.6g, %.6g]\n'], ...
    J_delta,T_sat,gap_y_low,gap_y_high,gap_phi_low,gap_phi_high);

% Figure 1：横向误差与性能边界。
figure(1);
plot(t, e_y, 'b', t, eyu, 'r', t, eyl, 'r', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$e_y$','FontSize', 16, 'Interpreter', 'latex');
legend('$e_y$', '$e_h$', '$e_l$', 'FontSize', 24, ...
    'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 20]);
setYLim([e_y;eyu;eyl]);

% Figure 2：航向误差与性能边界。
figure(2);
plot(t, e_phi, 'b', t, ephiu, 'r', t, ephil, 'r', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$e_\phi$', 'FontSize', 16, 'Interpreter', 'latex');
legend('$e_\phi$', '$e_h$', '$e_l$', 'FontSize', 24, ...
    'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 20]);
setYLim([e_phi;ephiu;ephil]);

% Figure 3：横向速度。
figure(3);
plot(t, v_y, 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$v_y$', 'FontSize', 16, 'Interpreter', 'latex');
legend('$v_y$', 'FontSize', 24, 'FontAngle', 'italic', ...
    'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 20]);
setYLim(v_y);

% Figure 4：横摆角速度。
figure(4);
plot(t, omega_z, 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$\omega_z$(rad/s)','FontSize', 16, 'Interpreter', 'latex');
legend('$\omega_z$', 'FontSize', 24, 'FontAngle', 'italic', ...
    'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 20]);
setYLim(omega_z);

% Figure 5：第一层横向 PI 变换误差。
figure(5);
plot(t, z1y, 'r--', t, s1y, 'b-', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$s_{1y}$','FontSize', 16, 'Interpreter', 'latex');
legend('$z_{1y}$', '$s_{1y}=z_{1y}+k_{1y}I_{1y}$', 'FontSize', 24, ...
    'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 20]);
setYLim([z1y;s1y]);

% Figure 6：第二层横向 PI 变换误差。
figure(6);
plot(t, z2y, 'r--', t, s2y, 'b-', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$s_{2y}$','FontSize', 16, 'Interpreter', 'latex');
legend('$z_{2y}$', '$s_{2y}=z_{2y}+k_{2y}I_{2y}$', 'FontSize', 24, ...
    'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 20]);
setYLim([z2y;s2y]);

% Figure 7：第一层航向 PI 变换误差。
figure(7);
plot(t, z1phi, 'r--', t, s1phi, 'b-', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$s_{1\phi}$','FontSize', 16, 'Interpreter', 'latex');
legend('$z_{1\phi}$', '$s_{1\phi}=z_{1\phi}+k_{1\phi}I_{1\phi}$', ...
    'FontSize', 24, 'FontAngle', 'italic', 'Interpreter', 'latex', ...
    'IconColumnWidth',50);
grid on;
xlim([0, 20]);
setYLim([z1phi;s1phi]);

% Figure 8：第二层航向 PI 变换误差。
figure(8);
plot(t, z2phi, 'r--', t, s2phi, 'b-', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$s_{2\phi}$','FontSize', 16, 'Interpreter', 'latex');
legend('$z_{2\phi}$', '$s_{2\phi}=z_{2\phi}+k_{2\phi}I_{2\phi}$', ...
    'FontSize', 24, 'FontAngle', 'italic', 'Interpreter', 'latex', ...
    'IconColumnWidth',50);
grid on;
xlim([0, 20]);
setYLim([z2phi;s2phi]);

% Figure 9：方向盘请求与饱和后的实际输入。
figure(9);
plot(t, delta, 'b-', t, delta1, 'r--', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
legend('$\delta$', '$sat(\delta)$', 'FontSize', 24, ...
    'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
yline(u_d, 'k:', 'u_d', 'LineWidth', 1.2, 'HandleVisibility', 'off');
yline(-u_d, 'k:', '-u_d', 'LineWidth', 1.2, 'HandleVisibility', 'off');
grid on;
xlim([0, 20]);
setYLim([delta;delta1;u_d;-u_d]);

% Figure 10：道路参考曲率 rho_0，不是柔性辅助状态 rho。
figure(10);
simulation_time = t(end);
if numel(rho_0_data) > 1
    t_rho = linspace(0, simulation_time, numel(rho_0_data));
else
    t_rho = [0, simulation_time];
    rho_0_data = [rho_0_data; rho_0_data];
end
stairs(t_rho, rho_0_data, 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
legend('$\rho_0$', 'FontSize', 24, 'Interpreter', 'latex', ...
    'IconColumnWidth',50);
grid on;
setYLim(rho_0_data);

% Figure 11：扰动信号与权重范数，使用同一坐标轴但不裁剪 W。
figure(11);
plot(t, w,'r', t, W, 'b','linewidth', 3);
xlabel('Time (sec)','FontSize', 16);
legend('$\omega$', '$\mathcal{W}=||W||$', 'FontSize', 24, ...
    'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 20]);
setYLim([w;W]);

% Figure 12：SFPPB 柔性辅助状态 rho。
figure(12);
plot(t, rho, 'm', 'linewidth', 3);
xlabel('Time (sec)','FontSize', 16);
 ylabel('$\rho$', 'FontSize', 16, 'Interpreter', 'latex');
legend('$\rho$', 'FontSize', 24, 'FontAngle', 'italic', ...
    'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 20]);
setYLim(rho);

% Figure 13：由 rho_0 恢复的曲线路径与实际 AGV 轨迹。
vx = 20;                                      % AGV_plant.m 中的纵向速度
rho_ref = interp1(t_rho, rho_0_data, t, 'previous', 'extrap');
psi_r = cumtrapz(t, vx*rho_ref);
X_r = cumtrapz(t, vx*cos(psi_r));
Y_r = cumtrapz(t, vx*sin(psi_r));
X = X_r-e_y.*sin(psi_r);
Y = Y_r+e_y.*cos(psi_r);

figure(13);
plot(X_r, Y_r, 'k--', X, Y, 'b-', 'linewidth', 2);
xlabel('$X$ (m)','FontSize', 16, 'Interpreter', 'latex');
ylabel('$Y$ (m)','FontSize', 16, 'Interpreter', 'latex');
legend('Reference curved path', 'Actual AGV path', ...
    'FontSize', 18, 'Interpreter', 'none');
grid on;
axis equal;

% 统一给曲线留出少量上下边距，避免边界或权重被坐标轴裁掉。
function setYLim(data)
data = data(isfinite(data));
if isempty(data)
    return;
end
low = min(data);
high = max(data);
span = max(high-low,1e-3);
margin = 0.10*span;
ylim([low-margin,high+margin]);
end
