close all;

% 从工作区读取原模型的记录信号。
t = out.t;
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

% Figure 1：横向误差与性能边界。
figure(1);
plot(t, e_y, 'b', t, eyu, 'r', t, eyl, 'r', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$e_y$','FontSize', 16, 'Interpreter', 'latex');
legend('$e_y$', '$e_h$', '$e_l$', 'FontSize', 24, ...
    'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 20]);
ylim([-0.5, 0.5]);

% Figure 2：航向误差与性能边界。
figure(2);
plot(t, e_phi, 'b', t, ephiu, 'r', t, ephil, 'r', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$e_\phi$', 'FontSize', 16, 'Interpreter', 'latex');
legend('$e_\phi$', '$e_h$', '$e_l$', 'FontSize', 24, ...
    'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 20]);
ylim([-0.05, 0.05]);

% Figure 3：横向速度。
figure(3);
plot(t, v_y, 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$v_y$', 'FontSize', 16, 'Interpreter', 'latex');
legend('$v_y$', 'FontSize', 24, 'FontAngle', 'italic', ...
    'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 10]);
ylim([-0.012, 0.007]);

% Figure 4：横摆角速度。
figure(4);
plot(t, omega_z, 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$\omega_z$(rad/s)','FontSize', 16, 'Interpreter', 'latex');
legend('$\omega_z$', 'FontSize', 24, 'FontAngle', 'italic', ...
    'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 10]);
ylim([-0.006, 0.02]);

% Figure 5：第一层横向 PI 变换误差。
figure(5);
plot(t, z1y, 'r--', t, s1y, 'b-', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$s_{1y}$','FontSize', 16, 'Interpreter', 'latex');
legend('$z_{1y}$', '$s_{1y}=z_{1y}+k_{1y}I_{1y}$', 'FontSize', 24, ...
    'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 20]);
ylim([-1.4, 1.0]);

% Figure 6：第二层横向 PI 变换误差。
figure(6);
plot(t, z2y, 'r--', t, s2y, 'b-', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$s_{2y}$','FontSize', 16, 'Interpreter', 'latex');
legend('$z_{2y}$', '$s_{2y}=z_{2y}+k_{2y}I_{2y}$', 'FontSize', 24, ...
    'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 20]);
ylim([-0.3, 0.4]);

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
ylim([-1, 1.3]);

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
ylim([-0.6, 0.8]);

% Figure 9：方向盘请求与饱和后的实际输入。
figure(9);
plot(t, delta, 'b-', t, delta1, 'r--', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
legend('$\delta$', '$sat(\delta)$', 'FontSize', 24, ...
    'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 20]);
ylim([-0.2, 1.6]);

% Figure 10：道路参考曲率 rho_0。
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
ylim([-0.025, 0.025]);

% Figure 11：扰动信号与权重范数。
figure(11);
plot(t, w,'r', t, W, 'b','linewidth', 3);
xlabel('Time (sec)','FontSize', 16);
legend('$\omega$', '$\mathcal{W}$', 'FontSize', 24, ...
    'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 10]);
ylim([-0.5, 1.5]);

% Figure 12：扰动信号。
figure(12);
plot(t, w, 'linewidth', 3);
xlabel('Time (sec)','FontSize', 16);
legend('$\omega$', 'FontSize', 24, 'FontAngle', 'italic', ...
    'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 10]);
ylim([-0.5, 1.5]);

% Figure 13：由 rho_0 恢复的 U 形参考轨迹与实际 AGV 轨迹。
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
legend('Reference U-shaped path', 'Actual AGV path', ...
    'FontSize', 18, 'Interpreter', 'none');
grid on;
axis equal;
