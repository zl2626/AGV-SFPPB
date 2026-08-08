
close all;

% 从工作区获取数据
t = out.t;
e_y = out.e_y;
eyu = out.eyu;
eyl = out.eyl;
eyu_ = out.eyu_;
eyl_ = out.eyl_;
e_phi = out.e_phi;
ephiu = out.ephiu;
ephil = out.ephil;
ephiu_ = out.ephiu_;
ephil_ = out.ephil_;
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

% 绘制 e_y
% figure(1);
% plot(t, e_y, 'b', t, eyu, 'r', t, eyl, 'r', t, eyu_, 'g--', t, eyl_, 'g--', 'linewidth', 2);
% xlabel('Time (sec)','FontSize', 16); ylabel('$e_y$','FontSize', 16, 'Interpreter', 'latex');
% legend('$e_y$', '$E_h$', '$E_l$', '$e_h$', '$e_l$', 'FontSize', 24, 'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
% grid on;
% % xlim([0, 20]);
% ylim([-0.5, 0.5]);

figure(1);
plot(t, e_y, 'b', t, eyu, 'r', t, eyl, 'r', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16); ylabel('$e_y$','FontSize', 16, 'Interpreter', 'latex');
legend('$e_y$', '$e_h$', '$e_l$', 'FontSize', 24, 'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 20]);
ylim([-0.5, 0.5]);

% 绘制e_phi
% figure(2);
% plot(t, e_phi, 'b', t, ephiu, 'r', t, ephil, 'r', t, ephiu_, 'g--', t, ephil_, 'g--', 'linewidth', 2);
% xlabel('Time (sec)','FontSize', 16); ylabel('$e_\varphi$', 'FontSize', 16, 'Interpreter', 'latex');
% legend('$e_\varphi$', '$E_h$', '$E_l$', '$e_h$', '$e_l$', 'FontSize', 24, 'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
% grid on;
% % xlim([0, 20]);
% ylim([-0.15, 0.15]);

figure(2);
plot(t, e_phi, 'b', t, ephiu, 'r', t, ephil, 'r', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16); ylabel('$e_\varphi$', 'FontSize', 16, 'Interpreter', 'latex');
legend('$e_\varphi$', '$e_h$', '$e_l$', 'FontSize', 24, 'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 20]);
ylim([-0.05, 0.05]);

% 绘制 v_y
figure(3);
plot(t, v_y, 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16); ylabel('$v_y$', 'FontSize', 16, 'Interpreter', 'latex');
legend('$v_y$', 'FontSize', 24, 'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 10]);
ylim([-0.012, 0.007]);

% 绘制omega_z
figure(4);
plot(t, omega_z, 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16); ylabel('$\omega_z$(rad/s)','FontSize', 16, 'Interpreter', 'latex');
legend('$\omega_z$', 'FontSize', 24, 'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 10]);
ylim([-0.006, 0.02]);

% 绘制 s1y 和 z1y
figure(5);
plot(t, z1y, 'r--', t, s1y, 'b-', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16); ylabel('$s_{1y}$','FontSize', 16, 'Interpreter', 'latex');
legend('PI compensation signal $s_{1y}$ \,\,\,', '$s_{1y}$ without PI control', 'FontSize', 24, 'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 20]);
ylim([-1.4, 1.0]);

% 绘制 s2y 和 z2y
figure(6);
plot(t, z2y, 'r--', t, s2y, 'b-', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16); ylabel('$s_{2y}$','FontSize', 16, 'Interpreter', 'latex');
legend('PI compensation signal $s_{2y}$ \,\,\,', '$s_{2y}$ without PI control', 'FontSize', 24, 'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 20]);
ylim([-0.3, 0.4]);

% 绘制 s1phi 和 z1phi
figure(7);
plot(t, z1phi, 'r--', t, s1phi, 'b-', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16); ylabel('$s_{1 \varphi}$','FontSize', 16, 'Interpreter', 'latex');
legend('PI compensation signal $s_{1\varphi}$ \,\,\,', '$s_{1\varphi}$ without PI control', 'FontSize', 24, 'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 20]);
ylim([-1, 1.3]);

% 绘制 s2phi 和 z2phi
figure(8);
plot(t, z2phi, 'r--', t, s2phi, 'b-', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16); ylabel('$s_{2 \varphi}$','FontSize', 16, 'Interpreter', 'latex');
legend('PI compensation signal $s_{2\varphi}$ \,\,\,', '$s_{2\varphi}$ without PI control', 'FontSize', 24, 'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 20]);
ylim([-0.6, 0.8]);

% 绘制 delta 和 delta1
figure(9);
plot(t, delta, 'b-', t, delta1, 'r--', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
legend('$\delta$', '$sat(\delta)$', 'FontSize', 24, 'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 20]);
ylim([-0.2, 1.6]);
% ylim([-150, 200]);

% 绘制 rho_0
figure(10);

% 检查out结构体中是否有rho_0
if isfield(out, 'rho_0')
    rho_0_data = out.rho_0;
    
    % 创建对应的时间向量
    simulation_time = t(end);
    time_per_step = simulation_time / length(rho_0_data);
    t_rho = (0:length(rho_0_data)-1) * time_per_step;
    
    stairs(t_rho, rho_0_data, 'linewidth', 2);
else
    % 使用默认值
    rho_0_values = [0 0.01 -0.01 0.01 0 0 0 0 0 0].';
    simulation_time = t(end);
    time_per_step = simulation_time / length(rho_0_values);
    t_rho = (0:length(rho_0_values)-1) * time_per_step;
    
    stairs(t_rho, rho_0_values, 'linewidth', 2);
end

xlabel('Time (sec)','FontSize', 16);
legend('$\rho_0$', 'FontSize', 24, 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
ylim([-0.015, 0.015]);

% 绘制 W
figure(11);
plot(t, w,'r', t, W, 'b','linewidth', 3);
xlabel('Time (sec)','FontSize', 16);
legend('$\omega$', '$\mathcal{W}$', 'FontSize', 24, 'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 10]);
ylim([-0.5, 1.5]);

% 绘制 w
figure(12);
plot(t, w, 'linewidth', 3);
xlabel('Time (sec)','FontSize', 16);
legend('$\omega$', 'FontSize', 24, 'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, 10]);
ylim([-0.5, 1.5]);
