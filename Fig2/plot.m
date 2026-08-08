% 创建新的图集窗口
combined_fig = figure('Position', [100, 100, 1200, 800]);

% 加载并复制第一个.fig文件
fig1 = openfig('1.fig');
ax1 = findobj(fig1, 'Type', 'axes');
subplot(2,2,1);
copyobj(ax1.Children, gca);
title('横向位移误差 e_y');
xlabel('时间 (s)'); ylabel('e_y (m)'); grid on;


% 加载并复制第二个.fig文件
fig2 = openfig('2.fig');
ax2 = findobj(fig2, 'Type', 'axes');
subplot(2,2,2);
copyobj(ax2.Children, gca);
title('航向角误差 e_\phi');
xlabel('时间 (s)'); ylabel('e_\phi (rad)'); grid on;


% 加载并复制第三个.fig文件
fig3 = openfig('3.fig');
ax3 = findobj(fig3, 'Type', 'axes');
subplot(2,2,3);
copyobj(ax3.Children, gca);
title('控制输入 \delta');
xlabel('时间 (s)'); ylabel('\delta (rad)'); grid on;


% 加载并复制第四个.fig文件
fig4 = openfig('4.fig');
ax4 = findobj(fig4, 'Type', 'axes');
subplot(2,2,4);
copyobj(ax4.Children, gca);
title('横向速度 v_y');
xlabel('时间 (s)'); ylabel('v_y (m/s)'); grid on;
