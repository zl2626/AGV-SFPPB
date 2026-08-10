close all;

% Logged simulation data
t = out.t(:);
e_y = out.e_y(:);
eyu = out.eyu(:);
eyl = out.eyl(:);
e_phi = out.e_phi(:);
ephiu = out.ephiu(:);
ephil = out.ephil(:);
v_y = out.v_y(:);
omega_z = out.omega_z(:);
s1y = out.s1y(:);
z1y = out.z1y(:);
s1phi = out.s1phi(:);
z1phi = out.z1phi(:);
s2y = out.s2y(:);
z2y = out.z2y(:);
s2phi = out.s2phi(:);
z2phi = out.z2phi(:);
delta_safe = out.delta(:);
delta_applied = out.delta1(:);
ctrl_diagnostics = out.ctrl_diagnostics;
delta_actor = ctrl_diagnostics(:,4);
WF_norm = ctrl_diagnostics(:,5);
Wc_norm = ctrl_diagnostics(:,6);
Wa_move = ctrl_diagnostics(:,7);

output_dir = fullfile(fileparts(mfilename('fullpath')),'Fig','paper');
if ~exist(output_dir,'dir')
    mkdir(output_dir);
end

%% Fig. 1: lateral tracking error and SFPPB
fig = figure(1);
set(fig,'Color','w','Units','centimeters','Position',[2,2,8.6,6.4]);
hold on;
plot(t,e_y,'b-','LineWidth',2.0);
plot(t,eyu,'r--','LineWidth',1.8);
plot(t,eyl,'r--','LineWidth',1.8,'HandleVisibility','off');
hold off;
formatAxes(gca,t(end),[-0.22,0.04]);
xlabel('Time (s)','Interpreter','latex','FontSize',14);
ylabel('$e_y\;(\mathrm{m})$','Interpreter','latex','FontSize',14);
legend({'$e_y$','SFPPB'},'Interpreter','latex','FontSize',11, ...
    'Location','southeast','Box','off');
exportFigure(fig,output_dir,'Fig1_error_y');

%% Fig. 2: heading tracking error and SFPPB
fig = figure(2);
set(fig,'Color','w','Units','centimeters','Position',[2,2,8.6,6.4]);
hold on;
plot(t,e_phi,'b-','LineWidth',2.0);
plot(t,ephiu,'r--','LineWidth',1.8);
plot(t,ephil,'r--','LineWidth',1.8,'HandleVisibility','off');
hold off;
formatAxes(gca,t(end),[-0.012,0.032]);
xlabel('Time (s)','Interpreter','latex','FontSize',14);
ylabel('$e_{\varphi}\;(\mathrm{rad})$','Interpreter','latex','FontSize',14);
legend({'$e_{\varphi}$','SFPPB'},'Interpreter','latex','FontSize',11, ...
    'Location','northeast','Box','off');
exportFigure(fig,output_dir,'Fig2_error_phi');

%% Fig. 3: Actor command, safety-filtered command, and applied steering
fig = figure(3);
set(fig,'Color','w','Units','centimeters','Position',[2,2,17.8,7.0]);
hold on;
h_actor = plot(t,delta_actor,'b-','LineWidth',2.0);
h_applied = plot(t,delta_applied,'k:','LineWidth',2.2);
h_safe = plot(t,delta_safe,'r--','LineWidth',1.8);
h_limit = yline(0.5,'k-.','LineWidth',1.3);
yline(-0.5,'k-.','LineWidth',1.3,'HandleVisibility','off');
hold off;
formatAxes(gca,t(end),[-0.52,0.52]);
xlabel('Time (s)','Interpreter','latex','FontSize',14);
ylabel('$\delta\;(\mathrm{rad})$','Interpreter','latex','FontSize',14);
legend([h_actor,h_safe,h_applied,h_limit], ...
    {'$\delta_{\mathrm{Actor}}$','$\delta_{\mathrm{safe}}$', ...
    '$\delta_{\mathrm{applied}}$','$\pm\delta_{\max}$'}, ...
    'Interpreter','latex','FontSize',11,'Location','northoutside', ...
    'Orientation','horizontal','NumColumns',4,'Box','off');
exportFigure(fig,output_dir,'Fig3_steering_allocation');

%% Fig. 4: Identifier-Critic-Actor learning states
fig = figure(4);
set(fig,'Color','w','Units','centimeters','Position',[2,2,17.8,6.8]);
hold on;
plot(t,WF_norm,'b-','LineWidth',2.0);
plot(t,Wc_norm,'r--','LineWidth',1.8);
plot(t,Wa_move,'k-.','LineWidth',1.3);
hold off;
formatAxes(gca,t(end),[0,0.09]);
xlabel('Time (s)','Interpreter','latex','FontSize',14);
ylabel('Weight magnitude','Interpreter','latex','FontSize',14);
legend({'$\Vert W_F\Vert_2$','$\Vert W_c\Vert_2$', ...
    '$\Vert W_a-W_{a0}\Vert_2$'},'Interpreter','latex', ...
    'FontSize',11,'Location','northoutside','Orientation','horizontal', ...
    'NumColumns',3,'Box','off');
exportFigure(fig,output_dir,'Fig4_learning_weights');

%% Fig. 5: vehicle lateral states
fig = figure(5);
set(fig,'Color','w','Units','centimeters','Position',[2,2,8.6,10.8]);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

ax = nexttile;
plot(ax,t,v_y,'b-','LineWidth',2.0);
formatAxes(ax,t(end),[-1.6,1.6]);
ylabel(ax,'$v_y\;(\mathrm{m\,s^{-1}})$','Interpreter','latex','FontSize',14);
text(ax,0.02,0.92,'(a)','Units','normalized','FontName','Times New Roman', ...
    'FontSize',12,'FontWeight','bold');

ax = nexttile;
plot(ax,t,omega_z,'b-','LineWidth',2.0);
formatAxes(ax,t(end),[-1.6,1.6]);
xlabel(ax,'Time (s)','Interpreter','latex','FontSize',14);
ylabel(ax,'$\omega_z\;(\mathrm{rad\,s^{-1}})$', ...
    'Interpreter','latex','FontSize',14);
text(ax,0.02,0.92,'(b)','Units','normalized','FontName','Times New Roman', ...
    'FontSize',12,'FontWeight','bold');
exportFigure(fig,output_dir,'Fig5_vehicle_states');

%% Fig. A1: transformed-state diagnostics for the appendix
fig = figure(6);
set(fig,'Color','w','Units','centimeters','Position',[2,2,17.8,12.0]);
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

ax = nexttile;
plotDiagnosticPair(ax,t,s1y,z1y,'$s_{1y}$','$z_{1y}$',[-6,7]);
ax = nexttile;
plotDiagnosticPair(ax,t,s1phi,z1phi,'$s_{1\varphi}$', ...
    '$z_{1\varphi}$',[-5,4]);
ax = nexttile;
plotDiagnosticPair(ax,t,s2y,z2y,'$s_{2y}$','$z_{2y}$',[-0.45,0.65]);
xlabel(ax,'Time (s)','Interpreter','latex','FontSize',14);
ax = nexttile;
plotDiagnosticPair(ax,t,s2phi,z2phi,'$s_{2\varphi}$', ...
    '$z_{2\varphi}$',[-0.28,0.25]);
xlabel(ax,'Time (s)','Interpreter','latex','FontSize',14);
exportFigure(fig,output_dir,'FigA1_transformed_states');

function formatAxes(ax,t_end,y_limits)
set(ax,'FontName','Times New Roman','FontSize',12,'LineWidth',1.0, ...
    'TickLabelInterpreter','latex','Box','on','Layer','top');
ax.Toolbar.Visible = 'off';
grid(ax,'on');
ax.GridLineStyle = ':';
ax.GridAlpha = 0.15;
xlim(ax,[0,t_end]);
ylim(ax,y_limits);
end

function plotDiagnosticPair(ax,t,s,z,s_label,z_label,y_limits)
hold(ax,'on');
plot(ax,t,s,'b-','LineWidth',2.0);
plot(ax,t,z,'r--','LineWidth',1.8);
hold(ax,'off');
formatAxes(ax,t(end),y_limits);
ylabel(ax,'Amplitude','Interpreter','latex','FontSize',14);
legend(ax,{s_label,z_label},'Interpreter','latex','FontSize',11, ...
    'Location','best','Box','off');
end

function exportFigure(fig,output_dir,file_stem)
exportgraphics(fig,fullfile(output_dir,[file_stem '.pdf']), ...
    'ContentType','vector');
exportgraphics(fig,fullfile(output_dir,[file_stem '.png']), ...
    'Resolution',600);
end
