close all;

% Simulation data
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
delta = out.delta(:);
delta1 = out.delta1(:);
W = out.W(:);
w = out.w(:);

% AGV-TFS scientific line convention
blue = [0, 0.4470, 0.7410];
red = [0.8500, 0.3250, 0.0980];
green = [0.4660, 0.6740, 0.1880];
purple = [0.4940, 0.1840, 0.5560];
gray = [0.25, 0.25, 0.25];
lw_response = 2.0;
lw_boundary = 1.7;
lw_reference = 1.4;

%% Figure 1: prescribed-performance tracking
figure(1);
set(gcf,'Color','w','Units','centimeters','Position',[2,2,18.5,15.5]);
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

ax = nexttile;
hold(ax,'on');
plot(t,e_y,'-','Color',blue,'LineWidth',lw_response);
plot(t,eyu,'--','Color',red,'LineWidth',lw_boundary);
plot(t,eyl,'--','Color',red,'LineWidth',lw_boundary);
hold(ax,'off');
styleAxes(ax,t(end));
ylabel(ax,'$e_y\;(\mathrm{m})$','Interpreter','latex');
title(ax,'\textbf{(a)}\quad Lateral tracking error','Interpreter','latex');
legend(ax,{'$e_y$','$\overline{B}_y$','$\underline{B}_y$'}, ...
    'Interpreter','latex','Location','southeast','Box','off');
padYLim(ax,[e_y;eyu;eyl]);

ax = nexttile;
hold(ax,'on');
plot(t,e_phi,'-','Color',blue,'LineWidth',lw_response);
plot(t,ephiu,'--','Color',red,'LineWidth',lw_boundary);
plot(t,ephil,'--','Color',red,'LineWidth',lw_boundary);
hold(ax,'off');
styleAxes(ax,t(end));
ylabel(ax,'$e_{\varphi}\;(\mathrm{rad})$','Interpreter','latex');
title(ax,'\textbf{(b)}\quad Heading tracking error','Interpreter','latex');
legend(ax,{'$e_{\varphi}$','$\overline{B}_{\varphi}$', ...
    '$\underline{B}_{\varphi}$'},'Interpreter','latex', ...
    'Location','northeast','Box','off');
padYLim(ax,[e_phi;ephiu;ephil]);

ax = nexttile;
hold(ax,'on');
plot(t,e_y,'-','Color',blue,'LineWidth',lw_response);
plot(t,eyu,'--','Color',red,'LineWidth',lw_boundary);
plot(t,eyl,'--','Color',red,'LineWidth',lw_boundary);
hold(ax,'off');
styleAxes(ax,t(end));
xlim(ax,[5,t(end)]);
ylim(ax,[-0.032,0.032]);
xlabel(ax,'Time (s)','Interpreter','latex');
ylabel(ax,'$e_y\;(\mathrm{m})$','Interpreter','latex');
title(ax,'\textbf{(c)}\quad Terminal-boundary detail','Interpreter','latex');
markDisturbanceWindows(ax);

ax = nexttile;
hold(ax,'on');
plot(t,1e3*e_phi,'-','Color',blue,'LineWidth',lw_response);
plot(t,1e3*ephiu,'--','Color',red,'LineWidth',lw_boundary);
plot(t,1e3*ephil,'--','Color',red,'LineWidth',lw_boundary);
hold(ax,'off');
styleAxes(ax,t(end));
xlim(ax,[5,t(end)]);
ylim(ax,[-5.5,5.5]);
xlabel(ax,'Time (s)','Interpreter','latex');
ylabel(ax,'$e_{\varphi}\;(\mathrm{mrad})$','Interpreter','latex');
title(ax,'\textbf{(d)}\quad Terminal-boundary detail','Interpreter','latex');
markDisturbanceWindows(ax);

%% Figure 2: constrained steering and learning
figure(2);
set(gcf,'Color','w','Units','centimeters','Position',[2,2,18.5,13.5]);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

ax = nexttile;
hold(ax,'on');
plot(t,delta,'-','Color',blue,'LineWidth',lw_response);
plot(t,delta1,'--','Color',red,'LineWidth',lw_boundary);
yline(0.5,'-.','Color',green,'LineWidth',lw_reference);
yline(-0.5,'-.','Color',green,'LineWidth',lw_reference);
hold(ax,'off');
styleAxes(ax,t(end));
ylim(ax,[-0.55,0.55]);
ylabel(ax,'$\delta\;(\mathrm{rad})$','Interpreter','latex');
title(ax,'\textbf{(a)}\quad Steering command and actuator constraint', ...
    'Interpreter','latex');
legend(ax,{'$\delta$','$\mathrm{sat}(\delta)$','$\delta_{\max}$', ...
    '$\delta_{\min}$'},'Interpreter','latex','Location','northoutside', ...
    'Orientation','horizontal','NumColumns',4,'Box','off');
markDisturbanceWindows(ax);

ax = nexttile;
hold(ax,'on');
plot(t,W,'-','Color',purple,'LineWidth',lw_response);
plot(t,w,'--','Color',red,'LineWidth',lw_boundary);
hold(ax,'off');
styleAxes(ax,t(end));
xlabel(ax,'Time (s)','Interpreter','latex');
ylabel(ax,'Magnitude','Interpreter','latex');
title(ax,'\textbf{(b)}\quad Adaptive weights and auxiliary state', ...
    'Interpreter','latex');
legend(ax,{'$\Vert W\Vert_2$','$w$'},'Interpreter','latex', ...
    'Location','best','Box','off');
padYLim(ax,[W;w]);
markDisturbanceWindows(ax);

%% Figure 3: transformed and sliding variables
figure(3);
set(gcf,'Color','w','Units','centimeters','Position',[2,2,18.5,15.5]);
tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
plotPair(nexttile,t,s1y,z1y,'$s_{1y}$','$z_{1y}$', ...
    '\textbf{(a)}\quad First-layer lateral variables',blue,red, ...
    lw_response,lw_boundary);
plotPair(nexttile,t,s1phi,z1phi,'$s_{1\varphi}$','$z_{1\varphi}$', ...
    '\textbf{(b)}\quad First-layer heading variables',blue,red, ...
    lw_response,lw_boundary);
plotPair(nexttile,t,s2y,z2y,'$s_{2y}$','$z_{2y}$', ...
    '\textbf{(c)}\quad Second-layer lateral variables',blue,red, ...
    lw_response,lw_boundary);
plotPair(nexttile,t,s2phi,z2phi,'$s_{2\varphi}$','$z_{2\varphi}$', ...
    '\textbf{(d)}\quad Second-layer heading variables',blue,red, ...
    lw_response,lw_boundary);
xlabel(tl,'Time (s)','Interpreter','latex','FontSize',13);

%% Figure 4: vehicle states and road curvature
figure(4);
set(gcf,'Color','w','Units','centimeters','Position',[2,2,18.5,15.5]);
tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

ax = nexttile;
plot(t,v_y,'-','Color',blue,'LineWidth',lw_response);
styleAxes(ax,t(end));
ylabel(ax,'$v_y\;(\mathrm{m\,s^{-1}})$','Interpreter','latex');
title(ax,'\textbf{(a)}\quad Lateral velocity','Interpreter','latex');
padYLim(ax,v_y);

ax = nexttile;
plot(t,omega_z,'-','Color',blue,'LineWidth',lw_response);
styleAxes(ax,t(end));
ylabel(ax,'$\omega_z\;(\mathrm{rad\,s^{-1}})$','Interpreter','latex');
title(ax,'\textbf{(b)}\quad Yaw rate','Interpreter','latex');
padYLim(ax,omega_z);

ax = nexttile([1,2]);
if isprop(out,'rho_0')
    rho_0 = out.rho_0(:);
    if numel(rho_0) == numel(t)
        t_rho = t;
    else
        t_rho = linspace(t(1),t(end),numel(rho_0)).';
    end
else
    rho_0 = [0;0.01;-0.01;0.01;zeros(6,1)];
    t_rho = linspace(t(1),t(end),numel(rho_0)).';
end
stairs(t_rho,rho_0,'-','Color',gray,'LineWidth',lw_response);
styleAxes(ax,t(end));
xlabel(ax,'Time (s)','Interpreter','latex');
ylabel(ax,'$\rho_0\;(\mathrm{m^{-1}})$','Interpreter','latex');
title(ax,'\textbf{(c)}\quad Road curvature command','Interpreter','latex');
padYLim(ax,rho_0);

function styleAxes(ax,t_end)
set(ax,'FontName','Times New Roman','FontSize',11,'LineWidth',1.0, ...
    'TickLabelInterpreter','latex','Box','on','Layer','top');
ax.Toolbar.Visible = 'off';
grid(ax,'on');
ax.GridLineStyle = ':';
ax.GridAlpha = 0.18;
ax.MinorGridAlpha = 0.10;
xlim(ax,[0,t_end]);
end

function padYLim(ax,data)
data = data(isfinite(data));
if isempty(data)
    return;
end
lo = min(data);
hi = max(data);
span = hi-lo;
if span <= eps(max(abs([lo,hi,1])))
    span = max(abs([lo,hi,1]))*0.1;
end
ylim(ax,[lo-0.08*span,hi+0.08*span]);
end

function plotPair(ax,t,s,z,s_label,z_label,panel_title,blue,red,lw_s,lw_z)
hold(ax,'on');
plot(ax,t,s,'-','Color',blue,'LineWidth',lw_s);
plot(ax,t,z,'--','Color',red,'LineWidth',lw_z);
hold(ax,'off');
styleAxes(ax,t(end));
ylabel(ax,'Amplitude','Interpreter','latex');
title(ax,panel_title,'Interpreter','latex');
legend(ax,{s_label,z_label},'Interpreter','latex','Location','best','Box','off');
padYLim(ax,[s;z]);
end

function markDisturbanceWindows(ax)
windows = [7,8;15,16];
yl = ylim(ax);
hold(ax,'on');
for k = 1:size(windows,1)
    xline(ax,windows(k,1),':','Color',[0.45,0.45,0.45], ...
        'LineWidth',0.9,'HandleVisibility','off');
    xline(ax,windows(k,2),':','Color',[0.45,0.45,0.45], ...
        'LineWidth',0.9,'HandleVisibility','off');
end
ylim(ax,yl);
hold(ax,'off');
end
