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
delta_applied = out.delta1(:);
ctrl_diagnostics = out.ctrl_diagnostics;
WF_norm = ctrl_diagnostics(:,5);
Wc_norm = ctrl_diagnostics(:,6);
Wa_move = ctrl_diagnostics(:,7);
delta_admissible = ctrl_diagnostics(:,9);
delta_RL = ctrl_diagnostics(:,10);
delta_safety_correction = ctrl_diagnostics(:,11);

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

%% Fig. 3: decomposition of steering authority
fig = figure(3);
set(fig,'Color','w','Units','centimeters','Position',[2,2,17.8,7.0]);
hold on;
h_admissible = plot(t,delta_admissible,'b-','LineWidth',2.0);
h_RL = plot(t,delta_RL,'r--','LineWidth',1.8);
h_safety = plot(t,delta_safety_correction,'k-.','LineWidth',1.5);
h_applied = plot(t,delta_applied,'Color',[0.45,0.45,0.45], ...
    'LineStyle',':','LineWidth',2.0);
yline(0.5,'Color',[0.45,0.45,0.45],'LineStyle','--', ...
    'LineWidth',1.0,'HandleVisibility','off');
yline(-0.5,'Color',[0.45,0.45,0.45],'LineStyle','--', ...
    'LineWidth',1.0,'HandleVisibility','off');
hold off;
formatAxes(gca,t(end),[-0.52,0.52]);
xlabel('Time (s)','Interpreter','latex','FontSize',14);
ylabel('$\delta\;(\mathrm{rad})$','Interpreter','latex','FontSize',14);
legend([h_admissible,h_RL,h_safety,h_applied], ...
    {'$\delta_{\mathrm{adm}}$','$\delta_{\mathrm{RL}}$', ...
    '$\Delta\delta_{\mathrm{safety}}$','$\delta_{\mathrm{applied}}$'}, ...
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

%% Fig. 6: direct online-versus-frozen differences
[delta_e_y,delta_e_phi,delta_J] = frozenPolicyDifference(out,t);
fig = figure(6);
set(fig,'Color','w','Units','centimeters','Position',[2,2,17.8,12.0]);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

ax = nexttile;
plot(ax,t,delta_e_y,'b-','LineWidth',2.0);
yline(ax,0,'r--','LineWidth',1.0);
formatAxes(ax,t(end),symmetricLimits(delta_e_y));
ylabel(ax,'$\Delta e_y\;(\mathrm{m})$', ...
    'Interpreter','latex','FontSize',14);

ax = nexttile;
plot(ax,t,delta_e_phi,'b-','LineWidth',2.0);
yline(ax,0,'r--','LineWidth',1.0);
formatAxes(ax,t(end),symmetricLimits(delta_e_phi));
ylabel(ax,'$\Delta e_{\varphi}\;(\mathrm{rad})$', ...
    'Interpreter','latex','FontSize',14);

ax = nexttile;
plot(ax,t,delta_J,'b-','LineWidth',2.0);
yline(ax,0,'r--','LineWidth',1.0);
formatAxes(ax,t(end),dataLimitsWithZero(delta_J));
xlabel(ax,'Time (s)','Interpreter','latex','FontSize',14);
ylabel(ax,'$\Delta J(0,t)$','Interpreter','latex','FontSize',14);
exportFigure(fig,output_dir,'Fig6_online_frozen_difference');

%% Fig. 7: flexible-boundary response under 0.4 rad saturation stress
out_stress = simulateControllerCase('100,1,1,0.25,0.4',0.4,t(end));
t_stress = out_stress.t(:);
fig = figure(7);
set(fig,'Color','w','Units','centimeters','Position',[2,2,17.8,13.0]);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

ax = nexttile;
hold(ax,'on');
h_error = plot(ax,t_stress,out_stress.e_y(:),'b-','LineWidth',2.0);
h_flexible = plot(ax,t_stress,out_stress.eyu(:),'r--','LineWidth',1.8);
plot(ax,t_stress,out_stress.eyl(:),'r--','LineWidth',1.8, ...
    'HandleVisibility','off');
h_nominal = plot(ax,t_stress,out_stress.eyl_(:),'k:', ...
    'LineWidth',1.5);
plot(ax,t_stress,out_stress.eyu_(:),'k:','LineWidth',1.5, ...
    'HandleVisibility','off');
hold(ax,'off');
formatAxes(ax,t(end),[-0.22,0.06]);
ylabel(ax,'$e_y\;(\mathrm{m})$','Interpreter','latex','FontSize',14);
legend(ax,[h_error,h_flexible,h_nominal], ...
    {'Error','Flexible boundary','Nominal boundary'}, ...
    'Interpreter','latex','FontSize',10,'Location','northoutside', ...
    'Orientation','horizontal','NumColumns',3,'Box','off');

ax = nexttile;
hold(ax,'on');
plot(ax,t_stress,out_stress.e_phi(:),'b-','LineWidth',2.0);
plot(ax,t_stress,out_stress.ephiu(:),'r--','LineWidth',1.8);
plot(ax,t_stress,out_stress.ephil(:),'r--','LineWidth',1.8);
plot(ax,t_stress,out_stress.ephil_(:),'k:','LineWidth',1.5);
plot(ax,t_stress,out_stress.ephiu_(:),'k:','LineWidth',1.5);
hold(ax,'off');
formatAxes(ax,t(end),[-0.025,0.04]);
ylabel(ax,'$e_{\varphi}\;(\mathrm{rad})$', ...
    'Interpreter','latex','FontSize',14);

ax = nexttile;
hold(ax,'on');
plot(ax,t_stress,out_stress.delta(:),'b-','LineWidth',2.0);
plot(ax,t_stress,out_stress.delta1(:),'r--','LineWidth',1.8);
yline(ax,0.4,'k:','LineWidth',1.3);
yline(ax,-0.4,'k:','LineWidth',1.3,'HandleVisibility','off');
hold(ax,'off');
formatAxes(ax,t(end),[-0.52,0.52]);
xlabel(ax,'Time (s)','Interpreter','latex','FontSize',14);
ylabel(ax,'$\delta\;(\mathrm{rad})$', ...
    'Interpreter','latex','FontSize',14);
legend(ax,{'Requested','Applied','$\pm0.4$ rad'}, ...
    'Interpreter','latex','FontSize',10,'Location','northoutside', ...
    'Orientation','horizontal','NumColumns',3,'Box','off');
exportFigure(fig,output_dir,'Fig7_saturation_stress');

%% Fig. 8: actuator-feasibility envelope with all controller gains frozen
[actuator_limits,envelope] = actuatorEnvelopeSweep(t(end));
fig = figure(8);
set(fig,'Color','w','Units','centimeters','Position',[2,2,17.8,12.0]);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

ax = nexttile;
hold(ax,'on');
plot(ax,actuator_limits,envelope.flexible_margin,'b-o', ...
    'LineWidth',2.0,'MarkerSize',5,'MarkerFaceColor','w');
plot(ax,actuator_limits,envelope.nominal_margin,'r--s', ...
    'LineWidth',1.8,'MarkerSize',5,'MarkerFaceColor','w');
yline(ax,0,'k:','LineWidth',1.3);
plot(ax,actuator_limits(~envelope.success), ...
    zeros(sum(~envelope.success),1),'kx','LineWidth',1.8, ...
    'MarkerSize',8,'HandleVisibility','off');
hold(ax,'off');
formatEnvelopeAxes(ax,actuator_limits,[-2.1,0.4]);
ylabel(ax,'$m_{\min}$','Interpreter','latex','FontSize',14);
legend(ax,{'Flexible SFPPB','Nominal PPB','Safety boundary'}, ...
    'Interpreter','latex','FontSize',10,'Location','northoutside', ...
    'Orientation','horizontal','NumColumns',3,'Box','off');

ax = nexttile;
yyaxis(ax,'left');
plot(ax,actuator_limits,envelope.saturation_ratio,'b-o', ...
    'LineWidth',2.0,'MarkerSize',5,'MarkerFaceColor','w');
ylabel(ax,'Saturation (\%)','Interpreter','latex','FontSize',14);
ylim(ax,[0,7.2]);
yyaxis(ax,'right');
plot(ax,actuator_limits,envelope.max_eta,'r--s', ...
    'LineWidth',1.8,'MarkerSize',5,'MarkerFaceColor','w');
ylabel(ax,'$\max|\eta|$','Interpreter','latex','FontSize',14);
ylim(ax,[0,0.055]);
formatEnvelopeAxes(ax,actuator_limits,[]);
legend(ax,{'Saturation time','$\max|\eta|$'}, ...
    'Interpreter','latex','FontSize',10,'Location','northwest', ...
    'Orientation','horizontal','NumColumns',2,'Box','off');

ax = nexttile;
yyaxis(ax,'left');
plot(ax,actuator_limits,envelope.max_gap,'b-o', ...
    'LineWidth',2.0,'MarkerSize',5,'MarkerFaceColor','w');
ylabel(ax,'Gap (rad)', ...
    'Interpreter','latex','FontSize',14);
ylim(ax,[0,0.14]);
yyaxis(ax,'right');
plot(ax,actuator_limits,envelope.conflicts,'r--s', ...
    'LineWidth',1.8,'MarkerSize',5,'MarkerFaceColor','w');
ylabel(ax,'Conflicts','Interpreter','latex','FontSize',14);
ylim(ax,[0,9]);
formatEnvelopeAxes(ax,actuator_limits,[]);
xlabel(ax,'Actuator limit (rad)','Interpreter','latex','FontSize',14);
legend(ax,{'Saturation gap','Safety conflicts'}, ...
    'Interpreter','latex','FontSize',10,'Location','northwest', ...
    'Orientation','horizontal','NumColumns',2,'Box','off');
exportFigure(fig,output_dir,'Fig8_actuator_envelope');

%% Fig. A1: transformed-state diagnostics for the appendix
fig = figure(9);
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

function [delta_e_y,delta_e_phi,delta_J] = frozenPolicyDifference(out,t)
out_frozen = simulateControllerCase( ...
    '100,0,1,0.25,0.5',0.5,t(end));

t_frozen = out_frozen.t(:);
delta_e_y = out.e_y(:)-interp1( ...
    t_frozen,out_frozen.e_y(:),t,'linear');
delta_e_phi = out.e_phi(:)-interp1( ...
    t_frozen,out_frozen.e_phi(:),t,'linear');

L_online = (out.s1y(:)/2).^2+(out.s1phi(:)/1.5).^2 ...
    + (out.z2y(:)/0.5).^2+(out.z2phi(:)/0.2).^2 ...
    + out.delta1(:).^2;
L_frozen = (out_frozen.s1y(:)/2).^2 ...
    + (out_frozen.s1phi(:)/1.5).^2 ...
    + (out_frozen.z2y(:)/0.5).^2 ...
    + (out_frozen.z2phi(:)/0.2).^2 ...
    + out_frozen.delta1(:).^2;
L_frozen = interp1(t_frozen,L_frozen,t,'linear');
delta_J = cumtrapz(t,L_online-L_frozen);
end

function out_case = simulateControllerCase( ...
    controller_parameters,actuator_limit,stop_time)
model = 'AGV_simulate';
load_system(model);
controller_block = [model '/S-Function3'];
plant_block = [model '/S-Function'];
assist_block = [model '/S-Function4'];
original_parameters = {get_param(controller_block,'Parameters'), ...
    get_param(plant_block,'Parameters'), ...
    get_param(assist_block,'Parameters')};
restore_parameters = onCleanup(@() restoreModelParameters( ...
    controller_block,plant_block,assist_block,original_parameters));
set_param(controller_block,'Parameters',controller_parameters);
set_param(plant_block,'Parameters',num2str(actuator_limit,16));
set_param(assist_block,'Parameters',num2str(actuator_limit,16));
out_case = sim(model,'StopTime',num2str(stop_time,16), ...
    'ReturnWorkspaceOutputs','on');
clear restore_parameters;
end

function [limits,data] = actuatorEnvelopeSweep(stop_time)
limits = (0.30:0.025:0.50).';
n = numel(limits);
data.success = false(n,1);
data.flexible_margin = nan(n,1);
data.nominal_margin = nan(n,1);
data.saturation_ratio = nan(n,1);
data.conflicts = nan(n,1);
data.max_eta = nan(n,1);
data.max_gap = nan(n,1);

for k = 1:n
    actuator_limit = limits(k);
    controller_parameters = sprintf( ...
        '100,1,1,0.25,%.16g',actuator_limit);
    try
        out_case = simulateControllerCase( ...
            controller_parameters,actuator_limit,stop_time);
    catch
        continue;
    end

    t_case = out_case.t(:);
    diagnostics = out_case.ctrl_diagnostics;
    requested = diagnostics(:,1);
    applied = diagnostics(:,2);
    data.flexible_margin(k) = min([ ...
        min(out_case.e_y(:)-out_case.eyl(:))/0.03; ...
        min(out_case.eyu(:)-out_case.e_y(:))/0.03; ...
        min(out_case.e_phi(:)-out_case.ephil(:))/0.005; ...
        min(out_case.ephiu(:)-out_case.e_phi(:))/0.005]);
    data.nominal_margin(k) = min([ ...
        min(out_case.e_y(:)-out_case.eyu_(:))/0.03; ...
        min(out_case.eyl_(:)-out_case.e_y(:))/0.03; ...
        min(out_case.e_phi(:)-out_case.ephiu_(:))/0.005; ...
        min(out_case.ephil_(:)-out_case.e_phi(:))/0.005]);
    saturated = abs(requested-applied) > 1e-8;
    data.saturation_ratio(k) = 100*trapz( ...
        t_case,double(saturated))/t_case(end);
    conflict = diagnostics(:,8) > 0.5;
    data.conflicts(k) = sum(diff([false;conflict]) > 0);
    relaxation = [out_case.eyu(:)-out_case.eyl_(:), ...
        out_case.eyu_(:)-out_case.eyl(:), ...
        out_case.ephiu(:)-out_case.ephil_(:), ...
        out_case.ephiu_(:)-out_case.ephil(:)];
    max_relaxation = max(relaxation(:));
    data.max_eta(k) = atanh(min(max(max_relaxation/0.4,0),1-eps));
    data.max_gap(k) = max(abs(requested-applied));
    data.success(k) = t_case(end) >= stop_time-1e-6 ...
        && data.flexible_margin(k) > 0;
end
end

function restoreModelParameters( ...
    controller_block,plant_block,assist_block,parameters)
set_param(controller_block,'Parameters',parameters{1});
set_param(plant_block,'Parameters',parameters{2});
set_param(assist_block,'Parameters',parameters{3});
end

function limits = symmetricLimits(signal)
limit = 1.08*max(abs(signal));
if limit <= eps
    limit = 1;
end
limits = [-limit,limit];
end

function limits = dataLimitsWithZero(signal)
lower = min([signal;0]);
upper = max([signal;0]);
span = upper-lower;
if span <= eps
    span = 1;
end
limits = [lower-0.08*span,upper+0.08*span];
end

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

function formatEnvelopeAxes(ax,limits,y_limits)
set(ax,'FontName','Times New Roman','FontSize',12,'LineWidth',1.0, ...
    'TickLabelInterpreter','latex','Box','on','Layer','top');
ax.Toolbar.Visible = 'off';
grid(ax,'on');
ax.GridLineStyle = ':';
ax.GridAlpha = 0.15;
xlim(ax,[limits(1)-0.005,limits(end)+0.005]);
xticks(ax,limits(1:2:end));
for axis_index = 1:numel(ax.YAxis)
    ax.YAxis(axis_index).Color = 'k';
end
if ~isempty(y_limits)
    ylim(ax,y_limits);
end
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
