%% compare_trajectories_pf.m
% Trabajo Integrador — Comparación de trayectorias ejecutadas en Gazebo:
%   O manual  (CU2 — pick_place_params.yaml:  point_O = [0.75, 0.00, 0.40])
%   O★ óptimo (Pilar 2 — selected_solution_final.yaml: point_O = [0.50, -0.004, 0.25])
%
%   Fig 1 — Trayectoria 3D del TCP                   (obstáculo AABB)
%   Fig 2 — Posición cartesiana  x(t), y(t), z(t)   [subplot 3×1]
%   Fig 3 — Cinemática cartesiana  ‖v‖, ‖a‖, ‖j‖    [subplot 3×1]
%   Fig 4 — Velocidades articulares  q̇₀..q̇₅          [subplot 3×2]
%   Fig 5 — Torques articulares  τ₀..τ₅              [subplot 3×2]  ← objetivo f₁
%
% Uso:
%   1. Asignar FILE_MANUAL y FILE_OSTAR (nombres de CSV en ur5_pick_place/data/).
%   2. Ejecutar desde MATLAB (no requiere ROS/Gazebo activo).
%
% Salida (si EXPORT_PNG = true):
%   ur5_trajectory_optimization/results/final/plots/traj_comparison/

clear; clc; close all;

% ═══════════════════════════════════════════════════════════════════════
%  CONFIGURACIÓN — completar antes de ejecutar
% ═══════════════════════════════════════════════════════════════════════

% Nombres de los CSV (en ur5_pick_place/data/) — OBLIGATORIO especificar
FILE_MANUAL = 'trajectory_20260629_145743_clamped_spline.csv';   % CSV de la corrida con O manual  [0.75, 0.00, 0.40]
FILE_OSTAR  = 'trajectory_20260628_182338_clamped_spline.csv';   % CSV de la corrida con O★        [0.50, -0.004, 0.25]

% Obstáculo AABB — coincide con pick_place_params.yaml:
%   obstacle_center:       [0.85, 0.00, 0.10]
%   obstacle_half_extents: [0.20, 0.30, 0.10]
OBS_CX = 0.85;  OBS_CY = 0.00;  OBS_CZ = 0.10;
OBS_HX = 0.20;  OBS_HY = 0.30;  OBS_HZ = 0.10;

% Exportar figuras (PNG 300 dpi + EPS vector)
EXPORT_PNG = true;

% ═══════════════════════════════════════════════════════════════════════
%  DIRECTORIOS
% ═══════════════════════════════════════════════════════════════════════
repo_root = fullfile(getenv('HOME'), 'ur5_ws', 'src', 'ur5_utec');
data_dir  = fullfile(repo_root, 'ur5_pick_place', 'data');
out_dir   = fullfile(repo_root, 'ur5_trajectory_optimization', ...
                     'results', 'final', 'plots', 'traj_comparison');

KP_NAMES = {'A', 'B', 'O', 'C', 'D'};

% ═══════════════════════════════════════════════════════════════════════
%  1. CARGAR CSVs
% ═══════════════════════════════════════════════════════════════════════
if isempty(FILE_MANUAL) || isempty(FILE_OSTAR)
    error(['Especifique FILE_MANUAL y FILE_OSTAR en la sección CONFIGURACIÓN.\n' ...
           'Ejecute pick_place_node con cada point_O y copie el nombre del CSV.']);
end

TM = readtable(fullfile(data_dir, FILE_MANUAL));
TS = readtable(fullfile(data_dir, FILE_OSTAR));
fprintf('O manual : %s  (%d filas)\n', FILE_MANUAL, height(TM));
fprintf('O*       : %s  (%d filas)\n', FILE_OSTAR,  height(TS));

% ── Extraer columnas ─────────────────────────────────────────────────
[tM,xM,yM,zM,wM,vxM,vyM,vzM,axM,ayM,azM,dqM,tauM,jxM,jyM,jzM] = extract_cols(TM);
[tS,xS,yS,zS,wS,vxS,vyS,vzS,axS,ayS,azS,dqS,tauS,jxS,jyS,jzS] = extract_cols(TS);

spdM  = sqrt(vxM.^2 + vyM.^2 + vzM.^2);
accnM = sqrt(axM.^2 + ayM.^2 + azM.^2);
jrkM  = sqrt(jxM.^2 + jyM.^2 + jzM.^2);
spdS  = sqrt(vxS.^2 + vyS.^2 + vzS.^2);
accnS = sqrt(axS.^2 + ayS.^2 + azS.^2);
jrkS  = sqrt(jxS.^2 + jyS.^2 + jzS.^2);

% Índices de keypoints (waypoint 1=A, 2=B, 3=O, 4=C, 5=D; 0=interpolado)
kpM = zeros(1,5);  kpS = zeros(1,5);
for k = 1:5
    idx = find(wM == k, 1, 'first');  if ~isempty(idx), kpM(k) = idx; end
    idx = find(wS == k, 1, 'first');  if ~isempty(idx), kpS(k) = idx; end
end

% Coordenadas reales del via-point O en cada CSV
if kpM(3) > 0
    via_M = [xM(kpM(3)) yM(kpM(3)) zM(kpM(3))];
else
    via_M = [NaN NaN NaN];
    fprintf('AVISO: trayectoria O manual incompleta (waypoint O no encontrado).\n');
end
if kpS(3) > 0
    via_S = [xS(kpS(3)) yS(kpS(3)) zS(kpS(3))];
else
    via_S = [NaN NaN NaN];
    fprintf('AVISO: trayectoria O* incompleta (waypoint O no encontrado).\n');
end

% ═══════════════════════════════════════════════════════════════════════
%  2. MÉTRICAS RESUMIDAS
% ═══════════════════════════════════════════════════════════════════════
arc_M    = sum(sqrt(diff(xM).^2 + diff(yM).^2 + diff(zM).^2));
arc_S    = sum(sqrt(diff(xS).^2 + diff(yS).^2 + diff(zS).^2));
effort_M = trapz(tM, sum(tauM.^2, 2));
effort_S = trapz(tS, sum(tauS.^2, 2));
cl_M     = tcp_clearance(xM,yM,zM, OBS_CX,OBS_CY,OBS_CZ,OBS_HX,OBS_HY,OBS_HZ);
cl_S     = tcp_clearance(xS,yS,zS, OBS_CX,OBS_CY,OBS_CZ,OBS_HX,OBS_HY,OBS_HZ);
maxdqM   = max(abs(dqM(:)));
maxdqS   = max(abs(dqS(:)));
rmsjM    = sqrt(mean(jrkM.^2));
rmsjS    = sqrt(mean(jrkS.^2));

pct_eff = (effort_M - effort_S) / effort_M * 100;   % + = O* mejor
pct_arc = (arc_M   - arc_S)    / arc_M    * 100;   % + = O* más corto
pct_cl  = (cl_S    - cl_M)     / cl_M     * 100;   % + = O* más holgado

fprintf('\n%-32s  %-22s  %-22s  %s\n', 'Métrica', 'O manual (CU2)', 'O* (Pilar 2)', '% mejora O*');
fprintf('%s\n', repmat('-', 1, 88));
fprintf('%-32s  %-22.4f  %-22.4f  %+.1f%%\n', 'Longitud de arco  [m]',    arc_M,    arc_S,    pct_arc);
fprintf('%-32s  %-22.1f  %-22.1f  %+.1f%%\n', 'Esfuerzo f1  [N^2*m^2*s]', effort_M, effort_S, pct_eff);
fprintf('%-32s  %-22.4f  %-22.4f  %+.1f%%\n', 'Clearance min TCP  [m]',   cl_M,     cl_S,     pct_cl);
fprintf('%-32s  %-22.4f  %-22.4f\n',           'Max |dq|  [rad/s]',        maxdqM,   maxdqS);
fprintf('%-32s  %-22.4f  %-22.4f\n',           'RMS jerk  [m/s^3]',        rmsjM,    rmsjS);
fprintf('\npoint_O manual    : [%.4f, %.4f, %.4f]  m\n', via_M(1),via_M(2),via_M(3));
fprintf('point_O* (Pilar 2): [%.4f, %.4f, %.4f]  m\n', via_S(1),via_S(2),via_S(3));

% ═══════════════════════════════════════════════════════════════════════
%  PALETA Y ESTILOS
% ═══════════════════════════════════════════════════════════════════════
lw   = 2.0;   fs  = 11;
c_M  = [0.8500 0.3250 0.0980];   % naranja — O manual (CU2)
c_S  = [0.0000 0.4470 0.7410];   % azul    — O* (Pilar 2)
c_ob = [0.6350 0.0780 0.1840];   % rojo oscuro — obstáculo

kp_col = {[0.4660 0.8740 0.1880],  % A  verde claro
          [0.1660 0.6740 0.1880],  % B  verde
          [0.4940 0.1840 0.5560],  % O  morado
          [0.8500 0.3250 0.0980],  % C  naranja
          [1.0000 0.6500 0.3000]}; % D  naranja claro
kp_sty = {':',  '--', '--', '--', ':'};
kp_mk  = {'v',  'o',  's',  '^',  'd'};

lbl_M = 'O manual (CU2)';
lbl_S = 'O^{*} (Pilar 2 — mono-objetivo)';

obs_V  = box_vertices(OBS_CX, OBS_CY, OBS_CZ, OBS_HX, OBS_HY, OBS_HZ);
obs_F  = [1 2 3 4; 5 6 7 8; 1 2 6 5; 4 3 7 8; 1 4 8 5; 2 3 7 6];
xlim_t = [min(tM(1), tS(1)),  max(tM(end), tS(end))];

% ═══════════════════════════════════════════════════════════════════════
%  Fig 1 — Trayectoria 3D del TCP
% ═══════════════════════════════════════════════════════════════════════
figure(1); clf;
set(gcf,'Color','w','Position',[80 220 900 680]);

h_ob = patch('Vertices',obs_V,'Faces',obs_F,'FaceColor',c_ob, ...
    'FaceAlpha',0.18,'EdgeColor',c_ob,'LineWidth',1.2);
hold on;
h_M1 = plot3(xM,yM,zM,'--','Color',c_M,'LineWidth',lw);
h_S1 = plot3(xS,yS,zS,'-', 'Color',c_S,'LineWidth',lw);

% Marcadores de keypoints (serie O*, siempre completa)
kp_off = [0.02  0.04  0.02;  0.02  0.04  0.02;  0.03  0.02  0.03;
          0.02 -0.09  0.02;  0.02 -0.09  0.02];
for k = 1:5
    if kpS(k) > 0
        plot3(xS(kpS(k)), yS(kpS(k)), zS(kpS(k)), kp_mk{k}, ...
            'MarkerSize',10,'MarkerFaceColor',kp_col{k}, ...
            'MarkerEdgeColor','k','LineWidth',1.1,'HandleVisibility','off');
        text(xS(kpS(k))+kp_off(k,1), yS(kpS(k))+kp_off(k,2), ...
             zS(kpS(k))+kp_off(k,3), KP_NAMES{k}, ...
            'FontSize',fs,'FontWeight','bold','Color',kp_col{k});
    end
end

% Resaltar punto O de cada método con anotación de coordenadas
if ~any(isnan(via_M))
    plot3(via_M(1),via_M(2),via_M(3),'s','MarkerSize',14, ...
        'MarkerFaceColor',c_M,'MarkerEdgeColor','k','LineWidth',1.5, ...
        'HandleVisibility','off');
    text(via_M(1)+0.03, via_M(2), via_M(3)+0.025, ...
        sprintf('$O_M$  [%.2f, %.2f, %.2f]', via_M(1),via_M(2),via_M(3)), ...
        'Interpreter','latex','Color',c_M,'FontSize',fs-1,'FontWeight','bold');
end
if ~any(isnan(via_S))
    plot3(via_S(1),via_S(2),via_S(3),'^','MarkerSize',14, ...
        'MarkerFaceColor',c_S,'MarkerEdgeColor','k','LineWidth',1.5, ...
        'HandleVisibility','off');
    text(via_S(1)+0.03, via_S(2), via_S(3)-0.045, ...
        sprintf('$O^{\\star}$  [%.2f, %.2f, %.2f]', via_S(1),via_S(2),via_S(3)), ...
        'Interpreter','latex','Color',c_S,'FontSize',fs-1,'FontWeight','bold');
end

text(OBS_CX+OBS_HX+0.02, OBS_CY, OBS_CZ+OBS_HZ+0.02, 'Obstáculo', ...
    'FontSize',fs-1,'FontWeight','bold','Color',c_ob,'Clipping','on');

xlabel('x [m]','FontSize',fs);  ylabel('y [m]','FontSize',fs);
zlabel('z [m]','FontSize',fs);
grid on; box on; set(gca,'FontSize',fs); view(42,22);
legend([h_M1, h_S1, h_ob], {lbl_M, lbl_S, 'Obstáculo AABB'}, ...
    'Location','best','FontSize',fs-1);
title('Trayectoria 3D del TCP — O manual (CU2) vs O^{*} (Pilar 2)', ...
    'FontSize',14,'FontWeight','bold');

% ═══════════════════════════════════════════════════════════════════════
%  Fig 2 — Posición cartesiana  x(t), y(t), z(t)
% ═══════════════════════════════════════════════════════════════════════
figure(2); clf;
set(gcf,'Color','w','Position',[1000 220 760 640]);
tl2 = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

dM2 = {xM,yM,zM};  dS2 = {xS,yS,zS};
yl2 = {'$x$ [m]','$y$ [m]','$z$ [m]'};
obs_zbands = [OBS_CZ-OBS_HZ,  OBS_CZ+OBS_HZ];   % rango z del obstáculo

hM2ref = [];  hS2ref = [];
for i = 1:3
    ax2 = nexttile(tl2); hold on;
    if i == 3   % banda de altura del obstáculo sobre z(t)
        patch([xlim_t(1) xlim_t(2) xlim_t(2) xlim_t(1)], ...
              [obs_zbands(1) obs_zbands(1) obs_zbands(2) obs_zbands(2)], ...
              c_ob,'FaceAlpha',0.10,'EdgeColor','none','HandleVisibility','off');
        yline(obs_zbands(1),':','Color',c_ob,'LineWidth',0.9,'HandleVisibility','off');
        yline(obs_zbands(2),':','Color',c_ob,'LineWidth',0.9,'HandleVisibility','off');
    end
    hM2 = plot(tM, dM2{i}, '--', 'Color',c_M, 'LineWidth',lw);
    hS2 = plot(tS, dS2{i}, '-',  'Color',c_S, 'LineWidth',lw);
    if i == 1, hM2ref = hM2; hS2ref = hS2; end
    for k = 1:5
        if kpM(k) > 0
            plot(tM(kpM(k)), dM2{i}(kpM(k)), kp_mk{k}, 'MarkerSize',7, ...
                'MarkerFaceColor',kp_col{k},'MarkerEdgeColor','k','HandleVisibility','off');
        end
        if kpS(k) > 0
            plot(tS(kpS(k)), dS2{i}(kpS(k)), kp_mk{k}, 'MarkerSize',7, ...
                'MarkerFaceColor',kp_col{k},'MarkerEdgeColor','k','HandleVisibility','off');
        end
    end
    add_kp_lines(ax2, tS, kpS, kp_col, kp_sty);
    ylabel(yl2{i},'Interpreter','latex','FontSize',fs);
    grid on; box on; set(ax2,'FontSize',fs); xlim(xlim_t);
    if i < 3, set(ax2,'XTickLabel',[]); end
end
xlabel('Tiempo [s]','FontSize',fs);
lgd2 = legend(nexttile(tl2,1), [hM2ref, hS2ref], {lbl_M, lbl_S}, ...
    'Orientation','horizontal','FontSize',fs-1);
lgd2.Layout.Tile = 'north';
title(tl2,'Posición cartesiana TCP — O manual vs O^{*}', ...
    'FontSize',14,'FontWeight','bold');

% ═══════════════════════════════════════════════════════════════════════
%  Fig 3 — Cinemática cartesiana  ‖v‖, ‖a‖, ‖j‖
% ═══════════════════════════════════════════════════════════════════════
figure(3); clf;
set(gcf,'Color','w','Position',[80 100 760 600]);
tl3 = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

dM3 = {spdM, accnM, jrkM};  dS3 = {spdS, accnS, jrkS};
yl3  = {'$\|\mathbf{v}\|$ [m/s]', '$\|\mathbf{a}\|$ [m/s$^2$]', '$\|\mathbf{j}\|$ [m/s$^3$]'};

hM3ref = [];  hS3ref = [];
for i = 1:3
    ax3 = nexttile(tl3); hold on;
    hM3 = plot(tM, dM3{i}, '--', 'Color',c_M, 'LineWidth',lw);
    hS3 = plot(tS, dS3{i}, '-',  'Color',c_S, 'LineWidth',lw);
    if i == 1, hM3ref = hM3; hS3ref = hS3; end
    for k = 1:5
        if kpM(k) > 0
            plot(tM(kpM(k)), dM3{i}(kpM(k)), kp_mk{k}, 'MarkerSize',7, ...
                'MarkerFaceColor',kp_col{k},'MarkerEdgeColor','k','HandleVisibility','off');
        end
        if kpS(k) > 0
            plot(tS(kpS(k)), dS3{i}(kpS(k)), kp_mk{k}, 'MarkerSize',7, ...
                'MarkerFaceColor',kp_col{k},'MarkerEdgeColor','k','HandleVisibility','off');
        end
    end
    add_kp_lines(ax3, tS, kpS, kp_col, kp_sty);
    ylabel(yl3{i},'Interpreter','latex','FontSize',fs);
    grid on; box on; set(ax3,'FontSize',fs); xlim(xlim_t);
    if i < 3, set(ax3,'XTickLabel',[]); end
end
xlabel('Tiempo [s]','FontSize',fs);
lgd3 = legend(nexttile(tl3,1), [hM3ref, hS3ref], {lbl_M, lbl_S}, ...
    'Orientation','horizontal','FontSize',fs-1);
lgd3.Layout.Tile = 'north';
title(tl3,'Cinemática cartesiana TCP — O manual vs O^{*}', ...
    'FontSize',14,'FontWeight','bold');

% ═══════════════════════════════════════════════════════════════════════
%  Fig 4 — Velocidades articulares  q̇₀..q̇₅
% ═══════════════════════════════════════════════════════════════════════
figure(4); clf;
set(gcf,'Color','w','Position',[80 100 1100 700]);
tl4 = tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

ax4_first = [];  hM4leg = [];  hS4leg = [];
for j = 1:6
    ax4 = nexttile(tl4); hold on;
    if isempty(ax4_first), ax4_first = ax4; end
    hM4 = plot(tM, dqM(:,j), '--', 'Color',c_M, 'LineWidth',lw);
    hS4 = plot(tS, dqS(:,j), '-',  'Color',c_S, 'LineWidth',lw);
    if j == 1, hM4leg = hM4; hS4leg = hS4; end
    add_kp_lines(ax4, tS, kpS, kp_col, kp_sty);
    ylabel(sprintf('$\\dot{q}_%d$ [rad/s]', j-1), 'Interpreter','latex','FontSize',fs-1);
    title(sprintf('Joint %d', j-1),'FontSize',fs-1);
    grid on; box on; set(ax4,'FontSize',fs-1); xlim(xlim_t);
end
xlabel('Tiempo [s]','FontSize',fs);
sgtitle('Velocidades articulares — O manual (CU2) vs O^{*} (Pilar 2)', ...
    'FontSize',13,'FontWeight','bold');
legend(ax4_first, [hM4leg, hS4leg], {lbl_M, lbl_S}, ...
    'Orientation','horizontal','FontSize',fs-1,'Location','best');

% ═══════════════════════════════════════════════════════════════════════
%  Fig 5 — Torques articulares  τ₀..τ₅  (objetivo f₁ = ∫Στ² dt)
% ═══════════════════════════════════════════════════════════════════════
figure(5); clf;
set(gcf,'Color','w','Position',[80 100 1100 700]);
tl5 = tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

ax5_first = [];  hM5leg = [];  hS5leg = [];
for j = 1:6
    ax5 = nexttile(tl5); hold on;
    if isempty(ax5_first), ax5_first = ax5; end
    hM5 = plot(tM, tauM(:,j), '--', 'Color',c_M, 'LineWidth',lw);
    hS5 = plot(tS, tauS(:,j), '-',  'Color',c_S, 'LineWidth',lw);
    if j == 1, hM5leg = hM5; hS5leg = hS5; end
    add_kp_lines(ax5, tS, kpS, kp_col, kp_sty);
    ylabel(sprintf('$\\tau_%d$ [N$\\cdot$m]', j-1), 'Interpreter','latex','FontSize',fs-1);
    title(sprintf('Joint %d', j-1),'FontSize',fs-1);
    grid on; box on; set(ax5,'FontSize',fs-1); xlim(xlim_t);
end
xlabel('Tiempo [s]','FontSize',fs);
sgtitle(sprintf(['Torques articulares  ($f_1 = \\int\\Sigma\\tau^2\\,dt$)' ...
                 ' — Manual: %.0f  |  O^{*}: %.0f  [N$^2\\cdot$m$^2\\cdot$s]'], ...
    effort_M, effort_S), ...
    'FontSize',13,'FontWeight','bold','Interpreter','latex');
legend(ax5_first, [hM5leg, hS5leg], {lbl_M, lbl_S}, ...
    'Orientation','horizontal','FontSize',fs-1,'Location','best');

% ═══════════════════════════════════════════════════════════════════════
%  EXPORTAR FIGURAS
% ═══════════════════════════════════════════════════════════════════════
if EXPORT_PNG
    if ~exist(out_dir,'dir'), mkdir(out_dir); end
    fig_names = {'trayectoria_3D', 'posicion_cartesiana', ...
                 'cinematica_cartesiana', 'velocidades_articulares', ...
                 'torques_articulares'};
    for f = 1:5
        if ~ishandle(f), continue; end
        exportgraphics(figure(f), fullfile(out_dir, [fig_names{f} '.png']), ...
            'Resolution', 300);
        exportgraphics(figure(f), fullfile(out_dir, [fig_names{f} '.eps']), ...
            'ContentType','vector','Resolution',600);
    end
    fprintf('\nFiguras exportadas en:\n  %s\n', out_dir);
end

% ═══════════════════════════════════════════════════════════════════════
%  FUNCIONES LOCALES
% ═══════════════════════════════════════════════════════════════════════

function [t,x,y,z,w,vx,vy,vz,ax,ay,az,dq,tau,jx,jy,jz] = extract_cols(T)
% Extrae columnas del CSV generado por pick_place_node.cpp.
    t   = T.time_s;
    x   = T.tcp_x;    y  = T.tcp_y;    z  = T.tcp_z;
    w   = T.waypoint;
    vx  = T.vel_x;    vy = T.vel_y;    vz = T.vel_z;
    ax  = T.acc_x;    ay = T.acc_y;    az = T.acc_z;
    dq  = [T.dq0 T.dq1 T.dq2 T.dq3 T.dq4 T.dq5];
    tau = [T.tau0 T.tau1 T.tau2 T.tau3 T.tau4 T.tau5];
    if ismember('jerk_x', T.Properties.VariableNames)
        jx = T.jerk_x;  jy = T.jerk_y;  jz = T.jerk_z;
    else
        dt = gradient(t);
        dt(dt == 0) = eps;
        jx = gradient(ax) ./ dt;
        jy = gradient(ay) ./ dt;
        jz = gradient(az) ./ dt;
    end
end

function d = tcp_clearance(x,y,z,cx,cy,cz,hx,hy,hz)
% Distancia mínima de la ruta TCP al obstáculo AABB (= "clearance" mínimo).
    pts     = [x y z];
    lo      = [cx-hx, cy-hy, cz-hz];
    hi      = [cx+hx, cy+hy, cz+hz];
    nearest = max(min(pts, hi), lo);
    d       = min(sqrt(sum((pts - nearest).^2, 2)));
end

function V = box_vertices(cx,cy,cz,hx,hy,hz)
% Genera los 8 vértices de la caja AABB para patch 3D.
    sx = [-1  1  1 -1 -1  1  1 -1]' .* hx + cx;
    sy = [-1 -1  1  1 -1 -1  1  1]' .* hy + cy;
    sz = [-1 -1 -1 -1  1  1  1  1]' .* hz + cz;
    V  = [sx sy sz];
end

function add_kp_lines(ax, t, kp_idx, kp_col, kp_sty)
% Líneas verticales en los instantes de cada keypoint.
    for k = 1:numel(kp_idx)
        if kp_idx(k) > 0 && kp_idx(k) <= numel(t)
            xline(ax, t(kp_idx(k)), kp_sty{k}, ...
                'Color',kp_col{k},'LineWidth',0.9,'HandleVisibility','off');
        end
    end
end
