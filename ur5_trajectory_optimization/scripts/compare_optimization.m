%% compare_optimization.m
% Compara las trayectorias ejecutadas en Gazebo con el via-point O hallado
% por NSGA-II y por ε-constraint (paquete ur5_trajectory_optimization).
%
% Datos de entrada:
%   · 2 CSV en ur5_pick_place/data/ (uno por cada método, clamped_spline)
%   · pareto_nsga2.csv  y  pareto_epsilon.csv  en  results/
%
% Figuras generadas:
%   Fig 1 — Trayectoria 3D del TCP  (obstáculo AABB)
%   Fig 2 — Posición cartesiana  x(t), y(t), z(t)
%   Fig 3 — Cinemática cartesiana  ‖v‖, ‖a‖, ‖j‖
%   Fig 4 — Velocidades articulares q̇₀..q̇₅  (diagnóstico abort JTC)
%   Fig 5 — Torques articulares τ₀..τ₅  (objetivo f₁ = ∫Στ² dt)
%   Fig 6 — Frentes de Pareto NSGA-II + ε-constraint
%
% Configuración de archivos CSV:
%   FILE_NSGA2   / FILE_EPSILON : '' = auto (2 CSV más recientes de data_dir)
%   Convenio auto: más antiguo = NSGA-II, más reciente = ε-constraint.
%   Para especificar manualmente, copie el nombre exacto del archivo.
%
% Salida: ur5_trajectory_optimization/plots/comparison/  (PNG 300 dpi + EPS)

clear; clc; close all;

% Interpreter 'latex' se aplica solo en los elementos con notación math

% ═══════════════════════════════════════════════════════════════════════
%  CONFIGURACIÓN
% ═══════════════════════════════════════════════════════════════════════

% Archivos de trayectoria  ('' = auto-detección)
FILE_NSGA2   = 'trajectory_20260622_120154_clamped_spline.csv';    % ej. 'trajectory_20260622_110000_clamped_spline.csv'
FILE_EPSILON = 'trajectory_20260622_120057_clamped_spline.csv';    % ej. 'trajectory_20260622_120000_clamped_spline.csv'

% Obstáculo AABB — debe coincidir con pick_place_params.yaml:
%   obstacle_center:       [0.85, 0.00, 0.10]
%   obstacle_half_extents: [0.20, 0.30, 0.10]
OBS_CX = 0.85;  OBS_CY = 0.00;  OBS_CZ = 0.10;
OBS_HX = 0.20;  OBS_HY = 0.30;  OBS_HZ = 0.10;

% Tolerancia de posición del JTC de Gazebo [rad]  (para anotación en Fig 4)
JTC_POS_TOL = 0.200;

% Exportar figuras
EXPORT_PNG = true;

% Número de prueba (0 = sin subdirectorio; 1..N → test1, test2, …)
%   results leídos de:  ur5_trajectory_optimization/results/testN/
%   figuras escritas en: ur5_trajectory_optimization/plots/comparison/testN/
TEST_ID = 2;

% ═══════════════════════════════════════════════════════════════════════
%  DIRECTORIOS
% ═══════════════════════════════════════════════════════════════════════
repo_root = fullfile(getenv('HOME'), 'ur5_ws', 'src', 'ur5_utec');
data_dir  = fullfile(repo_root, 'ur5_pick_place', 'data');
pkg_root  = fullfile(repo_root, 'ur5_trajectory_optimization');

if TEST_ID > 0
    test_sub = sprintf('test%d', TEST_ID);
    res_dir  = fullfile(pkg_root, 'results', test_sub);
    out_dir  = fullfile(pkg_root, 'results', test_sub, 'plots', 'traj_comparison');
else
    res_dir  = fullfile(pkg_root, 'results');
    out_dir  = fullfile(pkg_root, 'results', 'plots', 'traj_comparison');
end

KP_NAMES = {'A', 'B', 'O', 'C', 'D'};

% ═══════════════════════════════════════════════════════════════════════
%  1.  CARGAR CSVs DE TRAYECTORIA
% ═══════════════════════════════════════════════════════════════════════
if isempty(FILE_NSGA2) || isempty(FILE_EPSILON)
    csvs = dir(fullfile(data_dir, 'trajectory_*_clamped_spline.csv'));
    if numel(csvs) < 2
        error(['Necesita ≥2 CSV en:\n  %s\n' ...
               'Ejecute Gazebo con el via-point NSGA-II y luego con ε-constraint.'], ...
               data_dir);
    end
    [~, ord] = sort([csvs.datenum]);
    csvs = csvs(ord);                               % más antiguo primero
    if isempty(FILE_NSGA2),   FILE_NSGA2   = csvs(end-1).name; end
    if isempty(FILE_EPSILON), FILE_EPSILON = csvs(end).name;   end
end

TN = readtable(fullfile(data_dir, FILE_NSGA2));
TE = readtable(fullfile(data_dir, FILE_EPSILON));
fprintf('NSGA-II  : %s  (%d filas)\n', FILE_NSGA2,   height(TN));
fprintf('ε-constr : %s  (%d filas)\n', FILE_EPSILON, height(TE));

% ── Extraer columnas ─────────────────────────────────────────────────────
[tN,xN,yN,zN,wN,vxN,vyN,vzN,axN,ayN,azN,dqN,tauN,jxN,jyN,jzN] = extract_cols(TN);
[tE,xE,yE,zE,wE,vxE,vyE,vzE,axE,ayE,azE,dqE,tauE,jxE,jyE,jzE] = extract_cols(TE);

spdN  = sqrt(vxN.^2 + vyN.^2 + vzN.^2);
accnN = sqrt(axN.^2 + ayN.^2 + azN.^2);
jrkN  = sqrt(jxN.^2 + jyN.^2 + jzN.^2);
spdE  = sqrt(vxE.^2 + vyE.^2 + vzE.^2);
accnE = sqrt(axE.^2 + ayE.^2 + azE.^2);
jrkE  = sqrt(jxE.^2 + jyE.^2 + jzE.^2);

% Índices de keypoints  (waypoint 1=A, 2=B, 3=O, 4=C, 5=D;  0=interpolado)
kpN = zeros(1,5);  kpE = zeros(1,5);
for k = 1:5
    idx = find(wN == k, 1, 'first');  if ~isempty(idx), kpN(k) = idx; end
    idx = find(wE == k, 1, 'first');  if ~isempty(idx), kpE(k) = idx; end
end

nsga2_complete = all(kpN > 0);
if ~nsga2_complete
    miss = KP_NAMES(kpN == 0);
    fprintf('AVISO: trayectoria NSGA-II incompleta — waypoints ausentes: %s\n', ...
        strjoin(miss, ', '));
end

if kpN(3) > 0
    O_N = [xN(kpN(3)) yN(kpN(3)) zN(kpN(3))];
else
    O_N = [NaN NaN NaN];
end
O_E = [xE(kpE(3)) yE(kpE(3)) zE(kpE(3))];

% ═══════════════════════════════════════════════════════════════════════
%  2.  MÉTRICAS RESUMIDAS
% ═══════════════════════════════════════════════════════════════════════
arc_N    = sum(sqrt(diff(xN).^2 + diff(yN).^2 + diff(zN).^2));
arc_E    = sum(sqrt(diff(xE).^2 + diff(yE).^2 + diff(zE).^2));
effort_N = trapz(tN, sum(tauN.^2, 2));
effort_E = trapz(tE, sum(tauE.^2, 2));
cl_N     = tcp_clearance(xN,yN,zN, OBS_CX,OBS_CY,OBS_CZ,OBS_HX,OBS_HY,OBS_HZ);
cl_E     = tcp_clearance(xE,yE,zE, OBS_CX,OBS_CY,OBS_CZ,OBS_HX,OBS_HY,OBS_HZ);
maxdqN   = max(abs(dqN(:)));
maxdqE   = max(abs(dqE(:)));
rmsjN    = sqrt(mean(jrkN.^2));
rmsjE    = sqrt(mean(jrkE.^2));

fprintf('\n%-32s  %-16s  %-16s\n', 'Métrica', 'NSGA-II', 'ε-constraint');
fprintf('%s\n', repmat('─', 1, 66));
fprintf('%-32s  %-16.4f  %-16.4f\n', 'Longitud de arco  [m]',      arc_N,    arc_E);
fprintf('%-32s  %-16.1f  %-16.1f\n', 'Esfuerzo ∫Στ²  [N²·m²·s]',  effort_N, effort_E);
fprintf('%-32s  %-16.4f  %-16.4f\n', 'Clearance mín TCP  [m]',     cl_N,     cl_E);
fprintf('%-32s  %-16.4f  %-16.4f\n', 'Max |q̇|  [rad/s]',           maxdqN,   maxdqE);
fprintf('%-32s  %-16.4f  %-16.4f\n', 'RMS jerk  [m/s³]',           rmsjN,    rmsjE);
if nsga2_complete
    ns_str = 'sí';
else
    ns_str = 'no  (abort JTC)';
end
fprintf('%-32s  %-16s  %-16s\n', 'Trayectoria completa', ns_str, 'sí');
fprintf('%-32s  [%.3f, %.3f, %.3f]\n', 'point_O NSGA-II  [m]',  O_N(1),O_N(2),O_N(3));
fprintf('%-32s  [%.3f, %.3f, %.3f]\n', 'point_O ε-constr  [m]', O_E(1),O_E(2),O_E(3));

% ═══════════════════════════════════════════════════════════════════════
%  PALETA Y ESTILOS
% ═══════════════════════════════════════════════════════════════════════
lw   = 2.0;   fs  = 11;
c_N  = [0.8500 0.3250 0.0980];   % naranja  — NSGA-II
c_E  = [0.0000 0.4470 0.7410];   % azul     — ε-constraint
c_ob = [0.6350 0.0780 0.1840];   % rojo oscuro — obstáculo

kp_col = {[0.4660 0.8740 0.1880],  % A  verde claro
          [0.1660 0.6740 0.1880],  % B  verde
          [0.4940 0.1840 0.5560],  % O  morado
          [0.8500 0.3250 0.0980],  % C  naranja
          [1.0000 0.6500 0.3000]}; % D  naranja claro
kp_sty = {':',  '--', '--', '--', ':'};
kp_mk  = {'v',  'o',  's',  '^',  'd'};

if nsga2_complete
    lbl_N = 'NSGA-II';
else
    lbl_N = 'NSGA-II (incompleto)';
end

% Obstáculo: vértices y caras para patch 3D
obs_V = box_vertices(OBS_CX, OBS_CY, OBS_CZ, OBS_HX, OBS_HY, OBS_HZ);
obs_F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 4 3 7 8; 1 4 8 5; 2 3 7 6];

xlim_t = [min(tN(1), tE(1)),  max(tN(end), tE(end))];

% ═══════════════════════════════════════════════════════════════════════
%  Fig 1 — Trayectoria 3D
% ═══════════════════════════════════════════════════════════════════════
figure(1); clf;
set(gcf, 'Color','w', 'Position',[80 220 900 680]);

h_ob = patch('Vertices',obs_V,'Faces',obs_F,'FaceColor',c_ob, ...
    'FaceAlpha',0.18,'EdgeColor',c_ob,'LineWidth',1.2);
hold on;
h_N = plot3(xN,yN,zN,'--','Color',c_N,'LineWidth',lw);
h_E = plot3(xE,yE,zE,'-', 'Color',c_E,'LineWidth',lw);

% Waypoints  (posiciones del ε, que siempre está completo)
kp_off = [0.02  0.04  0.02;  0.02  0.04  0.02;  0.03  0.02  0.03;
          0.02 -0.09  0.02;  0.02 -0.09  0.02];
for k = 1:5
    if kpE(k) > 0
        plot3(xE(kpE(k)), yE(kpE(k)), zE(kpE(k)), kp_mk{k}, ...
            'MarkerSize',10,'MarkerFaceColor',kp_col{k}, ...
            'MarkerEdgeColor','k','LineWidth',1.1, ...
            'HandleVisibility','off');
        text(xE(kpE(k))+kp_off(k,1), yE(kpE(k))+kp_off(k,2), ...
             zE(kpE(k))+kp_off(k,3), KP_NAMES{k}, ...
            'FontSize',fs,'FontWeight','bold','Color',kp_col{k});
    end
end

% Puntos O de cada método
if ~any(isnan(O_N))
    plot3(O_N(1),O_N(2),O_N(3),'s','MarkerSize',13,'MarkerFaceColor',c_N, ...
        'MarkerEdgeColor','k','LineWidth',1.5,'HandleVisibility','off');
    text(O_N(1)+0.03, O_N(2), O_N(3)+0.02, ...
        sprintf('$O_N$  [%.2f, %.2f, %.2f]', O_N(1),O_N(2),O_N(3)), ...
        'Interpreter','latex','Color',c_N,'FontSize',fs-1,'FontWeight','bold');
end
plot3(O_E(1),O_E(2),O_E(3),'^','MarkerSize',13,'MarkerFaceColor',c_E, ...
    'MarkerEdgeColor','k','LineWidth',1.5,'HandleVisibility','off');
text(O_E(1)+0.03, O_E(2), O_E(3)-0.03, ...
    sprintf('$O_{\\varepsilon}$  [%.2f, %.2f, %.2f]', O_E(1),O_E(2),O_E(3)), ...
    'Interpreter','latex','Color',c_E,'FontSize',fs-1,'FontWeight','bold');

text(OBS_CX+OBS_HX+0.02, OBS_CY, OBS_CZ+OBS_HZ+0.02, 'Obstáculo', ...
    'FontSize',fs-1,'FontWeight','bold','Color',c_ob,'Clipping','on');

xlabel('x [m]','FontSize',fs);  ylabel('y [m]','FontSize',fs);
zlabel('z [m]','FontSize',fs);
grid on; box on; set(gca,'FontSize',fs); view(42,22);
legend([h_N, h_E, h_ob], {lbl_N, 'ε-constraint', 'Obstáculo AABB'}, ...
    'Location','best','FontSize',fs-1);
title('Trayectoria 3D del TCP — NSGA-II vs ε-constraint', ...
    'FontSize',14,'FontWeight','bold');

% ═══════════════════════════════════════════════════════════════════════
%  Fig 2 — Posición cartesiana
% ═══════════════════════════════════════════════════════════════════════
figure(2); clf;
set(gcf,'Color','w','Position',[1000 220 760 640]);
tl2 = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

dN2 = {xN,yN,zN};  dE2 = {xE,yE,zE};
yl2 = {'$x$ [m]','$y$ [m]','$z$ [m]'};
obs_zbands = [OBS_CZ-OBS_HZ,  OBS_CZ+OBS_HZ];   % [0.00  0.20]

hNp2 = []; hEp2 = [];
for i = 1:3
    ax2 = nexttile(tl2); hold on;
    if i == 3   % banda de altura del obstáculo en z(t)
        patch([xlim_t(1) xlim_t(2) xlim_t(2) xlim_t(1)], ...
              [obs_zbands(1) obs_zbands(1) obs_zbands(2) obs_zbands(2)], ...
              c_ob, 'FaceAlpha',0.10,'EdgeColor','none','HandleVisibility','off');
        yline(obs_zbands(1),':','Color',c_ob,'LineWidth',0.9,'HandleVisibility','off');
        yline(obs_zbands(2),':','Color',c_ob,'LineWidth',0.9,'HandleVisibility','off');
    end
    hN2 = plot(tN, dN2{i}, '--', 'Color',c_N, 'LineWidth',lw);
    hE2 = plot(tE, dE2{i}, '-',  'Color',c_E, 'LineWidth',lw);
    if i == 1, hNp2 = hN2; hEp2 = hE2; end
    for k = 1:5
        if kpN(k) > 0
            plot(tN(kpN(k)), dN2{i}(kpN(k)), kp_mk{k}, 'MarkerSize',7, ...
                'MarkerFaceColor',kp_col{k},'MarkerEdgeColor','k', ...
                'HandleVisibility','off');
        end
        if kpE(k) > 0
            plot(tE(kpE(k)), dE2{i}(kpE(k)), kp_mk{k}, 'MarkerSize',7, ...
                'MarkerFaceColor',kp_col{k},'MarkerEdgeColor','k', ...
                'HandleVisibility','off');
        end
    end
    add_kp_lines(ax2, tE, kpE, kp_col, kp_sty);
    ylabel(yl2{i},'Interpreter','latex','FontSize',fs);
    grid on; box on; set(ax2,'FontSize',fs); xlim(xlim_t);
    if i < 3, set(ax2,'XTickLabel',[]); end
end
xlabel('Tiempo [s]','FontSize',fs);
lgd2 = legend(nexttile(tl2,1), [hNp2, hEp2], {lbl_N,'ε-constraint'}, ...
    'Orientation','horizontal','FontSize',fs-1);
lgd2.Layout.Tile = 'north';
title(tl2,'Posición cartesiana TCP','FontSize',14,'FontWeight','bold');

% ═══════════════════════════════════════════════════════════════════════
%  Fig 3 — Cinemática cartesiana  (velocidad, aceleración, jerk)
% ═══════════════════════════════════════════════════════════════════════
figure(3); clf;
set(gcf,'Color','w','Position',[80 100 760 600]);
tl3 = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

dN3 = {spdN, accnN, jrkN};  dE3 = {spdE, accnE, jrkE};
yl3  = {'$\|\mathbf{v}\|$ [m/s]', '$\|\mathbf{a}\|$ [m/s$^2$]', '$\|\mathbf{j}\|$ [m/s$^3$]'};

hNk3 = []; hEk3 = [];
for i = 1:3
    ax3 = nexttile(tl3); hold on;
    hN3 = plot(tN, dN3{i}, '--', 'Color',c_N, 'LineWidth',lw);
    hE3 = plot(tE, dE3{i}, '-',  'Color',c_E, 'LineWidth',lw);
    if i == 1, hNk3 = hN3; hEk3 = hE3; end
    for k = 1:5
        if kpN(k) > 0
            plot(tN(kpN(k)), dN3{i}(kpN(k)), kp_mk{k}, 'MarkerSize',7, ...
                'MarkerFaceColor',kp_col{k},'MarkerEdgeColor','k', ...
                'HandleVisibility','off');
        end
        if kpE(k) > 0
            plot(tE(kpE(k)), dE3{i}(kpE(k)), kp_mk{k}, 'MarkerSize',7, ...
                'MarkerFaceColor',kp_col{k},'MarkerEdgeColor','k', ...
                'HandleVisibility','off');
        end
    end
    add_kp_lines(ax3, tE, kpE, kp_col, kp_sty);
    ylabel(yl3{i},'Interpreter','latex','FontSize',fs);
    grid on; box on; set(ax3,'FontSize',fs); xlim(xlim_t);
    if i < 3, set(ax3,'XTickLabel',[]); end
end
xlabel('Tiempo [s]','FontSize',fs);
lgd3 = legend(nexttile(tl3,1), [hNk3, hEk3], {lbl_N,'ε-constraint'}, ...
    'Orientation','horizontal','FontSize',fs-1);
lgd3.Layout.Tile = 'north';
title(tl3,'Cinemática cartesiana TCP','FontSize',14,'FontWeight','bold');

% ═══════════════════════════════════════════════════════════════════════
%  Fig 4 — Velocidades articulares  (diagnóstico abort JTC)
% ═══════════════════════════════════════════════════════════════════════
% El JTC en Gazebo reportó:
%   "Position Error: 0.204924, Position Tolerance: 0.200000"
% La alta velocidad de joint 1 (shoulder lift, q̇₁) impidió seguir la
% referencia de posición en el tramo B→O del NSGA-II, acumulando un error
% de posición que superó la tolerancia de 0.200 rad.
figure(4); clf;
set(gcf,'Color','w','Position',[80 100 1100 700]);
tl4 = tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

ax4_first = [];  hN4leg = [];  hE4leg = [];
for j = 1:6
    ax4 = nexttile(tl4); hold on;
    if isempty(ax4_first), ax4_first = ax4; end

    hN4 = plot(tN, dqN(:,j), '--', 'Color',c_N, 'LineWidth',lw);
    hE4 = plot(tE, dqE(:,j), '-',  'Color',c_E, 'LineWidth',lw);
    if j == 1, hN4leg = hN4; hE4leg = hE4; end

    % Joint 2 (MATLAB j=2 = q̇₁ shoulder lift) — causó el abort
    if j == 2
        ymx_n = max(abs(dqN(:,j)));  ymx_e = max(abs(dqE(:,j)));
        x_ann = xlim_t(1) + 0.05*(xlim_t(2)-xlim_t(1));
        text(x_ann, max(dqN(:,j))*0.82, ...
            sprintf('max = %.3f rad/s', ymx_n), ...
            'Color',c_N,'FontSize',fs-2,'FontWeight','bold');
        text(x_ann, max(dqE(:,j))*0.65, ...
            sprintf('max = %.3f rad/s', ymx_e), ...
            'Color',c_E,'FontSize',fs-2,'FontWeight','bold');
        ttl_j = sprintf('Joint %d  \\leftarrow abort  (tol pos. = %.3f rad)', j-1, JTC_POS_TOL);
    else
        ttl_j = sprintf('Joint %d', j-1);
    end

    add_kp_lines(ax4, tE, kpE, kp_col, kp_sty);
    ylabel(sprintf('$\\dot{q}_%d$ [rad/s]', j-1), 'Interpreter','latex', ...
        'FontSize',fs-1);
    title(ttl_j,'FontSize',fs-1);
    grid on; box on; set(ax4,'FontSize',fs-1); xlim(xlim_t);
end
xlabel('Tiempo [s]','FontSize',fs);
sgtitle('Velocidades articulares — diagnóstico abort JTC', ...
    'FontSize',13,'FontWeight','bold');
legend(ax4_first, [hN4leg, hE4leg], {lbl_N,'ε-constraint'}, ...
    'Orientation','horizontal','FontSize',fs-1,'Location','best');

% ═══════════════════════════════════════════════════════════════════════
%  Fig 5 — Torques articulares  (objetivo f₁ = ∫Στ² dt)
% ═══════════════════════════════════════════════════════════════════════
figure(5); clf;
set(gcf,'Color','w','Position',[80 100 1100 700]);
tl5 = tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

ax5_first = [];  hN5leg = [];  hE5leg = [];
for j = 1:6
    ax5 = nexttile(tl5); hold on;
    if isempty(ax5_first), ax5_first = ax5; end
    hN5 = plot(tN, tauN(:,j), '--', 'Color',c_N, 'LineWidth',lw);
    hE5 = plot(tE, tauE(:,j), '-',  'Color',c_E, 'LineWidth',lw);
    if j == 1, hN5leg = hN5; hE5leg = hE5; end
    add_kp_lines(ax5, tE, kpE, kp_col, kp_sty);
    ylabel(sprintf('$\\tau_%d$ [N$\\cdot$m]', j-1), 'Interpreter','latex','FontSize',fs-1);
    title(sprintf('Joint %d', j-1),'FontSize',fs-1);
    grid on; box on; set(ax5,'FontSize',fs-1); xlim(xlim_t);
end
xlabel('Tiempo [s]','FontSize',fs);
sgtitle(sprintf('Torques articulares ($f_1$) - NSGA-II: %.0f $|$ $\\varepsilon$: %.0f [N$^2\\cdot$m$^2\\cdot$s]', ...
    effort_N, effort_E), 'FontSize',13,'FontWeight','bold','Interpreter','latex');
legend(ax5_first, [hN5leg, hE5leg], {lbl_N,'ε-constraint'}, ...
    'Orientation','horizontal','FontSize',fs-1,'Location','best');

% ═══════════════════════════════════════════════════════════════════════
%  Fig 6 — Frentes de Pareto
% ═══════════════════════════════════════════════════════════════════════
pareto_n_file = fullfile(res_dir, 'pareto_nsga2.csv');
pareto_e_file = fullfile(res_dir, 'pareto_epsilon.csv');

if isfile(pareto_n_file) && isfile(pareto_e_file)
    PN = readtable(pareto_n_file);
    PE = readtable(pareto_e_file);

    c_pN = [0.9290 0.6940 0.1250];   % amarillo — NSGA-II frente
    c_pE = [0.4660 0.8740 0.1880];   % verde    — ε-constraint frente

    % Punto del Pareto más cercano al via-point ejecutado (en espacio de diseño)
    via_mat_N = [PN.via_x PN.via_y PN.via_z];
    via_mat_E = [PE.via_x PE.via_y PE.via_z];
    if ~any(isnan(O_N))
        [~, idx_pN] = min(sum((via_mat_N - O_N).^2, 2));
    else
        [~, idx_pN] = min(PN.f1_effort);
    end
    [~, idx_pE] = min(sum((via_mat_E - O_E).^2, 2));

    figure(6); clf;
    set(gcf,'Color','w','Position',[80 100 1100 490]);
    tl6 = tiledlayout(1,2,'TileSpacing','compact','Padding','compact');

    % ── F1 vs F2 ─────────────────────────────────────────────────────────
    nexttile(tl6); hold on;
    h6pN = scatter(PN.f1_effort, PN.f2_arclen, 48, c_pN, 'o', 'filled');
    h6pN.MarkerFaceAlpha = 0.55;
    h6pE = scatter(PE.f1_effort, PE.f2_arclen, 48, c_pE, '^', 'filled');
    h6pE.MarkerFaceAlpha = 0.80;
    h6sN = plot(PN.f1_effort(idx_pN), PN.f2_arclen(idx_pN), 'o', ...
        'MarkerSize',13,'MarkerFaceColor',c_N,'MarkerEdgeColor','k','LineWidth',1.5);
    h6sE = plot(PE.f1_effort(idx_pE), PE.f2_arclen(idx_pE), '^', ...
        'MarkerSize',13,'MarkerFaceColor',c_E,'MarkerEdgeColor','k','LineWidth',1.5);
    xlabel('$f_1$ - Esfuerzo [$\mathrm{N}^2\cdot\mathrm{m}^2\cdot\mathrm{s}$]','Interpreter','latex','FontSize',fs);
    ylabel('$f_2$ - Longitud de arco [m]','Interpreter','latex','FontSize',fs);
    legend([h6pN, h6pE, h6sN, h6sE], ...
        {'NSGA-II (Pareto)','ε-constraint (Pareto)','Sel. NSGA-II','Sel. ε-constr'}, ...
        'Location','best','FontSize',fs-1);
    grid on; box on; set(gca,'FontSize',fs);
    title('$f_1$ vs $f_2$','Interpreter','latex','FontSize',13,'FontWeight','bold');

    % ── F1 vs clearance (−f₃) ────────────────────────────────────────────
    nexttile(tl6); hold on;
    h6pNc = scatter(PN.f1_effort, -PN.f3_clearance, 48, c_pN, 'o', 'filled');
    h6pNc.MarkerFaceAlpha = 0.55;
    h6pEc = scatter(PE.f1_effort, -PE.f3_clearance, 48, c_pE, '^', 'filled');
    h6pEc.MarkerFaceAlpha = 0.80;
    h6sNc = plot(PN.f1_effort(idx_pN), -PN.f3_clearance(idx_pN), 'o', ...
        'MarkerSize',13,'MarkerFaceColor',c_N,'MarkerEdgeColor','k','LineWidth',1.5);
    h6sEc = plot(PE.f1_effort(idx_pE), -PE.f3_clearance(idx_pE), '^', ...
        'MarkerSize',13,'MarkerFaceColor',c_E,'MarkerEdgeColor','k','LineWidth',1.5);
    yline(0.05,'k--','LineWidth',1.1, ...
        'Label','$r_\mathrm{grip} = 0.05$ m','Interpreter','latex', ...
        'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom');
    xlabel('$f_1$ - Esfuerzo [$\mathrm{N}^2\cdot\mathrm{m}^2\cdot\mathrm{s}$]','Interpreter','latex','FontSize',fs);
    ylabel('Clearance TCP–obstáculo [m]','FontSize',fs);
    legend([h6pNc, h6pEc, h6sNc, h6sEc], ...
        {'NSGA-II (Pareto)','ε-constraint (Pareto)','Sel. NSGA-II','Sel. ε-constr'}, ...
        'Location','best','FontSize',fs-1);
    grid on; box on; set(gca,'FontSize',fs);
    title('$f_1$ vs Clearance','Interpreter','latex','FontSize',13,'FontWeight','bold');

    title(tl6,'Frentes de Pareto — NSGA-II vs ε-constraint', ...
        'FontSize',14,'FontWeight','bold');
else
    fprintf('AVISO: CSVs de Pareto no encontrados en:\n  %s\n', res_dir);
    fprintf('       Ejecute run_optimization primero.\n');
end

% ═══════════════════════════════════════════════════════════════════════
%  EXPORTAR FIGURAS  (PNG 300 dpi + EPS vector)
% ═══════════════════════════════════════════════════════════════════════
if EXPORT_PNG
    if ~exist(out_dir,'dir'), mkdir(out_dir); end
    fig_files = {'trayectoria_3D',         'posicion_cartesiana', ...
                 'cinematica_cartesiana',  'velocidades_articulares', ...
                 'torques_articulares',    'pareto_fronts'};
    for f = 1:6
        if ~ishandle(f), continue; end
        exportgraphics(figure(f), fullfile(out_dir, [fig_files{f} '.png']), ...
            'Resolution',300);
        exportgraphics(figure(f), fullfile(out_dir, [fig_files{f} '.eps']), ...
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
% Distancia mínima de la ruta TCP a la AABB del obstáculo.
    pts     = [x y z];
    lo      = [cx-hx, cy-hy, cz-hz];
    hi      = [cx+hx, cy+hy, cz+hz];
    nearest = max(min(pts, hi), lo);   % punto más cercano en la AABB (broadcast R2016b+)
    d       = min(sqrt(sum((pts - nearest).^2, 2)));
end

function V = box_vertices(cx,cy,cz,hx,hy,hz)
% Genera 8 vértices de la caja AABB para patch 3D.
    sx = [-1  1  1 -1 -1  1  1 -1]' .* hx + cx;
    sy = [-1 -1  1  1 -1 -1  1  1]' .* hy + cy;
    sz = [-1 -1 -1 -1  1  1  1  1]' .* hz + cz;
    V  = [sx sy sz];
end

function add_kp_lines(ax, t, kp_idx, kp_col, kp_sty)
% Añade líneas verticales de referencia en los instantes de keypoint.
    for k = 1:numel(kp_idx)
        if kp_idx(k) > 0 && kp_idx(k) <= numel(t)
            xline(ax, t(kp_idx(k)), kp_sty{k}, ...
                'Color', kp_col{k}, 'LineWidth', 0.9, 'HandleVisibility','off');
        end
    end
end
