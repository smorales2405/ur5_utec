%% compare_trajectories.m
% Comparación de métodos de interpolación: piecewise_linear vs clamped_spline.
%
%   Figura 1 — Trayectoria 3D del TCP (ambos métodos + obstáculo 3D)
%   Figura 2 — Posición cartesiana:   x(t), y(t), z(t)   [subplot 3×1]
%   Figura 3 — Norma de velocidad cartesiana  ||v(t)||
%   Figura 4 — Norma de aceleración cartesiana ||a(t)||
%
% Carga automáticamente el CSV más reciente de cada método en data_dir.
% Para fijar un archivo concreto, asigna su nombre en FILE_LINEAR / FILE_SPLINE.

clear; clc; close all;

% ── Configuración ─────────────────────────────────────────────────────────────
FILE_LINEAR = '';   % '' = auto (más reciente); o nombre exacto del CSV
FILE_SPLINE = '';   % '' = auto (más reciente); o nombre exacto del CSV

% Obstáculo (debe coincidir con plots_trajectory.m)
OBS_CX = 0.65;  OBS_CY = 0.00;  OBS_ZT = -0.375;
OBS_DX = 0.15;  OBS_DY = 0.15;  OBS_H  = 0.125;

EXPORT_PNG = false;

KP_NAMES  = {'A','B','O','C','D'};

% ── 1. Auto-detección y carga de CSVs ─────────────────────────────────────────
data_dir = fullfile(getenv('HOME'), 'ur5_ws', 'src', 'ur5_pick_place', 'data');

if isempty(FILE_LINEAR)
    f = dir(fullfile(data_dir, 'trajectory_20260517_201319_piecewise_linear.csv'));
    if isempty(f)
        error('No se encontró CSV de piecewise_linear en %s.\nEjecutar pick_place_node primero.', data_dir);
    end
    [~, i] = max([f.datenum]);
    FILE_LINEAR = f(i).name;
end

if isempty(FILE_SPLINE)
    f = dir(fullfile(data_dir, 'trajectory_20260517_201032_clamped_spline.csv'));
    if isempty(f)
        error('No se encontró CSV de clamped_spline en %s.\nEjecutar pick_place_node primero.', data_dir);
    end
    [~, i] = max([f.datenum]);
    FILE_SPLINE = f(i).name;
end

TL = readtable(fullfile(data_dir, FILE_LINEAR));
TS = readtable(fullfile(data_dir, FILE_SPLINE));
fprintf('Lineal : %s  (%d muestras)\n', FILE_LINEAR, height(TL));
fprintf('Spline : %s  (%d muestras)\n', FILE_SPLINE, height(TS));

% ── 2. Extraer columnas ───────────────────────────────────────────────────────
tL = TL.time_s;  xL = TL.tcp_x;  yL = TL.tcp_y;  zL = TL.tcp_z;
wL = TL.waypoint;
vxL = TL.vel_x; vyL = TL.vel_y; vzL = TL.vel_z;
axL = TL.acc_x; ayL = TL.acc_y; azL = TL.acc_z;
speedL = sqrt(vxL.^2 + vyL.^2 + vzL.^2);
accnL  = sqrt(axL.^2 + ayL.^2 + azL.^2);

tS = TS.time_s;  xS = TS.tcp_x;  yS = TS.tcp_y;  zS = TS.tcp_z;
wS = TS.waypoint;
vxS = TS.vel_x; vyS = TS.vel_y; vzS = TS.vel_z;
axS = TS.acc_x; ayS = TS.acc_y; azS = TS.acc_z;
speedS = sqrt(vxS.^2 + vyS.^2 + vzS.^2);
accnS  = sqrt(axS.^2 + ayS.^2 + azS.^2);

% Índices de keypoints
kpL = [find(wL==1,1), find(wL==2,1), find(wL==3,1), find(wL==4,1), find(wL==5,1)];
kpS = [find(wS==1,1), find(wS==2,1), find(wS==3,1), find(wS==4,1), find(wS==5,1)];

% ── 3. Paleta de colores ──────────────────────────────────────────────────────
lw    = 2.0;
fs    = 11;
c_lin = [0.8500 0.3250 0.0980];   % naranja  — piecewise linear
c_spl = [0.0000 0.4470 0.7410];   % azul     — clamped spline
c_obs = [0.6350 0.0780 0.1840];   % rojo     — obstáculo

% Colores individuales de keypoints (ídem plots_trajectory.m)
kp_col = {[0.4660 0.8740 0.1880], ...   % A  verde claro
          [0.1660 0.6740 0.1880], ...   % B  verde
          [0.4940 0.1840 0.5560], ...   % O  morado
          [0.8500 0.3250 0.0980], ...   % C  naranja
          [1.0000 0.6500 0.3000]};      % D  naranja claro
kp_sty = {':', '--', '--', '--', ':'};  % estilo de línea vertical
kp_mk  = {'v', 'o', 's', '^', 'd'};    % marcador por keypoint

% ── 4. Figura 1 — Trayectoria 3D ─────────────────────────────────────────────
figure(1); clf;
set(gcf, 'Color', 'w', 'Position', [80 220 920 700]);

% Caja 3D del obstáculo
xBL=OBS_CX-OBS_DX; xBR=OBS_CX+OBS_DX;
yBF=OBS_CY-OBS_DY; yBBk=OBS_CY+OBS_DY;
zBb=OBS_ZT-OBS_H;  zBt=OBS_ZT;
obs_V = [xBL yBF  zBb; xBR yBF  zBb; xBR yBBk zBb; xBL yBBk zBb;
         xBL yBF  zBt; xBR yBF  zBt; xBR yBBk zBt; xBL yBBk zBt];
obs_F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 4 3 7 8; 1 4 8 5; 2 3 7 6];
hObs = patch('Vertices', obs_V, 'Faces', obs_F, ...
    'FaceColor', c_obs, 'FaceAlpha', 0.18, 'EdgeColor', c_obs, 'LineWidth', 1.1);
hold on;

hLin3 = plot3(xL, yL, zL, '--', 'Color', c_lin, 'LineWidth', lw);
hSpl3 = plot3(xS, yS, zS, '-',  'Color', c_spl, 'LineWidth', lw);

% Marcadores en keypoints (una sola serie — spline; mismas posiciones)
for k = 1:5
    plot3(xS(kpS(k)), yS(kpS(k)), zS(kpS(k)), kp_mk{k}, ...
        'MarkerSize', 10, 'MarkerFaceColor', kp_col{k}, ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1.1);
end

% Etiquetas de keypoints
off = [0.02  0.03  0.01;   % A
       0.02  0.03  0.01;   % B
       0.02  0.03  0.02;   % O
       0.02 -0.09  0.01;   % C
       0.02 -0.09  0.01];  % D
for k = 1:5
    text(xS(kpS(k))+off(k,1), yS(kpS(k))+off(k,2), zS(kpS(k))+off(k,3), ...
        KP_NAMES{k}, 'FontSize', fs, 'FontWeight', 'bold', 'Color', kp_col{k});
end
text(OBS_CX+OBS_DX+0.02, OBS_CY, OBS_ZT+0.015, 'Obstáculo', ...
    'FontSize', fs-1, 'FontWeight', 'bold', 'Color', c_obs, 'Clipping', 'on');

xlabel('x [m]', 'FontSize', fs);
ylabel('y [m]', 'FontSize', fs);
zlabel('z [m]', 'FontSize', fs);
grid on; box on;
set(gca, 'FontSize', fs);
view(45, 25);
legend([hLin3, hSpl3, hObs], {'Lineal por tramos', 'Spline cúbico', 'Obstáculo'}, ...
    'Location', 'best', 'FontSize', fs-1);
title('Trayectoria 3D TCP — Comparación de métodos', 'FontSize', 14, 'FontWeight', 'bold');

% ── 5. Figura 2 — Posición cartesiana (subplot 3×1) ──────────────────────────
figure(2); clf;
set(gcf, 'Color', 'w', 'Position', [1020 220 760 640]);
tl2 = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

dataL2 = {xL, yL, zL};
dataS2 = {xS, yS, zS};
ylbl2  = {'$x$ [m]', '$y$ [m]', '$z$ [m]'};
xlim2  = [min(tL(1), tS(1)),  max(tL(end), tS(end))];

ax2_first = gobjects(1);
hL2_ref   = [];
hS2_ref   = [];

for i = 1:3
    ax2 = nexttile(tl2);
    hold on;
    hL2 = plot(tL, dataL2{i}, '--', 'Color', c_lin, 'LineWidth', lw);
    hS2 = plot(tS, dataS2{i}, '-',  'Color', c_spl, 'LineWidth', lw);
    if i == 1
        ax2_first = ax2;
        hL2_ref   = hL2;
        hS2_ref   = hS2;
    end
    % Marcadores en keypoints
    for k = 1:5
        plot(tL(kpL(k)), dataL2{i}(kpL(k)), kp_mk{k}, 'MarkerSize', 7, ...
            'MarkerFaceColor', kp_col{k}, 'MarkerEdgeColor', 'k', ...
            'Color', c_lin);
        plot(tS(kpS(k)), dataS2{i}(kpS(k)), kp_mk{k}, 'MarkerSize', 7, ...
            'MarkerFaceColor', kp_col{k}, 'MarkerEdgeColor', 'k', ...
            'Color', c_spl);
    end
    add_kp_lines(ax2, tS, kpS, kp_col, kp_sty);
    ylabel(ylbl2{i}, 'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on;
    set(ax2, 'FontSize', fs);
    xlim(xlim2);
    if i < 3, set(ax2, 'XTickLabel', []); end
end
xlabel('Tiempo [s]', 'FontSize', fs);

lgd2 = legend(ax2_first, [hL2_ref, hS2_ref], ...
    {'Lineal por tramos', 'Spline cúbico'}, ...
    'Orientation', 'horizontal', 'FontSize', fs-1);
lgd2.Layout.Tile = 'north';

title(tl2, 'Posición cartesiana TCP — Comparación', 'FontSize', 14, 'FontWeight', 'bold');

% ── 6. Figura 3 — Norma de velocidad cartesiana ───────────────────────────────
figure(3); clf;
set(gcf, 'Color', 'w', 'Position', [80 100 760 390]);
hold on;
hL3 = plot(tL, speedL, '--', 'Color', c_lin, 'LineWidth', lw);
hS3 = plot(tS, speedS, '-',  'Color', c_spl, 'LineWidth', lw);
for k = 1:5
    plot(tL(kpL(k)), speedL(kpL(k)), kp_mk{k}, 'MarkerSize', 7, ...
        'MarkerFaceColor', kp_col{k}, 'MarkerEdgeColor', 'k', 'Color', c_lin);
    plot(tS(kpS(k)), speedS(kpS(k)), kp_mk{k}, 'MarkerSize', 7, ...
        'MarkerFaceColor', kp_col{k}, 'MarkerEdgeColor', 'k', 'Color', c_spl);
end
add_kp_lines(gca, tS, kpS, kp_col, kp_sty);
ylabel('$\|\mathbf{v}\|$ [m/s]', 'Interpreter', 'latex', 'FontSize', fs);
xlabel('Tiempo [s]', 'FontSize', fs);
grid on; box on;
set(gca, 'FontSize', fs);
xlim([min(tL(1), tS(1)),  max(tL(end), tS(end))]);
legend([hL3, hS3], {'Lineal por tramos', 'Spline cúbico'}, ...
    'Location', 'best', 'FontSize', fs-1);
title('Norma velocidad cartesiana TCP — Comparación', 'FontSize', 14, 'FontWeight', 'bold');

% ── 7. Figura 4 — Norma de aceleración cartesiana ────────────────────────────
figure(4); clf;
set(gcf, 'Color', 'w', 'Position', [860 100 760 390]);
hold on;
hL4 = plot(tL, accnL, '--', 'Color', c_lin, 'LineWidth', lw);
hS4 = plot(tS, accnS, '-',  'Color', c_spl, 'LineWidth', lw);
for k = 1:5
    plot(tL(kpL(k)), accnL(kpL(k)), kp_mk{k}, 'MarkerSize', 7, ...
        'MarkerFaceColor', kp_col{k}, 'MarkerEdgeColor', 'k', 'Color', c_lin);
    plot(tS(kpS(k)), accnS(kpS(k)), kp_mk{k}, 'MarkerSize', 7, ...
        'MarkerFaceColor', kp_col{k}, 'MarkerEdgeColor', 'k', 'Color', c_spl);
end
add_kp_lines(gca, tS, kpS, kp_col, kp_sty);
ylabel('$\|\mathbf{a}\|\ [\mathrm{m/s}^2]$', 'Interpreter', 'latex', 'FontSize', fs);
xlabel('Tiempo [s]', 'FontSize', fs);
grid on; box on;
set(gca, 'FontSize', fs);
xlim([min(tL(1), tS(1)),  max(tL(end), tS(end))]);
legend([hL4, hS4], {'Lineal por tramos', 'Spline cúbico'}, ...
    'Location', 'best', 'FontSize', fs-1);
title('Norma aceleración cartesiana TCP — Comparación', 'FontSize', 14, 'FontWeight', 'bold');

% ── 8. Exportación PNG ────────────────────────────────────────────────────────
if EXPORT_PNG
    out_dir = fullfile(data_dir, 'plots', 'comparison');
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end
    exportgraphics(figure(1), fullfile(out_dir, 'comp_trayectoria_3D.png'),       'Resolution', 300);
    exportgraphics(figure(2), fullfile(out_dir, 'comp_posicion_cartesiana.png'),   'Resolution', 300);
    exportgraphics(figure(3), fullfile(out_dir, 'comp_velocidad_norma.png'),       'Resolution', 300);
    exportgraphics(figure(4), fullfile(out_dir, 'comp_aceleracion_norma.png'),     'Resolution', 300);
    exportgraphics(figure(1), fullfile(out_dir, 'comp_trayectoria_3D.eps'),       'ContentType','vector','Resolution',600);
    exportgraphics(figure(2), fullfile(out_dir, 'comp_posicion_cartesiana.eps'),   'ContentType','vector','Resolution',600);
    exportgraphics(figure(3), fullfile(out_dir, 'comp_velocidad_norma.eps'),       'ContentType','vector','Resolution',600);
    exportgraphics(figure(4), fullfile(out_dir, 'comp_aceleracion_norma.eps'),     'ContentType','vector','Resolution',600);
    fprintf('Figuras exportadas en: %s\n', out_dir);
end

% ── Función local ─────────────────────────────────────────────────────────────
function add_kp_lines(ax, t, kp_idx, kp_col, kp_sty)
    for k = 1:numel(kp_idx)
        xline(ax, t(kp_idx(k)), kp_sty{k}, 'Color', kp_col{k}, 'LineWidth', 0.9);
    end
end
