%% plots_trajectory.m
% Graficas de la trayectoria pick-and-place del UR5.
%
%   Figura 1 — Trayectoria 3D del TCP: keypoints A/B/O/C/D y obstáculo
%   Figura 2 — Posición cartesiana:    x(t), y(t), z(t)           [subplot 3×1]
%   Figura 3 — Velocidad cartesiana:   vx(t), vy(t), vz(t)        [subplot 3×1]
%   Figura 4 — Aceleración cartesiana: ax(t), ay(t), az(t)        [subplot 3×1]
%   Figura 5 — Posiciones articulares: q0..q5 vs t                [subplot 6×1]
%   Figura 6 — Velocidades articulares: dq0..dq5 vs t             [subplot 6×1]
%
% El CSV más reciente de ~/ur5_ws/src/ur5_pick_place/data/ se carga automáticamente.
% Para usar un archivo específico, asigna su nombre completo en FILE_OVERRIDE.

clear; clc; close all;

% ── Configuración ─────────────────────────────────────────────────────────────
FILE_OVERRIDE = '';          % '' = auto-detección del más reciente

% Obstáculo: caja 3D de la mesa_bidones en frame Pinocchio
%   Gazebo pose (x=0.30, y=0, z=0.29)  →  z_pinocchio = 0.29 - 0.63 = -0.34
OBS_CX = 0.65;   OBS_CY = 0.00;   OBS_ZT = -0.375;
OBS_DX = 0.15;   OBS_DY = 0.15;                    % semi-dimensiones [m]
OBS_H  = 0.125;                                     % altura de la caja [m]

% Exportar PNGs en data/plots/  (true/false)
EXPORT_PNG = false;

% Posiciones de referencia de los keypoints [x, y, z] en frame Pinocchio.
% Deben coincidir con los valores en pick_place_params.yaml.
KP_REF = [ 0.10,  0.70, -0.54;   % A  (point_A)
            0.75,  0.38, -0.5;   % B  (point_B)
            0.65,  0.00, -0.32;   % O  (point_O)
            0.75, -0.48, -0.5;   % C  (point_C)
            0.10, -0.70, -0.54];  % D  (point_D)
KP_NAMES = {'A','B','O','C','D'};

% ── 1. Carga del CSV ──────────────────────────────────────────────────────────
data_dir = fullfile(getenv('HOME'), 'ur5_ws', 'src', 'ur5_utec','ur5_pick_place', 'data');

if isempty(FILE_OVERRIDE)
    files = dir(fullfile(data_dir, 'trajectory_20260621_094326_clamped_spline.csv'));
    if isempty(files)
        error('No se encontró ningún CSV en %s.\nEjecutar pick_place_node primero.', ...
              data_dir);
    end
    [~, idx] = max([files.datenum]);
    csv_name = files(idx).name;
else
    csv_name = FILE_OVERRIDE;
end

filepath = fullfile(data_dir, csv_name);
if ~isfile(filepath)
    error('Archivo no encontrado: %s', filepath);
end

T = readtable(filepath);
fprintf('Cargado: %s  (%d muestras)\n', csv_name, height(T));

% ── 2. Extraer columnas ───────────────────────────────────────────────────────
t   = T.time_s;
x   = T.tcp_x;
y   = T.tcp_y;
z   = T.tcp_z;
wpt = T.waypoint;
%   0 = interpolado
%   1 = A   (aproximación sobre pick)
%   2 = B   (pick)
%   3 = O   (sobre obstáculo)
%   4 = C   (place)
%   5 = D   (retirada sobre place)

% Velocidades y aceleraciones cartesianas
vx = T.vel_x;   vy = T.vel_y;   vz = T.vel_z;
ax = T.acc_x;   ay = T.acc_y;   az = T.acc_z;

% Posiciones articulares [rad]
Q  = [T.q0, T.q1, T.q2, T.q3, T.q4, T.q5];

% Velocidades articulares [rad/s]
DQ = [T.dq0, T.dq1, T.dq2, T.dq3, T.dq4, T.dq5];

% Índices de keypoints
idxPre  = find(wpt == 1, 1);
idxA    = find(wpt == 2, 1);
idxVia  = find(wpt == 3, 1);
idxB    = find(wpt == 4, 1);
idxPost = find(wpt == 5, 1);

% Método a partir del nombre de archivo
name_noext   = strrep(csv_name, '.csv', '');
parts        = strsplit(name_noext, '_');
method_key   = strjoin(parts(4:end), '_');
method_label = strrep(method_key, '_', ' ');

% ── 3. Métricas ───────────────────────────────────────────────────────────────
kp_idx   = [idxPre, idxA, idxVia, idxB, idxPost];
speed    = sqrt(vx.^2 + vy.^2 + vz.^2);
accel_n  = sqrt(ax.^2 + ay.^2 + az.^2);

% 3a. Error en puntos de paso: distancia entre TCP muestreado y referencia YAML
err_kp = zeros(5, 1);
for k = 1:5
    tcp_k   = [x(kp_idx(k)), y(kp_idx(k)), z(kp_idx(k))];
    err_kp(k) = norm(tcp_k - KP_REF(k,:));
end

% 3b. Longitud de trayectoria (suma de segmentos euclídeos)
arc_len = sum(sqrt(diff(x).^2 + diff(y).^2 + diff(z).^2));

% 3c. Velocidad y aceleración máximas del TCP
v_max = max(speed);
a_max = max(accel_n);

% 3d. Cambio de velocidad (vector) en cada nodo
delta_v = zeros(5, 1);
for k = 1:5
    i_lo = max(kp_idx(k) - 1, 1);
    i_hi = min(kp_idx(k) + 1, length(t));
    delta_v(k) = norm([vx(i_hi)-vx(i_lo), vy(i_hi)-vy(i_lo), vz(i_hi)-vz(i_lo)]);
end

% ── Imprimir tabla ────────────────────────────────────────────────────────────
fprintf('\n%s\n', repmat('═', 1, 52));
fprintf('  Métricas — %s\n', method_label);
fprintf('%s\n', repmat('═', 1, 52));
fprintf('  Error en puntos de paso:\n');
for k = 1:5
    fprintf('    %s : %8.4f m\n', KP_NAMES{k}, err_kp(k));
end
fprintf('    Máximo                     : %8.4f m\n', max(err_kp));
fprintf('  Longitud de trayectoria      : %8.4f m\n', arc_len);
fprintf('  Velocidad máxima del TCP     : %8.4f m/s\n', v_max);
fprintf('  Aceleración máxima del TCP   : %8.4f m/s²\n', a_max);
fprintf('  Cambio de velocidad en nodos:\n');
[dv_max, dv_k] = max(delta_v);
for k = 1:5
    fprintf('    %s : %8.4f m/s\n', KP_NAMES{k}, delta_v(k));
end
fprintf('    Máximo (en %s)              : %8.4f m/s\n', KP_NAMES{dv_k}, dv_max);
fprintf('%s\n\n', repmat('═', 1, 52));

% ── 5. Paleta de colores (coherente con plots_IO_control.m) ──────────────────
lw       = 1.6;
fs       = 11;
c_traj   = [0.0000 0.4470 0.7410];   % azul         — trayectoria
c_pre    = [0.4660 0.8740 0.1880];   % verde claro  — A
c_A      = [0.1660 0.6740 0.1880];   % verde        — B (pick)
c_via    = [0.4940 0.1840 0.5560];   % morado       — O (via)
c_B      = [0.8500 0.3250 0.0980];   % naranja      — C (place)
c_post   = [1.0000 0.6500 0.3000];   % naranja claro— D
c_obs    = [0.6350 0.0780 0.1840];   % rojo         — obstáculo

% Colores por articulación (tableau-10)
c_joints = [0.1216 0.4667 0.7059;   % azul   — q0
            1.0000 0.4980 0.0549;   % naranja — q1
            0.1725 0.6275 0.1725;   % verde   — q2
            0.8392 0.1529 0.1569;   % rojo    — q3
            0.5804 0.4039 0.7412;   % violeta — q4
            0.5490 0.3373 0.2941];  % marrón  — q5

% ── 4. Figura 1 — Trayectoria 3D ─────────────────────────────────────────────
figure(1); clf;
set(gcf, 'Color', 'w', 'Position', [100 200 860 660]);

% Caja 3D del obstáculo
xL = OBS_CX - OBS_DX;  xR = OBS_CX + OBS_DX;
yF = OBS_CY - OBS_DY;  yBk = OBS_CY + OBS_DY;
zB = OBS_ZT - OBS_H;   zT  = OBS_ZT;

obs_V = [xL yF  zB;   % 1 — inferior-frontal-izq
         xR yF  zB;   % 2 — inferior-frontal-der
         xR yBk zB;   % 3 — inferior-trasero-der
         xL yBk zB;   % 4 — inferior-trasero-izq
         xL yF  zT;   % 5 — superior-frontal-izq
         xR yF  zT;   % 6 — superior-frontal-der
         xR yBk zT;   % 7 — superior-trasero-der
         xL yBk zT];  % 8 — superior-trasero-izq

obs_F = [1 2 3 4;    % cara inferior
         5 6 7 8;    % cara superior
         1 2 6 5;    % cara frontal
         4 3 7 8;    % cara trasera
         1 4 8 5;    % cara izquierda
         2 3 7 6];   % cara derecha

hObs = patch('Vertices', obs_V, 'Faces', obs_F, ...
             'FaceColor', c_obs, 'FaceAlpha', 0.18, ...
             'EdgeColor', c_obs, 'LineWidth', 1.2);
hold on;

hTraj = plot3(x, y, z, '-', 'Color', c_traj, 'LineWidth', lw);

hPre  = plot3(x(idxPre),  y(idxPre),  z(idxPre),  'v', 'MarkerSize', 10, ...
              'MarkerFaceColor', c_pre,  'MarkerEdgeColor', 'k', 'LineWidth', 1.2);
hA    = plot3(x(idxA),    y(idxA),    z(idxA),    'o', 'MarkerSize', 10, ...
              'MarkerFaceColor', c_A,    'MarkerEdgeColor', 'k', 'LineWidth', 1.2);
hVia  = plot3(x(idxVia),  y(idxVia),  z(idxVia),  's', 'MarkerSize', 10, ...
              'MarkerFaceColor', c_via,  'MarkerEdgeColor', 'k', 'LineWidth', 1.2);
hB    = plot3(x(idxB),    y(idxB),    z(idxB),    '^', 'MarkerSize', 10, ...
              'MarkerFaceColor', c_B,    'MarkerEdgeColor', 'k', 'LineWidth', 1.2);
hPost = plot3(x(idxPost), y(idxPost), z(idxPost), 'd', 'MarkerSize', 10, ...
              'MarkerFaceColor', c_post, 'MarkerEdgeColor', 'k', 'LineWidth', 1.2);

plot3([x(idxPre) x(idxA)],   [y(idxPre) y(idxA)],   [z(idxPre) z(idxA)],   ...
      ':', 'Color', [c_A   0.5], 'LineWidth', 1.0);
plot3([x(idxB)   x(idxPost)],[y(idxB)   y(idxPost)],[z(idxB)   z(idxPost)], ...
      ':', 'Color', [c_B   0.5], 'LineWidth', 1.0);

text(x(idxPre)+0.02,  y(idxPre)+0.03,  z(idxPre)+0.01,  'A', ...
     'FontSize', fs, 'FontWeight', 'bold', 'Color', c_pre);
text(x(idxA)+0.02,    y(idxA)+0.03,    z(idxA)+0.01,    'B', ...
     'FontSize', fs, 'FontWeight', 'bold', 'Color', c_A);
text(x(idxVia)+0.02,  y(idxVia)+0.03,  z(idxVia)+0.02,  'O', ...
     'FontSize', fs, 'FontWeight', 'bold', 'Color', c_via);
text(x(idxB)+0.02,    y(idxB)-0.09,    z(idxB)+0.01,    'C', ...
     'FontSize', fs, 'FontWeight', 'bold', 'Color', c_B);
text(x(idxPost)+0.02, y(idxPost)-0.09, z(idxPost)+0.01, 'D', ...
     'FontSize', fs, 'FontWeight', 'bold', 'Color', c_post);
text(OBS_CX+OBS_DX+0.02, OBS_CY, OBS_ZT+0.015, 'Obstáculo', ...
     'FontSize', fs-1, 'FontWeight', 'bold', 'Color', c_obs, 'Clipping', 'on');

xlabel('x [m]', 'FontSize', fs);
ylabel('y [m]', 'FontSize', fs);
zlabel('z [m]', 'FontSize', fs);
grid on; box on;
set(gca, 'FontSize', fs);
view(45, 25);

legend([hTraj, hPre, hA, hVia, hB, hPost, hObs], ...
       {'Trayectoria TCP', 'A', 'B', 'O', 'C', 'D', 'Mesa (obstáculo)'}, ...
       'Location', 'best', 'FontSize', fs-1);

title(sprintf('Trayectoria 3D TCP — %s', method_label), ...
      'FontSize', 14, 'FontWeight', 'bold');

% ── Helper local para figuras de subplots 3×1 con keypoints ──────────────────
function [ax_all, h_lgd] = make_subplot3(fig_num, data3, labels3, title_str, method_label, ...
                               t, idxPre, idxA, idxVia, idxB, idxPost, ...
                               c_traj, c_pre, c_A, c_via, c_B, c_post, lw, fs)
    figure(fig_num); clf;
    set(gcf, 'Color', 'w', 'Position', [1000 200 720 640]);
    tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax_all = gobjects(3, 1);
    h_lgd  = gobjects(6, 1);
    xlims  = [t(1), t(end)];

    for i = 1:3
        ax_all(i) = nexttile(tl);
        hold on;

        hLine = plot(t, data3{i}, '-', 'Color', c_traj, 'LineWidth', lw);

        xline(t(idxPre),  ':', 'Color', c_pre,  'LineWidth', 1.0);
        xline(t(idxA),    '--','Color', c_A,    'LineWidth', 1.0);
        xline(t(idxVia),  '--','Color', c_via,  'LineWidth', 1.0);
        xline(t(idxB),    '--','Color', c_B,    'LineWidth', 1.0);
        xline(t(idxPost), ':', 'Color', c_post, 'LineWidth', 1.0);

        hPre_pt  = plot(t(idxPre),  data3{i}(idxPre),  'v','MarkerSize',7, ...
                        'MarkerFaceColor',c_pre, 'MarkerEdgeColor','k');
        hA_pt    = plot(t(idxA),    data3{i}(idxA),    'o','MarkerSize',7, ...
                        'MarkerFaceColor',c_A,   'MarkerEdgeColor','k');
        hVia_pt  = plot(t(idxVia),  data3{i}(idxVia),  's','MarkerSize',7, ...
                        'MarkerFaceColor',c_via, 'MarkerEdgeColor','k');
        hB_pt    = plot(t(idxB),    data3{i}(idxB),    '^','MarkerSize',7, ...
                        'MarkerFaceColor',c_B,   'MarkerEdgeColor','k');
        hPost_pt = plot(t(idxPost), data3{i}(idxPost), 'd','MarkerSize',7, ...
                        'MarkerFaceColor',c_post,'MarkerEdgeColor','k');

        if i == 1
            h_lgd = [hLine, hPre_pt, hA_pt, hVia_pt, hB_pt, hPost_pt];
        end

        ylabel(labels3{i}, 'Interpreter', 'latex', 'FontSize', fs);
        grid on; box on;
        set(ax_all(i), 'FontSize', fs);
        xlim(xlims);

        if i < 3
            set(ax_all(i), 'XTickLabel', []);
        end
    end

    xlabel('Tiempo [s]', 'FontSize', fs);

    lgd = legend(ax_all(1), h_lgd, ...
                 {'señal', 'A', 'B', 'O', 'C', 'D'}, ...
                 'Orientation', 'horizontal', 'FontSize', fs-1);
    lgd.Layout.Tile = 'north';

    title(tl, sprintf('%s — %s', title_str, method_label), ...
          'FontSize', 14, 'FontWeight', 'bold');
end

% ── 5. Figura 2 — Posición cartesiana ────────────────────────────────────────
[~, ~] = make_subplot3(2, {x, y, z}, ...
    {'$x$ [m]', '$y$ [m]', '$z$ [m]'}, 'Posición TCP', method_label, ...
    t, idxPre, idxA, idxVia, idxB, idxPost, ...
    c_traj, c_pre, c_A, c_via, c_B, c_post, lw, fs);

% ── 6. Figura 3 — Velocidad cartesiana ───────────────────────────────────────
[~, ~] = make_subplot3(3, {vx, vy, vz}, ...
    {'$\dot{x}$ [m/s]', '$\dot{y}$ [m/s]', '$\dot{z}$ [m/s]'}, 'Velocidad TCP', method_label, ...
    t, idxPre, idxA, idxVia, idxB, idxPost, ...
    c_traj, c_pre, c_A, c_via, c_B, c_post, lw, fs);

% ── 7. Figura 4 — Aceleración cartesiana ─────────────────────────────────────
[~, ~] = make_subplot3(4, {ax, ay, az}, ...
    {'$\ddot{x}\ [\mathrm{m/s}^2]$', '$\ddot{y}\ [\mathrm{m/s}^2]$', '$\ddot{z}\ [\mathrm{m/s}^2]$'}, 'Aceleración TCP', method_label, ...
    t, idxPre, idxA, idxVia, idxB, idxPost, ...
    c_traj, c_pre, c_A, c_via, c_B, c_post, lw, fs);

% ── 8. Figura 5 — Posiciones articulares ─────────────────────────────────────
joint_labels_q  = {'$q_0$ [rad]','$q_1$ [rad]','$q_2$ [rad]', ...
                   '$q_3$ [rad]','$q_4$ [rad]','$q_5$ [rad]'};
joint_names     = {'pan','lift','elbow','wrist 1','wrist 2','wrist 3'};

figure(5); clf;
set(gcf, 'Color', 'w', 'Position', [100 100 760 900]);
tl5 = tiledlayout(6, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
xlims = [t(1), t(end)];

for j = 1:6
    nexttile(tl5);
    hold on;

    plot(t, Q(:,j), '-', 'Color', c_joints(j,:), 'LineWidth', lw);

    xline(t(idxPre),  ':', 'Color', c_pre,  'LineWidth', 0.9);
    xline(t(idxA),    '--','Color', c_A,    'LineWidth', 0.9);
    xline(t(idxVia),  '--','Color', c_via,  'LineWidth', 0.9);
    xline(t(idxB),    '--','Color', c_B,    'LineWidth', 0.9);
    xline(t(idxPost), ':', 'Color', c_post, 'LineWidth', 0.9);

    plot(t(idxPre),  Q(idxPre, j),  'v','MarkerSize',6,'MarkerFaceColor',c_pre, 'MarkerEdgeColor','k');
    plot(t(idxA),    Q(idxA,   j),  'o','MarkerSize',6,'MarkerFaceColor',c_A,   'MarkerEdgeColor','k');
    plot(t(idxVia),  Q(idxVia, j),  's','MarkerSize',6,'MarkerFaceColor',c_via, 'MarkerEdgeColor','k');
    plot(t(idxB),    Q(idxB,   j),  '^','MarkerSize',6,'MarkerFaceColor',c_B,   'MarkerEdgeColor','k');
    plot(t(idxPost), Q(idxPost,j),  'd','MarkerSize',6,'MarkerFaceColor',c_post,'MarkerEdgeColor','k');

    ylabel(joint_labels_q{j}, 'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);

    if j < 6
        set(gca, 'XTickLabel', []);
    end
end

xlabel('Tiempo [s]', 'FontSize', fs);
title(tl5, sprintf('Posiciones articulares — %s', method_label), ...
      'FontSize', 14, 'FontWeight', 'bold');

% ── 9. Figura 6 — Velocidades articulares ────────────────────────────────────
joint_labels_dq = {'$\dot{q}_0$ [rad/s]','$\dot{q}_1$ [rad/s]','$\dot{q}_2$ [rad/s]', ...
                   '$\dot{q}_3$ [rad/s]','$\dot{q}_4$ [rad/s]','$\dot{q}_5$ [rad/s]'};

figure(6); clf;
set(gcf, 'Color', 'w', 'Position', [900 100 760 900]);
tl6 = tiledlayout(6, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

for j = 1:6
    nexttile(tl6);
    hold on;

    plot(t, DQ(:,j), '-', 'Color', c_joints(j,:), 'LineWidth', lw);

    xline(t(idxPre),  ':', 'Color', c_pre,  'LineWidth', 0.9);
    xline(t(idxA),    '--','Color', c_A,    'LineWidth', 0.9);
    xline(t(idxVia),  '--','Color', c_via,  'LineWidth', 0.9);
    xline(t(idxB),    '--','Color', c_B,    'LineWidth', 0.9);
    xline(t(idxPost), ':', 'Color', c_post, 'LineWidth', 0.9);

    plot(t(idxPre),  DQ(idxPre, j),  'v','MarkerSize',6,'MarkerFaceColor',c_pre, 'MarkerEdgeColor','k');
    plot(t(idxA),    DQ(idxA,   j),  'o','MarkerSize',6,'MarkerFaceColor',c_A,   'MarkerEdgeColor','k');
    plot(t(idxVia),  DQ(idxVia, j),  's','MarkerSize',6,'MarkerFaceColor',c_via, 'MarkerEdgeColor','k');
    plot(t(idxB),    DQ(idxB,   j),  '^','MarkerSize',6,'MarkerFaceColor',c_B,   'MarkerEdgeColor','k');
    plot(t(idxPost), DQ(idxPost,j),  'd','MarkerSize',6,'MarkerFaceColor',c_post,'MarkerEdgeColor','k');

    ylabel(joint_labels_dq{j}, 'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);

    if j < 6
        set(gca, 'XTickLabel', []);
    end
end

xlabel('Tiempo [s]', 'FontSize', fs);
title(tl6, sprintf('Velocidades articulares — %s', method_label), ...
      'FontSize', 14, 'FontWeight', 'bold');

% ── 10. Exportación PNG (opcional) ────────────────────────────────────────────
if EXPORT_PNG
    out_dir = fullfile(data_dir, 'plots', method_key);
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    exportgraphics(figure(1), fullfile(out_dir, 'trayectoria_3D.png'),         'Resolution', 300);
    exportgraphics(figure(2), fullfile(out_dir, 'posicion_cartesiana.png'),     'Resolution', 300);
    exportgraphics(figure(3), fullfile(out_dir, 'velocidad_cartesiana.png'),    'Resolution', 300);
    exportgraphics(figure(4), fullfile(out_dir, 'aceleracion_cartesiana.png'),  'Resolution', 300);
    exportgraphics(figure(5), fullfile(out_dir, 'posiciones_articulares.png'),  'Resolution', 300);
    exportgraphics(figure(6), fullfile(out_dir, 'velocidades_articulares.png'), 'Resolution', 300);

    % Exportar figuras en EPS vectorial
    exportgraphics(figure(1), fullfile(out_dir, 'trayectoria_3D.eps'),         'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(2), fullfile(out_dir, 'posicion_cartesiana.eps'),     'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(3), fullfile(out_dir, 'velocidad_cartesiana.eps'),    'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(4), fullfile(out_dir, 'aceleracion_cartesiana.eps'),  'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(5), fullfile(out_dir, 'posiciones_articulares.eps'),  'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(6), fullfile(out_dir, 'velocidades_articulares.eps'), 'ContentType', 'vector', 'Resolution', 600);

    fprintf('Figuras exportadas en: %s\n', out_dir);
end
