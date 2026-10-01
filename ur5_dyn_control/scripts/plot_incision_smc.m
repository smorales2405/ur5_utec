function R = plot_incision_smc(varargin)
%% plot_incision_smc.m
% Graficas de una corrida de INCISION del SMC (Gazebo o UR5e real), a partir
% del CSV unificado de ur5_dyn_control (~/.ros/ur5_dyn_control/smc_<n>.csv).
%
% Uso:
%   plot_incision_smc                          % el smc_*.csv mas reciente
%   plot_incision_smc('smc_605.csv')           % se busca en ~/.ros/ur5_dyn_control
%   plot_incision_smc(ruta, 'Guardar', true)   % ademas guarda PNG 300 dpi + .fig
%   R = plot_incision_smc(...)                 % devuelve las metricas
%
% Figuras generadas:
%   Fig 1 — Trayectoria 3D del TCP: deseada vs simulada, con los puntos del
%           corte (entrada al tejido, inicio y fin del tramo a profundidad,
%           salida) y el plano de la superficie. Vista global y detalle.
%   Fig 2 — Posicion x(t), y(t), z(t): deseada vs simulada
%   Fig 3 — Orientacion roll/pitch/yaw: deseada vs simulada
%   Fig 4 — Errores: posicion del TCP, orientacion (inclinacion de la hoja y
%           giro alrededor de su eje) y articulares
%   Fig 5 — Torques articulares comandados
%
% De donde sale cada cosa:
%   El CSV trae la POSICION del TCP (x, y, z y sus _des), calculada por el nodo,
%   pero de la ORIENTACION solo el angulo theta_err = ||log(R_des' R)||. La
%   orientacion se calcula aqui por cinematica directa desde q y q_des, con la
%   cadena del URDF del UR5e (ur5_kinematics/urdf/ur5e.urdf). Como comprobacion,
%   la posicion que sale de esa cinematica se compara con la del CSV y se avisa
%   si difieren en mas de 0.01 mm: si la cadena o el offset del TCP no fuesen los
%   del nodo, las graficas de orientacion no serian fiables.
%
%   Los errores de orientacion se dan con el VECTOR DE ROTACION de R_des' R, no
%   con diferencias de angulos de Euler: esos saltan +-180 grados y no sirven
%   como metrica (mismo criterio que el nodo para theta_err).
%
% Opciones (nombre, valor):
%   'TcpOffsetZ'  0.162686  [m] tool0 -> punta de la hoja (smc_params.yaml)
%   'SurfaceZ'    0.03      [m] superficie del tejido (incision.surface_z)
%   'Guardar'     false     PNG 300 dpi + .fig en <carpeta del CSV>/plots/smc_<n>/
%   'Visible'     true
%
% Notas de interpretacion:
%   - El tiempo es relativo al inicio de TRACK: se ve la RAMPA previa (tiempos
%     negativos) y 1 s de HOLD_END.
%   - La franja sombreada es el CORTE: la punta a la profundidad de corte (z_des
%     en su minimo). Mismo criterio que analyze_smc.py, asi que el RMSE de TCP
%     que se imprime es comparable con el suyo.
%   - En Gazebo, wrist_3 esta congelada por el artefacto b*dt/I (docs/05_smc.md
%     §7.5). Su eje es el de la herramienta: no mueve la punta, pero gira la
%     hoja ~10 grados alrededor de su eje. Ese giro y el error de wrist_3 son del
%     simulador, no del controlador. En el robot real si serian reales.
%   - El torque es el COMANDADO. En Gazebo incluye la gravedad; en el robot real
%     no (compuerta G3: la compensa el propio robot).
%   - En Gazebo no hay tejido ni fuerza de corte: la hoja recorre el plano de
%     corte sin tocar nada (docs/07_gain_tuning.md §5.8.4).

% ═══════════════════════════════════════════════════════════════════════
%  OPCIONES Y DATOS
% ═══════════════════════════════════════════════════════════════════════
p = inputParser;
p.addOptional('csv', '', @(s) ischar(s) || isstring(s));
p.addParameter('TcpOffsetZ', 0.162686, @(x) isnumeric(x) && isscalar(x));
p.addParameter('SurfaceZ', 0.03, @(x) isnumeric(x) && isscalar(x));
p.addParameter('Guardar', false, @(x) islogical(x) || isnumeric(x));
p.addParameter('Visible', true, @(x) islogical(x) || isnumeric(x));
p.parse(varargin{:});
op = p.Results;

dataDir = fullfile(getenv('HOME'), '.ros', 'ur5_dyn_control');
csv = char(op.csv);
if isempty(csv)
    L = dir(fullfile(dataDir, 'smc_*.csv'));
    if isempty(L)
        error('No hay ningun smc_*.csv en %s', dataDir);
    end
    [~, k] = max([L.datenum]);
    csv = fullfile(L(k).folder, L(k).name);
elseif ~isfile(csv) && isfile(fullfile(dataDir, csv))
    csv = fullfile(dataDir, csv);
end
if ~isfile(csv)
    error('No existe el CSV: %s', csv);
end

meta = leerMeta(csv);
fprintf('\n=== %s  (test %s, git %s) ===\n', csv, meta.test_num, meta.git_sha);

opts = detectImportOptions(csv, 'CommentStyle', '#', 'Delimiter', ',');
opts = setvartype(opts, 'state', 'string');
T = readtable(csv, opts);

st = string(T.state);
iTrack = find(siguiendo(st));
if isempty(iTrack)
    error('El CSV no tiene fase de seguimiento (TRACK): %s', csv);
end
tAll = T.t_sim - T.t_sim(iTrack(1));
i0 = min([find(st == "RAMP", 1, 'first'); iTrack(1)]);
iEnd = find(tAll <= tAll(iTrack(end)) + 1.0, 1, 'last');
keep = (i0:iEnd)';
if any(st(keep) == "SAFE_HOLD")
    warning('La corrida entro en SAFE_HOLD (t = %.2f s): revisa el log del watchdog.', ...
            tAll(keep(find(st(keep) == "SAFE_HOLD", 1))));
end

t   = tAll(keep);
st  = st(keep);
Q   = T{keep, nombres('q%d')};
Qd  = T{keep, nombres('q%d_des')};
TAU = T{keep, nombres('tau%d')};
SAT = T{keep, nombres('tau%d_sat')};
P   = T{keep, {'x', 'y', 'z'}};
Pd  = T{keep, {'x_des', 'y_des', 'z_des'}};
thCsv = T.theta_err(keep);
esSim = T.t_sim(1) < 1e6;     % Gazebo empieza en ~0; el real en tiempo de pared

% ═══════════════════════════════════════════════════════════════════════
%  CINEMATICA DIRECTA (orientacion) Y COMPROBACION CONTRA EL CSV
% ═══════════════════════════════════════════════════════════════════════
[Pfk,  Ract] = fkUr5e(Q,  op.TcpOffsetZ);
[Pfkd, Rdes] = fkUr5e(Qd, op.TcpOffsetZ);
dfk = max([vecnorm(Pfk - P, 2, 2); vecnorm(Pfkd - Pd, 2, 2)]);
fprintf('Cinematica directa vs posicion del CSV: max %.4f mm\n', 1e3 * dfk);
if dfk > 1e-5
    warning(['La posicion por cinematica directa difiere %.3f mm de la del CSV. ' ...
             'Revisa ''TcpOffsetZ'' (tcp_offset_z del YAML): la orientacion ' ...
             'graficada puede no ser la del nodo.'], 1e3 * dfk);
end

n = numel(t);
rpyA = zeros(n, 3);  rpyD = zeros(n, 3);  rv = zeros(n, 3);
for k = 1:n
    rpyA(k, :) = rpyZYX(Ract(:, :, k));
    rpyD(k, :) = rpyZYX(Rdes(:, :, k));
    rv(k, :)   = rotvec(Rdes(:, :, k).' * Ract(:, :, k));   % en el marco del TCP deseado
end
% Roll y yaw valen ~180 grados, justo donde atan2 salta entre +180 y -180. Se
% desenvuelve la DESEADA en el tiempo, se lleva cada angulo a la rama en la que
% su mediana cae en [-90, 270) grados (asi ~180 se ve como 180 y no como -180), y
% la simulada se dibuja en la misma rama que la deseada.
rpyD = unwrap(rpyD, [], 1);
rpyD = rpyD - 2 * pi * floor((median(rpyD, 1) + pi / 2) / (2 * pi));
rpyA = rpyD + envolver(rpyA - rpyD);
thFk = vecnorm(rv, 2, 2);
fprintf('theta_err del CSV vs vector de rotacion: max %.2e rad\n', max(abs(thFk - thCsv)));
inclin = rad2deg(hypot(rv(:, 1), rv(:, 2)));   % la hoja se inclina
giro   = rad2deg(rv(:, 3));                     % gira sobre su propio eje

% ═══════════════════════════════════════════════════════════════════════
%  PUNTOS DEL CORTE
% ═══════════════════════════════════════════════════════════════════════
inT  = siguiendo(st);
zd   = Pd(:, 3);
zmin = min(zd(inT));
esIncision = zmin < op.SurfaceZ;
kT0 = find(inT, 1, 'first');
if esIncision
    aProf = inT & zd <= zmin + 1e-4;           % punta a la profundidad de corte
    kc0 = find(aProf, 1, 'first');   kc1 = find(aProf, 1, 'last');
    enTej = inT & zd < op.SurfaceZ;            % punta dentro del tejido
    ke0 = find(enTej, 1, 'first');   ke1 = find(enTej, 1, 'last');
    tc = [t(kc0), t(kc1)];
else
    warning(['La punta nunca baja de SurfaceZ = %.3f m: no parece una incision ' ...
             '(¿un barrido articular?). Se grafica sin marcar el corte.'], op.SurfaceZ);
    kc0 = []; kc1 = []; ke0 = []; ke1 = []; tc = [];
end

% ═══════════════════════════════════════════════════════════════════════
%  METRICAS (en el corte)
% ═══════════════════════════════════════════════════════════════════════
eP = (P - Pd) * 1e3;                           % [mm]
R = struct('csv', csv, 'test', meta.test_num, 'simulacion', esSim, ...
           'fk_vs_csv_mm', 1e3 * dfk, 'corte_s', tc);
if esIncision
    m = kc0:kc1;
    R.tcp_rmse_mm   = sqrt(mean(sum(eP(m, :).^2, 2)));
    R.tcp_max_mm    = max(vecnorm(eP(m, :), 2, 2));
    R.prof_rmse_mm  = sqrt(mean(eP(m, 3).^2));
    R.inclin_rms_deg = sqrt(mean(inclin(m).^2));
    R.giro_rms_deg   = sqrt(mean(giro(m).^2));
    R.rmse_q_rad    = sqrt(mean((Q(m, :) - Qd(m, :)).^2, 1));
    R.tau_max_Nm    = max(abs(TAU(m, :)), [], 1);
    fprintf('Corte: %.2f .. %.2f s de TRACK\n', tc);
    fprintf('  TCP           RMSE %.3f mm   max %.3f mm\n', R.tcp_rmse_mm, R.tcp_max_mm);
    fprintf('  profundidad   RMSE %.3f mm\n', R.prof_rmse_mm);
    fprintf('  hoja: inclinacion RMS %.3f deg   giro sobre su eje RMS %.2f deg%s\n', ...
            R.inclin_rms_deg, R.giro_rms_deg, ...
            ternario(esSim, '  (el giro es el artefacto de wrist_3 en Gazebo)', ''));
end
R.n_saturados = sum(SAT(inT, :), 1);
if any(R.n_saturados)
    fprintf('  ciclos con el par saturado por junta: %s\n', mat2str(R.n_saturados));
end

% ═══════════════════════════════════════════════════════════════════════
%  FIGURAS
% ═══════════════════════════════════════════════════════════════════════
cDes = [0 0 0];  cSim = [0 0.447 0.741];  cErr = lines(6);
vis = ternario(op.Visible, 'on', 'off');
fuente = ternario(esSim, 'Gazebo', 'UR5e real');
titulo = sprintf('SMC · smc\\_%s · %s', meta.test_num, fuente);
nota = 'franja: corte (punta a la profundidad de corte) · t = 0: inicio de TRACK';
figs = gobjects(0);  nomFig = {};

% ── Fig 1: trayectoria 3D ──────────────────────────────────────────────
f = figure('Name', 'Trayectoria 3D', 'Color', 'w', 'Visible', vis, ...
           'Position', [60 60 1400 600]);
tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for vista = 1:2
    ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');
    if vista == 1
        s = 1;  u = 'm';  sel = true(n, 1);
    else
        s = 1e3;  u = 'mm';
        sel = inT & zd < op.SurfaceZ + 0.02;   % el corte y 2 cm por encima
        if ~any(sel), sel = inT; end
    end
    plot3(ax, s*Pd(sel,1), s*Pd(sel,2), s*Pd(sel,3), '--', 'Color', cDes, ...
          'LineWidth', 1.3, 'DisplayName', 'deseada');
    plot3(ax, s*P(sel,1), s*P(sel,2), s*P(sel,3), '-', 'Color', cSim, ...
          'LineWidth', 1.3, 'DisplayName', ternario(esSim, 'simulada', 'medida'));
    if esIncision
        plano(ax, s*Pd(ke0:ke1, :), s*op.SurfaceZ, s*0.02);
        % Color FIJO por tipo de punto: el panel de detalle no lleva leyenda y
        % tiene que leerse con la del panel global.
        mk = {kT0, 'o', 'inicio de TRACK',   [0.93 0.69 0.13];
              ke0, 'v', 'entrada al tejido', [0.49 0.18 0.56];
              kc0, '>', 'inicio del corte',  [0.47 0.67 0.19];
              kc1, 's', 'fin del corte',     [0.30 0.75 0.93];
              ke1, '^', 'salida del tejido', [0.85 0.10 0.55]};
        if vista == 2, mk(1, :) = []; end
        for j = 1:size(mk, 1)
            kk = mk{j, 1};
            plot3(ax, s*Pd(kk,1), s*Pd(kk,2), s*Pd(kk,3), mk{j, 2}, ...
                  'MarkerSize', 9, 'MarkerFaceColor', mk{j, 4}, ...
                  'MarkerEdgeColor', 'k', 'DisplayName', mk{j, 3});
        end
    end
    xlabel(ax, ['x [' u ']']);  ylabel(ax, ['y [' u ']']);  zlabel(ax, ['z [' u ']']);
    view(ax, -35, 25);
    if vista == 1
        axis(ax, 'equal');  title(ax, 'Trayectoria del TCP (punta de la hoja)');
        legend(ax, 'Location', 'northeastoutside');
    else
        title(ax, 'Detalle del corte (ejes sin escalar por igual)');
    end
end
title(tl, titulo, 'FontWeight', 'bold');
figs(end+1) = f;  nomFig{end+1} = '01_trayectoria_3d';

% ── Fig 2: x, y, z ─────────────────────────────────────────────────────
f = figure('Name', 'Posicion', 'Color', 'w', 'Visible', vis, 'Position', [80 80 1100 800]);
tl = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ejes = {'x', 'y', 'z'};
for j = 1:3
    ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');
    sombrear(ax, tc);
    plot(ax, t, Pd(:, j), '--', 'Color', cDes, 'LineWidth', 1.3, 'DisplayName', 'deseada');
    plot(ax, t, P(:, j), '-', 'Color', cSim, 'LineWidth', 1.1, ...
         'DisplayName', ternario(esSim, 'simulada', 'medida'));
    if j == 3 && esIncision
        yline(ax, op.SurfaceZ, ':', 'superficie del tejido', 'HandleVisibility', 'off', ...
              'LabelHorizontalAlignment', 'left');
        yline(ax, zmin, ':', 'profundidad de corte', 'HandleVisibility', 'off', ...
              'LabelHorizontalAlignment', 'right', 'LabelVerticalAlignment', 'bottom');
    end
    ylabel(ax, [ejes{j} ' [m]']);
    if j == 1, legend(ax, 'Location', 'best'); end
end
xlabel(ax, 't [s]');
title(tl, {titulo, nota}, 'FontWeight', 'bold');
figs(end+1) = f;  nomFig{end+1} = '02_posicion_xyz';

% ── Fig 3: orientacion ─────────────────────────────────────────────────
f = figure('Name', 'Orientacion', 'Color', 'w', 'Visible', vis, 'Position', [100 100 1100 800]);
tl = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ang = {'roll (x)', 'pitch (y)', 'yaw (z)'};
for j = 1:3
    ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');
    sombrear(ax, tc);
    plot(ax, t, rad2deg(rpyD(:, j)), '--', 'Color', cDes, 'LineWidth', 1.3, 'DisplayName', 'deseada');
    plot(ax, t, rad2deg(rpyA(:, j)), '-', 'Color', cSim, 'LineWidth', 1.1, ...
         'DisplayName', ternario(esSim, 'simulada', 'medida'));
    ylabel(ax, [ang{j} ' [deg]']);
    if j == 1, legend(ax, 'Location', 'best'); end
    if j == 3 && esSim
        title(ax, ['la diferencia en yaw es el giro de la hoja sobre su eje: ' ...
                   'artefacto de wrist\_3 en Gazebo (05\_smc §7.5)'], 'FontWeight', 'normal');
    end
end
xlabel(ax, 't [s]');
title(tl, {titulo, 'Orientacion del TCP, Euler ZYX (R = R_z(yaw) R_y(pitch) R_x(roll), convenio del URDF)', nota}, ...
      'FontWeight', 'bold');
figs(end+1) = f;  nomFig{end+1} = '03_orientacion';

% ── Fig 4: errores ─────────────────────────────────────────────────────
f = figure('Name', 'Errores', 'Color', 'w', 'Visible', vis, 'Position', [120 120 1400 850]);
tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');  sombrear(ax, tc);
plot(ax, t, eP(:, 1), 'Color', cErr(1, :), 'DisplayName', 'e_x');
plot(ax, t, eP(:, 2), 'Color', cErr(2, :), 'DisplayName', 'e_y');
plot(ax, t, eP(:, 3), 'Color', cErr(3, :), 'DisplayName', 'e_z');
plot(ax, t, vecnorm(eP, 2, 2), 'k', 'LineWidth', 1.3, 'DisplayName', '||e||');
ylabel(ax, 'error de posicion del TCP [mm]');  legend(ax, 'Location', 'best');
if esIncision
    title(ax, sprintf('TCP en el corte: RMSE %.3f mm, max %.3f mm', R.tcp_rmse_mm, R.tcp_max_mm));
end

ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');  sombrear(ax, tc);
plot(ax, t, inclin, 'Color', cErr(4, :), 'LineWidth', 1.2, 'DisplayName', 'inclinacion de la hoja');
plot(ax, t, giro, 'Color', cErr(5, :), 'LineWidth', 1.2, 'DisplayName', 'giro sobre su eje');
ylabel(ax, 'error de orientacion [deg]');  legend(ax, 'Location', 'best');
title(ax, ternario(esSim, ...
      'el giro sobre el eje es el artefacto de wrist\_3 de Gazebo (05\_smc §7.5)', ...
      'vector de rotacion de R_{des}^T R, marco del TCP deseado'));

ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');  sombrear(ax, tc);
jn = {'shoulder\_pan', 'shoulder\_lift', 'elbow', 'wrist\_1', 'wrist\_2', 'wrist\_3'};
for j = 1:5
    plot(ax, t, 1e3 * (Q(:, j) - Qd(:, j)), 'Color', cErr(j, :), 'DisplayName', jn{j});
end
ylabel(ax, 'q - q_{des} [mrad]');  legend(ax, 'Location', 'best', 'NumColumns', 2);
title(ax, 'Error articular (wrist\_3 aparte: su escala es otra)');
xlabel(ax, 't [s]');

ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');  sombrear(ax, tc);
plot(ax, t, 1e3 * (Q(:, 6) - Qd(:, 6)), 'Color', cErr(6, :), 'LineWidth', 1.2);
ylabel(ax, 'q_6 - q_{6,des} [mrad]');
title(ax, ternario(esSim, 'wrist\_3: congelada por el artefacto b·dt/I de Gazebo', 'wrist\_3'));
xlabel(ax, 't [s]');
title(tl, {titulo, nota}, 'FontWeight', 'bold');
figs(end+1) = f;  nomFig{end+1} = '04_errores';

% ── Fig 5: torques ─────────────────────────────────────────────────────
f = figure('Name', 'Torques', 'Color', 'w', 'Visible', vis, 'Position', [140 140 1400 850]);
tl = tiledlayout(f, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for j = 1:6
    ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');  sombrear(ax, tc);
    plot(ax, t, TAU(:, j), 'Color', cSim, 'LineWidth', 1.0);
    ks = SAT(:, j) > 0;
    if any(ks)
        plot(ax, t(ks), TAU(ks, j), 'r.', 'MarkerSize', 6, 'DisplayName', 'saturado');
        legend(ax, 'Location', 'best');
    end
    ylabel(ax, sprintf('\\tau_%d [N·m]', j));
    title(ax, sprintf('%s · max |\\tau| = %.2f N·m', jn{j}, max(abs(TAU(:, j)))));
    if j >= 5, xlabel(ax, 't [s]'); end
end
title(tl, {titulo, ['Par COMANDADO' ternario(esSim, ' (en Gazebo incluye la gravedad)', ...
          ' (sin gravedad: la compensa el robot, G3)')], nota}, 'FontWeight', 'bold');
figs(end+1) = f;  nomFig{end+1} = '05_torques';

% ═══════════════════════════════════════════════════════════════════════
%  GUARDADO
% ═══════════════════════════════════════════════════════════════════════
if op.Guardar
    outDir = fullfile(fileparts(csv), 'plots', sprintf('smc_%s', meta.test_num));
    if ~isfolder(outDir), mkdir(outDir); end
    for j = 1:numel(figs)
        exportgraphics(figs(j), fullfile(outDir, [nomFig{j} '.png']), 'Resolution', 300);
        savefig(figs(j), fullfile(outDir, [nomFig{j} '.fig']));
    end
    fprintf('Figuras guardadas en %s\n', outDir);
    R.carpeta = outDir;
end
end

% ═══════════════════════════════════════════════════════════════════════
%  FUNCIONES LOCALES
% ═══════════════════════════════════════════════════════════════════════
function b = siguiendo(st)
% Fase de SEGUIMIENTO. En la incision el nodo la registra como "TRACK"; en un
% barrido articular, con la etiqueta de cada tramo ("SWEEP_0.050_POS", ...).
b = ~ismember(st, ["PRE_HOLD", "WAIT_STATE", "HOLD_START", "RAMP", ...
                   "HOLD_END", "SAFE_HOLD", "DONE"]);
end

function c = nombres(fmt)
c = arrayfun(@(i) sprintf(fmt, i), 1:6, 'UniformOutput', false);
end

function meta = leerMeta(csv)
% Cabecera '# clave=valor' del CSV unificado.
meta = struct('test_num', '?', 'git_sha', '?');
fid = fopen(csv, 'r');
c = onCleanup(@() fclose(fid));
while true
    l = fgetl(fid);
    if ~ischar(l) || ~startsWith(l, '#'), break; end
    kv = regexp(l, '^#\s*(\w+)=(.*)$', 'tokens', 'once');
    if ~isempty(kv), meta.(kv{1}) = strtrim(kv{2}); end
end
end

function [P, Rm] = fkUr5e(Q, tcp)
% Cinematica directa del UR5e hasta la punta de la hoja, con la cadena EXACTA
% de ur5_kinematics/urdf/ur5e.urdf (origenes xyz/rpy de cada junta) y el TCP
% a `tcp` metros de tool0 en su z local, como Ur5Dynamics. Marco: base_link.
persistent C
if isempty(C)
    C.base   = tf(rpy(0, 0, pi), [0 0 0]);                    % base_link -> base_link_inertia
    C.J{1}   = tf(eye(3), [0 0 0.1625]);
    C.J{2}   = tf(rpy(1.570796327, 0, 0), [0 0 0]);
    C.J{3}   = tf(eye(3), [-0.425 0 0]);
    C.J{4}   = tf(eye(3), [-0.3922 0 0.1333]);
    C.J{5}   = tf(rpy(1.570796327, 0, 0), [0 -0.0997 -2.044881182297852e-11]);
    C.J{6}   = tf(rpy(1.570796326589793, pi, pi), [0 0.0996 -2.042830148012698e-11]);
    C.flange = tf(rpy(0, -pi/2, -pi/2), [0 0 0]);             % wrist_3_link -> flange
    C.tool0  = tf(rpy(pi/2, 0, pi/2), [0 0 0]);               % flange -> tool0
end
Ttcp = tf(eye(3), [0 0 tcp]);
n = size(Q, 1);
P = zeros(n, 3);  Rm = zeros(3, 3, n);
for k = 1:n
    A = C.base;
    for j = 1:6
        A = A * C.J{j} * tf(rz(Q(k, j)), [0 0 0]);
    end
    A = A * C.flange * C.tool0 * Ttcp;
    P(k, :) = A(1:3, 4).';
    Rm(:, :, k) = A(1:3, 1:3);
end
end

function T = tf(R, p)
T = [R, p(:); 0 0 0 1];
end

function R = rpy(r, p, y)
% Convenio del URDF: R = Rz(y) * Ry(p) * Rx(r).
R = rz(y) * ry(p) * rx(r);
end

function R = rx(a), R = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)]; end
function R = ry(a), R = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)]; end
function R = rz(a), R = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1]; end

function a = rpyZYX(R)
% [roll pitch yaw] con R = Rz(yaw) Ry(pitch) Rx(roll).
a = [atan2(R(3, 2), R(3, 3)), atan2(-R(3, 1), hypot(R(3, 2), R(3, 3))), atan2(R(2, 1), R(1, 1))];
end

function w = rotvec(R)
% log(R) como vector de rotacion [rad]. Robusto para angulos pequenos, que es
% el caso aqui (theta < 0.2 rad).
v = 0.5 * [R(3, 2) - R(2, 3); R(1, 3) - R(3, 1); R(2, 1) - R(1, 2)];
s = norm(v);  c = (trace(R) - 1) / 2;
th = atan2(s, c);
if s < 1e-12
    w = v.';
else
    w = (th / s) * v.';
end
end

function a = envolver(a)
a = mod(a + pi, 2 * pi) - pi;
end

function sombrear(ax, tc)
if ~isempty(tc)
    xregion(ax, tc(1), tc(2), 'FaceColor', [0.85 0.33 0.10], 'FaceAlpha', 0.10, ...
            'HandleVisibility', 'off');
end
xline(ax, 0, ':', 'HandleVisibility', 'off');
end

function plano(ax, Pin, z, margen)
% Superficie del tejido: rectangulo en z que cubre el tramo dentro del tejido.
x = [min(Pin(:, 1)) - margen, max(Pin(:, 1)) + margen];
y = [min(Pin(:, 2)) - margen, max(Pin(:, 2)) + margen];
patch(ax, x([1 2 2 1]), y([1 1 2 2]), z * ones(1, 4), [0.80 0.60 0.45], ...
      'FaceAlpha', 0.20, 'EdgeColor', [0.6 0.4 0.3], 'DisplayName', 'superficie del tejido');
end

function v = ternario(c, a, b)
if c, v = a; else, v = b; end
end
