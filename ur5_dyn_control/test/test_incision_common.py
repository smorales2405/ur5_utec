"""
B0.2 / B0.3 / B0.5: las tres leyes ejecutan la MISMA incisión, etiquetada.

Por qué existe. Cada YAML de ley llevaba su propio bloque de trayectoria y
habían divergido sin ningún síntoma: el FL partía 21.7 mm más abajo (TCP viejo)
y el SMC cortaba en 0.85 mientras los otros cortaban en 0.75 (V4 y V6 de
docs/Journal_UR5eControl_plan.md). Una comparación de controladores sobre
referencias distintas no compara controladores. Desde B0 la geometría vive en
config/incision_common.yaml y aquí se comprueba, con los nodos de verdad, que
los tres producen la tabla byte a byte.

Los nodos corren en `dry_run` (no publican par) en un ROS_DOMAIN_ID propio y
escriben en un directorio temporal: no tocan ~/.ros/ur5_dyn_control.
"""

import glob
import os
import random
import signal
import subprocess
import sys
import time

import pytest
import yaml

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_CONFIG = os.path.join(_PKG, "config")
sys.path.insert(0, os.path.join(_PKG, "launch"))
from param_layers import declared_trajectory_type, incision_layers  # noqa: E402

#: YAML de cada ley de la comparación -> ejecutable.
LAWS = {
    "incision_params.yaml": "gz_fl_control_node",       # CT (línea base)
    "lqr_sdre_params.yaml": "gz_lqr_sdre_control_node",
    "smc_params.yaml": "gz_smc_control_node",
}
#: YAML completos que NO son de incisión: no deben recibir el común.
OTHERS = ["fl_control_params.yaml", "sweep_params.yaml",
          "sweep_smc_params.yaml", "gravity_comp_params.yaml"]

PHASES = ["approach", "contact", "penetration", "cut", "withdraw"]


def _flat(d, pre=""):
    out = {}
    for k, v in d.items():
        if isinstance(v, dict):
            out.update(_flat(v, pre + k + "."))
        else:
            out[pre + k] = v
    return out


def _params(name):
    with open(os.path.join(_CONFIG, name), encoding="utf-8") as fh:
        doc = yaml.safe_load(fh)
    (section,) = doc.values()
    return _flat(section["ros__parameters"])


# ─────────────────────────────────────────────────────────────────────────────
# Estáticos: estructura de los YAML y regla de capas
# ─────────────────────────────────────────────────────────────────────────────

def test_los_yaml_de_ley_no_repiten_el_bloque_comun():
    """Una clave del común repetida en una ley la pisaría solo para esa ley:
    exactamente la divergencia que B0.2 elimina."""
    common = set(_params("incision_common.yaml"))
    for law in LAWS:
        repes = (set(_params(law)) & common) - {"trajectory_type"}
        assert not repes, f"{law} repite claves del bloque comun: {sorted(repes)}"


def test_regla_de_capas():
    for law in LAWS:
        assert declared_trajectory_type(os.path.join(_CONFIG, law)) == "incision"
        layers = incision_layers(os.path.join(_CONFIG, law),
                                 common=os.path.join(_CONFIG, "incision_common.yaml"))
        assert [os.path.basename(p) for p in layers] == ["incision_common.yaml", law]
    for other in OTHERS:
        path = os.path.join(_CONFIG, other)
        assert declared_trajectory_type(path) != "incision", other
        assert incision_layers(path) == [path], other


def test_el_comun_no_fija_trajectory_type():
    """Si lo fijara, cargarlo por error con un YAML de waypoints o de barrido
    lo convertiria en una incision sin avisar."""
    assert "trajectory_type" not in _params("incision_common.yaml")
    assert _params("incision_common.yaml")["incision_common"]


# ─────────────────────────────────────────────────────────────────────────────
# Dinámicos: los tres nodos de verdad
# ─────────────────────────────────────────────────────────────────────────────

def _executable(name):
    from ament_index_python.packages import get_package_prefix
    exe = os.path.join(get_package_prefix("ur5_dyn_control"), "lib",
                       "ur5_dyn_control", name)
    if not os.path.exists(exe):
        pytest.fail(f"no esta instalado {exe}: compilar antes de testear")
    return exe


def _dry_run(exe, param_files, out_dir, test_num, extra=()):
    """Arranca el nodo en dry_run, espera a que exporte la tabla y lo para.

    Devuelve (ruta de la tabla, ruta del fichero lateral de parametros). El
    fichero lateral se escribe DESPUES de la tabla, asi que su aparicion marca
    que la tabla esta completa.
    """
    ref = os.path.join(out_dir, f"ref_{test_num}.csv")
    # <prefijo del nodo>_<test_num>.params.yaml; el prefijo lo pone el nodo.
    sidecar = os.path.join(out_dir, f"*_{test_num}.params.yaml")
    cmd = [exe, "--ros-args"]
    for f in param_files:
        cmd += ["--params-file", f]
    for k, v in [("dry_run", "true"), ("perform_switch", "false"),
                 ("perform_unpause", "false"), ("use_sim_time", "false"),
                 ("csv_output_dir", out_dir), ("test_num", str(test_num)),
                 ("reference_table_out", ref), *extra]:
        cmd += ["-p", f"{k}:={v}"]
    env = dict(os.environ, ROS_DOMAIN_ID=str(random.randint(150, 200)),
               ROS_LOCALHOST_ONLY="1")
    # A un fichero, no a un PIPE: el nodo escribe mas de lo que cabe en el
    # bufer de la tuberia y, si nadie lo lee, se bloquea antes de exportar.
    log = open(os.path.join(out_dir, f"node_{test_num}.log"), "w")
    proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                            text=True)
    proc.log_path = log.name
    return proc, ref, sidecar


def _wait(procs, timeout=120.0):
    t0 = time.time()
    pending = dict(procs)
    while pending and time.time() - t0 < timeout:
        for key, (proc, ref, sidecar) in list(pending.items()):
            if glob.glob(sidecar):
                time.sleep(0.2)
                del pending[key]
            elif proc.poll() is not None:
                pytest.fail(f"{key} murio antes de exportar la tabla:\n"
                            f"{_tail(proc.log_path)}")
        time.sleep(0.2)
    for key, (proc, _, _) in procs.items():
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
    assert not pending, f"sin tabla tras {timeout} s: {sorted(pending)}"


def _tail(path, n=2000):
    with open(path, encoding="utf-8", errors="replace") as fh:
        return fh.read()[-n:]


def _rows(path):
    with open(path, encoding="utf-8") as fh:
        return [ln.rstrip("\n") for ln in fh if not ln.startswith("#")]


@pytest.fixture(scope="module")
def tablas(tmp_path_factory):
    out = str(tmp_path_factory.mktemp("incision_common"))
    common = os.path.join(_CONFIG, "incision_common.yaml")
    procs = {}
    for i, (law, exe_name) in enumerate(LAWS.items()):
        procs[law] = _dry_run(_executable(exe_name),
                              [common, os.path.join(_CONFIG, law)], out, 9100 + i)
    _wait(procs)
    return out, {law: (ref, glob.glob(sidecar)[0])
                 for law, (_, ref, sidecar) in procs.items()}


def test_las_tres_leyes_ejecutan_la_misma_tabla(tablas):
    _, res = tablas
    rows = {law: _rows(ref) for law, (ref, _) in res.items()}
    base = rows["smc_params.yaml"]
    assert len(base) > 1000
    for law, r in rows.items():
        assert r == base, f"{law} no ejecuta la misma tabla que el SMC"


def test_fases_y_meseta_del_corte(tablas):
    """B0.3: la tabla trae la fase geometrica y la meseta de feed constante;
    el evaluador y el analisis leen las ventanas de aqui."""
    _, res = tablas
    ref, _ = res["smc_params.yaml"]
    rows = _rows(ref)
    hdr = rows[0].split(",")
    assert hdr[-2:] == ["phase", "plateau"]
    phase = [r.split(",")[-2] for r in rows[1:]]
    plateau = [r.split(",")[-1] == "1" for r in rows[1:]]
    # Las cinco fases, en orden y sin volver atras.
    seq = [p for i, p in enumerate(phase) if i == 0 or p != phase[i - 1]]
    assert seq == PHASES
    # La meseta: contigua, dentro del corte y de cut_length / v_cut segundos.
    idx = [i for i, p in enumerate(plateau) if p]
    assert idx == list(range(idx[0], idx[-1] + 1))
    assert all(phase[i] == "cut" for i in idx)
    common = _params("incision_common.yaml")
    dt = 1.0 / common["control_rate"]
    t_plateau = (len(idx) - 1) * dt
    assert t_plateau == pytest.approx(
        common["incision.cut_length"] / common["incision.v_cut"], abs=2 * dt)


def test_el_fichero_lateral_repite_la_corrida(tablas):
    """B0.5: <prefijo>_<n>.params.yaml basta por si solo para repetir la
    corrida (aqui, la misma tabla)."""
    out, res = tablas
    ref, sidecar = res["smc_params.yaml"]
    # Solo el fichero lateral; los -p de _dry_run pisan test_num y la ruta de
    # la tabla, que son lo unico que tiene que cambiar.
    proc = _dry_run(_executable("gz_smc_control_node"), [sidecar], out, 9199)
    _wait({"replay": proc})
    assert _rows(proc[1]) == _rows(ref)


def test_incision_sin_bloque_comun_no_arranca(tmp_path):
    """Sin incision_common.yaml el nodo usaria los defaults del codigo y
    ejecutaria otra tabla: tiene que negarse."""
    exe = _executable("gz_smc_control_node")
    proc, _, _ = _dry_run(exe, [os.path.join(_CONFIG, "smc_params.yaml")],
                          str(tmp_path), 9198)
    try:
        proc.wait(timeout=60)
    except subprocess.TimeoutExpired:
        proc.kill()
        pytest.fail("el nodo arranco una incision sin el bloque comun")
    assert proc.returncode != 0
    assert "incision_common" in _tail(proc.log_path)


#: Direccion hacia la que mira el filo, en el frame tool0. Trazada sobre la malla
#: del acople (urdf/scalpel_tool.xacro, "Orientacion de la hoja"): el tramo recto
#: del filo esta en y = -0.9 .. -1.2 mm y el dorso de la hoja en y > 0.
BLADE_EDGE_TOOL0 = (0.0, -1.0, 0.0)


def test_el_filo_va_por_delante_y_wrist3_en_pi(tablas):
    """D18 del plan: la direccion del corte la fija el filo. Con wrist_3 en pi el
    filo mira hacia la base, asi que el corte va de lejos a cerca de la base con
    el filo por delante, y en la recta x = -d4 ni la base ni wrist_3 se mueven
    durante el corte."""
    import numpy as np
    import pinocchio as pin
    from ament_index_python.packages import get_package_share_directory
    _, res = tablas
    ref, _ = res["smc_params.yaml"]
    rows = _rows(ref)
    hdr = rows[0].split(",")
    data = [r.split(",") for r in rows[1:]]
    col = [hdr.index(f"q{i}") for i in range(1, 7)]
    q = np.array([[float(r[c]) for c in col] for r in data])
    plateau = np.array([r[-1] == "1" for r in data])

    urdf = os.path.join(get_package_share_directory("ur5_kinematics"), "ur5e.urdf")
    model = pin.buildModelFromUrdf(urdf)
    d = model.createData()
    tool0 = model.getFrameId("tool0")
    p, edge = [], []
    for qk in q[plateau]:
        pin.framesForwardKinematics(model, d, qk)
        p.append(d.oMf[tool0].translation.copy())
        edge.append(d.oMf[tool0].rotation @ np.array(BLADE_EDGE_TOOL0))
    p, edge = np.array(p), np.array(edge)
    advance = (p[-1] - p[0]) / np.linalg.norm(p[-1] - p[0])
    # De lejos a cerca de la base: |base_y| decrece.
    assert abs(p[-1][1]) < abs(p[0][1])
    # El filo por delante, alineado con el avance (< 0.5 grados).
    assert np.degrees(np.arccos(np.clip((edge @ advance).min(), -1, 1))) < 0.5
    # wrist_3 en pi y la base quieta durante todo el corte.
    np.testing.assert_allclose(q[plateau, 5], np.pi, atol=1e-3)
    assert np.ptp(q[plateau, 0]) < 1e-6

