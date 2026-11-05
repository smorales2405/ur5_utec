"""
R10 y D25: en el robot real el nodo NO arranca si su ley, sobre la planta medida
(M del URDF + rotor de B4.6), no llega a x2 con el retardo de tubería medido (5
ciclos, plan V15).

Se arrancan los nodos de verdad con la configuración del real
(`gravity_in_command:=false`, retardo, rotor y margen mínimo) pero sin robot: el
chequeo corre en el constructor, antes de comandar nada. Fuera de `dry_run`,
en un ROS_DOMAIN_ID propio y con los CSV en un directorio temporal.
"""

import os
import random
import re
import subprocess
import time

import pytest

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_CONFIG = os.path.join(_PKG, "config")
_GAINS = os.path.join(os.path.dirname(_PKG), "ur5_trajectory_optimization", "results",
                      "gain_tuning")
#: Sintonizadas con 1 ciclo de retardo y la M del URDF.
_V5_G6 = os.path.join(_GAINS, "smc_v5_g6", "selected_gains.yaml")
#: Las de `smc_712`, que entró en ciclo límite.
_V4_G5 = os.path.join(_GAINS, "smc_v4_g5", "selected_gains.yaml")
#: La candidata de la re-sintonía con el rotor medido.
_CANDIDATA = os.path.join(_GAINS, "smc_v7_rotor", "candidata_fila46.yaml")
_INCISION = ["incision_common.yaml", "smc_params.yaml"]

#: Lo que fija `ur5e_real.launch.py`; `test_el_real_y_el_evaluador_coinciden` lo comprueba.
_ROTOR = (1.0324, 1.2466, 1.0545, 0.1247, 0.1386, 0.0719)
_MARGIN_MIN = 2.0


def _list(values):
    return "[" + ",".join(f"{float(v):.6f}" for v in values) + "]"


def _executable(name):
    from ament_index_python.packages import get_package_prefix
    exe = os.path.join(get_package_prefix("ur5_dyn_control"), "lib",
                       "ur5_dyn_control", name)
    if not os.path.exists(exe):
        pytest.fail(f"no esta instalado {exe}: compilar antes de testear")
    return exe


def _margin(log):
    return float(re.findall(r"margen de ganancia x([\d.]+)", log)[-1])


def _start(tmp_path, exe, files, test_num, extra=(), measured_plant=True):
    """Arranca el nodo como en el real y espera a que salga o a que entre en
    su lazo. Devuelve (codigo de salida o None si sigue vivo, log). Con
    `measured_plant=False`, el rotor y el margen mínimo se dejan por defecto."""
    if measured_plant:
        extra = [("discrete_check.rotor_inertia", _list(_ROTOR)),
                 ("discrete_check.margin_min", str(_MARGIN_MIN)), *extra]
    cmd = [_executable(exe), "--ros-args", "-r", f"__node:={exe}"]
    for f in files:
        cmd += ["--params-file", os.path.join(_CONFIG, f) if "/" not in f else f]
    for k, v in [("use_sim_time", "false"), ("perform_switch", "false"),
                 ("perform_unpause", "false"), ("gravity_in_command", "false"),
                 ("pipeline_delay_steps", "5"), ("csv_output_dir", str(tmp_path)),
                 ("test_num", str(test_num)), *extra]:
        cmd += ["-p", f"{k}:={v}"]
    env = dict(os.environ, ROS_DOMAIN_ID=str(random.randint(150, 200)),
               ROS_LOCALHOST_ONLY="1")
    log_path = tmp_path / f"node_{test_num}.log"
    with open(log_path, "w") as log:
        proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            t0 = time.time()
            while time.time() - t0 < 120:
                if proc.poll() is not None:
                    return proc.returncode, log_path.read_text()
                if "Lazo de control" in log_path.read_text():
                    return None, log_path.read_text()
                time.sleep(0.5)
            pytest.fail("el nodo ni arranco ni salio en 120 s")
        finally:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    proc.kill()


@pytest.mark.skipif(not os.path.exists(_V4_G5), reason="sin smc_v4_g5")
def test_la_ley_de_712_no_arranca_en_el_real(tmp_path):
    """Estable en el modelo rígido, pero con menos margen del exigido."""
    rc, log = _start(tmp_path, "gz_smc_control_node", [*_INCISION, _V4_G5], 41)
    assert rc == 1, log[-2000:]
    assert "MARGEN INSUFICIENTE con 5 ciclos de retardo" in log
    assert 1.0 < _margin(log) < _MARGIN_MIN


@pytest.mark.skipif(not os.path.exists(_CANDIDATA), reason="sin smc_v7_rotor")
def test_la_candidata_arranca_con_el_margen_de_g7(tmp_path):
    rc, log = _start(tmp_path, "gz_smc_control_node", [*_INCISION, _CANDIDATA], 45)
    assert rc is None, log[-2000:]
    assert _margin(log) == pytest.approx(3.54, abs=0.05)
    assert "planta del chequeo: M del URDF + rotor medido" in log


@pytest.mark.skipif(not os.path.exists(_CANDIDATA), reason="sin smc_v7_rotor")
def test_sin_el_rotor_la_candidata_no_arranca(tmp_path):
    """El rotor es lo que la hace estable en el modelo: tiene que venir del launch."""
    rc, log = _start(tmp_path, "gz_smc_control_node", [*_INCISION, _CANDIDATA], 46,
                     extra=[("discrete_check.rotor_inertia", _list([0.0] * 6))])
    assert rc == 1, log[-2000:]
    assert "INESTABLE con 5 ciclos de retardo" in log


@pytest.mark.skipif(not os.path.exists(_V4_G5), reason="sin smc_v4_g5")
def test_se_puede_forzar_con_el_override(tmp_path):
    rc, log = _start(tmp_path, "gz_smc_control_node", [*_INCISION, _V4_G5], 42,
                     extra=[("discrete_check.allow_unstable", "true")])
    assert rc is None, log[-2000:]
    assert "MARGEN INSUFICIENTE con 5 ciclos de retardo" in log


@pytest.mark.skipif(not os.path.exists(_V5_G6), reason="sin smc_v5_g6")
def test_alpha_g_llega_a_la_ley(tmp_path):
    """B4.4: el fichero de ganancias lleva alpha_g, y entra en K y en el chequeo."""
    rc, log = _start(tmp_path, "gz_smc_control_node", [*_INCISION, _V5_G6], 44,
                     extra=[("alpha_g", "0.0543")])
    assert rc is None, log[-2000:]
    assert "alpha M/C/g=0.30/0.30/0.05" in log


def test_el_ct_nominal_arranca_con_margen(tmp_path):
    rc, log = _start(tmp_path, "gz_fl_control_node",
                     ["incision_common.yaml", "incision_params.yaml"], 43)
    assert rc is None, log[-2000:]
    assert _margin(log) > _MARGIN_MIN


def test_por_defecto_el_chequeo_es_el_de_siempre(tmp_path):
    """Gazebo: la planta no lleva rotor, y solo bloquearia lo inestable."""
    rc, log = _start(tmp_path, "gz_fl_control_node",
                     ["incision_common.yaml", "incision_params.yaml"], 47,
                     measured_plant=False)
    assert rc is None, log[-2000:]
    assert "margen de ganancia x6.95" in log
    assert "planta del chequeo" not in log


@pytest.mark.skipif(not os.path.exists(_CANDIDATA), reason="sin smc_v7_rotor")
@pytest.mark.parametrize("step", [0, 3])
def test_los_escalones_de_la_rampa_pasan_el_chequeo(tmp_path, step):
    """Sobre la tabla del barrido del codo, el que más sube K en el peor modo."""
    yaml = pytest.importorskip("yaml")
    ramp = pytest.importorskip("ur5_identification.gain_ramp")
    closed_loop = pytest.importorskip("ur5_trajectory_optimization.gain_tuning.closed_loop")
    sweeps = pytest.importorskip("ur5_identification.gravity_sweeps")
    g = next(iter(yaml.safe_load(open(_CANDIDATA)).values()))["ros__parameters"]
    lam, phi = ramp.step_gains(g["lambda"], g["phi_joint"], ramp.step_r(step))
    q = sweeps.incision_posture(closed_loop.Plant(closed_loop.default_urdf()), *ramp.RAMP_TIP)
    rc, log = _start(tmp_path, "gz_smc_control_node", ["gain_ramp_params.yaml", _CANDIDATA],
                     48 + step, extra=[("lambda", _list(lam)), ("phi_joint", _list(phi)),
                                       ("q_init", _list(q)), ("sweep.q_fixed", _list(q)),
                                       ("sweep.joint", "2")])
    assert rc is None, log[-2000:]
    assert _margin(log) >= _MARGIN_MIN


def _launch_real():
    import ast
    src = open(os.path.join(_PKG, "launch", "ur5e_real.launch.py"), encoding="utf-8").read()
    values = {}
    for node in ast.walk(ast.parse(src)):
        if isinstance(node, ast.Dict) and any(
                isinstance(k, ast.Constant) and k.value == "pipeline_delay_steps"
                for k in node.keys):
            assert not values
            for key, value in zip(node.keys, node.values):
                if isinstance(key, ast.Constant) and isinstance(value, (ast.Constant, ast.List)):
                    values[key.value] = ast.literal_eval(value)
    return values


def test_el_real_y_el_evaluador_coinciden():
    """El launch del real fija el retardo, el rotor y el margen del chequeo; el evaluador,
    los de la simulacion y de g7. Si divergen, se sintoniza para un lazo y se comprueba
    otro."""
    from ur5_trajectory_optimization.gain_tuning.closed_loop import PIPELINE_DELAY_STEPS
    from ur5_trajectory_optimization.gain_tuning.problem import (MARGIN_MIN_DEFAULT,
                                                                 ROTOR_INERTIA_REAL)
    real = _launch_real()
    assert real["pipeline_delay_steps"] == PIPELINE_DELAY_STEPS == 5
    assert real["discrete_check.margin_min"] == MARGIN_MIN_DEFAULT == _MARGIN_MIN
    assert tuple(real["discrete_check.rotor_inertia"]) == tuple(ROTOR_INERTIA_REAL) == _ROTOR
