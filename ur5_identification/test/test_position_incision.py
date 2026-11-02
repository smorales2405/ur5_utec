"""B1/B2: el CSV de la incision por posicion lo lee el estimador como uno por par.

Los `e2e` ejecutan el runner de verdad contra `fake_ur_driver.py`, en un
ROS_DOMAIN_ID propio: el robot real vive en otro y no se entera. Existen porque
el runner llego al robot sin haberse ejecutado nunca y no arrancaba desde q_init
(2026-11-02).
"""

import json
import os
import random
import subprocess
import sys

import numpy as np
import pytest

_TEST = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.join(_TEST, "..", "scripts")
sys.path.insert(0, _SCRIPTS)
from run_position_incision import write_csv, zero_time  # noqa: E402
from ur5_identification.force_estimator import estimate_from_csv  # noqa: E402

try:
    from ur5_trajectory_optimization.gain_tuning.closed_loop import (
        Q_INIT, Plant, default_urdf)
    URDF = default_urdf()
except Exception:  # pragma: no cover
    URDF = ""


def test_el_cero_cae_en_el_reposo_antes_del_contacto():
    phase = np.array(["approach"] * 100 + ["contact"] * 10)
    assert zero_time(0.01, phase) == pytest.approx(1.0 - 0.15)


@pytest.mark.skipif(not os.path.exists(URDF), reason="URDF/evaluador no instalado")
def test_csv_de_posicion_compatible_con_el_estimador(tmp_path):
    plant = Plant(URDF)
    n, dt = 50, 0.002
    q = np.tile(Q_INIT, (n, 1))
    z = np.zeros((n, 6))
    phase = np.array(["approach"] * 20 + ["contact"] * 30)
    plateau = np.zeros(n, int)
    ref = (dt, q, z, z, phase, plateau)
    samples = [(100.0 + k * dt, q[k], z[k], np.ones(6)) for k in range(n)]
    # F/T en tool0_controller: 1 N a lo largo de x de la herramienta.
    ft = [(100.0 + k * dt, np.array([1.0, 0, 0, 0, 0, 0])) for k in range(n)]
    out = tmp_path / "pos_1.csv"
    assert write_csv(str(out), samples, ft, 100.03, ref, {"test_num": 1}, plant)
    r = estimate_from_csv(str(out), plant=plant, gazebo=True)
    assert r["F"].shape == (n, 3)
    # Con la hoja hacia abajo (A4), x de la herramienta = -x de base_link.
    np.testing.assert_allclose(r["F_ft"][:, 0], -1.0, atol=1e-9)
    st = np.genfromtxt(str(out), delimiter=",", names=True, dtype=None,
                       encoding="utf-8", skip_header=1)
    assert list(st["ft_zero"][:15]) == [0] * 15 and st["ft_zero"][-1] == 1


# --- De punta a punta, contra el driver falso ------------------------------------

def _exe_table():
    """El runner construye la tabla con el nodo de C++ instalado."""
    try:
        from ament_index_python.packages import get_package_prefix
        return os.path.join(get_package_prefix("ur5_dyn_control"), "lib",
                            "ur5_dyn_control", "gz_smc_control_node")
    except Exception:  # pragma: no cover
        return ""


e2e = pytest.mark.skipif(not (os.path.exists(URDF) and os.path.exists(_exe_table())),
                         reason="sin ur5_dyn_control o el evaluador instalados")


def _run_against_fake(tmp_path, q0, scaling=1.0, test_num=1, timeout=120):
    """Devuelve (resultado del runner, eventos del driver falso)."""
    domain = random.choice([d for d in range(150, 200)
                            if str(d) != os.environ.get("ROS_DOMAIN_ID")])
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1",
               TMPDIR=str(tmp_path))
    log = tmp_path / "fake.jsonl"
    fake = subprocess.Popen(
        [sys.executable, os.path.join(_TEST, "fake_ur_driver.py"),
         "--domain", str(domain), "--scaling", str(scaling), "--log", str(log),
         "--q0", *[str(v) for v in q0]], env=env)
    try:
        res = subprocess.run(
            [sys.executable, os.path.join(_SCRIPTS, "run_position_incision.py"),
             "--yes", "--test-num", str(test_num), "--out-dir", str(tmp_path)],
            env=env, capture_output=True, text=True, timeout=timeout)
    finally:
        fake.terminate()
        fake.wait(timeout=10)
    events = ([json.loads(x) for x in log.read_text().splitlines()]
              if log.exists() else [])
    return res, events


@e2e
def test_e2e_desde_q_init_se_coloca_y_corre(tmp_path):
    """El caso del usuario: robot en q_init, que NO es el inicio de la tabla."""
    res, events = _run_against_fake(tmp_path, Q_INIT)
    assert res.returncode == 0, res.stdout[-1500:] + res.stderr[-1500:]
    out = tmp_path / "pos_1.csv"
    assert out.exists()

    trajs = [e for e in events if e["kind"] == "trajectory"]
    assert [t["points"] for t in trajs][0] == 1, "primero el traslado, de un punto"
    assert len(trajs) == 2
    # La tabla manda su primer punto en t = 0: el JTC salta a el. Tras el traslado
    # el salto tiene que ser nulo.
    assert trajs[1]["points"] > 1000 and trajs[1]["jump"] < 1e-3

    meta = open(out).readline()
    assert meta.startswith("#")
    d = np.genfromtxt(str(out), delimiter=",", names=True, dtype=None,
                      encoding="utf-8", skip_header=sum(
                          1 for x in open(out) if x.startswith("#")))
    # Las fases y q_des alineadas con lo que hace el brazo. Con las muestras
    # viejas de la cola dentro t0 se adelanta y esto da 0.08 rad (medido quitando
    # el vaciado; el driver falso va a 250 Hz: 0.4 s, en el real serian 0.2 s).
    err = max(np.abs(d[f"q{i}"] - d[f"q{i}_des"]).max() for i in range(1, 7))
    assert err < 0.01

    # El cero del F/T, en el reposo antes del contacto y con la hoja quieta.
    k = int(np.flatnonzero(d["ft_zero"] == 1)[0])
    assert d["state"][k] == "approach"
    assert 7.65 < d["t_sim"][k] < 7.95
    assert max(abs(d[f"dq{i}"][k]) for i in range(1, 7)) < 0.02
    assert "ft_zero_ok=True" in open(out).read(400)

    r = estimate_from_csv(str(out), plant=Plant(URDF), gazebo=True)
    assert r["F"].shape[0] == len(d)


@e2e
def test_e2e_sin_external_control_no_mueve_ni_gasta_la_corrida(tmp_path):
    res, events = _run_against_fake(tmp_path, Q_INIT, scaling=0.0)
    assert res.returncode != 0
    assert "External Control" in res.stderr + res.stdout
    assert not [e for e in events if e["kind"] == "trajectory"]
    assert not (tmp_path / "pos_1.csv").exists()


# --- Decisiones de arranque, sin ROS ---------------------------------------------

@pytest.mark.parametrize("value, word", [
    (0.0, "External Control"), (0.5, "50 %"), (50.0, "50 %"),
    (1.0, None), (100.0, None)])
def test_escala_de_velocidad(value, word):
    from run_position_incision import scaling_problem
    problem = scaling_problem(value)
    assert (problem is None) if word is None else (word in problem)


@pytest.mark.skipif(not os.path.exists(URDF), reason="URDF/evaluador no instalado")
def test_traslado_al_inicio():
    from run_position_incision import MAX_MOVE, start_move
    plant = Plant(URDF)
    # Inicio y final de la tabla de incision_common.yaml.
    q0 = np.array([1.5708, -1.56938, 1.51406, -1.51547, -1.5708, 3.1416])
    q_end = np.array([1.5708, -1.0855, 1.7772, -2.2625, -1.5708, 3.1416])

    err, problem = start_move(plant, Q_INIT, q0)
    assert err == pytest.approx(0.0567, abs=1e-3) and problem is None
    # La corrida anterior acaba en q_end: tiene que poder recolocarse sola.
    assert start_move(plant, q_end, q0)[1] is None

    lejos = q0.copy()
    lejos[0] += MAX_MOVE + 0.1
    assert "pendant" in start_move(plant, lejos, q0)[1]

    # wrist_1 de un lado al otro del punto en que la punta queda mas baja (no es
    # la vertical: la punta esta a d5 del eje): pasa por debajo de los dos extremos.
    from ur5_identification.gravity_sweeps import tip_position
    grid = q0[3] + np.linspace(-1.0, 1.0, 201)
    zs = [tip_position(plant, np.r_[q0[:3], v, q0[4:]])[2] for v in grid]
    low = grid[int(np.argmin(zs))]
    a, b = q0.copy(), q0.copy()
    a[3], b[3] = low + 0.45, low - 0.45
    assert "bajaria" in start_move(plant, a, b)[1]
