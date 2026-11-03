"""B1: la inclinacion mide ganancia y signo del F/T con una masa conocida.

El `e2e` ejecuta el runner contra `fake_ur_driver.py`, en un ROS_DOMAIN_ID propio:
el robot real vive en otro y no se entera.
"""

import os
import random
import subprocess
import sys

import numpy as np
import pytest

_TEST = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.join(_TEST, "..", "scripts")
sys.path.insert(0, _SCRIPTS)
from run_ft_tilt_check import (  # noqa: E402
    Q_BASE, TILT, fit_gain, plan_problems, read_csv, sequence, verdict,
    weight_in_tool, write_csv)

try:
    from ur5_trajectory_optimization.gain_tuning.closed_loop import Plant, default_urdf
    URDF = default_urdf()
except Exception:  # pragma: no cover
    URDF = ""

needs_plant = pytest.mark.skipif(not os.path.exists(URDF),
                                 reason="URDF/evaluador no instalado")
MASS = 0.179


def _rows(plant, gain, offset=(0.7, -0.4, 1.1), drift=0.0, mass=MASS):
    """Lo que leeria un sensor de esa ganancia, con offset y deriva lineal."""
    return [(name, q, gain * weight_in_tool(plant, q, mass) + np.asarray(offset)
             + drift * k * np.ones(3))
            for k, (name, q) in enumerate(sequence())]


def test_cada_inclinacion_entre_dos_reposos():
    seq = sequence()
    assert [n for n, _ in seq[1::2]] == ["wrist_1+", "wrist_1-", "wrist_2+", "wrist_2-"]
    assert all(n == "reposo" for n, _ in seq[0::2])
    assert seq[-1][0] == "reposo"
    np.testing.assert_allclose(np.abs(seq[1][1] - Q_BASE).max(), TILT)


@needs_plant
@pytest.mark.parametrize("gain", [1.0, -1.0, 0.0, 0.5])
def test_recupera_ganancia_y_signo(gain):
    plant = Plant(URDF)
    r = fit_gain(plant, _rows(plant, gain), MASS)
    assert r["gain"] == pytest.approx(gain, abs=1e-9)
    assert r["rms"] < 1e-9


@needs_plant
def test_offset_y_deriva_lineal_no_entran():
    """Es la razon del ensayo: ni un cero anterior ni la deriva lenta cuentan."""
    plant = Plant(URDF)
    r = fit_gain(plant, _rows(plant, 1.0, offset=(30.0, -20.0, 9.0), drift=0.15), MASS)
    assert r["gain"] == pytest.approx(1.0, abs=1e-9)


@needs_plant
def test_la_senal_es_medible_con_el_acople():
    """Inclinar 0.5 rad con 1.75 N tiene que dar ~0.84 N, frente a 0.11 N de ruido."""
    plant = Plant(URDF)
    r = fit_gain(plant, _rows(plant, 1.0), MASS)
    for _, _, exp in r["detail"]:
        assert np.abs(exp).max() == pytest.approx(MASS * 9.8 * np.sin(TILT), rel=0.02)


def _result(gain, x=(1.0, 0.03), y=(1.0, 0.03), z=(1.0, 0.03)):
    return {"gain": gain, "axes": {"x": x, "y": y, "z": z}}


@pytest.mark.parametrize("result, word", [
    (_result(1.02), "EXTERNA"),
    (_result(-0.95, x=(-1.0, 0.03), y=(-1.0, 0.03), z=(-1.0, 0.03)), "brida sobre la carga"),
    (_result(0.03), "NO VE"),
    (_result(0.5), "FUERA DE RANGO"),
    (_result(0.874, x=(0.669, 0.047), y=(1.052, 0.031), z=(1.071, 0.155)), "de x se sale"),
])
def test_veredicto(result, word):
    assert word in verdict(result)


def test_un_eje_sin_senal_no_entra_en_el_veredicto():
    assert verdict(_result(1.0, z=(0.3, 0.155))).startswith("OK")


@needs_plant
def test_ganancias_por_eje_e_incertidumbre():
    """Un eje flojo no puede quedar escondido en el `gain` global (tilt_1)."""
    plant = Plant(URDF)
    gains = np.array([0.67, 1.05, 1.0])
    rows = [(n, q, gains * weight_in_tool(plant, q, MASS) + np.array([3.0, -3.5, 0.3]))
            for n, q in sequence()]
    r = fit_gain(plant, rows, MASS)
    assert r["axes"]["x"][0] == pytest.approx(0.67, abs=1e-6)
    assert r["axes"]["y"][0] == pytest.approx(1.05, abs=1e-6)
    rng = np.random.default_rng(1)
    noisy = [(n, q, f + rng.normal(0.0, 0.05, 3)) for n, q, f in rows]
    err = fit_gain(plant, noisy, MASS)["axes"]["x"][1]
    assert 0.01 < err < 0.2


@needs_plant
def test_csv_ida_y_vuelta(tmp_path):
    plant = Plant(URDF)
    poses = [(k, n, q.tolist(), [(1.0, tuple(f), (0.0, 0.0, 0.0))])
             for k, (n, q, f) in enumerate(_rows(plant, -1.0))]
    path = tmp_path / "tilt_1.csv"
    write_csv(str(path), {"masa_kg": MASS}, poses)
    mass, rows = read_csv(str(path))
    assert mass == MASS and len(rows) == len(poses)
    assert fit_gain(plant, rows, mass)["gain"] == pytest.approx(-1.0, abs=1e-4)


@needs_plant
def test_plan_desde_q_init_y_desde_el_final_de_la_tabla():
    plant = Plant(URDF)
    assert plan_problems(plant, Q_BASE) == []
    q_end = np.array([1.5708, -1.0855, 1.7772, -2.2625, -1.5708, 3.1416])
    assert plan_problems(plant, q_end) == []
    assert plan_problems(plant, Q_BASE, tilt=1.0) == []
    lejos = Q_BASE.copy()
    lejos[0] += 1.5
    assert any("pendant" in p for p in plan_problems(plant, lejos))


def _exe_ok():
    try:
        from ament_index_python.packages import get_package_prefix
        return bool(get_package_prefix("ur5_identification"))
    except Exception:  # pragma: no cover
        return False


@needs_plant
@pytest.mark.skipif(not _exe_ok(), reason="sin el entorno de ROS")
def test_e2e_contra_el_driver_falso(tmp_path):
    domain = random.choice([d for d in range(150, 200)
                            if str(d) != os.environ.get("ROS_DOMAIN_ID")])
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1",
               TMPDIR=str(tmp_path))
    fake = subprocess.Popen(
        [sys.executable, os.path.join(_TEST, "fake_ur_driver.py"),
         "--domain", str(domain), "--log", str(tmp_path / "fake.jsonl"),
         "--ft-gain-kg", str(MASS), "--q0", *[str(v) for v in Q_BASE]], env=env)
    try:
        res = subprocess.run(
            [sys.executable, os.path.join(_SCRIPTS, "run_ft_tilt_check.py"),
             "--yes", "--test-num", "1", "--masa-kg", str(MASS), "--hold", "1.0",
             "--out-dir", str(tmp_path)],
            env=env, capture_output=True, text=True, timeout=200)
    finally:
        fake.terminate()
        fake.wait(timeout=10)
    assert res.returncode == 0, res.stdout[-1500:] + res.stderr[-1500:]
    mass, rows = read_csv(str(tmp_path / "tilt_1.csv"))
    r = fit_gain(Plant(URDF), rows, mass)
    assert r["gain"] == pytest.approx(1.0, abs=0.05)
    assert "EXTERNA" in res.stdout and "eje x" in res.stdout
