"""Rampa de ganancias del SMC (plan V16, D25): escalones, orden, geometría y criterios."""

import csv
import os
import sys

import numpy as np
import pytest
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_HERE))
_SCRIPTS = os.path.join(os.path.dirname(_HERE), "scripts")
sys.path.insert(0, _SCRIPTS)

from ur5_identification.gain_ramp import (AMPLITUDE_FLOOR, PRE_FAILURE_BAND,  # noqa: E402
                                          PRE_FAILURE_FRACTION, PRE_FAILURE_LEVEL,
                                          RAMP_AMPLITUDE, RAMP_JOINTS, RAMP_SPEED, RAMP_STEPS,
                                          RAMP_TIP,
                                          RIPPLE_MAX, SAT_MAX, TESTED_CLEAN_KP, check_run,
                                          over_tested, plan_runs, step_gains)

_RESULTS = os.path.join(_SRC, "ur5_trajectory_optimization", "results", "gain_tuning")
_V7 = os.path.join(_RESULTS, "smc_v7_rotor")
_CANDIDATA = os.path.join(_V7, "candidata_fila46.yaml")
_CONFIG = os.path.join(_SRC, "ur5_dyn_control", "config")
_CSV = os.path.expanduser("~/.ros/ur5_dyn_control")
_BAND2 = (40.0, 55.0)


def _gains(path=_CANDIDATA):
    return next(iter(yaml.safe_load(open(path)).values()))["ros__parameters"]


def _plant():
    closed_loop = pytest.importorskip(
        "ur5_trajectory_optimization.gain_tuning.closed_loop")
    try:
        return closed_loop.Plant(closed_loop.default_urdf())
    except Exception as exc:     # sin URDF instalado
        pytest.skip(f"sin URDF: {exc}")


def _law(g, lam, phi):
    from ur5_trajectory_optimization.gain_tuning.closed_loop import SmcLaw
    return SmcLaw(lam=lam, eta=g["eta"], phi=phi, alpha=g["alpha"], use_sat=True,
                  alpha_g=g.get("alpha_g", g["alpha"]))


def _quiet():
    return {"ripple": np.full(6, 0.001), "sat": np.zeros(6),
            "bands": {PRE_FAILURE_BAND: np.full(6, 0.005), _BAND2: np.full(6, 0.005)}}


# ── escalones ───────────────────────────────────────────────────────────────


def test_r_uno_son_las_ganancias_de_la_candidata():
    lam, phi = step_gains([1.0, 2.0], [0.1, 0.2], 1.0)
    np.testing.assert_allclose(lam, [1.0, 2.0])
    np.testing.assert_allclose(phi, [0.1, 0.2])


@pytest.mark.parametrize("r", [0.0, -0.5, 1.1])
def test_r_fuera_de_rango_se_rechaza(r):
    with pytest.raises(ValueError):
        step_gains([1.0], [0.1], r)


def test_la_rampa_acaba_en_la_candidata_y_solo_sube():
    assert RAMP_STEPS[-1] == 1.0
    assert list(RAMP_STEPS) == sorted(set(RAMP_STEPS))


def test_kp_escala_con_r_cuadrado_y_g_con_r():
    from ur5_trajectory_optimization.gain_tuning.closed_loop import smc_loop_gains
    plant, g, z = _plant(), _gains(), np.zeros(6)
    q = np.array([1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 3.1416])
    loop = {}
    for r in (1.0, 0.5):
        lam, phi = step_gains(g["lambda"], g["phi_joint"], r)
        _, KP, KD = smc_loop_gains(_law(g, lam, phi), plant.model, plant.data, q, z, z)
        loop[r] = np.diag(KP), np.diag(KD)
    np.testing.assert_allclose(loop[0.5][0], 0.25 * loop[1.0][0])
    np.testing.assert_allclose(loop[0.5][1], 0.5 * loop[1.0][1])


def test_probado_limpio_sale_de_las_corridas():
    from ur5_trajectory_optimization.gain_tuning.closed_loop import smc_loop_gains
    plant, z = _plant(), np.zeros(6)
    g = _gains(os.path.join(_RESULTS, "smc_v4_g5", "selected_gains.yaml"))
    q = np.array([1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 3.1416])
    kp = np.zeros(6)
    for lam in ([20] * 6, [20, 20, 110, 20, 20, 20], [20, 20, 130.7, 20, 20, 20],
                [20, 60, 55, 20, 20, 20]):
        _, KP, _ = smc_loop_gains(_law(g, lam, g["phi_joint"]), plant.model, plant.data,
                                  q, z, z)
        kp = np.maximum(kp, np.diag(KP))
    np.testing.assert_allclose(kp, TESTED_CLEAN_KP, atol=0.1)


def test_la_postura_y_los_escalones_cumplen_lo_prometido():
    """El primer escalón no sale de lo probado, y el margen se mantiene sobre x2."""
    from run_gain_ramp_real import step_table
    from ur5_identification.gravity_sweeps import incision_posture
    from ur5_trajectory_optimization.gain_tuning.problem import ROTOR_INERTIA_REAL
    plant = _plant()
    table = step_table(plant, incision_posture(plant, *RAMP_TIP), _gains(), ROTOR_INERTIA_REAL)
    assert over_tested(table[0]["kp"]) == []
    assert set(over_tested(table[-1]["kp"])) == {"shoulder_pan", "wrist_1", "wrist_2", "wrist_3"}
    margins = [s["margin"] for s in table]
    assert margins == sorted(margins, reverse=True)
    assert margins[-1] >= 2.0
    assert max(s["g"].max() for s in table) <= 120.0


# ── orden de las corridas ───────────────────────────────────────────────────


def test_numeros_unicos_y_comparan_con_la_misma_junta():
    runs = plan_runs(1400)
    assert len(runs) == len(RAMP_STEPS) * len(RAMP_JOINTS)
    nums = [r["test_num"] for r in runs]
    assert len(set(nums)) == len(nums)
    by_num = {r["test_num"]: r for r in runs}
    for r in runs:
        if r["step"] == 0:
            assert r["previous"] is None and r["ratio"] is None
        else:
            prev = by_num[r["previous"]]
            assert prev["joint"] == r["joint"] and prev["step"] == r["step"] - 1
            assert r["ratio"] == pytest.approx(r["r"] / prev["r"])


def test_las_cuatro_juntas_por_encima_de_lo_probado_y_el_codo_se_barren():
    assert set(RAMP_JOINTS) == {0, 2, 3, 4, 5}


def test_un_escalon_no_empieza_sin_acabar_el_anterior():
    steps = [r["step"] for r in plan_runs(0)]
    assert steps == sorted(steps)


# ── geometría ───────────────────────────────────────────────────────────────


def test_barridos_y_traslado_no_se_acercan_al_tablero():
    from run_gain_ramp_real import geometry_problems
    from ur5_identification.gravity_sweeps import incision_posture
    from ur5_trajectory_optimization.gain_tuning.closed_loop import Q_INIT
    plant = _plant()
    clear, move, problems = geometry_problems(plant, incision_posture(plant, *RAMP_TIP), Q_INIT)
    assert problems == []
    assert min(clear.values()) > 0.05 and move > 0.05


def test_una_postura_del_corte_no_pasaria():
    """Es la razón de no barrer en la postura del trazo."""
    from run_gain_ramp_real import geometry_problems
    from ur5_identification.gravity_sweeps import incision_posture
    from ur5_trajectory_optimization.gain_tuning.closed_loop import Q_INIT
    plant = _plant()
    _, _, problems = geometry_problems(plant, incision_posture(plant, 0.70, 0.06), Q_INIT)
    assert any("elbow" in p for p in problems)


# ── criterios de parada ─────────────────────────────────────────────────────


def test_corrida_limpia_no_para():
    assert check_run(_quiet()) == []
    assert check_run(_quiet(), _quiet(), 1.2) == []


def test_rizado_y_saturacion_paran():
    ind = _quiet()
    ind["ripple"][3] = RIPPLE_MAX * 1.01
    ind["sat"][5] = SAT_MAX * 1.01
    reasons = check_run(ind)
    assert len(reasons) == 2
    assert "wrist_1" in reasons[0] and "wrist_3" in reasons[1]


@pytest.mark.parametrize("j", sorted(PRE_FAILURE_LEVEL))
def test_el_nivel_previo_al_fallo_para(j):
    ind = _quiet()
    ind["bands"][PRE_FAILURE_BAND][j] = PRE_FAILURE_FRACTION * PRE_FAILURE_LEVEL[j]
    assert len(check_run(ind)) == 1


def test_crecer_mas_del_doble_de_lo_esperado_para():
    prev, ind = _quiet(), _quiet()
    prev["bands"][_BAND2][3] = 0.015
    ind["bands"][_BAND2][3] = 0.015 * 2.0 * 1.2 * 1.01
    reasons = check_run(ind, prev, 1.2)
    assert len(reasons) == 1 and "wrist_1" in reasons[0]


def test_crecer_lo_esperado_no_para():
    prev, ind = _quiet(), _quiet()
    prev["bands"][_BAND2][3] = 0.015
    ind["bands"][_BAND2][3] = 0.015 * 2.0 * 1.2 * 0.99
    assert check_run(ind, prev, 1.2) == []


def test_por_debajo_del_suelo_de_ruido_no_mira_el_crecimiento():
    prev, ind = _quiet(), _quiet()
    prev["bands"][_BAND2][3] = 1e-4
    ind["bands"][_BAND2][3] = AMPLITUDE_FLOOR * 0.99
    assert check_run(ind, prev, 1.2) == []


def test_comparar_sin_ratio_es_un_error():
    with pytest.raises(ValueError):
        check_run(_quiet(), _quiet())


def _csv_real(n):
    path = os.path.join(_CSV, f"smc_{n}.csv")
    if not os.path.exists(path):
        pytest.skip(f"sin {path}")
    return path


def test_la_rampa_limpia_del_codo_no_para():
    """`smc_713`–`717`, con el barrido del codo que subía: G de 30 a 109."""
    from run_gain_ramp_real import run_indicators
    g = [29.9, 43.2, 60.8, 82.8, 109.3]
    ind = [run_indicators(_csv_real(n), 0.30) for n in (713, 714, 715, 716, 717)]
    assert check_run(ind[0]) == []
    for i in range(1, len(ind)):
        assert check_run(ind[i], ind[i - 1], g[i] / g[i - 1]) == []


def test_el_fallo_de_712_para():
    from run_gain_ramp_real import run_indicators
    reasons = check_run(run_indicators(_csv_real(712), 0.30))
    assert any("rizado" in r for r in reasons)
    assert any("saturados" in r for r in reasons)


# ── ficheros ────────────────────────────────────────────────────────────────


def test_la_candidata_es_la_fila_46_del_frente():
    rows = list(csv.reader(open(os.path.join(_V7, "pareto.csv"))))
    idx = {}
    for i, h in enumerate(rows[0]):
        idx.setdefault(h, i)
    row = [r for r in rows[1:] if r[0] == "nsga2"][46]
    g = _gains()
    for key, name in (("lambda", "lambda"), ("eta", "eta"), ("phi_joint", "phi")):
        expected = [float(row[idx[f"{name}{j}"]]) for j in range(1, 7)]
        np.testing.assert_allclose(g[key], expected, atol=1e-8)   # el YAML redondea a 8
    assert g["alpha_g"] == 0.0543


def test_los_parametros_de_la_rampa_solo_cambian_el_barrido():
    def load(name):
        return next(iter(yaml.safe_load(open(os.path.join(_CONFIG, name))).values()))[
            "ros__parameters"]
    ramp, base = load("gain_ramp_params.yaml"), load("sweep_smc_params.yaml")
    assert ramp["sweep"]["amplitude"] == RAMP_AMPLITUDE
    assert max(ramp["sweep"]["velocities"]) == RAMP_SPEED
    for doc in (ramp, base):
        del doc["sweep"]["amplitude"], doc["sweep"]["velocities"]
    assert ramp == base


def test_la_sesion_nunca_fuerza_el_chequeo_de_arranque():
    """Si el nodo rechaza un escalón, ese escalón no se corre."""
    code = [line for line in open(os.path.join(_SCRIPTS, "run_gain_ramp_real.py"),
                                  encoding="utf-8")
            if "allow_unstable" in line and "Nunca pasa" not in line]
    assert code == []


def test_el_launch_recibe_el_escalon_y_las_guardas():
    from run_gain_ramp_real import launch_args
    args = launch_args(np.arange(6.0), "/g.yaml", np.full(6, 2.5), np.full(6, 0.25))
    assert "gains_file:=/g.yaml" in args
    assert "lambda_joint:=" + ",".join(["2.500000"] * 6) in args
    assert "phi_joint:=" + ",".join(["0.250000"] * 6) in args
    assert "friction_compensation:=viscous_coulomb" in args
    assert "watchdog_sat_frac_max:=0.1" in args and "watchdog_q_err_max:=0.3" in args


def test_ensayo_en_seco_no_pide_nada(capsys):
    from run_gain_ramp_real import main
    _plant()
    params = os.path.join(_CONFIG, "gain_ramp_params.yaml")
    assert main(["--gains-file", _CANDIDATA, "--params-file", params, "--dry-run"]) == 0
    out = capsys.readouterr().out
    assert "20 corridas" in out and "r = 1.00" in out and "allow_unstable" not in out


def test_sin_base_ni_payload_no_ejecuta(capsys):
    from run_gain_ramp_real import main
    _plant()
    params = os.path.join(_CONFIG, "gain_ramp_params.yaml")
    assert main(["--gains-file", _CANDIDATA, "--params-file", params]) == 1
    assert "obligatorios" in capsys.readouterr().out
