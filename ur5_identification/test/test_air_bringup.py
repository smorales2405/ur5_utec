"""B7.1 (plan D29): el SMC en aire con las ganancias finales, la base y la incisión."""

import os
import sys

import numpy as np
import pytest
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_HERE))
_SCRIPTS = os.path.join(os.path.dirname(_HERE), "scripts")
sys.path.insert(0, _SCRIPTS)

from ur5_identification.air_bringup import (BASE_JOINT, BASE_TIP,  # noqa: E402
                                            INCISION_MIN_CLEARANCE, INCISION_Q_ERR,
                                            INCISION_STEPS, INCISION_TCP_MAX, MODE_BAND,
                                            PAN_INCISION_R, PAN_STEPS, base_runs,
                                            check_incision, incision_indicators,
                                            incision_r_vector, incision_run, incision_runs,
                                            label, r_vector)
from ur5_identification.gain_ramp import (PRE_FAILURE_BAND, RAMP_STEPS,  # noqa: E402
                                          check_run, step_r)

_CANDIDATA = os.path.join(_SRC, "ur5_trajectory_optimization", "results", "gain_tuning",
                          "smc_v7_rotor", "candidata_fila46.yaml")
_CONFIG = os.path.join(_SRC, "ur5_dyn_control", "config")


def _plant():
    closed_loop = pytest.importorskip(
        "ur5_trajectory_optimization.gain_tuning.closed_loop")
    try:
        return closed_loop.Plant(closed_loop.default_urdf())
    except Exception as exc:     # sin URDF instalado
        pytest.skip(f"sin URDF: {exc}")


def _gains():
    return next(iter(yaml.safe_load(open(_CANDIDATA)).values()))["ros__parameters"]


def _common():
    return next(iter(yaml.safe_load(open(os.path.join(_CONFIG, "incision_common.yaml")))
                     .values()))["ros__parameters"]


# ── ganancias y corridas ────────────────────────────────────────────────────


def test_solo_baja_shoulder_pan():
    np.testing.assert_allclose(r_vector(0.71), [0.71, 1, 1, 1, 1, 1])
    assert label(r_vector(0.5)) == "1.00 (shoulder_pan 0.50)"


@pytest.mark.parametrize("r", [0.0, 1.2])
def test_r_fuera_de_rango_se_rechaza(r):
    with pytest.raises(ValueError):
        r_vector(r)


def test_las_ganancias_de_la_incision_son_las_de_d28():
    """Con 0.71 en la base, el último escalón de la rampa (`smc_1530`–`1533`)."""
    np.testing.assert_allclose(r_vector(PAN_STEPS[-1]), step_r(3))


def test_la_base_sube_de_050_a_071_y_compara_junta_a_junta():
    runs = base_runs(1600)
    assert [r["test_num"] for r in runs] == [1600, 1601]
    assert [r["pan_r"] for r in runs] == [0.50, 0.71]
    assert all(r["joint"] == BASE_JOINT for r in runs)
    assert runs[0]["previous"] is None and runs[1]["previous"] == 1600
    np.testing.assert_allclose(runs[1]["ratio"], [0.71 / 0.50, 1, 1, 1, 1, 1])


def test_la_incision_va_por_escalones_con_la_base_en_050_y_wrist_1_en_1():
    """D31: tras `smc_1610`, los escalones de la rampa, y `shoulder_pan` donde dejó el paso 1.
    D34: `wrist_1` a r = 1, porque con 0.50 se pegaba en el corte (`smc_1640`)."""
    assert INCISION_STEPS == RAMP_STEPS and PAN_INCISION_R == PAN_STEPS[0]
    np.testing.assert_allclose(incision_r_vector(0.50), [0.50, 0.50, 0.50, 1.0, 0.50, 0.50])
    np.testing.assert_allclose(incision_r_vector(0.71), [0.50, 0.71, 0.71, 1.0, 0.71, 0.71])
    np.testing.assert_allclose(incision_r_vector(1.00), [0.50] + [1.0] * 5)
    assert label(incision_r_vector(0.50)) == "0.50 (wrist_1 1.00)"
    assert label(incision_r_vector(0.71)) == "0.71 (shoulder_pan 0.50, wrist_1 1.00)"
    with pytest.raises(ValueError):
        incision_r_vector(0.6)


def test_la_incision_no_pisa_los_numeros_ya_usados():
    nums = [r["test_num"] for r in incision_runs(1600)]
    assert nums == [1670, 1671, 1672, 1673]
    assert not set(nums) & {r["test_num"] for r in base_runs(1600)}
    assert not {1610, 1620, 1630, 1640} & set(nums)


def test_cada_escalon_se_compara_con_el_anterior_junta_a_junta():
    run = incision_run(1600, 0.71)
    assert run["previous"] == 1670
    np.testing.assert_allclose(run["ratio"], [1.0, 0.71 / 0.5, 0.71 / 0.5, 1.0, 0.71 / 0.5,
                                              0.71 / 0.5])
    assert incision_run(1600, 0.50)["previous"] is None


def test_la_banda_del_modo_tambien_mira_el_crecimiento():
    """La de `smc_1610` (34.9 y 36.5 Hz) cae entre las de la rampa."""
    assert MODE_BAND[0] < 35.0 < 36.5 < MODE_BAND[1] and MODE_BAND[1] <= 40.0
    assert not PRE_FAILURE_BAND[0] <= 36.5 <= PRE_FAILURE_BAND[1]
    quiet = {"ripple": np.full(6, 0.001), "sat": np.zeros(6),
             "bands": {PRE_FAILURE_BAND: np.full(6, 0.005), MODE_BAND: np.full(6, 0.02)}}
    grown = {**quiet, "bands": {**quiet["bands"], MODE_BAND: np.array([0.02, 0.10, 0.02, 0.02,
                                                                       0.02, 0.02])}}
    reasons = check_run(grown, quiet, incision_run(1600, 0.71)["ratio"])
    assert len(reasons) == 1 and "shoulder_lift" in reasons[0] and "30-40" in reasons[0]


# ── la postura del paso 1 ───────────────────────────────────────────────────


def test_la_postura_de_la_base_tiene_la_inercia_de_la_incision():
    """Lo que se quiere medir: el modo de 33 Hz con la `G` y la `M₁₁` del corte, no las de la
    rampa (2026-11-05, `09c` §8.2)."""
    import pinocchio as pin
    from run_air_bringup_real import gains_at, incision_geometry
    from ur5_identification.gravity_sweeps import incision_posture
    from ur5_trajectory_optimization.gain_tuning.problem import ROTOR_INERTIA_REAL
    plant, g = _plant(), _gains()
    q_base = incision_posture(plant, *BASE_TIP)
    (y, z), _ = incision_geometry(_common())
    q_cut = incision_posture(plant, y, z)
    m = [pin.crba(plant.model, plant.data, q)[0, 0] for q in (q_base, q_cut)]
    assert m[0] == pytest.approx(m[1], rel=0.10)
    r = r_vector(PAN_STEPS[-1])
    g_base = gains_at(plant, q_base, g, r, ROTOR_INERTIA_REAL)["g"][0]
    g_cut = gains_at(plant, q_cut, g, r, ROTOR_INERTIA_REAL)["g"][0]
    assert g_base == pytest.approx(g_cut, rel=0.05)


def test_la_postura_de_la_base_cumple_g6_g7_y_la_holgura():
    from run_air_bringup_real import plan_base
    from ur5_trajectory_optimization.gain_tuning.problem import ROTOR_INERTIA_REAL
    from ur5_identification.gravity_sweeps import MIN_CLEARANCE
    _, rows, clear, problems = plan_base(_plant(), _gains(), ROTOR_INERTIA_REAL, 1600)
    assert problems == []
    assert clear >= MIN_CLEARANCE
    for s in rows:
        assert s["g"].max() <= 120.0 and s["margin"] >= 2.0


# ── la incisión ─────────────────────────────────────────────────────────────


def test_la_incision_deja_holgura_y_la_guarda_salta_antes_del_tablero():
    from run_air_bringup_real import incision_geometry
    from ur5_identification.gravity_sweeps import TABLE_Z
    (y, _), z_min = incision_geometry(_common())
    clearance = z_min - TABLE_Z
    assert clearance == pytest.approx(0.037, abs=1e-3)
    assert clearance >= INCISION_MIN_CLEARANCE
    assert INCISION_Q_ERR * 0.70 < clearance     # 0.7 m de alcance
    assert y == pytest.approx(0.70)              # empieza el corte lejos de la base


def test_los_escalones_de_la_incision_cumplen_g6_y_g7():
    from run_air_bringup_real import plan_incision
    from ur5_trajectory_optimization.gain_tuning.problem import ROTOR_INERTIA_REAL
    _, rows, _, problems = plan_incision(_plant(), _gains(), ROTOR_INERTIA_REAL, _common(), 1600)
    assert problems == []
    assert [s["r"] for s in rows] == list(INCISION_STEPS)
    for s in rows:
        assert s["g"].max() <= 120.0 and s["margin"] >= 2.0
    g_base = [s["g"][0] for s in rows]
    assert max(g_base) == pytest.approx(min(g_base), rel=0.01), "la base no sube"


def _incision_csv(err_z=0.0, z_offset=0.0):
    cols = ["t_wall", "x", "y", "z", "x_des", "y_des", "z_des"]
    n = 100
    z_des = np.concatenate([np.linspace(0.3, 0.0, 50), np.full(50, -0.009)])
    d = np.zeros((n, len(cols)))
    d[:, 0] = np.arange(n) * 0.002
    d[:, 5], d[:, 6] = 0.65, z_des
    d[:, 2], d[:, 3] = 0.65, z_des + err_z + z_offset
    return d, {c: k for k, c in enumerate(cols)}


def test_indicadores_de_la_incision():
    d, i = _incision_csv(err_z=0.001)
    ind = incision_indicators(d, i)
    assert ind["cut_rows"] == 50
    assert ind["tcp_rms_cut"] == pytest.approx(0.001)
    assert ind["z_min"] == pytest.approx(-0.008)
    assert check_incision(ind, -0.046) == []


def test_la_incision_para_si_baja_demasiado_o_se_aparta():
    d, i = _incision_csv(err_z=-0.02)
    reasons = check_incision(incision_indicators(d, i), -0.046)
    assert any("tablero" in r for r in reasons)
    assert any("TCP" in r for r in reasons)
    d, i = _incision_csv(err_z=INCISION_TCP_MAX * 0.9)
    assert check_incision(incision_indicators(d, i), -0.046) == []


def test_el_launch_de_la_incision():
    from run_air_bringup_real import incision_args
    args = incision_args("/g.yaml", np.full(6, 2.5), np.full(6, 0.25))
    assert "gains_file:=/g.yaml" in args
    assert "friction_compensation:=table" in args, "D33: la fricción medida"
    assert f"watchdog_q_err_max:={INCISION_Q_ERR}" in args
    assert "gravity_correction:=true" in args, "D32: la incisión corrige la gravedad"
    assert not any(a.startswith(("q_init", "sweep")) for a in args)


def test_run_one_no_mezcla_incision_y_barrido():
    from run_friction_campaign_real import run_one
    with pytest.raises(ValueError):
        run_one(0, 1, "/p.yaml", 0.3, "/tmp", trajectory_type="incision")
    with pytest.raises(ValueError):
        run_one(None, 1, "/p.yaml", 0.3, "/tmp", trajectory_type="circulo")


# ── la sesión ───────────────────────────────────────────────────────────────


def test_la_sesion_nunca_fuerza_el_chequeo_de_arranque():
    code = [line for line in open(os.path.join(_SCRIPTS, "run_air_bringup_real.py"),
                                  encoding="utf-8")
            if "allow_unstable" in line and "Nunca pasa" not in line]
    assert code == []


def test_la_confirmacion_va_antes_de_lanzar():
    src = open(os.path.join(_SCRIPTS, "run_air_bringup_real.py"), encoding="utf-8").read()
    loop = src[src.index("for k, s in enumerate(runs)"):]
    assert loop.index("confirm(word)") < loop.index("run_one(")


def test_ensayo_en_seco_de_la_base(capsys):
    from run_air_bringup_real import main
    _plant()
    assert main(["--gains-file", _CANDIDATA, "--paso", "base", "--test-base", "1600",
                 "--dry-run"]) == 0
    out = capsys.readouterr().out
    assert "smc_1600" in out and "smc_1601" in out and "2 corrida(s)" in out
    assert "ros2 launch" not in out and "allow_unstable" not in out


@pytest.fixture
def _no_runs(tmp_path, monkeypatch):
    """Las corridas hechas se miran en un directorio vacío, no en ~/.ros."""
    import run_air_bringup_real
    monkeypatch.setattr(run_air_bringup_real, "csv_path",
                        lambda n: str(tmp_path / f"smc_{n}.csv"))
    return tmp_path


def test_ensayo_en_seco_de_la_incision(capsys, _no_runs):
    from run_air_bringup_real import main
    _plant()
    assert main(["--gains-file", _CANDIDATA, "--paso", "incision", "--r", "0.5",
                 "--test-base", "1600", "--dry-run"]) == 0
    out = capsys.readouterr().out
    assert "smc_1670" in out and "37 mm" in out
    assert "r = 0.71 (shoulder_pan 0.50, wrist_1 1.00)" in out and "D34" in out
    assert "D32" in out and "D33" in out
    assert "ros2 launch" not in out


def test_un_escalon_exige_el_anterior(capsys, _no_runs, monkeypatch):
    import run_air_bringup_real
    from run_air_bringup_real import main
    _plant()
    monkeypatch.setattr(run_air_bringup_real, "INCISION_MAX_R", 1.0)   # sin el bloqueo de D35
    args = ["--gains-file", _CANDIDATA, "--paso", "incision", "--r", "0.71", "--test-base",
            "1600", "--dry-run"]
    assert main(args) == 1
    assert "smc_1670" in capsys.readouterr().out
    (_no_runs / "smc_1670.csv").write_text("t_wall\n")
    assert main(args) == 0
    assert "smc_1671" in capsys.readouterr().out


@pytest.mark.parametrize("r", ["0.71", "0.85", "1.0"])
def test_los_escalones_por_encima_de_050_estan_bloqueados(capsys, _no_runs, r):
    """D35: con `shoulder_lift` deslizando, el modo de 31.6 Hz se haría inestable hacia r ≈ 0.64.
    Aunque exista el escalón anterior, la sesión no los corre."""
    from run_air_bringup_real import main
    from ur5_identification.air_bringup import INCISION_MAX_R
    _plant()
    assert INCISION_MAX_R == 0.50
    for n in (1670, 1671, 1672):
        (_no_runs / f"smc_{n}.csv").write_text("t_wall\n")
    assert main(["--gains-file", _CANDIDATA, "--paso", "incision", "--r", r, "--test-base",
                 "1600", "--dry-run"]) == 1
    assert "bloqueado (D35)" in capsys.readouterr().out


def test_la_incision_pide_el_escalon(capsys):
    from run_air_bringup_real import main
    assert main(["--gains-file", _CANDIDATA, "--paso", "incision", "--dry-run"]) == 1
    assert "--r" in capsys.readouterr().out
