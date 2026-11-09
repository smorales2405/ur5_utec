"""D36: la medida de fricción de shoulder_lift a baja velocidad, con la carga del corte."""

import os
import sys

import numpy as np
import pytest
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.join(os.path.dirname(_HERE), "scripts")
sys.path.insert(0, _SCRIPTS)

from ur5_identification.lift_friction import (AMPLITUDE, LIFT, SPEEDS,  # noqa: E402
                                              duration_s, sweep_overrides)


def _plant():
    closed_loop = pytest.importorskip("ur5_trajectory_optimization.gain_tuning.closed_loop")
    try:
        return closed_loop.Plant(closed_loop.default_urdf())
    except Exception as exc:
        pytest.skip(f"sin URDF: {exc}")


def test_la_postura_tiene_la_carga_del_corte_y_holgura():
    """La campaña de agosto midió en q_init, con la mitad de carga gravitatoria."""
    import pinocchio as pin
    from run_lift_friction_real import plan
    from ur5_identification.gravity_sweeps import MIN_CLEARANCE
    plant = _plant()
    q, clear, move, g_lift, problems = plan(plant)
    q_cut = np.array([1.5708, -0.8481, 1.4684, -2.1911, -1.5708, 3.1416])
    g_cut = pin.computeGeneralizedGravity(plant.model, plant.data, q_cut)[LIFT]
    assert g_lift == pytest.approx(g_cut, rel=0.10)
    assert problems == [] and clear >= MIN_CLEARANCE and move >= MIN_CLEARANCE


def test_las_velocidades_cubren_el_corte_y_enlazan_con_agosto():
    assert list(SPEEDS) == sorted(SPEEDS)
    assert 0.015 in SPEEDS, "la de shoulder_lift en el corte de smc_1650"
    assert 0.02 in SPEEDS and 0.05 in SPEEDS, "para compararlas con la tabla de agosto"
    assert min(SPEEDS) < 0.015 and AMPLITUDE <= 0.06
    assert duration_s() < 200


def test_solo_cambia_el_barrido(tmp_path):
    from run_lift_friction_real import share, write_params
    q = np.array([1.5708, -0.9964, 1.4083, -1.9827, -1.5708, 3.1416])
    new = yaml.safe_load(open(write_params(q, str(tmp_path / "p.yaml"))))
    base = yaml.safe_load(open(share("sweep_params.yaml")))
    (node,) = new
    for doc in (new, base):
        doc[node]["ros__parameters"].pop("q_init", None)
        for key in sweep_overrides(q):
            doc[node]["ros__parameters"]["sweep"].pop(key, None)
    assert new == base
    sweep = yaml.safe_load(open(tmp_path / "p.yaml"))[node]["ros__parameters"]["sweep"]
    assert sweep["joint"] == LIFT and all(isinstance(v, float) for v in sweep["velocities"])


def test_ensayo_en_seco_no_lanza_nada(capsys, tmp_path):
    from run_lift_friction_real import main
    _plant()
    assert main(["--dry-run", "--csv-dir", str(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert "cur_1660" in out and "ros2 launch" not in out and "ros2 run" not in out


def test_no_sobrescribe_una_corrida(capsys, tmp_path):
    from run_lift_friction_real import main
    _plant()
    (tmp_path / "cur_1660.csv").write_text("t_sim\n")
    assert main(["--dry-run", "--csv-dir", str(tmp_path)]) == 1
    assert "no se sobrescriben" in capsys.readouterr().out


def test_la_confirmacion_va_antes_del_barrido():
    src = open(os.path.join(_SCRIPTS, "run_lift_friction_real.py"), encoding="utf-8").read()
    assert src.index("confirm(JOINT_NAME") < src.index('"run_current_sweep.py"')
    assert src.index('chk.set_friction(*LEVELS["0.0"])') < src.index("confirm(JOINT_NAME")
    assert "por posición" in src[src.index("confirm(JOINT_NAME"):][:80], "aquí no se comanda par"


def test_la_tabla_del_barrido_se_construye_sin_esperar_el_timeout(tmp_path):
    """D36: el nodo escribe la tabla y se queda en PRE_HOLD; antes se le esperaban 120 s sin decir
    nada, y la sesión pareció colgada. Ahora se para en cuanto la anuncia."""
    import time
    from ament_index_python.packages import get_package_prefix
    from run_current_sweep import build_reference, load_reference
    from run_lift_friction_real import plan, write_params
    exe = os.path.join(get_package_prefix("ur5_dyn_control"), "lib", "ur5_dyn_control",
                       "gz_fl_control_node")
    if not os.path.exists(exe):
        pytest.skip("sin gz_fl_control_node instalado")
    q = plan(_plant())[0]
    params = write_params(q, str(tmp_path / "p.yaml"))
    t0 = time.time()
    assert build_reference(LIFT, params, str(tmp_path / "ref.csv"))
    assert time.time() - t0 < 60
    dt, q_ref, dq_ref, phase = load_reference(str(tmp_path / "ref.csv"))
    labels = {p for p in phase if p.startswith("SWEEP_0")}
    assert {f"SWEEP_{v:.3f}_POS" for v in SPEEDS} <= labels
    assert np.abs(q_ref[:, LIFT] - q[LIFT]).max() == pytest.approx(AMPLITUDE, abs=1e-6)
