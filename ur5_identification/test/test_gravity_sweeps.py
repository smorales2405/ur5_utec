"""B4.4: posturas, holgura y la parte par que separa g_real de g_UR."""

import os
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from ur5_identification.force_estimator import K_CURRENT  # noqa: E402
from ur5_identification.gravity_sweeps import (  # noqa: E402
    CUT_X, DEFAULT_POSTURES, GRAVITY_JOINTS, MIN_CLEARANCE, Q_SEED, TABLE_Z,
    analyze_run, even_part, incision_posture, sweep_min_tip_z, tip_position)

try:
    from ur5_trajectory_optimization.gain_tuning.closed_loop import (
        Q_INIT, TOOL_SCALPEL, Plant, default_urdf)
    URDF = default_urdf()
except Exception:  # pragma: no cover
    URDF = ""

needs_model = pytest.mark.skipif(not os.path.exists(URDF),
                                 reason="URDF/evaluador no instalado")


@needs_model
def test_la_ik_reproduce_el_inicio_de_la_incision():
    plant = Plant(URDF)
    q = incision_posture(plant, 0.4919, 0.3469)
    assert np.abs(q - Q_SEED).max() < 1e-3
    assert q[1] + q[2] + q[3] == pytest.approx(-np.pi / 2, abs=1e-9)
    assert tip_position(plant, q) == pytest.approx([CUT_X, 0.4919, 0.3469], abs=1e-5)


@needs_model
def test_los_barridos_no_bajan_a_la_mesa():
    plant = Plant(URDF)
    for name, y, z in DEFAULT_POSTURES:
        q = incision_posture(plant, y, z)
        for j in GRAVITY_JOINTS:
            assert sweep_min_tip_z(plant, q, j, 0.05) - TABLE_Z > MIN_CLEARANCE, name


def test_la_parte_par_cancela_la_friccion():
    q = np.r_[np.linspace(-0.05, 0.05, 400), np.linspace(0.05, -0.05, 400)]
    sign = np.r_[np.ones(400), -np.ones(400)]
    g = 3.0 + 20.0 * q                                   # gravedad que varía con q
    y = g + sign * (6.76 + 11.55 * 0.05)                 # Coulomb + viscoso, impares
    c, e = even_part(q, y, sign)
    assert len(c) > 15
    assert np.abs(e - (3.0 + 20.0 * c)).max() < 0.06    # error de bin (pendiente)


def _synthetic_run(tmp_path, plant, j, delta, eps, delay):
    """Barrido de la junta j en el real: g_real = ĝ + delta (junta j),
    g_UR = ĝ + eps (todas), fricción impar, par del motor = corriente·k, y el
    par mandado actúa `delay` filas después."""
    import pinocchio as pin
    q_fixed = incision_posture(plant, 0.65, 0.06)
    rows, dt = [], 0.002
    for v in (0.05, 0.10):
        n = int(0.1 / v / dt)
        for sgn, path in ((+1, np.linspace(-0.05, 0.05, n)),
                          (-1, np.linspace(0.05, -0.05, n))):
            for a in path:
                rows.append((a, sgn * v, f"SWEEP_{v:.3f}_{'POS' if sgn > 0 else 'NEG'}"))
        rows += [(-0.05, 0.0, "SWEEP_RAMP")] * 20
    n = len(rows)
    q = np.tile(q_fixed, (n, 1))
    dq = np.zeros((n, 6))
    q[:, j] += [r[0] for r in rows]
    dq[:, j] = [r[1] for r in rows]
    g = np.array([pin.computeGeneralizedGravity(plant.model, plant.data, qi) for qi in q])
    c = np.array([pin.rnea(plant.model, plant.data, qi, dqi, np.zeros(6)) - gi
                  for qi, dqi, gi in zip(q, dq, g)])
    motor = g + c
    motor[:, j] += delta + np.sign(dq[:, j]) * 6.76 + 11.55 * dq[:, j]
    g_ur = g + eps
    tau = np.empty_like(motor)
    tau[:n - delay] = motor[delay:] - g_ur[delay:]
    tau[n - delay:] = tau[n - delay - 1]
    cur = motor / K_CURRENT
    path = tmp_path / "fl_1.csv"
    with open(path, "w") as fh:
        fh.write("# controller_id=fl\n")
        cols = (["t_sim"] + [f"q{i}" for i in range(1, 7)]
                + [f"dq{i}_des" for i in range(1, 7)] + [f"cur{i}" for i in range(1, 7)]
                + [f"tau{i}" for i in range(1, 7)] + ["state"])
        fh.write(",".join(cols) + "\n")
        for k in range(n):
            vals = [1.79e9 + k * dt, *q[k], *dq[k], *cur[k], *tau[k]]
            fh.write(",".join(f"{x:.9f}" for x in vals) + f",{rows[k][2]}\n")
    side = {"gz_fl_control_node": {"ros__parameters": {
        "sweep.joint": j, "sweep.q_fixed": [float(x) for x in q_fixed],
        "tool_mass": TOOL_SCALPEL.mass, "tool_com": list(TOOL_SCALPEL.com),
        "tool_inertia": list(TOOL_SCALPEL.inertia), "tcp_offset_z": 0.162686}}}
    with open(tmp_path / "fl_1.params.yaml", "w") as fh:
        yaml.safe_dump(side, fh)
    return str(path)


@needs_model
@pytest.mark.parametrize("j", [1, 3])
def test_recupera_g_real_y_g_ur_con_retardo(tmp_path, j):
    plant = Plant(URDF)
    eps = np.array([0.0, -0.4, 0.3, 0.05, 0.0, 0.0])
    delta = 0.8 if j == 1 else -0.1
    csv = _synthetic_run(tmp_path, plant, j, delta, eps, delay=5)
    r = analyze_run(csv)                      # real -> D = 5 y k = K_CURRENT
    assert r["real"] and r["delay"] == 5 and r["joint"] == j
    for lv in r["levels"].values():
        assert lv["g_real_minus_ghat"] == pytest.approx(delta, abs=0.02)
    assert r["g_ur_minus_ghat"] == pytest.approx(eps, abs=1e-6)
    assert r["delta"] == pytest.approx(delta - eps[j], abs=0.02)
    # Con el retardo mal supuesto, g_UR sale contaminada por la rampa del par.
    assert analyze_run(csv, delay=0)["g_ur_minus_ghat_std"][j] > \
        10 * r["g_ur_minus_ghat_std"][j] + 1e-9


@needs_model
def test_firma_del_payload_igual_al_modelo_con_acople():
    import pinocchio as pin
    from check_session_config import COG_ACOPLE_MM, payload_signature
    a, b = Plant(URDF), Plant(URDF, tool=None)
    expected = (pin.computeGeneralizedGravity(a.model, a.data, Q_INIT)
                - pin.computeGeneralizedGravity(b.model, b.data, Q_INIT))
    sig = payload_signature(TOOL_SCALPEL.mass, COG_ACOPLE_MM, Q_INIT)
    assert sig == pytest.approx(expected, abs=1e-6)
    assert payload_signature(0.0, COG_ACOPLE_MM).any() == False  # noqa: E712
