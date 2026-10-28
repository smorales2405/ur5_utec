"""B1: estimador de fuerza por corriente (ur5_identification/force_estimator.py)."""

import os

import numpy as np
import pytest

from ur5_identification.force_estimator import (
    FRICTION_REAL, estimate_force, mean_resolution)

try:
    from ur5_trajectory_optimization.gain_tuning.closed_loop import (
        Q_INIT, Plant, default_urdf)
    URDF = default_urdf()
except Exception:  # pragma: no cover - sin el paquete del evaluador
    URDF = ""


def _synthetic_motor_torque(plant, q, dq, ddq, F, friction):
    """Par de motor de una planta que empuja con F [N] desde la punta."""
    import pinocchio as pin
    tau = pin.rnea(plant.model, plant.data, q, dq, ddq)
    tau += np.array([p.torque(np.array([v]))[0] for p, v in zip(friction, dq)])
    J = pin.computeFrameJacobian(plant.model, plant.data, q, plant.tcp,
                                 pin.LOCAL_WORLD_ALIGNED)
    return tau + J[:3].T @ F


@pytest.mark.skipif(not os.path.exists(URDF), reason="URDF/evaluador no instalado")
def test_recupera_la_fuerza_con_todas_las_juntas_en_movimiento():
    plant = Plant(URDF)
    rng = np.random.default_rng(3)
    for _ in range(20):
        q = Q_INIT + rng.uniform(-0.5, 0.5, 6)
        dq = rng.choice([-1, 1], 6) * rng.uniform(0.01, 0.3, 6)
        ddq = rng.uniform(-1, 1, 6)
        F = rng.uniform(-20, 20, 3)
        tau = _synthetic_motor_torque(plant, q, dq, ddq, F, FRICTION_REAL)
        F_hat, obs = estimate_force(plant, q, dq, ddq, tau, FRICTION_REAL)
        assert obs.all()
        np.testing.assert_allclose(F_hat, F, atol=1e-8)


@pytest.mark.skipif(not os.path.exists(URDF), reason="URDF/evaluador no instalado")
def test_en_la_incision_la_fuerza_lateral_no_es_observable():
    """D18: en el corte se mueven solo hombro, codo y wrist_1 (plano del brazo).
    F_y y F_z salen; F_x solo la ven juntas paradas, cuya friccion es
    indeterminada: tiene que salir NaN, no un numero."""
    plant = Plant(URDF)
    q = Q_INIT.copy()
    dq = np.array([0.0, 0.015, -0.025, 0.010, 0.0, 0.0])
    ddq = np.zeros(6)
    F = np.array([4.0, 3.0, -5.0])
    tau = _synthetic_motor_torque(plant, q, dq, ddq, F, FRICTION_REAL)
    # Una junta parada puede llevar cualquier friccion en [-F_c, F_c]: se le
    # suma un sesgo que el estimador no debe atribuir a la fuerza.
    tau[[0, 4, 5]] += np.array([3.0, -1.0, 0.7])
    F_hat, obs = estimate_force(plant, q, dq, ddq, tau, FRICTION_REAL)
    assert list(obs) == [False, True, True]
    assert np.isnan(F_hat[0])
    # 1e-4 N y no 1e-8: q_init lleva 1.5708 y no pi/2 exacto, asi que la F_x
    # (4 N) se cuela ~1e-6 por las juntas del plano. Despreciable.
    np.testing.assert_allclose(F_hat[1:], F[1:], atol=1e-4)


def test_resolucion_de_la_media_corrige_la_autocorrelacion():
    rng = np.random.default_rng(0)
    n = 4000
    blanco = rng.standard_normal(n)
    m, se = mean_resolution(blanco)
    assert se == pytest.approx(1 / np.sqrt(n), rel=0.15)
    # AR(1) con rho = 0.9: el error tipico crece sqrt((1+rho)/(1-rho)) = 4.36x.
    x = np.zeros(n)
    e = rng.standard_normal(n) * np.sqrt(1 - 0.81)
    for i in range(1, n):
        x[i] = 0.9 * x[i - 1] + e[i]
    _, se_ar = mean_resolution(x)
    assert se_ar / (np.std(x) / np.sqrt(n)) == pytest.approx(4.36, rel=0.25)
