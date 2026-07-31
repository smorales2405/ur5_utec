"""
Tests del estimador de fricción sobre datos SINTÉTICOS (FASE 2).

Los datos sintéticos permiten verificar la matemática del estimador de forma
determinista y en milisegundos, separándola de la campaña en Gazebo (que valida
además el acoplamiento con la planta). Si estos tests fallan, el problema está
en el estimador; si pasan y la campaña no recupera los parámetros, el problema
está en la planta o en el residuo.
"""

import numpy as np
import pytest

from ur5_identification.estimator import cross_validate, fit, fit_stribeck
from ur5_identification.friction_models import (FrictionParams, friction_torque,
                                                regressor)


class FakeWindow:
    """Meseta sintética con la misma interfaz que residual.SweepWindow."""

    def __init__(self, velocity, residual_mean, n=1000, noise_std=0.0, rng=None):
        self.velocity = velocity
        self.joint = 0
        rng = rng or np.random.default_rng(0)
        self.dq = np.full(n, velocity)
        self.residual = residual_mean + noise_std * rng.standard_normal(n)
        self.t = np.arange(n) * 0.002

    @property
    def dq_mean(self):
        return float(np.mean(self.dq))

    @property
    def residual_mean(self):
        return float(np.mean(self.residual))

    @property
    def residual_std(self):
        return float(np.std(self.residual, ddof=1))


LEVELS = [0.02, 0.05, 0.10, 0.20, 0.35, 0.50, 0.75, 1.00]


def make_windows(p: FrictionParams, noise_std=0.0, seed=0):
    rng = np.random.default_rng(seed)
    out = []
    for lv in LEVELS:
        for s in (+1.0, -1.0):
            v = s * lv
            tau = float(friction_torque(np.array([v]), p)[0])
            out.append(FakeWindow(v, tau, noise_std=noise_std, rng=rng))
    return out


# ── Recuperación exacta sin ruido ────────────────────────────────────────────
def test_recovers_viscous_coulomb_exactly_without_noise():
    truth = FrictionParams(f_v=1.5, f_c=2.5, model="viscous_coulomb")
    r = fit(make_windows(truth), "viscous_coulomb")
    assert abs(r.params.f_v - 1.5) < 1e-9
    assert abs(r.params.f_c - 2.5) < 1e-9
    assert r.r2 > 1 - 1e-12
    assert r.n_obs == 2 * len(LEVELS)


def test_recovers_viscous_only():
    truth = FrictionParams(f_v=0.8, f_c=0.0, model="viscous")
    r = fit(make_windows(truth), "viscous")
    assert abs(r.params.f_v - 0.8) < 1e-9


# ── Criterio de aceptación del plan: 10 % con ruido realista ─────────────────
def test_meets_ten_percent_criterion_with_noise():
    truth = FrictionParams(f_v=1.5, f_c=2.5, model="viscous_coulomb")
    # sigma = 0.05 N·m por meseta es holgadamente peor que lo medido en Gazebo
    # (donde la dispersión intra-meseta fue de milinewton-metro).
    for seed in range(5):
        r = fit(make_windows(truth, noise_std=0.05, seed=seed), "viscous_coulomb")
        assert abs(r.params.f_v - 1.5) / 1.5 < 0.10, f"seed={seed}"
        assert abs(r.params.f_c - 2.5) / 2.5 < 0.10, f"seed={seed}"


def test_confidence_interval_covers_truth():
    truth = FrictionParams(f_v=1.5, f_c=2.5, model="viscous_coulomb")
    covered = 0
    for seed in range(20):
        r = fit(make_windows(truth, noise_std=0.05, seed=seed), "viscous_coulomb")
        if r.ci95[0, 0] <= 1.5 <= r.ci95[0, 1] and r.ci95[1, 0] <= 2.5 <= r.ci95[1, 1]:
            covered += 1
    # Un IC del 95 % debe cubrir la verdad en la gran mayoría de las réplicas.
    assert covered >= 17, f"cobertura {covered}/20"


# ── Validación cruzada ───────────────────────────────────────────────────────
def test_cross_validation_is_good_for_correct_model():
    truth = FrictionParams(f_v=1.5, f_c=2.5, model="viscous_coulomb")
    cv = cross_validate(make_windows(truth, noise_std=0.01), "viscous_coulomb")
    assert cv.r2 > 0.99
    # Deja fuera un nivel completo cada vez.
    assert len(cv.per_fold) == len(LEVELS)


def test_cross_validation_penalises_underfitting_model():
    """
    Con fricción de Coulomb presente, el modelo SOLO viscoso debe validar
    claramente peor. Es la comprobación de que la validación cruzada discrimina
    y no premia cualquier ajuste.
    """
    truth = FrictionParams(f_v=1.5, f_c=2.5, model="viscous_coulomb")
    w = make_windows(truth, noise_std=0.01)
    cv_good = cross_validate(w, "viscous_coulomb")
    cv_bad = cross_validate(w, "viscous")
    assert cv_bad.rmse > 10 * cv_good.rmse
    assert cv_good.r2 > cv_bad.r2


# ── Signo y simetría ─────────────────────────────────────────────────────────
def test_friction_is_odd_in_velocity():
    p = FrictionParams(f_v=1.5, f_c=2.5, model="viscous_coulomb")
    v = np.array([0.1, 0.5, 1.0])
    assert np.allclose(friction_torque(v, p), -friction_torque(-v, p))


def test_regressor_matches_model():
    p = FrictionParams(f_v=1.5, f_c=2.5, model="viscous_coulomb")
    v = np.array([-1.0, -0.1, 0.1, 1.0])
    phi = regressor(v, "viscous_coulomb")
    assert np.allclose(phi @ np.array([1.5, 2.5]), friction_torque(v, p))


# ── Stribeck ─────────────────────────────────────────────────────────────────
def test_stribeck_recovers_injected_curve():
    truth = FrictionParams(f_v=1.0, f_c=2.0, f_s=3.0, v_s=0.05, delta=2.0,
                           model="stribeck")
    r, v_s, delta = fit_stribeck(make_windows(truth))
    assert abs(r.params.f_v - 1.0) < 0.05
    assert abs(r.params.f_c - 2.0) < 0.1
    assert r.params.f_s > r.params.f_c          # estático por encima del dinámico
    assert 0.02 < v_s < 0.15                    # rejilla cerca del valor real


def test_fit_rejects_insufficient_data():
    truth = FrictionParams(f_v=1.5, f_c=2.5, model="viscous_coulomb")
    with pytest.raises(ValueError):
        fit(make_windows(truth)[:2], "viscous_coulomb")
