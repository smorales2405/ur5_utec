"""B7.0: el retardo de tuberia se recupera de los escalones del par mandado."""

import numpy as np
import pytest

from ur5_identification.pipeline_delay import (event_average, fit_shift,
                                               step_events, xcorr_delay)


def synthetic(delay, n_steps=20, noise=0.05, seed=0):
    """Par con escalones de 7 N.m en rampa de 3 muestras (como el feedforward
    de Coulomb) y corriente = par retrasado `delay` muestras + ruido."""
    rng = np.random.default_rng(seed)
    n = 60 * n_steps
    u = np.zeros(n)
    level = -3.5
    for i in range(n):
        if i % 60 == 30:
            level = -level
        u[i] = level
    u = np.convolve(u, [0.25, 0.5, 0.25], mode="same")      # rampa de 3 muestras
    k = np.arange(n, dtype=float)
    y = np.interp(k - delay, k, u) + noise * rng.standard_normal(n)
    return u, y


@pytest.mark.parametrize("delay", [1.0, 2.6, 5.0, 5.4])
def test_recupera_el_retardo_fraccionario(delay):
    u, y = synthetic(delay)
    ev = step_events(u, 2.0)
    assert len(ev) >= 15
    U, Y, n = event_average(u, y, ev)
    D, a, r = fit_shift(U, Y)
    assert D == pytest.approx(delay, abs=0.05)
    assert a == pytest.approx(1.0, abs=0.02)
    assert r < 0.02


def test_ganancia_distinta_de_uno_no_mueve_el_retardo():
    u, y = synthetic(5.0)
    U, Y, _ = event_average(u, 0.8 * y + 3.0, step_events(u, 2.0))
    D, a, _ = fit_shift(U, Y)
    assert D == pytest.approx(5.0, abs=0.05)
    assert a == pytest.approx(0.8, abs=0.02)


def test_sin_escalones_no_hay_eventos():
    u = np.sin(np.linspace(0, 20, 2000))
    assert len(step_events(u, 2.0)) == 0
    assert event_average(u, u, np.array([], int))[2] == 0


def test_xcorr_en_banda_ancha():
    rng = np.random.default_rng(1)
    u = rng.standard_normal(5000)
    y = np.roll(u, 3)
    lag, rho = xcorr_delay(u, y)
    assert lag == pytest.approx(3.0, abs=0.1)
    assert rho > 0.9
