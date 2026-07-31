"""
Tests del estimador por DIFERENCIACIÓN entre sentidos (FASE 2).

Lo que hay que demostrar es la propiedad que motiva el método: que la gravedad
—y en general cualquier término PAR en la velocidad— se cancela exactamente, de
modo que el resultado no depende de conocer el modelo dinámico. Por eso los
tests construyen un CSV sintético con una gravedad ARBITRARIA y comprueban que
la estimación no cambia.
"""

import csv
import os
import tempfile

import numpy as np
import pytest

from ur5_identification.differencing import (difference_points, fit_difference,
                                             identify_by_differencing)

LEVELS = [0.05, 0.10, 0.20, 0.35, 0.50, 0.75, 1.00]
F_V_TRUE, F_C_TRUE = 1.5, 2.5

HEADER = (["t"] + [f"q{i}" for i in range(1, 7)] + [f"dq{i}" for i in range(1, 7)] +
          [f"q{i}_des" for i in range(1, 7)] + [f"dq{i}_des" for i in range(1, 7)] +
          [f"ddq{i}_des" for i in range(1, 7)] + [f"tau{i}" for i in range(1, 7)] +
          ["x", "y", "z", "x_des", "y_des", "z_des", "state"])


def write_sweep_csv(path, gravity_fn, f_v=F_V_TRUE, f_c=F_C_TRUE,
                    joint=0, amplitude=0.7854, n=300):
    """
    CSV sintético de barrido. El par comandado es

        tau = gravity_fn(q) + f_v*dq + f_c*sign(dq)

    donde gravity_fn es cualquier función SOLO de la posición: es el término que
    la diferenciación debe cancelar.
    """
    rows = []
    t = 0.0
    dt = 0.002
    for speed in LEVELS:
        for side, sgn in (("POS", +1.0), ("NEG", -1.0)):
            q_traj = np.linspace(-sgn * amplitude, sgn * amplitude, n)
            for q in q_traj:
                dq = sgn * speed
                tau = gravity_fn(q) + f_v * dq + f_c * np.sign(dq)
                row = [t] + [0.0] * 36 + [0.0] * 6 + [0.0] * 6 + ["placeholder"]
                # Reconstrucción explícita para no depender del orden de arriba.
                vals = {name: 0.0 for name in HEADER}
                vals["t"] = t
                vals[f"q{joint + 1}"] = q
                vals[f"dq{joint + 1}"] = dq
                vals[f"tau{joint + 1}"] = tau
                vals["state"] = f"SWEEP_{speed:.3f}_{side}"
                rows.append([vals[name] for name in HEADER])
                t += dt
            # Reposo entre barridos, que debe ser ignorado.
            for _ in range(20):
                vals = {name: 0.0 for name in HEADER}
                vals["t"] = t
                vals["state"] = "SWEEP_MOVE"
                rows.append([vals[name] for name in HEADER])
                t += dt

    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(HEADER)
        w.writerows(rows)


@pytest.fixture
def tmp_csv():
    fd, path = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    yield path
    os.unlink(path)


# ── La propiedad central: la gravedad se cancela ─────────────────────────────
@pytest.mark.parametrize("name,gravity_fn", [
    ("sin gravedad", lambda q: 0.0),
    ("gravedad lineal", lambda q: 12.0 * q),
    ("gravedad senoidal", lambda q: 20.0 * np.sin(q) + 5.0 * np.cos(2 * q)),
    ("offset grande", lambda q: 50.0),
])
def test_gravity_cancels_regardless_of_its_shape(tmp_csv, name, gravity_fn):
    write_sweep_csv(tmp_csv, gravity_fn)
    r = identify_by_differencing(tmp_csv, joint=0)
    assert abs(r.params.f_v - F_V_TRUE) < 1e-6, name
    assert abs(r.params.f_c - F_C_TRUE) < 1e-6, name
    assert r.r2 > 1 - 1e-9, name


def test_result_is_independent_of_gravity_model(tmp_csv):
    """Dos gravedades muy distintas deben dar EXACTAMENTE el mismo resultado."""
    write_sweep_csv(tmp_csv, lambda q: 0.0)
    a = identify_by_differencing(tmp_csv, joint=0)
    write_sweep_csv(tmp_csv, lambda q: 37.0 * np.sin(3 * q) - 11.0)
    b = identify_by_differencing(tmp_csv, joint=0)
    assert abs(a.params.f_v - b.params.f_v) < 1e-9
    assert abs(a.params.f_c - b.params.f_c) < 1e-9


# ── Extracción de las ventanas ───────────────────────────────────────────────
def test_uses_only_plateau_windows(tmp_csv):
    write_sweep_csv(tmp_csv, lambda q: 3.0 * q)
    pts = difference_points(tmp_csv, joint=0)
    assert len(pts) == len(LEVELS)
    assert [p.speed for p in pts] == sorted(LEVELS)
    # Los dos sentidos recorren el mismo rango, así que el solape es casi total.
    for p in pts:
        assert p.q_overlap > 1.3, f"solape insuficiente a |v|={p.speed}"


def test_recovers_a_different_parameter_set(tmp_csv):
    write_sweep_csv(tmp_csv, lambda q: 8.0 * np.sin(q), f_v=0.42, f_c=0.13)
    r = identify_by_differencing(tmp_csv, joint=0)
    assert abs(r.params.f_v - 0.42) < 1e-6
    assert abs(r.params.f_c - 0.13) < 1e-6


def test_pure_viscous_gives_zero_coulomb(tmp_csv):
    write_sweep_csv(tmp_csv, lambda q: 5.0 * q, f_v=1.1, f_c=0.0)
    r = identify_by_differencing(tmp_csv, joint=0)
    assert abs(r.params.f_v - 1.1) < 1e-6
    assert abs(r.params.f_c) < 1e-6


def test_requires_enough_levels():
    from ur5_identification.differencing import DifferencePoint
    pts = [DifferencePoint(0.1, 2.6, 0.0, 1.5, 400),
           DifferencePoint(0.2, 2.8, 0.0, 1.5, 400)]
    with pytest.raises(ValueError):
        fit_difference(pts)
