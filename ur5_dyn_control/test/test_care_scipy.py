"""B4.2: la CARE de 18 estados del LQI-SDRE (solveCare, C++) frente a
scipy.linalg.solve_continuous_are, con pesos FISICOS y Coriolis no nulo."""

import os
import subprocess

import numpy as np
import pytest


def _sdc(rng):
    a = rng.uniform(-1, 1, (6, 6))
    d = np.sqrt([2.6, 2.6, 0.9, 0.03, 0.006, 3e-4])
    M = np.diag(d) @ (0.3 * a @ a.T + np.eye(6)) @ np.diag(d)
    C = rng.uniform(-0.05, 0.05, (6, 6))
    Mi = np.linalg.inv(M)
    I, Z = np.eye(6), np.zeros((6, 6))
    A = np.block([[Z, I, Z], [Z, Z, I], [Z, Z, -Mi @ C]])
    B = np.vstack([Z, Z, Mi])
    return A, B


def test_care_18_igual_que_scipy():
    exe = os.environ.get("DUMP_CARE", "")
    if not os.path.exists(exe):
        pytest.fail(f"DUMP_CARE no apunta al ejecutable de prueba: {exe!r}")
    from scipy.linalg import solve_continuous_are
    rng = np.random.default_rng(7)
    for _ in range(5):
        A, B = _sdc(rng)
        Q = np.diag(np.r_[np.full(6, 1e4), np.full(6, 1e6), np.full(6, 1e3)])
        R = np.diag(rng.uniform(1e-2, 1.0, 6))
        txt = "18 6\n" + "\n".join(" ".join(repr(v) for v in row)
                                    for X in (A, B, Q, R) for row in X) + "\n"
        out = subprocess.run([exe], input=txt, capture_output=True, text=True,
                             timeout=30).stdout.split("\n")
        assert out[0].strip() == "1", "solveCare no encontro solucion"
        K = np.array([r.split() for r in out[1:7]], dtype=float)
        P = solve_continuous_are(A, B, Q, R)
        K_ref = np.linalg.solve(R, B.T @ P)
        assert np.abs(K - K_ref).max() / np.abs(K_ref).max() < 1e-8
