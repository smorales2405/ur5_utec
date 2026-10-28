"""
B4.3: test dorado de la ley SMC — el nodo (C++, smc_law.hpp) y el evaluador
offline (Python, SmcLaw) calculan el MISMO par.

Por qué existe. El optimizador de la FASE 7 sintoniza sobre `SmcLaw`; si la ley
del nodo y su espejo divergen, las ganancias se optimizan para un controlador
distinto del que va al robot, sin ningún síntoma. Pasó con la planta (B0.4) y
podía pasar con la ley al introducir R1 y R2.
"""

import os
import subprocess

import numpy as np
import pytest
import yaml

_CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "config")
N_CASOS = 200
TOL = 1e-9


def test_ley_smc_cpp_y_python_coinciden():
    exe = os.environ.get("DUMP_SMC_LAW", "")
    if not os.path.exists(exe):
        pytest.fail(f"DUMP_SMC_LAW no apunta al ejecutable de prueba: {exe!r}")
    from ur5_trajectory_optimization.gain_tuning.closed_loop import (
        Q_INIT, Plant, SmcLaw, default_urdf)

    with open(os.path.join(_CONFIG, "incision_common.yaml")) as fh:
        p = yaml.safe_load(fh)["/**"]["ros__parameters"]
    urdf = default_urdf()
    args = [exe, urdf, str(p["gravity"]), str(p["tcp_offset_z"]),
            str(p["tool_mass"])] + [str(v) for v in p["tool_com"]] \
        + [str(v) for v in p["tool_inertia"]]

    rng = np.random.default_rng(43)
    casos, lineas = [], []
    for k in range(N_CASOS):
        q = Q_INIT + rng.uniform(-0.6, 0.6, 6)
        dq = rng.uniform(-0.5, 0.5, 6)
        qd = q + rng.uniform(-0.02, 0.02, 6)        # dentro y fuera de la capa
        dqd = dq + rng.uniform(-0.2, 0.2, 6)
        ddqd = rng.uniform(-2.0, 2.0, 6)
        lam = rng.uniform(5.0, 200.0, 6)
        eta = rng.uniform(0.01, 5.0, 6)
        phi = rng.uniform(0.05, 1.0, 6)
        a = rng.uniform(0.0, 1.0, 3)
        sat = float(k % 4 != 0)                     # un cuarto con sign
        casos.append((q, dq, qd, dqd, ddqd, lam, eta, phi, a, sat))
        lineas.append(" ".join(repr(float(v)) for v in np.r_[
            q, dq, qd, dqd, ddqd, lam, eta, phi, a, sat]))
    out = subprocess.run(args, input="\n".join(lineas) + "\n", capture_output=True,
                         text=True, timeout=60, check=True).stdout.split("\n")
    filas = [np.array(r.split(), dtype=float) for r in out if r.strip()]
    assert len(filas) == N_CASOS

    plant = Plant(urdf, gravity=p["gravity"])
    err = 0.0
    for (q, dq, qd, dqd, ddqd, lam, eta, phi, a, sat), r in zip(casos, filas):
        law = SmcLaw(lam, eta, phi, use_sat=bool(sat),
                     alpha_M=a[0], alpha_C=a[1], alpha_g=a[2])
        tau, s, _ = law(plant.model, plant.data, q, dq, qd, dqd, ddqd)
        err = max(err, np.abs(tau - r[:6]).max(), np.abs(s - r[6:12]).max())
    assert err < TOL, f"la ley del nodo y la del evaluador difieren en {err:.3e}"
