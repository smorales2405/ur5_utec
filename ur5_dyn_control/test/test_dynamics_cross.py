"""
B0.4: el evaluador offline (Python) y el nodo (C++) usan la MISMA dinámica.

Por qué existe. El optimizador de la FASE 7 sintonizó sobre `Plant`, que hasta
B0 era el brazo SIN el acople del bisturí y con el TCP de 0.141 m, mientras el
nodo lleva los 0.182 kg y la punta a 0.162686 m (V2 del plan): las G que
predecía el evaluador salían un 3-5 % por debajo de las que imprimía el nodo.
Aquí se comparan M(q), g(q) y C(q, dq) de `Ur5Dynamics` (C++) con los de
`Plant` en configuraciones aleatorias, con la herramienta y el TCP que dice
config/incision_common.yaml.
"""

import os
import subprocess

import numpy as np
import pytest
import yaml

_CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "config")
N_CONFIGS = 50
TOL = 1e-9


def test_dinamica_cpp_y_python_coinciden():
    exe = os.environ.get("DUMP_DYNAMICS", "")
    if not os.path.exists(exe):
        pytest.fail(f"DUMP_DYNAMICS no apunta al ejecutable de prueba: {exe!r}")
    from ur5_trajectory_optimization.gain_tuning.closed_loop import (
        Plant, default_urdf)
    import pinocchio as pin

    with open(os.path.join(_CONFIG, "incision_common.yaml")) as fh:
        p = yaml.safe_load(fh)["/**"]["ros__parameters"]
    urdf = default_urdf()
    args = [exe, urdf, str(p["gravity"]), str(p["tcp_offset_z"]),
            str(p["tool_mass"])] + [str(v) for v in p["tool_com"]] \
        + [str(v) for v in p["tool_inertia"]]

    rng = np.random.default_rng(20261027)
    q = rng.uniform(-np.pi, np.pi, (N_CONFIGS, 6))
    dq = rng.uniform(-2.0, 2.0, (N_CONFIGS, 6))
    stdin = "\n".join(" ".join(repr(v) for v in np.r_[a, b]) for a, b in zip(q, dq))
    out = subprocess.run(args, input=stdin + "\n", capture_output=True,
                         text=True, timeout=60, check=True).stdout.split("\n")
    rows = [np.array(r.split(), dtype=float) for r in out if r.strip()]
    assert len(rows) == N_CONFIGS

    # La planta por DEFECTO del evaluador: tiene que ser la de la incision.
    plant = Plant(urdf, gravity=p["gravity"])
    m, d = plant.model, plant.data
    err = {"M": 0.0, "g": 0.0, "C": 0.0}
    for qk, dqk, r in zip(q, dq, rows):
        M = pin.crba(m, d, qk)
        M = np.triu(M) + np.triu(M, 1).T
        g = pin.computeGeneralizedGravity(m, d, qk)
        C = pin.computeCoriolisMatrix(m, d, qk, dqk)
        err["M"] = max(err["M"], np.abs(M - r[:36].reshape(6, 6)).max())
        err["g"] = max(err["g"], np.abs(g - r[36:42]).max())
        err["C"] = max(err["C"], np.abs(C - r[42:].reshape(6, 6)).max())
    assert max(err.values()) < TOL, f"C++ y Python difieren: {err}"

    # Y el acople importa: sin el, la diferencia es grande (el test no es vacuo).
    sin = Plant(urdf, gravity=p["gravity"], tool=None)
    g0 = pin.computeGeneralizedGravity(sin.model, sin.data, q[0])
    assert np.abs(g0 - rows[0][36:42]).max() > 0.1
