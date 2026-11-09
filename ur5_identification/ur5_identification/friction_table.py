"""D33: fricción medida por velocidad, para la compensación del nodo (`friction_compensation: table`).

El modelo F_c + F_v·|q̇| se ajustó a la campaña de agosto (`docs/02_friction_real.md`), pero la
fricción no es lineal a baja velocidad: en `smc_1630`, el codo iba a 0.027 rad/s y la compensación
sobraba ~0.9 N·m, porque a 0.02 rad/s la campaña midió 6.49 N·m frente a 7.74 del modelo. Aquí se
toman los puntos medidos tal cual, del mismo dato y con el mismo método con que se ajustó el modelo:

    F_j(v) = ½ (i⁺ − i⁻) · k_j      en cada par de mesetas ±v de `cur_95j.csv`

con G4 a 0.0 (fricción física), en `q_init` y ±45°. La semidiferencia cancela la gravedad y todo
lo que no cambia de signo con el sentido. `k_j` es la de `docs/02_friction_real.md` §8.4.
Salvedad: la fricción cambia con la postura y la temperatura, del orden de 0.4 N·m (`09c` §8).
"""

from __future__ import annotations

import csv
import os

import numpy as np

from .force_estimator import K_CURRENT

#: Campaña de agosto con G4 a 0.0: `cur_950 + junta` (`02_friction_real.md` §10).
CAMPAIGN_G4_0 = tuple(950 + j for j in range(6))


def _load(path: str) -> tuple:
    with open(path, errors="replace") as fh:
        rows = list(csv.reader(line for line in fh if not line.startswith("#")))
    head = rows[0]
    rows = [r for r in rows[1:] if len(r) == len(head)]
    ix = {name: i for i, name in enumerate(head)}
    state = np.array([r[ix["state"]] for r in rows])
    num = {name: np.array([float(r[ix[name]]) for r in rows])
           for name in head if name != "state"}
    return num, state


def joint_friction(path: str, joint: int, k: float) -> dict:
    """{velocidad nominal [rad/s]: fricción [N·m]} de una corrida de la campaña."""
    num, state = _load(path)
    out = {}
    levels = sorted({s.rsplit("_", 1)[0] for s in state
                     if s.startswith("SWEEP_") and s.rsplit("_", 1)[-1] in ("POS", "NEG")})
    for lv in levels:
        pos, neg = state == f"{lv}_POS", state == f"{lv}_NEG"
        if pos.sum() < 20 or neg.sum() < 20:
            continue
        cur = num[f"cur{joint + 1}"]
        out[float(lv.split("_")[1])] = 0.5 * (cur[pos].mean() - cur[neg].mean()) * k
    return out


def build_table(csv_dir: str, runs=CAMPAIGN_G4_0, k=K_CURRENT) -> tuple:
    """(velocidades [N], tabla [6 × N]) de la campaña; las mismas velocidades en las seis."""
    per_joint = [joint_friction(os.path.join(csv_dir, f"cur_{n}.csv"), j, float(k[j]))
                 for j, n in enumerate(runs)]
    speeds = sorted(set.intersection(*(set(f) for f in per_joint)))
    if not speeds:
        raise ValueError("ninguna velocidad común a las seis juntas")
    return np.array(speeds), np.array([[f[v] for v in speeds] for f in per_joint])


def interp(speeds, values, v: float) -> float:
    """F(|v|) de una junta, como el nodo: lineal entre puntos; por debajo del primero, el
    primero; por encima del último, la pendiente del último tramo."""
    s, f, a = np.asarray(speeds, float), np.asarray(values, float), abs(float(v))
    if a <= s[0]:
        return float(f[0])
    if a >= s[-1]:
        return float(f[-1] + (f[-1] - f[-2]) / (s[-1] - s[-2]) * (a - s[-1]))
    return float(np.interp(a, s, f))
