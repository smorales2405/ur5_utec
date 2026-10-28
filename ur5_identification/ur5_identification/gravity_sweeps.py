"""
B4.4: α_g, el error de gravedad que ve el SMC en el robot real, con barridos.

Qué hay que medir (y qué no)
----------------------------
En el real el robot suma su propia gravedad `g_UR(q)` al par que mandamos (G3),
y el nodo manda la ley MENOS su `ĝ(q)`. En lazo cerrado `ĝ` se cancela entera:

    M q̈ + C q̇ + g_real + τ_f = τ_ley − ĝ + g_UR + ĝ − ĝ   ⇒   error = g_real − g_UR

Comparar `g_UR` con `ĝ` (lo que decía el plan hasta la Rev. 7) mide la
diferencia entre dos modelos, ninguno de los cuales es el error del lazo.

Cómo
----
Con una junta a velocidad constante (q̈ = 0) y la misma `q` en los dos sentidos:

    g_real(q) = ½ [k⊙i(q, +v) + k⊙i(q, −v)] − C(q, q̇) q̇        la fricción es impar
    g_UR(q)   = k⊙i(k) − τ_cmd(k − D)                            en cualquier muestra

`D` es el retardo de tubería (B7.0): el par de la fila `k` actúa `D` filas
después. Las dos se dan como diferencia con `ĝ` del modelo del nodo, así que
`Δ = g_real − g_UR = (g_real − ĝ) − (g_UR − ĝ)` y `α_g = |Δ| / |ĝ|` en cada
postura. Quieto no sirve: la fricción estática de hombro y codo (±7 N·m) es
mayor que el error que se busca.

En Gazebo (`gravity_in_command: true`) `g_UR = 0` y `g_real = ĝ` por
construcción: es la prueba del método, no una medida. Con fricción inyectada
la columna de esfuerzo de Gazebo es el par NETO (aplicado menos fricción): la
parte par no cambia, pero `g_UR` sale con la fricción alternando de signo
(media nula, dispersión grande). En el real la corriente es el par del motor.
"""

from __future__ import annotations

import os
import re

import numpy as np

from .force_estimator import K_CURRENT, load_columns
from .pipeline_delay import is_real

#: Retardo de tubería del real medido en B7.0 [ciclos de 2 ms]; Gazebo, 1.
DELAY_REAL = 5
DELAY_GAZEBO = 1

#: Línea del corte (incision_common.yaml): x fija en base_link, q1 = π/2,
#: q5 = −π/2, q6 = π y la herramienta vertical (q2 + q3 + q4 = −π/2).
CUT_X = -0.1333
Q_SEED = np.array([1.5708, -1.569385, 1.514062, -1.515474, -1.5708, 3.1416])
#: Juntas con gravedad en esta orientación: hombro, codo, wrist_1 (0-based).
GRAVITY_JOINTS = (1, 2, 3)

#: Posturas por defecto: (nombre, y, z) de la PUNTA en base_link [m]. El
#: inicio de la aproximación y tres puntos sobre el trazo (y 0.70 → 0.60), a
#: 60 mm sobre la cota del corte (z ≈ 0) para que los barridos no bajen a la
#: mesa (tablero en z = −0.046).
DEFAULT_POSTURES = (
    ("inicio", 0.4919, 0.3469),
    ("corte_lejos", 0.70, 0.06),
    ("corte_medio", 0.65, 0.06),
    ("corte_cerca", 0.60, 0.06),
)
#: Tablero de la mesa en base_link y holgura mínima de la punta [m].
TABLE_Z = -0.046
MIN_CLEARANCE = 0.05

_LEVEL = re.compile(r"SWEEP_(\d+\.\d+)_(POS|NEG)$")


def tip_position(plant, q) -> np.ndarray:
    import pinocchio as pin
    pin.framesForwardKinematics(plant.model, plant.data, np.asarray(q, float))
    return plant.data.oMf[plant.tcp].translation.copy()


def incision_posture(plant, y: float, z: float, seed=Q_SEED,
                     tol: float = 1e-6, iters: int = 50) -> np.ndarray:
    """q con la punta en (CUT_X, y, z) y la orientación de la incisión.

    Newton sobre (q2, q3) con q4 = −π/2 − q2 − q3 (herramienta vertical). La
    rama es la de la semilla (codo arriba, la de la incisión).
    """
    q = np.array(seed, float)

    def close(q2, q3):
        out = q.copy()
        out[1], out[2], out[3] = q2, q3, -np.pi / 2 - q2 - q3
        return out

    x = np.array([q[1], q[2]])
    target = np.array([y, z])
    for _ in range(iters):
        p = tip_position(plant, close(*x))[1:]
        r = p - target
        if np.linalg.norm(r) < tol:
            break
        J = np.empty((2, 2))
        for c in range(2):
            dx = np.zeros(2)
            dx[c] = 1e-6
            J[:, c] = (tip_position(plant, close(*(x + dx)))[1:] - p) / 1e-6
        x = x - np.linalg.solve(J, r)
    q = close(*x)
    p = tip_position(plant, q)
    if np.linalg.norm(p - [CUT_X, y, z]) > 1e-4:
        raise ValueError(f"IK sin solucion para y={y}, z={z}: punta en {p}")
    return q


def sweep_min_tip_z(plant, q_fixed, joint: int, amplitude: float,
                    n: int = 21) -> float:
    """Cota mínima de la punta mientras `joint` barre ±amplitude."""
    zs = []
    for a in np.linspace(-amplitude, amplitude, n):
        q = np.array(q_fixed, float)
        q[joint] += a
        zs.append(tip_position(plant, q)[2])
    return float(min(zs))


def _sidecar(csv_path: str) -> dict:
    """Parámetros efectivos de la corrida (`<prefijo>_<n>.params.yaml`)."""
    import yaml
    path = os.path.splitext(csv_path)[0] + ".params.yaml"
    if not os.path.exists(path):
        return {}
    with open(path) as fh:
        doc = yaml.safe_load(fh) or {}
    for sec in doc.values():
        if isinstance(sec, dict) and "ros__parameters" in sec:
            return sec["ros__parameters"]
    return {}


def plant_for_run(params: dict):
    """El modelo del NODO en esa corrida: con el acople si `tool_mass` > 0."""
    from ur5_trajectory_optimization.gain_tuning.closed_loop import (
        TCP_OFFSET, Plant, Tool, default_urdf)
    mass = float(params.get("tool_mass", 0.0))
    tool = None
    if mass > 0.0:
        tool = Tool(mass=mass, com=tuple(params["tool_com"]),
                    inertia=tuple(params["tool_inertia"]))
    return Plant(default_urdf(), tcp_offset=float(params.get("tcp_offset_z", TCP_OFFSET)),
                 tool=tool)


def even_part(q, y, sign, bin_width: float = 0.004, min_n: int = 5):
    """½[y(+) + y(−)] a q emparejada, por bins. Devuelve (centros, valores)."""
    q, y, sign = map(np.asarray, (q, y, sign))
    pos, neg = sign > 0, sign < 0
    if pos.sum() < min_n or neg.sum() < min_n:
        return np.zeros(0), np.zeros(0)
    lo = max(q[pos].min(), q[neg].min())
    hi = min(q[pos].max(), q[neg].max())
    centers, vals = [], []
    for b0 in np.arange(lo, hi - bin_width / 2, bin_width):
        s = (q >= b0) & (q < b0 + bin_width)
        if (s & pos).sum() >= min_n and (s & neg).sum() >= min_n:
            centers.append(b0 + bin_width / 2)
            vals.append(0.5 * (np.median(y[s & pos]) + np.median(y[s & neg])))
    return np.asarray(centers), np.asarray(vals)


def analyze_run(csv_path: str, plant=None, delay: int | None = None,
                k=None) -> dict:
    """Una corrida de barrido: g_real, g_UR y ĝ en su postura, para su junta."""
    import pinocchio as pin
    d = load_columns(csv_path)
    params = _sidecar(csv_path)
    real = is_real(d)
    if delay is None:
        delay = DELAY_REAL if real else DELAY_GAZEBO
    if k is None:
        k = K_CURRENT if real else np.ones(6)
    if plant is None:
        plant = plant_for_run(params)
    col = lambda p: np.column_stack([d[p % i] for i in range(1, 7)])  # noqa: E731
    q, dq_des, cur, tau = col("q%d"), col("dq%d_des"), col("cur%d"), col("tau%d")
    if "sweep.joint" in params:
        j = int(params["sweep.joint"])
    else:
        j = int(np.argmax(np.ptp(q, axis=0)))

    labels = [_LEVEL.match(s) for s in d["state"]]
    plateau = np.array([m is not None for m in labels])
    level = np.array([float(m.group(1)) if m else np.nan for m in labels])
    sign = np.array([(1 if m.group(2) == "POS" else -1) if m else 0 for m in labels])
    if plateau.sum() == 0:
        raise ValueError("sin mesetas SWEEP_<v>_POS/NEG: ¿es un barrido?")

    # Modelo en cada muestra de meseta: ĝ(q) y C(q, q̇_d) q̇_d.
    idx = np.where(plateau)[0]
    idx = idx[idx >= delay]
    g_hat = np.empty((len(idx), 6))
    c_term = np.empty((len(idx), 6))
    z6 = np.zeros(6)
    for n, r in enumerate(idx):
        g_hat[n] = pin.computeGeneralizedGravity(plant.model, plant.data, q[r])
        c_term[n] = pin.rnea(plant.model, plant.data, q[r], dq_des[r], z6) - g_hat[n]
    motor = cur[idx] * np.asarray(k, float)
    # g_UR − ĝ: lo que añade el robot, alineado con el par que lo acompaña.
    g_ur_minus = motor - tau[idx - delay] - g_hat
    # g_real − ĝ en la junta barrida: parte par a q emparejada, por nivel.
    r_j = motor[:, j] - c_term[:, j] - g_hat[:, j]
    levels = {}
    for v in sorted(set(level[idx][np.isfinite(level[idx])])):
        m = level[idx] == v
        c, e = even_part(q[idx][m, j], r_j[m], sign[idx][m])
        if len(e):
            levels[v] = {"bins": len(e), "g_real_minus_ghat": float(np.mean(e)),
                         "spread": float(np.std(e))}
    if not levels:
        raise ValueError("sin bins emparejados en los dos sentidos")
    q_fixed = np.asarray(params.get("sweep.q_fixed", np.median(q[idx], axis=0)), float)
    g_fixed = pin.computeGeneralizedGravity(plant.model, plant.data, q_fixed)
    real_minus = float(np.mean([lv["g_real_minus_ghat"] for lv in levels.values()]))
    ur_minus = np.mean(g_ur_minus, axis=0)
    delta = real_minus - float(ur_minus[j])
    return {
        "csv": csv_path, "real": real, "joint": j, "delay": delay,
        "q_fixed": q_fixed, "g_hat": g_fixed,
        "levels": levels,
        "g_real_minus_ghat": real_minus,
        "g_ur_minus_ghat": ur_minus,
        "g_ur_minus_ghat_std": np.std(g_ur_minus, axis=0),
        "delta": delta,                       # g_real − g_UR en la junta barrida
        "alpha": abs(delta) / max(abs(float(g_fixed[j])), 1e-9),
    }


def alpha_by_joint(results: list) -> dict:
    """α_g por junta: el peor |Δ| / |ĝ| entre posturas."""
    out = {}
    for r in results:
        j = r["joint"]
        out[j] = max(out.get(j, 0.0), r["alpha"])
    return out
