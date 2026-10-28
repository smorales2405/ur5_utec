"""
B1: fuerza en la punta de la hoja estimada por CORRIENTE de motor.

Es la segunda vía de medida de la fuerza de corte, independiente del sensor F/T
integrado (D4 del plan): `k ⊙ i` es el par TOTAL que entregan los motores, lo
fija la física y no depende de nuestro mando (02_friction_real.md). Lo que
sobra respecto al modelo es la fuerza externa:

    M q̈ + n(q, q̇) + τ_f(q̇) = k ⊙ i − Jᵀ w_amb      (w_amb: entorno sobre la hoja)
    ⇒   Jᵀ w = k ⊙ i − M q̈ − n − τ_f                  (w = −w_amb: hoja sobre el entorno)

`w` es la fuerza que la HOJA ejerce sobre el tejido, el mismo convenio que
`CuttingForce` del evaluador offline.

Lo que limita la estimación, y hay que declararlo
-------------------------------------------------
**Una junta parada no aporta ecuación.** Con la junta quieta la fricción es
cualquier valor en `[−F_c, +F_c]` (stiction): su residuo no se puede atribuir a
la fuerza externa. En la incisión (D18: recta x = −d4) la base, `wrist_2` y
`wrist_3` están QUIETAS durante todo el corte, así que solo cuentan hombro, codo
y `wrist_1`, las tres juntas del plano del brazo. Con ellas:

  - `F_y` (arrastre a lo largo del corte) y `F_z` (normal) son observables;
  - `F_x` (lateral) NO: solo la ven juntas paradas. Sale NaN, no un número.

Se resuelve por mínimos cuadrados SOLO la fuerza (momento en la punta nulo: la
hoja corta en su plano), con las juntas en movimiento de cada muestra, y cada
eje se declara observable o no a partir del proyector sobre el espacio de filas.

En Gazebo la columna `cur` del CSV es el esfuerzo de la junta [N·m]: `k = 1`.
"""

from __future__ import annotations

import numpy as np

from .friction_models import DEFAULT_DEADBAND, FrictionParams

#: Constante corriente → par [N·m/A] por junta, medida por diferencia entre
#: sentidos en el robot real (docs/02_friction_real.md 8.4; 0.1–0.2 %).
K_CURRENT = np.array([11.139, 11.124, 11.001, 8.666, 8.542, 8.644])

#: Fricción FÍSICA (vía de corriente, G4 = 0.0), robot rodado: la misma de
#: incision_common.yaml (docs/02_friction_real.md 8.4).
FRICTION_REAL = [FrictionParams(f_v=v, f_c=c) for v, c in zip(
    [13.64, 11.55, 13.74, 1.28, 1.35, 2.12],
    [6.85, 6.76, 7.46, 1.78, 1.96, 2.35])]

#: Umbral de observabilidad: diagonal del proyector sobre el espacio de filas.
OBSERVABLE_MIN = 0.99


def load_columns(path: str) -> dict:
    """Todas las columnas del CSV unificado, por nombre (`state` como texto).

    Salta la cabecera de trazabilidad `#` (genfromtxt con names=True no lo hace
    sola) y tolera una ultima fila a medio escribir.
    """
    n_meta = 0
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            if not line.startswith("#"):
                break
            n_meta += 1
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=None,
                         encoding="utf-8", skip_header=n_meta,
                         invalid_raise=False)
    return {n: (np.asarray(data[n], dtype=str) if n == "state"
                else np.asarray(data[n], dtype=float)) for n in data.dtype.names}


def residual_torque(plant, q, dq, ddq, tau_motor, friction=None,
                    deadband: float = DEFAULT_DEADBAND):
    """`k⊙i − M q̈ − n − τ_f` y la máscara de juntas en movimiento."""
    import pinocchio as pin
    m, d = plant.model, plant.data
    tau_model = pin.rnea(m, d, np.asarray(q, float), np.asarray(dq, float),
                         np.asarray(ddq, float))
    moving = np.abs(dq) > deadband
    tau_f = np.zeros(6)
    if friction is not None:
        tau_f = np.array([p.torque(np.array([v]))[0] for p, v in zip(friction, dq)])
    return np.asarray(tau_motor, float) - tau_model - tau_f, moving


def estimate_force(plant, q, dq, ddq, tau_motor, friction=None,
                   deadband: float = DEFAULT_DEADBAND):
    """Fuerza [N] que la hoja ejerce sobre el entorno, en base_link.

    Devuelve `(F, observable)`: `F` con NaN en los ejes no observables con las
    juntas en movimiento de esta muestra.
    """
    import pinocchio as pin
    r, moving = residual_torque(plant, q, dq, ddq, tau_motor, friction, deadband)
    F = np.full(3, np.nan)
    observable = np.zeros(3, dtype=bool)
    if moving.sum() == 0:
        return F, observable
    J = pin.computeFrameJacobian(plant.model, plant.data, np.asarray(q, float),
                                 plant.tcp, pin.LOCAL_WORLD_ALIGNED)
    A = J[:3, moving].T          # filas: juntas en movimiento; columnas: Fx, Fy, Fz
    A_pinv = np.linalg.pinv(A, rcond=1e-6)
    observable = np.diag(A_pinv @ A) > OBSERVABLE_MIN
    F_ls = A_pinv @ r[moving]
    F[observable] = F_ls[observable]
    return F, observable


def estimate_from_csv(csv_path: str, plant=None, k=K_CURRENT, gazebo=False,
                      friction=FRICTION_REAL, ddq_source: str = "desired",
                      deadband: float = DEFAULT_DEADBAND) -> dict:
    """Estimación fila a fila sobre el CSV de una corrida.

    `ddq_source`: "desired" (la de la referencia; en la meseta del corte es 0
    y no mete ruido) o "measured" (derivada de `dq`, ruidosa).
    `gazebo=True`: `cur` ya es par, `k = 1` y sin fricción (la planta de Gazebo
    no la tiene salvo que se inyecte).
    """
    if plant is None:
        from ur5_trajectory_optimization.gain_tuning.closed_loop import (
            Plant, default_urdf)
        plant = Plant(default_urdf())
    d = load_columns(csv_path)
    col = lambda p: np.column_stack([d[p % i] for i in range(1, 7)])  # noqa: E731
    q, dq = col("q%d"), col("dq%d")
    cur = col("cur%d")
    if ddq_source == "desired":
        ddq = col("ddq%d_des")
    else:
        ddq = np.gradient(dq, d["t_sim"], axis=0)
    if gazebo:
        k, friction = np.ones(6), None
    tau_motor = cur * np.asarray(k, float)
    n = len(q)
    F = np.full((n, 3), np.nan)
    obs = np.zeros((n, 3), dtype=bool)
    for i in range(n):
        F[i], obs[i] = estimate_force(plant, q[i], dq[i], ddq[i], tau_motor[i],
                                      friction, deadband)
    out = {"t_sim": d["t_sim"], "F": F, "observable": obs, "state": d.get("state")}
    if "wrench1" in d:
        out["F_ft"] = np.column_stack([d[f"wrench{i}"] for i in (1, 2, 3)])
    return out


def mean_resolution(x: np.ndarray, max_lag: int = 200) -> tuple:
    """Media de una serie y su error típico con la autocorrelación corregida.

    Sobre la meseta (8 s, ~4 000 muestras) el ruido no es blanco: el número
    efectivo de muestras es `N / (1 + 2 Σ ρ_k)`. Es lo que decide qué fuerza
    resuelve el sensor integrado promediando (B1).
    """
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    n = len(x)
    if n < 3:
        return float("nan"), float("nan")
    xc = x - x.mean()
    var = xc @ xc / n
    if var == 0.0:
        return float(x.mean()), 0.0
    rho_sum = 0.0
    for lag in range(1, min(max_lag, n - 1)):
        rho = (xc[:-lag] @ xc[lag:]) / (n * var)
        if rho <= 0.0:          # se corta en el primer cruce por cero
            break
        rho_sum += rho
    n_eff = n / (1.0 + 2.0 * rho_sum)
    return float(x.mean()), float(np.sqrt(var / n_eff))
