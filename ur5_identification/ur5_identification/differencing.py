"""
Estimación de fricción por DIFERENCIACIÓN entre sentidos, sin usar el modelo
dinámico (FASE 2).

Método
------
A la MISMA posición ``q`` y a la misma rapidez ``|q̇|``, se compara el par
comandado en los dos sentidos de barrido:

    tau_f(v) = [ tau_cmd(q, +v) - tau_cmd(q, -v) ] / 2

Los términos de la dinámica que son PARES en la velocidad se cancelan
exactamente en la diferencia:

- **gravedad** ``g(q)``: no depende de q̇ en absoluto → se cancela;
- **Coriolis/centrífugo** ``C(q,q̇) q̇``: cuadrático en q̇ → par → se cancela;
- **inercia** ``M(q) q̈``: nula en la meseta por construcción.

Lo que queda es exactamente la parte IMPAR, que es la fricción.

Por qué importa (protocolo del OpenMANIPULATOR-X)
-------------------------------------------------
El estimador de ``estimator.py`` usa el residuo ``tau_cmd - RNEA(q,q̇,q̈)`` y por
tanto **depende de que el URDF sea exacto**: cualquier error de masas, centro de
masa o calibración se absorbe dentro de la "fricción" estimada. En el UR5e real
eso no es hipotético — el supuesto **A1** declara explícitamente que se desprecia
la masa del acople del bisturí.

Este método no toca el URDF ni Pinocchio, así que es inmune a ese error. En
Gazebo la relación se invierte (allí el modelo es perfecto y RNEA es óptimo),
por lo que conviene reportar AMBOS: si coinciden, el modelo es bueno; si
discrepan, **la discrepancia mide el error de modelado**, y ese número es en sí
mismo un resultado publicable.

Limitación: exige que los dos sentidos recorran un rango de posiciones común, lo
cual el barrido de `JointSweepGenerator` garantiza por construcción (cada nivel
se recorre −A→+A y +A→−A).
"""

from __future__ import annotations

import re
from dataclasses import dataclass

import numpy as np

from .friction_models import FrictionParams
from .residual import load_csv

_PLATEAU_RE = re.compile(r"^SWEEP_([0-9.]+)_(POS|NEG)$")


@dataclass
class DifferencePoint:
    """Fricción estimada a un nivel de rapidez, por diferencia de sentidos."""

    speed: float            # |q̇| nominal [rad/s]
    tau_friction: float     # [N·m]
    tau_std: float          # dispersión a lo largo del rango común
    q_overlap: float        # amplitud del rango de posiciones comparado [rad]
    n_grid: int


@dataclass
class DifferenceResult:
    points: list
    params: FrictionParams
    r2: float
    rmse: float
    stderr: np.ndarray

    def summary(self) -> str:
        lines = [f"  diferenciación entre sentidos (sin URDF): "
                 f"{len(self.points)} niveles"]
        lines.append(f"    F_v = {self.params.f_v:9.4f}  ± {self.stderr[0]:7.4f}")
        lines.append(f"    F_c = {self.params.f_c:9.4f}  ± {self.stderr[1]:7.4f}")
        lines.append(f"    R² = {self.r2:.4f}   RMSE = {self.rmse:.4f} N·m")
        return "\n".join(lines)


def _plateau_traces(csv_path: str, joint: int) -> dict:
    """{|v| : {'POS': (q, tau), 'NEG': (q, tau)}} de las mesetas."""
    d = load_csv(csv_path)
    out: dict = {}
    q_col = d["q"][:, joint]
    tau_col = d["tau"][:, joint]
    for k, st in enumerate(d["state"]):
        m = _PLATEAU_RE.match(st)
        if m is None:
            continue
        speed = float(m.group(1))
        side = m.group(2)
        out.setdefault(speed, {}).setdefault(side, [[], []])
        out[speed][side][0].append(q_col[k])
        out[speed][side][1].append(tau_col[k])
    return {s: {k: (np.asarray(v[0]), np.asarray(v[1])) for k, v in d2.items()}
            for s, d2 in out.items()}


def difference_points(csv_path: str, joint: int, n_grid: int = 400,
                      trim_fraction: float = 0.05) -> list:
    """
    Calcula tau_f(|v|) por media diferencia entre sentidos, interpolando ambos
    trazos sobre una malla común de POSICIONES.
    """
    traces = _plateau_traces(csv_path, joint)
    points: list[DifferencePoint] = []

    for speed in sorted(traces):
        sides = traces[speed]
        if "POS" not in sides or "NEG" not in sides:
            continue
        qp, tp = sides["POS"]
        qn, tn = sides["NEG"]

        lo = max(qp.min(), qn.min())
        hi = min(qp.max(), qn.max())
        if not (hi > lo):
            continue
        # Recorte de los extremos del solape: allí un trazo puede estar aún
        # asentando mientras el otro ya está en régimen.
        span = hi - lo
        lo += trim_fraction * span
        hi -= trim_fraction * span
        grid = np.linspace(lo, hi, n_grid)

        def interp(q, tau):
            order = np.argsort(q)
            return np.interp(grid, q[order], tau[order])

        diff = 0.5 * (interp(qp, tp) - interp(qn, tn))
        points.append(DifferencePoint(speed=speed,
                                      tau_friction=float(diff.mean()),
                                      tau_std=float(diff.std(ddof=1)),
                                      q_overlap=float(hi - lo),
                                      n_grid=n_grid))
    return points


def fit_difference(points: list) -> DifferenceResult:
    """Ajuste LS de tau_f = F_v·|v| + F_c sobre los niveles de rapidez."""
    if len(points) < 3:
        raise ValueError(f"hacen falta al menos 3 niveles, hay {len(points)}")

    v = np.array([p.speed for p in points])
    y = np.array([p.tau_friction for p in points])
    phi = np.column_stack([v, np.ones_like(v)])

    beta, *_ = np.linalg.lstsq(phi, y, rcond=None)
    resid = y - phi @ beta
    dof = max(len(y) - 2, 1)
    sigma2 = float(resid @ resid) / dof
    try:
        stderr = np.sqrt(np.clip(np.diag(sigma2 * np.linalg.inv(phi.T @ phi)), 0, None))
    except np.linalg.LinAlgError:
        stderr = np.full(2, np.nan)

    ss_res = float(resid @ resid)
    ss_tot = float(((y - y.mean()) ** 2).sum())
    return DifferenceResult(
        points=points,
        params=FrictionParams(f_v=float(beta[0]), f_c=float(beta[1]),
                              model="viscous_coulomb"),
        r2=1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan"),
        rmse=float(np.sqrt(ss_res / len(y))),
        stderr=stderr)


def identify_by_differencing(csv_path: str, joint: int, **kwargs) -> DifferenceResult:
    return fit_difference(difference_points(csv_path, joint, **kwargs))
