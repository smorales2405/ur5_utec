"""
Modelos de fricción articular y sus regresores lineales (FASE 2).

Todos los modelos se escriben como ``tau_f(q̇) = Phi(q̇) · beta``, donde ``Phi``
es el regresor y ``beta`` los parámetros a estimar. Mantenerlos lineales en los
parámetros es lo que permite resolverlos por mínimos cuadrados con solución
cerrada, covarianza analítica y bandas de confianza — sin optimización no lineal
ni semillas.

La excepción es Stribeck, que es no lineal en ``v_s`` y ``delta``: se trata
fijando esos dos y resolviendo el resto por LS, barriendo una rejilla pequeña.

Convenio de signos
------------------
La ecuación de la planta es

    M(q) q̈ + C(q,q̇) q̇ + g(q) + tau_f(q̇) = tau_cmd

es decir la fricción se OPONE al movimiento y aparece sumando en el lado
izquierdo. Por tanto el residuo

    tau_residual = tau_cmd - RNEA(q, q̇, q̈)

es directamente ``+tau_f(q̇)``, con ``F_v`` y ``F_c`` positivos.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# Umbral de velocidad por debajo del cual sgn(q̇) no es fiable (ruido de signo).
# Las muestras con |q̇| menor se descartan antes de ajustar.
DEFAULT_DEADBAND = 1e-3


@dataclass
class FrictionParams:
    """Parámetros identificados de una junta."""

    f_v: float = 0.0          # viscoso [N·m·s/rad]
    f_c: float = 0.0          # Coulomb [N·m]
    f_s: float = 0.0          # estático (Stribeck) [N·m]; f_s > f_c
    v_s: float = 0.0          # velocidad de Stribeck [rad/s]
    delta: float = 2.0        # exponente de Stribeck
    model: str = "viscous_coulomb"

    def torque(self, dq: np.ndarray) -> np.ndarray:
        return friction_torque(dq, self)

    def as_dict(self) -> dict:
        d = {"model": self.model, "f_v": float(self.f_v), "f_c": float(self.f_c)}
        if self.model == "stribeck":
            d.update({"f_s": float(self.f_s), "v_s": float(self.v_s),
                      "delta": float(self.delta)})
        return d


def regressor(dq: np.ndarray, model: str, v_s: float = 0.0,
              delta: float = 2.0) -> np.ndarray:
    """
    Matriz de regresión ``Phi`` tal que ``tau_f = Phi · beta``.

    - ``viscous``          : beta = [F_v]
    - ``viscous_coulomb``  : beta = [F_v, F_c]
    - ``stribeck``         : beta = [F_v, F_c, (F_s - F_c)] con v_s y delta FIJOS
    """
    dq = np.asarray(dq, dtype=float)
    s = np.sign(dq)
    if model == "viscous":
        return dq.reshape(-1, 1)
    if model == "viscous_coulomb":
        return np.column_stack([dq, s])
    if model == "stribeck":
        if not (v_s > 0.0):
            raise ValueError("stribeck requiere v_s > 0")
        return np.column_stack([dq, s, np.exp(-np.abs(dq / v_s) ** delta) * s])
    raise ValueError(f"modelo desconocido: {model!r}")


def params_from_beta(beta: np.ndarray, model: str, v_s: float = 0.0,
                     delta: float = 2.0) -> FrictionParams:
    """Traduce el vector de LS a parámetros con nombre."""
    beta = np.asarray(beta, dtype=float).ravel()
    if model == "viscous":
        return FrictionParams(f_v=beta[0], model=model)
    if model == "viscous_coulomb":
        return FrictionParams(f_v=beta[0], f_c=beta[1], model=model)
    if model == "stribeck":
        f_c = beta[1]
        return FrictionParams(f_v=beta[0], f_c=f_c, f_s=f_c + beta[2],
                              v_s=v_s, delta=delta, model=model)
    raise ValueError(f"modelo desconocido: {model!r}")


def friction_torque(dq: np.ndarray, p: FrictionParams) -> np.ndarray:
    """Par de fricción predicho por el modelo (mismo convenio de signos)."""
    dq = np.asarray(dq, dtype=float)
    s = np.sign(dq)
    tau = p.f_v * dq + p.f_c * s
    if p.model == "stribeck" and p.v_s > 0.0:
        tau = tau + (p.f_s - p.f_c) * np.exp(-np.abs(dq / p.v_s) ** p.delta) * s
    return tau


def n_params(model: str) -> int:
    return {"viscous": 1, "viscous_coulomb": 2, "stribeck": 3}[model]
