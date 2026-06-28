"""
Pilar 3 — Validación por EDO: dinámica directa Euler vs. RK4 (Unidad 6.1–6.2).

Contraste con la dinámica INVERSA del CU3 (RNEA): aquí se integra hacia adelante
la dinámica DIRECTA del manipulador sobre los torques del óptimo para validar la
factibilidad dinámica antes de Gazebo.

Sistema de EDOs de primer orden, estado ``y = [q (6), q̇ (6)]`` (12 estados):

    q̇  = q̇
    q̈  = M(q)⁻¹ (τ − C(q,q̇)q̇ − g(q))   ← ``pin.aba`` (Articulated Body Algorithm)

Integradores implementados a mano (igual estructura que el laboratorio del RLC):
  * ``integrate_euler`` — Euler explícito (orden 1).
  * ``integrate_rk4``   — Runge-Kutta clásico de 4º orden.
"""

from __future__ import annotations
import numpy as np
import pinocchio as pin
from typing import Callable, Tuple


# ─────────────────────────────────────────────────────────────────────────────
# Dinámica directa (ABA)
# ─────────────────────────────────────────────────────────────────────────────

def closed_dynamics(
    model: pin.Model,
    data:  pin.Data,
    q:     np.ndarray,
    dq:    np.ndarray,
    tau:   np.ndarray,
) -> np.ndarray:
    """
    Aceleración articular ``q̈ = M(q)⁻¹(τ − C(q,q̇)q̇ − g(q))`` vía ``pin.aba``
    (Articulated Body Algorithm), la inversa exacta de la RNEA del CU3.
    """
    return pin.aba(model, data, q, dq, tau)


def _deriv(model, data, q, dq, tau):
    """f(y) del sistema de 12 estados: (q̇, q̈)."""
    return dq, closed_dynamics(model, data, q, dq, tau)


# ─────────────────────────────────────────────────────────────────────────────
# Euler explícito (orden 1)
# ─────────────────────────────────────────────────────────────────────────────

def integrate_euler(
    model:    pin.Model,
    data:     pin.Data,
    q0:       np.ndarray,
    dq0:      np.ndarray,
    tau_of_t: Callable[[float], np.ndarray],
    t_grid:   np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Euler explícito de un paso sobre ``y = [q, q̇]``.

        q_{k+1}  = q_k  + h·q̇_k
        q̇_{k+1} = q̇_k + h·q̈_k,   q̈_k = aba(q_k, q̇_k, τ(t_k))

    Returns (qs, dqs) con forma ``(len(t_grid), nq)``.
    """
    n = len(t_grid)
    qs  = np.zeros((n, len(q0)))
    dqs = np.zeros((n, len(dq0)))
    qs[0], dqs[0] = q0, dq0
    for k in range(n - 1):
        h = t_grid[k + 1] - t_grid[k]
        q, dq = qs[k], dqs[k]
        dq_dot, ddq = _deriv(model, data, q, dq, tau_of_t(t_grid[k]))
        qs[k + 1]  = q  + h * dq_dot
        dqs[k + 1] = dq + h * ddq
    return qs, dqs


# ─────────────────────────────────────────────────────────────────────────────
# Runge-Kutta clásico de 4º orden
# ─────────────────────────────────────────────────────────────────────────────

def integrate_rk4(
    model:    pin.Model,
    data:     pin.Data,
    q0:       np.ndarray,
    dq0:      np.ndarray,
    tau_of_t: Callable[[float], np.ndarray],
    t_grid:   np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    RK4 clásico sobre el sistema de 12 estados ``y = [q, q̇]``.

    Cuatro evaluaciones de la pendiente por paso (k1..k4) y combinación
    ponderada ``y_{k+1} = y_k + h/6·(k1 + 2k2 + 2k3 + k4)``.

    Returns (qs, dqs) con forma ``(len(t_grid), nq)``.
    """
    n = len(t_grid)
    qs  = np.zeros((n, len(q0)))
    dqs = np.zeros((n, len(dq0)))
    qs[0], dqs[0] = q0, dq0
    for k in range(n - 1):
        h  = t_grid[k + 1] - t_grid[k]
        t  = t_grid[k]
        q, dq = qs[k], dqs[k]

        k1q, k1v = _deriv(model, data, q,                 dq,                 tau_of_t(t))
        k2q, k2v = _deriv(model, data, q + 0.5*h*k1q,     dq + 0.5*h*k1v,     tau_of_t(t + 0.5*h))
        k3q, k3v = _deriv(model, data, q + 0.5*h*k2q,     dq + 0.5*h*k2v,     tau_of_t(t + 0.5*h))
        k4q, k4v = _deriv(model, data, q + h*k3q,         dq + h*k3v,         tau_of_t(t + h))

        qs[k + 1]  = q  + (h / 6.0) * (k1q + 2*k2q + 2*k3q + k4q)
        dqs[k + 1] = dq + (h / 6.0) * (k1v + 2*k2v + 2*k3v + k4v)
    return qs, dqs


# ─────────────────────────────────────────────────────────────────────────────
# Utilidades de comparación
# ─────────────────────────────────────────────────────────────────────────────

def make_tau_interp(
    times: np.ndarray,
    taus:  np.ndarray,
) -> Callable[[float], np.ndarray]:
    """
    Construye ``τ(t)`` por interpolación lineal de un perfil tabulado
    ``taus`` (N×nq) sobre ``times`` (N,).  Fuera del rango satura en los extremos.
    """
    times = np.asarray(times, float)
    taus  = np.asarray(taus,  float)

    def tau_of_t(t: float) -> np.ndarray:
        return np.array([np.interp(t, times, taus[:, j]) for j in range(taus.shape[1])])

    return tau_of_t


def tracking_error(
    qs_int:  np.ndarray,
    t_grid:  np.ndarray,
    times:   np.ndarray,
    qs_ref:  np.ndarray,
) -> Tuple[float, float]:
    """
    Error de seguimiento entre la trayectoria integrada y la referencia IK
    (interpolada a ``t_grid``).  Devuelve ``(error_max, error_rms)`` en rad,
    sobre todas las articulaciones y todo el horizonte.
    """
    ref_grid = np.column_stack([
        np.interp(t_grid, times, qs_ref[:, j]) for j in range(qs_ref.shape[1])
    ])
    err = qs_int - ref_grid
    if not np.all(np.isfinite(err)):
        return float('inf'), float('inf')
    return float(np.max(np.abs(err))), float(np.sqrt(np.mean(err ** 2)))
