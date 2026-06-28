"""
Integrandos callables para el estudio de cuadraturas (Pilar 1).

Construye, reutilizando la maquinaria del CU3 (``trajectory_model`` +
``ik_interface`` + Pinocchio), las dos funciones ``t -> valor`` que aparecen en
los objetivos del problema:

  * Longitud de arco  ``‖ṗ(t)‖``  — integrando analítico y suave (derivada del
    spline).  Showcase principal de Gauss-Legendre.
  * Esfuerzo articular ``Σᵢ τᵢ(t)²`` — costoso: muestrea ``q(t)`` por IK, estima
    ``q̇, q̈`` por diferencias centradas y llama ``pin.rnea``.  Sólo definido
    donde la IK converge; se compara contra una referencia de malla muy fina.

Ambos integrandos viven sobre el dominio temporal completo de la trayectoria
``[t_A, t_D]`` y respetan la estructura de splines del CU3:
    spline 1 = [A, B]        sobre [t_A, t_B]
    spline 2 = [B, O, C]     sobre [t_B, t_O, t_C]
    spline 3 = [C, D]        sobre [t_C, t_D]
"""

from __future__ import annotations
import numpy as np
import pinocchio as pin
from dataclasses import dataclass
from typing import Callable, List

from .trajectory_model import CartesianWaypoint, _compute_clamped_coeffs, _slerp


# ─────────────────────────────────────────────────────────────────────────────
# Tabla de segmentos: muestreo de pose cartesiana p(t), R(t) en t arbitrario
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class _Segment:
    t0: float
    t1: float
    cx: tuple          # (a, b, c, d) coeficientes del spline en s∈[0,1], eje x
    cy: tuple
    cz: tuple
    R0: np.ndarray     # orientación en t0
    R1: np.ndarray     # orientación en t1


def _build_segment_table(keypoints: List[CartesianWaypoint]) -> List[_Segment]:
    """
    Construye la lista de segmentos cúbicos (en orden temporal) a partir de los
    5 keypoints ordenados ``[A, B, O, C, D]``, replicando la partición de
    splines clamped del CU3 (``build_trajectory``).
    """
    if len(keypoints) != 5:
        raise ValueError("Se esperan 5 keypoints [A, B, O, C, D]")

    splines = [
        keypoints[0:2],   # [A, B]
        keypoints[1:4],   # [B, O, C]
        keypoints[3:5],   # [C, D]
    ]

    segs: List[_Segment] = []
    for sp in splines:
        px = [kp.position[0] for kp in sp]
        py = [kp.position[1] for kp in sp]
        pz = [kp.position[2] for kp in sp]
        cx = _compute_clamped_coeffs(px)
        cy = _compute_clamped_coeffs(py)
        cz = _compute_clamped_coeffs(pz)
        for i in range(len(sp) - 1):
            segs.append(_Segment(
                t0=sp[i].timestamp,   t1=sp[i + 1].timestamp,
                cx=cx[i], cy=cy[i], cz=cz[i],
                R0=sp[i].orientation, R1=sp[i + 1].orientation,
            ))
    return segs


def _locate(segs: List[_Segment], t: float) -> tuple[_Segment, float]:
    """Devuelve (segmento que contiene t, parámetro local s∈[0,1])."""
    t = min(max(t, segs[0].t0), segs[-1].t1)   # clamp al dominio
    for seg in segs:
        if seg.t0 <= t <= seg.t1:
            s = (t - seg.t0) / (seg.t1 - seg.t0)
            return seg, s
    # fallback numérico (t justo en el borde)
    seg = segs[-1]
    return seg, 1.0


def _position(seg: _Segment, s: float) -> np.ndarray:
    s2, s3 = s * s, s * s * s
    return np.array([
        seg.cx[0] + seg.cx[1] * s + seg.cx[2] * s2 + seg.cx[3] * s3,
        seg.cy[0] + seg.cy[1] * s + seg.cy[2] * s2 + seg.cy[3] * s3,
        seg.cz[0] + seg.cz[1] * s + seg.cz[2] * s2 + seg.cz[3] * s3,
    ])


def _velocity(seg: _Segment, s: float) -> np.ndarray:
    """dp/dt = (dp/ds) / (t1 - t0),  dp/ds = b + 2c·s + 3d·s²."""
    dt = seg.t1 - seg.t0
    s2 = s * s
    return np.array([
        (seg.cx[1] + 2 * seg.cx[2] * s + 3 * seg.cx[3] * s2) / dt,
        (seg.cy[1] + 2 * seg.cy[2] * s + 3 * seg.cy[3] * s2) / dt,
        (seg.cz[1] + 2 * seg.cz[2] * s + 3 * seg.cz[3] * s2) / dt,
    ])


# ─────────────────────────────────────────────────────────────────────────────
# Helpers de construcción de keypoints
# ─────────────────────────────────────────────────────────────────────────────

def keypoints_from_fixed(via: np.ndarray, fixed: dict) -> List[CartesianWaypoint]:
    """Arma los 5 keypoints [A, B, O, C, D] (igual que ``evaluate``)."""
    R = fixed['R_tcp']
    return [
        CartesianWaypoint(np.asarray(fixed['A'], float), R, fixed['t_A']),
        CartesianWaypoint(np.asarray(fixed['B'], float), R, fixed['t_B']),
        CartesianWaypoint(np.asarray(via,        float), R, fixed['t_O']),
        CartesianWaypoint(np.asarray(fixed['C'], float), R, fixed['t_C']),
        CartesianWaypoint(np.asarray(fixed['D'], float), R, fixed['t_D']),
    ]


def time_domain(fixed: dict) -> tuple[float, float]:
    """Dominio temporal completo [t_A, t_D] de la trayectoria."""
    return float(fixed['t_A']), float(fixed['t_D'])


# ─────────────────────────────────────────────────────────────────────────────
# Integrando de longitud de arco:  ‖ṗ(t)‖   (analítico, suave)
# ─────────────────────────────────────────────────────────────────────────────

def make_arclength_integrand(
    keypoints:   List[CartesianWaypoint],
    pts_per_seg: int = 8,
) -> Callable[[float], float]:
    """
    Devuelve el integrando ``f(t) = ‖ṗ(t)‖`` (rapidez cartesiana) evaluado de
    forma analítica a partir de la derivada del spline clamped.

    Su integral sobre ``[t_A, t_D]`` es exactamente ``f2`` (longitud de arco).
    Es continuo y suave en el interior de cada spline (la rapidez se anula en
    los extremos clamped t_A, t_B, t_C, t_D), por lo que es el showcase ideal
    para Gauss-Legendre / Romberg.

    ``pts_per_seg`` se mantiene por compatibilidad de firma; la derivada es
    analítica y no requiere muestreo.
    """
    segs = _build_segment_table(keypoints)

    def integrand(t: float) -> float:
        seg, s = _locate(segs, t)
        v = _velocity(seg, s)
        return float(np.linalg.norm(v))

    return integrand


# ─────────────────────────────────────────────────────────────────────────────
# Integrando de esfuerzo:  Σᵢ τᵢ(t)²   (IK + RNEA, costoso)
# ─────────────────────────────────────────────────────────────────────────────

def make_effort_integrand(
    via:        np.ndarray,
    fixed:      dict,
    evaluator,                 # TrajectoryEvaluator (acceso a IK + Pinocchio)
    h_fd:       float = 2.0e-2,
    ik_tol:     float = 1.0e-8,
    ik_max_iter: int  = 500,
) -> Callable[[float], float]:
    """
    Devuelve el integrando ``f(t) = Σᵢ τᵢ(t)²`` del esfuerzo articular.

    Para cada ``t``:
      1. Resuelve la pose cartesiana ``p(t), R(t)`` del spline.
      2. Resuelve IK en ``t`` (warm-start con la solución cacheada más cercana)
         y en ``t ± h_fd`` (warm-start desde ``q(t)`` para mantener la misma
         rama y obtener ``q̇, q̈`` limpios).
      3. Estima ``q̇, q̈`` por diferencias centradas y evalúa ``τ = pin.rnea``.

    Costoso y sólo bien definido donde la IK converge; por eso en el estudio se
    compara contra una referencia de malla muy fina y no contra un valor
    analítico.  ``h_fd`` es el paso temporal de las diferencias finitas [s].

    Importante — precisión de la IK
    --------------------------------
    La estimación de ``q̈`` por diferencias centradas amplifica el ruido de la
    IK por ``1/h_fd²``.  La IK del CU3 (``tol≈1e-4``) está afinada para
    velocidad y produce un integrando dominado por ruido; aquí se usa una IK
    mucho más estricta (``ik_tol=1e-8``, ``ik_max_iter=500``) para que el
    integrando sea físicamente representativo y suave.
    """
    keypoints = keypoints_from_fixed(via, fixed)
    segs = _build_segment_table(keypoints)
    t_a, t_d = time_domain(fixed)

    mdl    = evaluator._pin_mdl
    dat    = evaluator._pin_dat
    ik     = evaluator._ik
    q_home = evaluator._q_home
    # IK estricta para diferencias finitas limpias (sobre-escribe la del CU3).
    ik_kw  = dict(evaluator._ik_kwargs)
    ik_kw['tol']      = ik_tol
    ik_kw['max_iter'] = ik_max_iter

    # Cache (t, q) para warm-start continuo entre evaluaciones dispersas.
    cache_t: List[float]      = []
    cache_q: List[np.ndarray] = []

    def _pose(t: float):
        seg, s = _locate(segs, t)
        R = _slerp(seg.R0, seg.R1, s)
        return _position(seg, s), R

    def _seed(t: float) -> np.ndarray:
        if not cache_t:
            return q_home.copy()
        idx = int(np.argmin(np.abs(np.asarray(cache_t) - t)))
        return cache_q[idx].copy()

    def _solve(t: float, seed: np.ndarray) -> np.ndarray:
        p, R = _pose(t)
        q, _ = ik.solve(seed, p, R, **ik_kw)
        return q

    def integrand(t: float) -> float:
        q0 = _solve(t, _seed(t))
        cache_t.append(t)
        cache_q.append(q0)

        # Paso de diferencias finitas acotado al dominio.
        h = min(h_fd, 0.5 * (t_d - t_a))
        tm, tp = max(t - h, t_a), min(t + h, t_d)
        qm = _solve(tm, q0)
        qp = _solve(tp, q0)

        dq  = (qp - qm) / (tp - tm)
        ddq = (qp - 2.0 * q0 + qm) / (0.5 * (tp - tm)) ** 2

        tau = pin.rnea(mdl, dat, q0, dq, ddq)
        return float(np.dot(tau, tau))

    return integrand
