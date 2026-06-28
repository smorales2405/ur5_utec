"""
Pilar 2 — Optimización mono-objetivo con restricciones (Unidad 7, "Alternativa 1b").

Problema:
    min  f1(O)              (esfuerzo articular ∫Στ²dt)
    s.a. d_min(O) ≥ d_safe  (holgura al obstáculo)
         O ∈ caja           (cotas del via-point)

con  ``d_min = r_grip − g1``  y  ``d_safe = r_grip + delta_safe``.

Tres solvers que reutilizan el ``TrajectoryEvaluator`` del CU3:
  * ``steepest_descent``  — penalización exterior + máxima inclinación
                            (gradiente por diferencias finitas, búsqueda lineal),
                            con proyección sobre la caja de cotas.
  * ``direct_search``     — método sin gradiente (búsqueda por coordenadas),
                            verificación de robustez frente al ruido del evaluador.
  * ``slsqp_reference``   — SLSQP de scipy con las restricciones explícitas
                            (el "ode45" del problema: referencia profesional).

Todos devuelven un dict homogéneo:
    {'x', 'f1', 'd_min', 'n_eval', 'n_iter', 'history', 'success'}
"""

from __future__ import annotations
import numpy as np
from scipy.optimize import minimize as scipy_minimize, minimize_scalar
from typing import Callable, Dict, List, Tuple


# ─────────────────────────────────────────────────────────────────────────────
# Objetivo escalar y objetivo penalizado
# ─────────────────────────────────────────────────────────────────────────────

def scalar_objective(evaluator, via: np.ndarray) -> Tuple[float, float]:
    """
    Evalúa el esfuerzo ``f1`` y la holgura ``d_min`` para un via-point.

    Returns
    -------
    f1    : esfuerzo articular [N²·m²·s]  (``_PENALTY`` si la IK es infactible).
    d_min : holgura mínima TCP-obstáculo [m] = ``r_grip − g1``.
    """
    f1, _f2, _f3, g1 = evaluator.evaluate(np.asarray(via, dtype=float))
    r_grip = evaluator._obstacle['r_grip']
    d_min  = r_grip - g1
    return float(f1), float(d_min)


def penalized_objective(
    evaluator,
    via:    np.ndarray,
    mu:     float,
    d_safe: float,
    bounds: np.ndarray,
) -> float:
    """
    Función penalizada exterior (cuadrática):

        J̃(O) = f1(O)
              + mu · max(0, d_safe − d_min)²          (violación de holgura)
              + mu · Σⱼ [max(0, xlⱼ − Oⱼ)² + max(0, Oⱼ − xuⱼ)²]   (violación de cotas)

    Penaliza acercarse al obstáculo por debajo de ``d_safe`` y salir de la caja.
    """
    f1, d_min = scalar_objective(evaluator, via)

    pen_clear = max(0.0, d_safe - d_min) ** 2

    via = np.asarray(via, dtype=float)
    xl, xu = bounds[:, 0], bounds[:, 1]
    viol = np.maximum(xl - via, 0.0) ** 2 + np.maximum(via - xu, 0.0) ** 2
    pen_box = float(viol.sum())

    return float(f1 + mu * pen_clear + mu * pen_box)


def _project(via: np.ndarray, bounds: np.ndarray) -> np.ndarray:
    """Proyecta sobre la caja de cotas (las restricciones de borde son triviales)."""
    return np.clip(via, bounds[:, 0], bounds[:, 1])


def _projected_gradient(grad: np.ndarray, x: np.ndarray, bounds: np.ndarray,
                        eps: float = 1e-9) -> np.ndarray:
    """
    Gradiente proyectado sobre las cotas activas: anula las componentes que
    empujan hacia AFUERA de una cota activa.  Su norma es la medida correcta de
    estacionariedad KKT (se anula en un mínimo sobre la frontera, a diferencia
    del gradiente crudo, que ahí sigue siendo grande).
    """
    g = grad.copy()
    at_lo = x <= bounds[:, 0] + eps
    at_hi = x >= bounds[:, 1] - eps
    g[np.logical_and(at_lo, grad > 0.0)] = 0.0
    g[np.logical_and(at_hi, grad < 0.0)] = 0.0
    return g


# ─────────────────────────────────────────────────────────────────────────────
# Máxima inclinación (steepest descent) con penalización
# ─────────────────────────────────────────────────────────────────────────────

def _fd_gradient(J: Callable[[np.ndarray], float], x: np.ndarray,
                 h: float = 1.0e-3) -> np.ndarray:
    """Gradiente por diferencias centradas (Unidad 7 / laboratorio)."""
    g = np.zeros_like(x)
    for i in range(len(x)):
        xp = x.copy(); xp[i] += h
        xm = x.copy(); xm[i] -= h
        g[i] = (J(xp) - J(xm)) / (2.0 * h)
    return g


def steepest_descent(
    evaluator,
    x0:       np.ndarray,
    bounds:   np.ndarray,
    mu:       float,
    d_safe:   float,
    tol:      float = 1.0e-4,
    max_iter: int   = 50,
    fd_step:  float = 1.0e-3,
) -> Dict:
    """
    Máxima inclinación sobre la función penalizada ``J̃``.

    - Gradiente ``∇J̃`` por diferencias finitas centradas.
    - Dirección de descenso ``d = −∇J̃``.
    - Paso por búsqueda lineal acotada (``minimize_scalar(method='bounded')``,
      análogo al ``fminbnd`` del laboratorio), con los iterados proyectados sobre
      la caja de cotas (proyección de las restricciones de borde, exactas).

    Guarda el historial ``(x_k, J_k, ‖∇J̃‖)`` para graficar la trayectoria de
    optimización.
    """
    evaluator.reset_eval_counter()
    x0 = _project(np.asarray(x0, dtype=float), bounds)

    def J(v):
        return penalized_objective(evaluator, _project(v, bounds), mu, d_safe, bounds)

    x = x0.copy()
    Jx = J(x)
    box_diag = float(np.linalg.norm(bounds[:, 1] - bounds[:, 0]))
    history: List[Tuple[np.ndarray, float, float]] = []

    success = False
    n_iter = 0
    for k in range(max_iter):
        n_iter = k + 1
        grad  = _fd_gradient(J, x, h=fd_step)
        pgrad = _projected_gradient(grad, x, bounds)
        gnorm = float(np.linalg.norm(pgrad))         # estacionariedad KKT
        history.append((x.copy(), Jx, gnorm))

        if gnorm < tol:                              # gradiente proyectado ≈ 0
            success = True
            break

        # Dirección de descenso proyectada (se mueve sólo por caras libres).
        direction = -pgrad / gnorm
        alpha_max = 0.5 * box_diag                   # paso máximo acotado

        def line(alpha):
            return J(_project(x + alpha * direction, bounds))

        res = minimize_scalar(line, bounds=(0.0, alpha_max), method='bounded',
                              options={'xatol': 1e-5})
        x_new = _project(x + res.x * direction, bounds)
        J_new = J(x_new)

        if np.linalg.norm(x_new - x) < tol:          # iterado estancado
            x, Jx = x_new, J_new
            success = True
            break
        x, Jx = x_new, J_new

    g_fin = _projected_gradient(_fd_gradient(J, x, h=fd_step), x, bounds)
    history.append((x.copy(), Jx, float(np.linalg.norm(g_fin))))
    f1, d_min = scalar_objective(evaluator, x)
    return {
        'x':       x,
        'f1':      f1,
        'd_min':   d_min,
        'n_eval':  evaluator.n_eval,
        'n_iter':  n_iter,
        'history': history,
        'success': success,
    }


# ─────────────────────────────────────────────────────────────────────────────
# Búsqueda directa (sin gradiente): búsqueda por coordenadas / patrón
# ─────────────────────────────────────────────────────────────────────────────

def direct_search(
    evaluator,
    x0:        np.ndarray,
    bounds:    np.ndarray,
    mu:        float,
    d_safe:    float,
    init_step: float = 0.05,
    tol:       float = 1.0e-4,
    max_iter:  int   = 100,
) -> Dict:
    """
    Búsqueda por coordenadas (compass / pattern search) sobre ``J̃``, sin
    gradiente.  En cada iteración prueba ``±step`` en cada eje; si algún
    movimiento mejora, se desplaza al mejor; si ninguno mejora, reduce el paso a
    la mitad.  Robusto frente al ruido / no diferenciabilidad del evaluador.

    Termina cuando el paso baja de ``tol`` o se agotan las iteraciones.
    """
    evaluator.reset_eval_counter()
    x = _project(np.asarray(x0, dtype=float), bounds)

    def J(v):
        return penalized_objective(evaluator, v, mu, d_safe, bounds)

    Jx = J(x)
    step = init_step
    n = len(x)
    history: List[Tuple[np.ndarray, float, float]] = [(x.copy(), Jx, step)]

    success = False
    n_iter = 0
    for k in range(max_iter):
        n_iter = k + 1
        best_x, best_J = x, Jx
        for i in range(n):
            for sgn in (+1.0, -1.0):
                cand = x.copy()
                cand[i] += sgn * step
                cand = _project(cand, bounds)
                Jc = J(cand)
                if Jc < best_J:
                    best_x, best_J = cand, Jc
        if best_J < Jx:                      # movimiento aceptado
            x, Jx = best_x, best_J
        else:                                # estancado → reducir paso
            step *= 0.5
        history.append((x.copy(), Jx, step))
        if step < tol:
            success = True
            break

    f1, d_min = scalar_objective(evaluator, x)
    return {
        'x':       x,
        'f1':      f1,
        'd_min':   d_min,
        'n_eval':  evaluator.n_eval,
        'n_iter':  n_iter,
        'history': history,
        'success': success,
    }


# ─────────────────────────────────────────────────────────────────────────────
# Referencia SLSQP (restricciones explícitas)
# ─────────────────────────────────────────────────────────────────────────────

def slsqp_reference(
    evaluator,
    x0:      np.ndarray,
    bounds:  np.ndarray,
    d_safe:  float,
) -> Dict:
    """
    Adaptación del bloque SLSQP del CU3 (``run_epsilon_constraint``) al problema
    mono-objetivo:

        min f1(O)
        s.a.  d_min − d_safe ≥ 0     (holgura segura)
              −g1 ≥ 0                 (no colisión: d_min ≥ r_grip)
              O ∈ caja

    Es la referencia profesional contra la que se contrastan máxima inclinación
    y búsqueda directa.
    """
    evaluator.reset_eval_counter()
    x0 = _project(np.asarray(x0, dtype=float), bounds)
    scipy_bounds = [(bounds[i, 0], bounds[i, 1]) for i in range(3)]

    def objective(x):
        f1, _f2, _f3, _g1 = evaluator.evaluate(x)
        return f1

    def clearance_ineq(x):
        # d_min − d_safe ≥ 0
        _f1, d_min = scalar_objective(evaluator, x)
        return d_min - d_safe

    def collision_ineq(x):
        # −g1 ≥ 0  (d_min ≥ r_grip)
        _f1, _f2, _f3, g1 = evaluator.evaluate(x)
        return -g1

    constraints = [
        {'type': 'ineq', 'fun': clearance_ineq},
        {'type': 'ineq', 'fun': collision_ineq},
    ]

    res = scipy_minimize(
        objective, x0, method='SLSQP',
        bounds=scipy_bounds, constraints=constraints,
        options={'maxiter': 200, 'ftol': 1e-6},
    )

    f1, d_min = scalar_objective(evaluator, res.x)
    return {
        'x':       np.asarray(res.x, dtype=float),
        'f1':      f1,
        'd_min':   d_min,
        'n_eval':  evaluator.n_eval,
        'n_iter':  int(res.nit),
        'history': None,
        'success': bool(res.success),
    }
