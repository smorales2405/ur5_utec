"""
Métodos de optimización de la FASE 7: NSGA-II, ε-restricción con SLSQP,
certificación KKT y la línea base de suma ponderada contra la que se compara.

Reutiliza el instrumental del CU3 (`..metrics`, `..multiobjective_optimizer`)
en vez de duplicarlo: son las mismas definiciones de hipervolumen, IGD,
cobertura y punto de rodilla que ya se reportaron allí, así que las tablas de
los dos trabajos son directamente comparables.
"""

from __future__ import annotations

import multiprocessing as mp
import time

import numpy as np
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.core.callback import Callback
from pymoo.core.problem import Problem
from pymoo.optimize import minimize as pymoo_minimize
from pymoo.termination.default import DefaultMultiObjectiveTermination
from scipy.optimize import minimize as scipy_minimize, nnls

from ..metrics import compute_hv
from .closed_loop import Plant, load_reference
from .problem import (CHI_THRESHOLD, PENALTY, GainEvaluator, GainTuningProblem,
                      make_evaluator)

# ─────────────────────────────────────────────────────────────────────────────
# Evaluación en paralelo
# ─────────────────────────────────────────────────────────────────────────────
#
# El modelo de Pinocchio no viaja bien por pickle, así que en vez de mandar el
# evaluador a los workers se manda su RECETA (rutas y escalares) y cada worker
# construye el suyo una sola vez en el init. Es también lo que hace que la caché
# sea por proceso, que es lo correcto: cada worker repite sus propios puntos.

_WORKER: dict = {}


def _worker_init(urdf, ref_path, mode, alpha, f_cut, chi_safety, tcp_tol_mm):
    _WORKER["ev"] = make_evaluator(
        load_reference(ref_path), Plant(urdf), mode=mode, alpha=alpha,
        f_cut=f_cut, chi_safety=chi_safety, tcp_tol_mm=tcp_tol_mm)


def _worker_eval(x):
    F, G = _WORKER["ev"].evaluate(x)
    return np.concatenate([F, G])


def _worker_result(x):
    """Devuelve el `EvalResult` completo (dataclass de escalares: picklea bien)."""
    return _WORKER["ev"].result(x)


def _pool_recipe(evaluator: GainEvaluator, urdf: str, ref_path: str) -> tuple:
    return (urdf, ref_path, evaluator.param.mode, evaluator.alpha,
            evaluator.force.f_cut if evaluator.force else 0.0,
            evaluator.chi_safety, evaluator.tcp_tol_mm)


class _PooledProblem(Problem):
    """Problema pymoo vectorizado que reparte la población en un pool."""

    def __init__(self, evaluator: GainEvaluator, pool):
        xl, xu = evaluator.param.bounds()
        super().__init__(n_var=evaluator.param.n_var, n_obj=3,
                         n_ieq_constr=GainEvaluator.N_CON, xl=xl, xu=xu)
        self._pool = pool

    def _evaluate(self, X, out, *args, **kwargs):
        rows = np.array(self._pool.map(_worker_eval, list(X)))
        out["F"] = rows[:, :3]
        out["G"] = rows[:, 3:]


class _HVCallback(Callback):
    """
    Hipervolumen y tamaño del frente por generación (curva de convergencia).

    El punto de referencia se FIJA con el nadir de la primera generación que
    tenga soluciones útiles y ya no se mueve. Un punto móvil (el nadir de cada
    generación) haría que el HV subiera y bajara por cambiar la vara de medir,
    no por mejorar el frente, y la curva de convergencia no significaría nada.
    """

    def __init__(self, ref_point=None):
        super().__init__()
        self._ref = None if ref_point is None else np.asarray(ref_point, dtype=float)
        self.history: list = []

    def notify(self, algorithm):
        opt = getattr(algorithm, "opt", None)
        if opt is None or len(opt) == 0:
            return
        F = opt.get("F")
        if F is None or len(F) == 0:
            return
        feas = np.all(F < PENALTY / 10.0, axis=1)
        if self._ref is None and feas.any():
            self._ref = hv_reference_point(F[feas])
        try:
            hv = (compute_hv(F[feas], self._ref)
                  if feas.any() and self._ref is not None else float("nan"))
        except Exception:
            hv = float("nan")
        self.history.append((int(algorithm.n_gen), hv, int(feas.sum())))

    @property
    def ref_point(self):
        return self._ref


def hv_reference_point(F: np.ndarray, margin: float = 1.10) -> np.ndarray:
    """
    Punto de referencia del hipervolumen: el nadir del frente con un margen.

    A diferencia del CU3 —donde el punto está FIJADO en `metrics.py` para que
    corridas distintas sean comparables— aquí las escalas de f2 y f3 dependen
    del controlador y de la parametrización, así que se deriva del frente y se
    REPORTA junto a los resultados. Sin reportarlo, un HV no significa nada.
    """
    F = np.asarray(F, dtype=float)
    F = F[np.all(F < PENALTY / 10.0, axis=1)]
    if len(F) == 0:
        return np.ones(3)
    nadir = F.max(axis=0)
    span = np.where(nadir > 0, nadir, 1.0)
    return nadir + (margin - 1.0) * np.abs(span)


# ─────────────────────────────────────────────────────────────────────────────
# NSGA-II
# ─────────────────────────────────────────────────────────────────────────────

def run_nsga2(evaluator: GainEvaluator, urdf: str, ref_path: str,
              pop_size: int = 40, n_gen: int = 60, seed: int = 42,
              n_jobs: int = 1, verbose: bool = True,
              hv_ref: np.ndarray | None = None, x0: np.ndarray | None = None) -> dict:
    """
    NSGA-II sobre la caja de la parametrización.

    `x0` siembra un individuo con ganancias conocidas (las de la FASE 5) para
    que el frente nunca sea PEOR que el punto de partida manual: sin eso, una
    comparación «optimizador vs sintonía a mano» puede salir favorable por puro
    azar de la población inicial.
    """
    ref_point = np.asarray(hv_ref) if hv_ref is not None else np.ones(3)
    callback = _HVCallback(ref_point)
    algorithm = NSGA2(pop_size=pop_size, sampling=_seeded_sampling(evaluator, pop_size, x0, seed))
    termination = DefaultMultiObjectiveTermination(n_max_gen=n_gen)

    t0 = time.time()
    pool = None
    try:
        if n_jobs > 1:
            pool = mp.Pool(n_jobs, initializer=_worker_init,
                           initargs=_pool_recipe(evaluator, urdf, ref_path))
            problem = _PooledProblem(evaluator, pool)
        else:
            problem = GainTuningProblem(evaluator)

        result = pymoo_minimize(problem, algorithm, termination, seed=seed,
                                verbose=verbose, save_history=False,
                                callback=callback)
    finally:
        if pool is not None:
            pool.close()
            pool.join()
    elapsed = time.time() - t0

    X = result.X if result.X is not None else result.pop.get("X")
    F = result.F if result.F is not None else result.pop.get("F")
    G = result.G if result.G is not None else result.pop.get("G")
    X = np.atleast_2d(X)
    F = np.atleast_2d(F)
    n_eval = int(getattr(result.algorithm.evaluator, "n_eval", 0))
    return {"X": X, "F": F, "G": np.atleast_2d(G) if G is not None else None,
            "convergence": callback.history, "n_eval": n_eval,
            "elapsed_s": elapsed,
            "sec_per_eval": elapsed / max(n_eval, 1),
            # Referencia con la que se midió la CURVA de convergencia. No es la
            # misma que la del HV final (esa se deriva del frente completo), así
            # que se reporta aparte: comparar los dos números sería un error.
            "convergence_hv_ref": callback.ref_point}


def _seeded_sampling(evaluator: GainEvaluator, pop_size: int,
                     x0: np.ndarray | None, seed: int):
    """Muestreo LHS con los puntos de `x0` inyectados en las primeras filas."""
    from pymoo.operators.sampling.lhs import LHS
    if x0 is None:
        return LHS()

    xl, xu = evaluator.param.bounds()
    seeds = np.atleast_2d(np.asarray(x0, dtype=float))
    base = np.asarray(LHS().do(GainTuningProblem(evaluator), pop_size).get("X"),
                      dtype=float)
    n = min(len(seeds), pop_size)
    base[:n] = np.clip(seeds[:n], xl, xu)
    return base


def seed_points(evaluator: GainEvaluator, d_bound: np.ndarray,
                dt: float | None = None) -> np.ndarray:
    """
    Población inicial sembrada con puntos DERIVADOS del análisis, no aleatorios.

    En 13 dimensiones un muestreo puramente aleatorio gasta casi todo el
    presupuesto lejos de la escala útil: `η` tiene que cubrir la perturbación de
    corte `d` y `φ` tiene que respetar `χ ≤ límite`, que acopla las dos. Se
    siembran combinaciones de margen sobre `d` y de `φ`, más las ganancias de la
    FASE 5 como referencia — así el frente nunca puede salir PEOR que el punto
    de partida manual, que es la comparación honesta.
    """
    p = evaluator.param
    dt = evaluator.ref.dt if dt is None else dt
    lo_phi, hi_phi = p.phi_bounds
    pts = [p.encode(np.full(6, 20.0), p.inertia * 1.0, 0.05)]   # FASE 5
    for lam in (30.0, 100.0, 250.0):
        for margin in (1.1, 1.5, 2.5):
            eta = d_bound * margin + p.inertia * 0.5
            # φ mínimo que respeta χ en la junta que más aprieta, con holgura.
            phi = float(np.clip(
                1.25 * np.max(eta * dt /
                              (evaluator.chi_safety * CHI_THRESHOLD * p.inertia)),
                lo_phi, hi_phi))
            pts.append(p.encode(np.full(6, lam), eta, phi))
    # Se recortan a la caja: el `η` de la FASE 5 en `wrist_3` (2.6e-4 N·m) cae
    # POR DEBAJO de la cota inferior anclada al actuador (1e-4·τ_max = 2.8e-3),
    # así que sin recortar la semilla saldría del dominio de búsqueda.
    xl, xu = p.bounds()
    return np.clip(np.array(pts), xl, xu)


# ─────────────────────────────────────────────────────────────────────────────
# ε-restricción con SLSQP
# ─────────────────────────────────────────────────────────────────────────────

#: Paso de las diferencias finitas de SLSQP, en el espacio de BÚSQUEDA (log10
#: para λ y η, lineal para φ). El de scipy por defecto es √ε_máquina ≈ 1.5e-8:
#: sobre la salida de un lazo cerrado de 13 802 pasos —donde f3 es la variación
#: total del par, muy sensible— una perturbación de ese tamaño devuelve ruido
#: numérico en vez de una derivada, y SLSQP no converge nunca. 1e-3 es un paso
#: con significado físico: 0.23 % en λ y η, 0.1 % del rango de φ.
FD_STEP = 1e-3


class _GradientPrefetcher:
    """
    Rellena la caché del evaluador con el punto y sus vecinos de diferencias
    finitas, calculados EN PARALELO.

    SLSQP pide los puntos de uno en uno, así que sin esto la ε-restricción corre
    en serie: `n_var + 1` simulaciones por iteración, ~14 s en 13 variables. Al
    adelantar el vecindario completo en un solo `pool.map`, la iteración cuesta
    lo que una sola simulación.
    """

    def __init__(self, evaluator: GainEvaluator, pool, step: float):
        self.ev = evaluator
        self.pool = pool
        self.step = step

    def prime(self, x: np.ndarray) -> None:
        x = np.asarray(x, dtype=float)
        pts = [x] + [x + self.step * e for e in np.eye(x.size)]
        missing = [p for p in pts if self.ev._key(p) not in self.ev._cache]
        if not missing:
            return
        for p, r in zip(missing, self.pool.map(_worker_result, missing)):
            self.ev._cache[self.ev._key(p)] = r
            self.ev.n_eval += 1


def run_epsilon_constraint(evaluator: GainEvaluator, pareto_F: np.ndarray,
                           pareto_X: np.ndarray, eps_obj_idx: int = 0,
                           n_steps: int = 12, maxiter: int | None = None,
                           urdf: str | None = None, ref_path: str | None = None,
                           n_jobs: int = 1, fd_step: float = FD_STEP,
                           verbose: bool = True) -> dict:
    """
    Para cada nivel ε del objetivo `eps_obj_idx`:

        min  f_a_norm(x) + f_b_norm(x)
        s.a. f_eps(x) ≤ ε,  g1..g4 ≤ 0,  x ∈ caja

    NORMALIZANDO los dos objetivos que van a la suma. Es la diferencia con el
    bloque equivalente del CU3, y no es cosmética: aquí f2 (esfuerzo, ~1e4) y
    f3 (variación total, ~1e2–1e6) difieren en varios órdenes de magnitud, así
    que sumarlos en crudo equivaldría a optimizar solo el mayor de los dos.
    Los factores salen del frente de NSGA-II y se reportan.

    Presupuesto: SLSQP con gradientes numéricos gasta `n_var + 1` evaluaciones
    por iteración —la caché hace que el objetivo y las cinco restricciones
    compartan simulación, si no serían seis veces más— y las pide de una en una.
    Con `n_jobs > 1` el `_GradientPrefetcher` adelanta ese vecindario en
    paralelo, con lo que la iteración cuesta una sola simulación en vez de 14.
    `maxiter` se escala además con la dimensión.
    """
    if maxiter is None:
        maxiter = max(15, int(200 / max(pareto_X.shape[1], 1)))
    n_var = pareto_X.shape[1]
    other = [i for i in range(3) if i != eps_obj_idx]
    feas = np.all(pareto_F < PENALTY / 10.0, axis=1)
    F_ok = pareto_F[feas] if feas.any() else pareto_F
    lo, hi = F_ok.min(axis=0), F_ok.max(axis=0)
    scale = np.where(hi - lo > 0, hi - lo, 1.0)

    f_eps_col = pareto_F[:, eps_obj_idx]
    eps_values = np.linspace(F_ok[:, eps_obj_idx].min(),
                             F_ok[:, eps_obj_idx].max(), n_steps)
    xl, xu = evaluator.param.bounds()
    bounds = list(zip(xl, xu))

    X_out, F_out, ok_out, nit_out = [], [], [], []
    t0 = time.time()
    pool = prefetch = None
    if n_jobs > 1 and urdf and ref_path:
        pool = mp.Pool(n_jobs, initializer=_worker_init,
                       initargs=_pool_recipe(evaluator, urdf, ref_path))
        prefetch = _GradientPrefetcher(evaluator, pool, fd_step)

    def _solve(eps, x0):
        def objective(x, _o=other, _s=scale, _l=lo):
            if prefetch is not None:
                prefetch.prime(x)
            F = evaluator.objectives(x)
            return float(((F[_o] - _l[_o]) / _s[_o]).sum())

        cons = [{"type": "ineq",
                 "fun": lambda x: float(eps - evaluator.objectives(x)[eps_obj_idx])}]
        for j in range(GainEvaluator.N_CON):
            cons.append({"type": "ineq",
                         "fun": lambda x, j=j: float(-evaluator.constraints(x)[j])})

        return scipy_minimize(objective, x0, method="SLSQP", bounds=bounds,
                              constraints=cons,
                              options={"maxiter": maxiter, "ftol": 1e-6,
                                       "eps": fd_step})

    try:
        for k, eps in enumerate(eps_values):
            x0 = pareto_X[int(np.argmin(np.abs(f_eps_col - eps)))]
            res = _solve(eps, x0)
            X_out.append(res.x)
            F_out.append(evaluator.objectives(res.x))
            ok_out.append(bool(res.success))
            nit_out.append(int(res.nit))
            if verbose:
                print(f"  ε[{k + 1:02d}/{n_steps}] = {eps:.6g}  "
                      f"{'OK' if res.success else 'falló'}  ({res.nit} iter)"
                      f"  [{res.message}]")
    finally:
        if pool is not None:
            pool.close()
            pool.join()

    return {"X": np.array(X_out), "F": np.array(F_out),
            "eps_values": eps_values, "success": np.array(ok_out),
            "nit": np.array(nit_out, dtype=int), "scale": scale, "lo": lo,
            "fd_step": fd_step, "maxiter": maxiter, "n_var": n_var,
            "elapsed_s": time.time() - t0}


# ─────────────────────────────────────────────────────────────────────────────
# Certificación KKT
# ─────────────────────────────────────────────────────────────────────────────

#: Escalas para adimensionalizar las restricciones antes de decidir cuáles
#: están ACTIVAS. Sin esto habría que comparar N·m con rad/s y con un número
#: adimensional usando la misma tolerancia, que no significa nada.
_CON_SCALE = np.array([150.0, np.pi, 1.0, 1.0])   # g1 N·m, g2 rad/s, g3 –, g4 mm


def certify_kkt(evaluator: GainEvaluator, x: np.ndarray, eps_obj_idx: int,
                eps: float, scale: np.ndarray, lo: np.ndarray,
                h: float = 1e-3, active_tol: float = 1e-3) -> dict:
    """
    Certifica KKT del problema que SLSQP resolvió REALMENTE, que es el de
    ε-restricción, no un escalarizado ponderado cualquiera:

        min  Σ_{i≠ε} (f_i(x) − lo_i)/scale_i
        s.a. (f_ε(x) − ε)/scale_ε ≤ 0,   g_j(x) ≤ 0,   xl ≤ x ≤ xu

    Certificar otro problema daría siempre «sin restricciones activas» y residuo
    relativo 1, porque el punto no es estacionario para ESE otro problema.

    Estacionariedad:   ∇f + Σ_j μ_j ∇c_j − ν_lo + ν_hi = 0,   μ, ν ≥ 0
    Complementariedad: μ_j c_j = 0   (solo entran las restricciones activas)

    Los multiplicadores salen de mínimos cuadrados NO NEGATIVOS sobre las
    restricciones activas, que es la forma limpia de imponer μ ≥ 0. Los
    gradientes son por diferencias centradas; gracias a la caché del evaluador
    el coste total es 2n+1 simulaciones, no 2n+1 por cada función.

    Se devuelve el residuo, no un veredicto binario: con gradientes numéricos
    sobre un lazo cerrado de 13 802 pasos, exigir residuo exactamente cero
    sería teatro.
    """
    x = np.asarray(x, dtype=float)
    n = x.size
    xl, xu = evaluator.param.bounds()
    other = [i for i in range(3) if i != eps_obj_idx]

    def stacked(z):
        F = evaluator.objectives(z)
        G = evaluator.constraints(z)
        obj = float(((F[other] - lo[other]) / scale[other]).sum())
        c_eps = float((F[eps_obj_idx] - eps) / scale[eps_obj_idx])
        return np.concatenate([[obj, c_eps], G / _CON_SCALE])

    base = stacked(x)
    J = np.zeros((base.size, n))
    for i in range(n):
        step = h * max(1.0, abs(x[i]))
        xp, xm = x.copy(), x.copy()
        xp[i] = min(x[i] + step, xu[i])
        xm[i] = max(x[i] - step, xl[i])
        denom = xp[i] - xm[i]
        if denom == 0.0:
            continue
        J[:, i] = (stacked(xp) - stacked(xm)) / denom

    grad_f, grad_c = J[0], J[1:]
    c_val = base[1:]
    labels_all = ["eps"] + [f"g{j + 1}" for j in range(GainEvaluator.N_CON)]

    cols, labels, active_vals = [], [], []
    for j, cj in enumerate(c_val):
        if cj >= -active_tol:                       # restricción activa
            cols.append(grad_c[j])
            labels.append(labels_all[j])
            active_vals.append(cj)
    for i in range(n):
        if x[i] <= xl[i] + active_tol:
            e = np.zeros(n); e[i] = -1.0
            cols.append(e); labels.append(f"x{i}_lo"); active_vals.append(0.0)
        if x[i] >= xu[i] - active_tol:
            e = np.zeros(n); e[i] = 1.0
            cols.append(e); labels.append(f"x{i}_hi"); active_vals.append(0.0)

    if cols:
        mu, residual = nnls(np.array(cols).T, -grad_f)
    else:
        mu, residual = np.array([]), float(np.linalg.norm(grad_f))

    gnorm = float(np.linalg.norm(grad_f))
    return {
        "stationarity_residual": float(residual),
        "stationarity_relative": float(residual / gnorm) if gnorm > 0 else 0.0,
        "grad_f_norm": gnorm,
        "max_violation": float(max(0.0, c_val.max())),
        "complementarity": float(np.abs(np.asarray(mu) * np.asarray(active_vals)).sum())
        if len(mu) else 0.0,
        "active": labels,
        "multipliers": {lab: float(m) for lab, m in zip(labels, mu)},
        "constraints_normalised": {lab: float(v)
                                   for lab, v in zip(labels_all, c_val)},
    }


# ─────────────────────────────────────────────────────────────────────────────
# Estudio de sensibilidad a alpha
# ─────────────────────────────────────────────────────────────────────────────

def alpha_sensitivity(evaluator: GainEvaluator, x: np.ndarray,
                      alphas=(0.1, 0.3, 0.5, 1.0)) -> list:
    """
    Reevalúa unas ganancias FIJAS variando `α`, la fracción de los términos
    nominales que la ley asume incierta (barrido que pide el plan).

    Responde a «¿las ganancias elegidas siguen siendo factibles si se asume más
    incertidumbre?», que es lo que importa de cara a la FASE 8: `α` entra en
    `K = η + α·|cota|`, así que subirlo sube `K` y con ello `χ`. Unas ganancias
    que solo son factibles con `α` pequeño no sirven bajo desajuste paramétrico.

    Es barato (una simulación por nivel) porque NO reoptimiza. Reoptimizar en
    cada `α` respondería a otra pregunta —cómo se mueve el óptimo— y cuesta una
    campaña entera por nivel.

    > Aviso de la FASE 5 que sigue vigente: en Gazebo el modelo es perfecto, así
    > que un `α` pequeño sale «mejor» porque no hay incertidumbre que dominar.
    > El barrido solo es interpretable junto con el de desajuste (FASE 8).
    """
    rows = []
    alpha_0 = evaluator.alpha
    try:
        for a in alphas:
            evaluator.alpha = float(a)
            evaluator._cache.clear()      # α cambia la ley: la caché ya no vale
            r = evaluator.result(x)
            rows.append({
                "alpha": float(a),
                "f1_iae": r.f1_iae, "f2_effort": r.f2_effort,
                "f3_chatter": r.f3_chatter,
                "tcp_rmse_mm": r.rmse_tcp_mm, "rmse_q": r.rmse_q,
                "s_max": r.s_max, "chi": r.chi_max,
                "g1_tau": r.g1_tau, "g2_dq": r.g2_dq,
                "feasible": bool(r.g1_tau <= 0 and r.g2_dq <= 0
                                 and np.max(np.asarray(r.chi_joint) /
                                            CHI_THRESHOLD)
                                 <= evaluator.chi_safety),
            })
    finally:
        evaluator.alpha = alpha_0
        evaluator._cache.clear()
    return rows


# ─────────────────────────────────────────────────────────────────────────────
# Línea base: suma ponderada sobre sustituto polinómico cúbico
# ─────────────────────────────────────────────────────────────────────────────

def _cubic_features(X: np.ndarray) -> np.ndarray:
    """Base polinómica completa de grado 3 (con términos cruzados)."""
    X = np.atleast_2d(X)
    n, d = X.shape
    cols = [np.ones(n)]
    for i in range(d):
        cols.append(X[:, i])
    for i in range(d):
        for j in range(i, d):
            cols.append(X[:, i] * X[:, j])
    for i in range(d):
        for j in range(i, d):
            for k in range(j, d):
                cols.append(X[:, i] * X[:, j] * X[:, k])
    return np.column_stack(cols)


def run_weighted_sum_baseline(evaluator: GainEvaluator, weights, scale, lo,
                              n_samples: int = 60, seed: int = 42,
                              verbose: bool = True) -> dict:
    """
    Reproduce el método PREVIO al que se compara la fase: suma ponderada de los
    objetivos, minimizada sobre un **sustituto polinómico cúbico** ajustado a un
    muestreo del espacio de diseño (es lo que hacía la tesis de partida).

    Se ejecuta sobre la parametrización escalar; en 13 variables la base cúbica
    completa tiene 560 términos y el ajuste no sería honesto con 60 muestras.

    Devuelve el óptimo del sustituto EVALUADO CON EL SIMULADOR REAL: comparar
    predicciones del sustituto contra objetivos reales sería hacer trampa a
    favor del método nuevo.
    """
    xl, xu = evaluator.param.bounds()
    rng = np.random.default_rng(seed)
    X = rng.uniform(xl, xu, size=(n_samples, xl.size))

    y, F_all = [], []
    for i, xi in enumerate(X):
        F = evaluator.objectives(xi)
        G = evaluator.constraints(xi)
        # La suma ponderada clásica no distingue factible de infactible: se
        # penaliza la violación, que es como se hacía y como hay que reportarlo.
        pen = float(np.sum(np.clip(G, 0.0, None) ** 2))
        y.append(float((weights * (F - lo) / scale).sum()) + 1e3 * pen)
        F_all.append(F)
        if verbose and (i + 1) % 20 == 0:
            print(f"  muestreo {i + 1}/{n_samples}")

    y = np.array(y)
    finite = np.isfinite(y) & (y < PENALTY / 10.0)
    A = _cubic_features(X[finite])
    coef, *_ = np.linalg.lstsq(A, y[finite], rcond=None)

    def surrogate(z):
        return float(_cubic_features(np.atleast_2d(z)) @ coef)

    best = None
    for x0 in rng.uniform(xl, xu, size=(20, xl.size)):
        res = scipy_minimize(surrogate, x0, method="L-BFGS-B",
                             bounds=list(zip(xl, xu)))
        if best is None or res.fun < best.fun:
            best = res

    x_star = np.clip(best.x, xl, xu)
    F_star = evaluator.objectives(x_star)
    G_star = evaluator.constraints(x_star)
    return {
        "x": x_star, "F": F_star, "G": G_star,
        "feasible": bool(np.all(G_star <= 0)),
        "surrogate_pred": surrogate(x_star),
        "scalar_true": float((weights * (F_star - lo) / scale).sum()),
        "n_samples": int(finite.sum()),
        "fit_r2": float(1.0 - np.var(A @ coef - y[finite]) / max(np.var(y[finite]), 1e-30)),
    }
