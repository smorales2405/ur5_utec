"""
Multi-objective optimizer for the CU3 via-point problem.

Provides two complementary methods:
  1. NSGA-II (pymoo)     — approximates the Pareto front (3 objectives)
  2. ε-constraint (scipy) — finds knee/compromise solutions along the front

Decision variable: x = [via_x, via_y, via_z]  (3-D, Pinocchio frame)
Objectives:        F = [f1, f2, f3]
Constraint:        G = [g1] = [r_grip - d_min] ≤ 0
"""

from __future__ import annotations
import numpy as np
import pinocchio as pin
from typing import Dict, List, Optional, Tuple

from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.core.problem import ElementwiseProblem
from pymoo.optimize import minimize as pymoo_minimize
from pymoo.termination.default import DefaultMultiObjectiveTermination
from scipy.optimize import minimize as scipy_minimize

from .trajectory_model import build_trajectory, CartesianWaypoint
from .ik_interface import IKInterface
from .objective_evaluators import evaluate_candidate
from .constraints import is_feasible


# ─────────────────────────────────────────────────────────────────────────────
# Shared evaluation kernel (used by both methods)
# ─────────────────────────────────────────────────────────────────────────────

class TrajectoryEvaluator:
    """
    Thread-unsafe (each worker must own its own instance in multiprocessing).
    Holds Pinocchio model/data and IK interface for repeated evaluations.
    """

    # Large penalty returned when a candidate is infeasible
    _PENALTY = 1e6

    def __init__(self, config: Dict, urdf_path: str) -> None:
        self._cfg     = config
        self._ik      = IKInterface(urdf_path)
        self._pin_mdl = pin.buildModelFromUrdf(urdf_path)
        self._pin_dat = self._pin_mdl.createData()

        # Fixed trajectory parameters
        pp  = config['pre_post_duration']
        td  = config['total_duration']
        t0  = 0.0
        self._fixed = {
            'A':   np.array(config['point_A']),
            'B':       np.array(config['point_B']),
            'C':       np.array(config['point_C']),
            'D':  np.array(config['point_D']),
            'R_tcp':   config['R_tcp'],
            't_A': t0,
            't_B':     t0 + pp,
            't_O':   t0 + pp + td / 2.0,
            't_C':     t0 + pp + td,
            't_D':t0 + pp + td + pp,
        }

        self._q_home       = np.array(config['home_joint_angles'])
        self._pts_per_seg  = config['pts_per_seg']
        self._q_min        = np.array(config['joint_q_min'])
        self._q_max        = np.array(config['joint_q_max'])
        self._obstacle     = {
            'center':       np.array(config['obstacle_center']),
            'half_extents': np.array(config['obstacle_half_extents']),
            'r_grip':       config['obstacle_r_grip'],
        }
        self._ik_kwargs = {
            'max_iter': config.get('ik_max_iter', 80),
            'tol':      config.get('ik_tol',      1e-4),
            'lam':      config.get('ik_lambda',   0.05),
            'alpha':    config.get('ik_alpha',    0.8),
        }

    # ------------------------------------------------------------------
    def evaluate(self, via: np.ndarray) -> Tuple[float, float, float, float]:
        """
        Evaluate objectives and constraint for one via-point candidate.
        Returns (f1, f2, f3, g1).  Infeasible → large penalty values.
        """
        traj = build_trajectory(via, self._fixed, self._pts_per_seg)

        qs, ik_ok = self._ik.solve_trajectory(
            traj, self._q_home, **self._ik_kwargs)

        feasible, _ = is_feasible(
            ik_ok, qs, self._q_min, self._q_max)
        if not feasible:
            return self._PENALTY, self._PENALTY, self._PENALTY, self._PENALTY

        # Build keypoints list for f2 (arc-length of spline segments)
        R = self._fixed['R_tcp']
        keypoints = [
            CartesianWaypoint(self._fixed['A'],  R, self._fixed['t_A']),
            CartesianWaypoint(self._fixed['B'],       R, self._fixed['t_B']),
            CartesianWaypoint(via,                    R, self._fixed['t_O']),
            CartesianWaypoint(self._fixed['C'],       R, self._fixed['t_C']),
            CartesianWaypoint(self._fixed['D'], R, self._fixed['t_D']),
        ]

        result = evaluate_candidate(
            traj, keypoints, qs, ik_ok,
            self._pin_mdl, self._pin_dat,
            self._obstacle, self._pts_per_seg,
        )
        if result is None:
            return self._PENALTY, self._PENALTY, self._PENALTY, self._PENALTY

        return result   # (f1, f2, f3, g1)


# ─────────────────────────────────────────────────────────────────────────────
# NSGA-II via pymoo
# ─────────────────────────────────────────────────────────────────────────────

class _ViaPointProblem(ElementwiseProblem):
    def __init__(self, evaluator: TrajectoryEvaluator, bounds: np.ndarray) -> None:
        xl = bounds[:, 0]
        xu = bounds[:, 1]
        super().__init__(n_var=3, n_obj=3, n_ieq_constr=1, xl=xl, xu=xu)
        self._ev = evaluator

    def _evaluate(self, x, out, *args, **kwargs):
        f1, f2, f3, g1 = self._ev.evaluate(x)
        out['F'] = [f1, f2, f3]
        out['G'] = [g1]


def run_nsga2(
    evaluator:   TrajectoryEvaluator,
    bounds:      np.ndarray,         # shape (3,2): [[xmin,xmax],[ymin,ymax],[zmin,zmax]]
    pop_size:    int   = 60,
    n_gen:       int   = 120,
    seed:        int   = 42,
    verbose:     bool  = True,
) -> Dict:
    """
    Run NSGA-II and return the Pareto-front approximation.

    Returns dict with keys:
      X  — decision variables of non-dominated solutions, shape (n, 3)
      F  — objective values,                             shape (n, 3)
      G  — constraint violations,                        shape (n, 1)
    """
    problem = _ViaPointProblem(evaluator, bounds)
    algorithm = NSGA2(pop_size=pop_size)
    termination = DefaultMultiObjectiveTermination(n_max_gen=n_gen)

    result = pymoo_minimize(
        problem, algorithm, termination,
        seed=seed, verbose=verbose, save_history=False,
    )

    # result.X/F/G are None when no feasible (non-dominated) solutions exist.
    # Fall back to the final population so callers always get arrays.
    if result.X is not None:
        return {'X': result.X, 'F': result.F, 'G': result.G}

    pop = result.pop
    return {
        'X': pop.get('X'),
        'F': pop.get('F'),
        'G': pop.get('G'),
    }


# ─────────────────────────────────────────────────────────────────────────────
# ε-constraint via scipy
# ─────────────────────────────────────────────────────────────────────────────

def run_epsilon_constraint(
    evaluator:     TrajectoryEvaluator,
    pareto_F:      np.ndarray,    # (n_pareto, 3) from NSGA-II
    pareto_X:      np.ndarray,    # (n_pareto, 3) from NSGA-II
    bounds:        np.ndarray,    # (3, 2)
    eps_obj_idx:   int   = 0,     # index of f to sweep as ε (0=f1, 1=f2)
    n_steps:       int   = 25,
    verbose:       bool  = True,
) -> Dict:
    """
    ε-constraint method: for n_steps values of ε spanning the range of
    f[eps_obj_idx] on the Pareto front, minimize the sum of the other two
    objectives subject to f[eps_obj_idx] ≤ ε.

    Returns dict with keys:
      X           — (n_steps, 3) optimal decision vars per ε level
      F           — (n_steps, 3) objective values
      eps_values  — (n_steps,)   ε levels used
      success     — (n_steps,)   bool, scipy optimisation converged
    """
    other_idx = [i for i in range(3) if i != eps_obj_idx]
    f_eps_col = pareto_F[:, eps_obj_idx]
    eps_min, eps_max = f_eps_col.min(), f_eps_col.max()
    eps_values = np.linspace(eps_min, eps_max, n_steps)

    scipy_bounds = [(bounds[i, 0], bounds[i, 1]) for i in range(3)]

    result_X, result_F, successes = [], [], []

    for k, eps in enumerate(eps_values):
        # Warm-start from the Pareto-front member closest to current ε
        closest_idx = np.argmin(np.abs(f_eps_col - eps))
        x0 = pareto_X[closest_idx]

        def objective(x):
            f1, f2, f3, _ = evaluator.evaluate(x)
            fs = [f1, f2, f3]
            return fs[other_idx[0]] + fs[other_idx[1]]

        def eps_ineq(x):
            # f[eps_obj_idx] ≤ ε  →  ε - f[eps_obj_idx] ≥ 0  (scipy 'ineq')
            fs = evaluator.evaluate(x)
            return eps - fs[eps_obj_idx]

        def obstacle_ineq(x):
            # g1 = r_grip - d_min ≤ 0  →  -g1 ≥ 0
            _, _, _, g1 = evaluator.evaluate(x)
            return -g1

        constraints = [
            {'type': 'ineq', 'fun': eps_ineq},
            {'type': 'ineq', 'fun': obstacle_ineq},
        ]

        res = scipy_minimize(
            objective, x0,
            method='SLSQP',
            bounds=scipy_bounds,
            constraints=constraints,
            options={'maxiter': 200, 'ftol': 1e-6},
        )

        f1, f2, f3, _ = evaluator.evaluate(res.x)
        result_X.append(res.x)
        result_F.append([f1, f2, f3])
        successes.append(res.success)

        if verbose:
            status = 'OK' if res.success else 'failed'
            print(f"  ε[{k+1:02d}/{n_steps}] = {eps:.4f}  "
                  f"f={[f1, f2, f3]}  {status}")

    return {
        'X':          np.array(result_X),
        'F':          np.array(result_F),
        'eps_values': eps_values,
        'success':    np.array(successes),
    }


# ─────────────────────────────────────────────────────────────────────────────
# Knee-point selector (min-distance to utopia on normalised front)
# ─────────────────────────────────────────────────────────────────────────────

def select_knee_point(F: np.ndarray) -> int:
    """
    Returns the index of the solution closest to the utopia point
    on the normalised objective space (min-distance method).
    """
    utopia   = F.min(axis=0)
    nadir    = F.max(axis=0)
    denom    = nadir - utopia
    denom[denom == 0] = 1.0
    F_norm   = (F - utopia) / denom
    dist     = np.linalg.norm(F_norm, axis=1)
    return int(np.argmin(dist))
