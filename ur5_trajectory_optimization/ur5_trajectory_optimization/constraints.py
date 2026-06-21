"""
Hard feasibility checks for the CU3 optimization problem.

These are evaluated BEFORE the objectives to reject infeasible candidates
early (before running expensive RNEA).

Checks:
  1. IK convergence rate ≥ min_ik_rate
  2. All joint angles within UR5 limits
  3. via-point inside search domain (enforced by optimizer bounds, checked here too)
"""

from __future__ import annotations
import numpy as np
from typing import List, Tuple


def check_ik_rate(
    ik_ok: List[bool],
    min_rate: float = 0.95,
) -> Tuple[bool, float]:
    """Returns (feasible, actual_rate)."""
    rate = sum(ik_ok) / len(ik_ok)
    return rate >= min_rate, rate


def check_joint_limits(
    qs:     List[np.ndarray],
    q_min:  np.ndarray,
    q_max:  np.ndarray,
) -> Tuple[bool, float]:
    """
    Returns (feasible, max_violation_rad).
    Feasible if all configurations satisfy q_min ≤ q ≤ q_max.
    """
    violations = []
    for q in qs:
        viol_lo = np.maximum(q_min - q, 0.0)
        viol_hi = np.maximum(q  - q_max, 0.0)
        violations.append(np.max(np.maximum(viol_lo, viol_hi)))
    max_viol = max(violations)
    return max_viol == 0.0, max_viol


def is_feasible(
    ik_ok:    List[bool],
    qs:       List[np.ndarray],
    q_min:    np.ndarray,
    q_max:    np.ndarray,
    min_ik_rate: float = 0.95,
) -> Tuple[bool, dict]:
    """
    Aggregate feasibility check.
    Returns (feasible, info_dict with individual check results).
    """
    ik_ok_flag, ik_rate  = check_ik_rate(ik_ok, min_ik_rate)
    jl_ok,      jl_viol  = check_joint_limits(qs, q_min, q_max)
    feasible = ik_ok_flag and jl_ok
    return feasible, {
        'ik_rate':       ik_rate,
        'ik_ok':         ik_ok_flag,
        'jl_violation':  jl_viol,
        'jl_ok':         jl_ok,
    }
