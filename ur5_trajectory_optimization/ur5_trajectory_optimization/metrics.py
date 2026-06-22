"""
Pareto-front quality metrics for CU3 (ur5_trajectory_optimization).

HV reference point (fixed, documented)
---------------------------------------
HV_REF_POINT = [20 000, 3.0, 0.0]

  f1 ref = 20 000 N²·m²·s  (> NSGA-II max ≈ 16 300)
  f2 ref = 3.0 m            (> observed max ≈ 2.78)
  f3 ref = 0.0              (f3 = −clearance ≤ −0.05 for feasible solutions)

A fixed reference (not the per-run nadir) ensures that HV values from
different optimisation runs are directly comparable.
"""

from __future__ import annotations
import numpy as np
from pymoo.indicators.hv import HV as _HV
from pymoo.indicators.igd import IGD as _IGD

HV_REF_POINT: np.ndarray = np.array([20_000.0, 3.0, 0.0])


def compute_hv(
    F: np.ndarray,
    ref_point: np.ndarray = HV_REF_POINT,
) -> float:
    """
    Hypervolume dominated by F w.r.t. ref_point (minimisation).
    Solutions not strictly below ref_point are silently excluded so
    the indicator is always well-defined.
    """
    mask = np.all(F < ref_point, axis=1)
    if not mask.any():
        return 0.0
    return float(_HV(ref_point=ref_point)(F[mask]))


def compute_spacing(F: np.ndarray) -> float:
    """
    Spacing metric S (Schott 1995).
    S = 0  →  perfectly uniform distribution.
    Larger S  →  more irregular spacing between solutions.
    """
    n = len(F)
    if n < 2:
        return 0.0
    # Manhattan distance to nearest neighbour for each solution
    dists = np.array([
        np.min(np.sum(np.abs(F - F[i]), axis=1)[np.arange(n) != i])
        for i in range(n)
    ])
    d_bar = dists.mean()
    return float(np.sqrt(((dists - d_bar) ** 2).sum() / (n - 1)))


def compute_igd(F: np.ndarray, F_ref: np.ndarray) -> float:
    """
    Inverted Generational Distance from approximation F to reference F_ref.
    Lower IGD  →  F is closer to and covers F_ref better.
    """
    return float(_IGD(F_ref)(F))


def compute_coverage(F_A: np.ndarray, F_B: np.ndarray) -> float:
    """
    C(A, B): fraction of B weakly dominated by at least one member of A.
      C(A,B) = 1  →  A completely dominates B.
      C(A,B) = 0  →  no member of B is dominated by A.
    """
    if len(F_A) == 0 or len(F_B) == 0:
        return 0.0
    count = sum(
        1 for b in F_B
        if np.any(np.all(F_A <= b, axis=1))
    )
    return count / len(F_B)


def filter_nondominated(F: np.ndarray) -> np.ndarray:
    """Return boolean mask: True for non-dominated rows in F (O(n²), n ≤ 200)."""
    n = len(F)
    dominated = np.zeros(n, dtype=bool)
    for i in range(n):
        if dominated[i]:
            continue
        for j in range(n):
            if i != j and not dominated[j]:
                if np.all(F[j] <= F[i]) and np.any(F[j] < F[i]):
                    dominated[i] = True
                    break
    return ~dominated
