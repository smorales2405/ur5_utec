"""
Objective function evaluators for CU3 multi-objective optimization.

Objectives (all to be minimized):
  f1 — Joint effort:      ∫₀ᵀ Σᵢ τᵢ(t)² dt   [N²·m²·s]
  f2 — Arc length:        ∫₀ᵀ ‖ṗ(t)‖ dt        [m]
  f3 — Obstacle penalty:  −d_min               [m] (minimize → maximize clearance)

Hard constraint (for NSGA-II G vector):
  g1 = r_grip − d_min ≤ 0   (gripper does not penetrate obstacle surface)

All computations are offline (no ROS 2 / Gazebo required).
"""

from __future__ import annotations
import numpy as np
import pinocchio as pin
from typing import List, Dict, Tuple

from .trajectory_model import CartesianWaypoint, spline_arc_length


# ─────────────────────────────────────────────────────────────────────────────
# RNEA helpers
# ─────────────────────────────────────────────────────────────────────────────

def _finite_diff(vals: List[np.ndarray], times: List[float]) -> List[np.ndarray]:
    """Central finite differences (forward/backward at endpoints)."""
    N = len(vals)
    dvs = []
    for i in range(N):
        if i == 0:
            dt = times[1] - times[0]
            dvs.append((vals[1] - vals[0]) / dt)
        elif i == N - 1:
            dt = times[-1] - times[-2]
            dvs.append((vals[-1] - vals[-2]) / dt)
        else:
            dt = times[i + 1] - times[i - 1]
            dvs.append((vals[i + 1] - vals[i - 1]) / dt)
    return dvs


def compute_rnea_torques(
    model: pin.Model,
    data:  pin.Data,
    qs:    List[np.ndarray],
    times: List[float],
) -> List[np.ndarray]:
    """
    Compute joint torques τ = M(q)q̈ + C(q,q̇)q̇ + g(q) via Pinocchio RNEA.
    q̇ and q̈ estimated by central finite differences on the IK solution.
    Returns list of τ arrays (6,).
    """
    dq  = _finite_diff(qs,  times)
    ddq = _finite_diff(dq, times)
    taus = []
    for q, dqi, ddqi in zip(qs, dq, ddq):
        taus.append(pin.rnea(model, data, q, dqi, ddqi))
    return taus


# ─────────────────────────────────────────────────────────────────────────────
# Objective f1: joint effort
# ─────────────────────────────────────────────────────────────────────────────

def f1_joint_effort(
    taus:  List[np.ndarray],
    times: List[float],
) -> float:
    """
    ∫₀ᵀ Σᵢ τᵢ(t)² dt  via trapezoidal integration.
    Units: N²·m²·s
    """
    tau_sq = np.array([np.dot(t, t) for t in taus])
    return float(np.trapz(tau_sq, times))


# ─────────────────────────────────────────────────────────────────────────────
# Objective f2: Cartesian arc length
# ─────────────────────────────────────────────────────────────────────────────

def f2_arc_length(
    keypoints:   List[CartesianWaypoint],
    pts_per_seg: int,
    n_quad:      int = 20,
) -> float:
    """
    ∫₀ᵀ ‖ṗ(t)‖ dt via Gauss-Legendre quadrature on spline coefficients.
    Computed analytically — no IK required.
    Units: m
    """
    return spline_arc_length(keypoints, pts_per_seg, n_quad)


# ─────────────────────────────────────────────────────────────────────────────
# Objective f3: obstacle clearance  (and hard constraint g1)
# ─────────────────────────────────────────────────────────────────────────────

def _aabb_distance(point: np.ndarray, center: np.ndarray, half: np.ndarray) -> float:
    """
    Minimum distance from 'point' to the surface of an AABB.
    Returns 0 if point is inside the box.
    """
    closest = np.clip(point, center - half, center + half)
    return float(np.linalg.norm(point - closest))


def f3_obstacle_clearance(
    tcp_positions: List[np.ndarray],
    obstacle:      Dict,
) -> Tuple[float, float]:
    """
    Compute:
      d_min  — minimum distance from any TCP position to the obstacle AABB surface [m]
      f3     — -d_min  (objective to minimize → maximizes clearance)
      g1     — r_grip - d_min  (≤ 0 for feasibility: gripper clears the box)

    obstacle dict keys:
      center       — np.ndarray (3,)  AABB centre in Pinocchio frame
      half_extents — np.ndarray (3,)  half-dimensions [m]
      r_grip       — float            gripper enveloping radius [m]
    """
    center = obstacle['center']
    half   = obstacle['half_extents']
    r_grip = obstacle['r_grip']

    d_min = min(_aabb_distance(p, center, half) for p in tcp_positions)
    f3 = -d_min
    g1 = r_grip - d_min    # ≤ 0 means clearance ≥ r_grip (no penetration)
    return f3, g1


# ─────────────────────────────────────────────────────────────────────────────
# Full evaluation: returns (f1, f2, f3, g1) or None on IK failure
# ─────────────────────────────────────────────────────────────────────────────

def evaluate_candidate(
    traj:        List[CartesianWaypoint],
    keypoints:   List[CartesianWaypoint],
    ik_joint:    List[np.ndarray],
    ik_ok:       List[bool],
    pin_model:   pin.Model,
    pin_data:    pin.Data,
    obstacle:    Dict,
    pts_per_seg: int,
    min_ik_rate: float = 0.95,
) -> Tuple[float, float, float, float] | None:
    """
    Evaluate all three objectives for one candidate via-point.

    Returns (f1, f2, f3, g1) or None if IK convergence rate < min_ik_rate.
    """
    if sum(ik_ok) / len(ik_ok) < min_ik_rate:
        return None

    times = [wp.timestamp for wp in traj]

    # f1
    taus = compute_rnea_torques(pin_model, pin_data, ik_joint, times)
    f1   = f1_joint_effort(taus, times)

    # f2 (analytic from spline segments kp1=[pre_A,A], kp2=[A,via,B], kp3=[B,post_B])
    f2 = (
        spline_arc_length([keypoints[0], keypoints[1]], pts_per_seg)
        + spline_arc_length([keypoints[1], keypoints[2], keypoints[3]], pts_per_seg)
        + spline_arc_length([keypoints[3], keypoints[4]], pts_per_seg)
    )

    # f3 + g1
    tcp_positions = [wp.position for wp in traj]
    f3, g1 = f3_obstacle_clearance(tcp_positions, obstacle)

    return f1, f2, f3, g1
