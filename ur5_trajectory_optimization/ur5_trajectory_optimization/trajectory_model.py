"""
Python port of the C++ clamped cubic spline trajectory generator.

Algorithm is identical to TrajectoryGenerator::clampedCubicSpline() in
ur5_pick_place/src/trajectory_generator.cpp so that the optimizer evaluates
the exact same trajectory shape as the ROS 2 node.

Segment layout (mirrors pick_place_node.cpp):
  kp1 = [A, B]      → 2-point spline (velocity = 0 at both ends)
  kp2 = [B, O, C]   → 3-point spline (velocity = 0 at both ends)
  kp3 = [C, D]      → 2-point spline (velocity = 0 at both ends)

Combined by discarding the duplicate boundary point at each junction.
Total samples = 4 * pts_per_seg + 1.
"""

from __future__ import annotations
import numpy as np
from dataclasses import dataclass, field
from typing import List


@dataclass
class CartesianWaypoint:
    position:    np.ndarray        # shape (3,)
    orientation: np.ndarray        # shape (3,3) rotation matrix
    timestamp:   float
    jerk:        np.ndarray = field(default_factory=lambda: np.zeros(3))


def _compute_clamped_coeffs(y: List[float]):
    """
    Clamped cubic spline coefficients for one scalar axis.
    Boundary condition: first derivative = 0 at both endpoints.
    Returns list of (a, b, c, d) per segment, s ∈ [0, 1].
    Exact port of TrajectoryGenerator::computeClampedCoeffs().
    """
    n = len(y) - 1
    if n < 1:
        raise ValueError("Need at least 2 keypoints")

    diag  = [4.0] * (n + 1)
    upper = [1.0] * n
    lower = [1.0] * n
    rhs   = [0.0] * (n + 1)

    # Clamped BC at s=0: 2*M[0] + M[1] = 3*(y[1]-y[0])
    diag[0] = 2.0
    rhs[0]  = 3.0 * (y[1] - y[0])

    for i in range(1, n):
        rhs[i] = 3.0 * (y[i + 1] - 2.0 * y[i] + y[i - 1])

    # Clamped BC at s=1: M[n-1] + 2*M[n] = 3*(y[n-1]-y[n])
    diag[n] = 2.0
    rhs[n]  = 3.0 * (y[n - 1] - y[n])

    # Thomas algorithm (forward sweep)
    c_prime = [0.0] * (n + 1)
    d_prime = [0.0] * (n + 1)
    c_prime[0] = upper[0] / diag[0]
    d_prime[0] = rhs[0]   / diag[0]
    for i in range(1, n + 1):
        sub = lower[i - 1] if i < n else lower[n - 1]
        denom      = diag[i] - sub * c_prime[i - 1]
        c_prime[i] = (upper[i] / denom) if i < n else 0.0
        d_prime[i] = (rhs[i] - lower[i - 1] * d_prime[i - 1]) / denom

    # Back substitution
    M = [0.0] * (n + 1)
    M[n] = d_prime[n]
    for i in range(n - 1, -1, -1):
        M[i] = d_prime[i] - c_prime[i] * M[i + 1]

    coeffs = []
    for i in range(n):
        a = y[i]
        b = (y[i + 1] - y[i]) - (2.0 * M[i] + M[i + 1]) / 3.0
        c = M[i]
        d = (M[i + 1] - M[i]) / 3.0
        coeffs.append((a, b, c, d))
    return coeffs


def _slerp(R0: np.ndarray, R1: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between two rotation matrices."""
    from scipy.spatial.transform import Rotation, Slerp
    rots = Rotation.from_matrix(np.stack([R0, R1]))
    slerp = Slerp([0.0, 1.0], rots)
    return slerp(t).as_matrix()


def clamped_cubic_spline(
    keypoints: List[CartesianWaypoint],
    pts_per_seg: int,
) -> List[CartesianWaypoint]:
    """
    Dense trajectory by clamped cubic spline (exact C++ port).
    keypoints: list of CartesianWaypoint with timestamps in seconds.
    pts_per_seg: samples per spline segment (last segment gets pts_per_seg+1).
    Returns list of CartesianWaypoint at uniform sub-segment steps.
    """
    nk = len(keypoints)
    if nk < 2:
        raise ValueError("Need at least 2 keypoints")

    px = [kp.position[0] for kp in keypoints]
    py = [kp.position[1] for kp in keypoints]
    pz = [kp.position[2] for kp in keypoints]

    cx = _compute_clamped_coeffs(px)
    cy = _compute_clamped_coeffs(py)
    cz = _compute_clamped_coeffs(pz)

    n_segs = nk - 1
    out: List[CartesianWaypoint] = []

    for seg in range(n_segs):
        dt_seg = keypoints[seg + 1].timestamp - keypoints[seg].timestamp
        steps  = pts_per_seg + 1 if seg == n_segs - 1 else pts_per_seg

        inv_dt3  = 1.0 / (dt_seg ** 3)
        seg_jerk = np.array([
            6.0 * cx[seg][3] * inv_dt3,
            6.0 * cy[seg][3] * inv_dt3,
            6.0 * cz[seg][3] * inv_dt3,
        ])

        a_x, b_x, c_x, d_x = cx[seg]
        a_y, b_y, c_y, d_y = cy[seg]
        a_z, b_z, c_z, d_z = cz[seg]

        for j in range(steps):
            s  = j / pts_per_seg
            s2 = s * s
            s3 = s2 * s

            pos = np.array([
                a_x + b_x * s + c_x * s2 + d_x * s3,
                a_y + b_y * s + c_y * s2 + d_y * s3,
                a_z + b_z * s + c_z * s2 + d_z * s3,
            ])
            R   = _slerp(keypoints[seg].orientation, keypoints[seg + 1].orientation, s)
            t   = keypoints[seg].timestamp + s * dt_seg

            out.append(CartesianWaypoint(
                position=pos, orientation=R,
                timestamp=t, jerk=seg_jerk.copy(),
            ))
    return out


def build_trajectory(
    point_O: np.ndarray,
    fixed_pts: dict,
    pts_per_seg: int,
) -> List[CartesianWaypoint]:
    """
    Build the full pick-place trajectory replicating pick_place_node.cpp logic.

    fixed_pts keys: A, B, C, D, R_tcp, t_A, t_B, t_O, t_C, t_D
    point_O is the decision variable [x, y, z] in Pinocchio frame.

    Segment structure:
      kp1 = [A, B]   → clampedCubicSpline
      kp2 = [B, O, C] → clampedCubicSpline
      kp3 = [C, D]   → clampedCubicSpline
    Combined with boundary deduplication.
    """
    R = fixed_pts['R_tcp']

    t_A  = fixed_pts['t_A']
    t_B      = fixed_pts['t_B']
    t_O    = fixed_pts['t_O']
    t_C      = fixed_pts['t_C']
    t_D = fixed_pts['t_D']

    kp_A  = CartesianWaypoint(fixed_pts['A'], R, t_A)
    kp_B      = CartesianWaypoint(fixed_pts['B'],     R, t_B)
    kp_O    = CartesianWaypoint(point_O,           R, t_O)
    kp_C      = CartesianWaypoint(fixed_pts['C'],     R, t_C)
    kp_D = CartesianWaypoint(fixed_pts['D'],R, t_D)

    t1 = clamped_cubic_spline([kp_A, kp_B],              pts_per_seg)
    t2 = clamped_cubic_spline([kp_B, kp_O, kp_C],          pts_per_seg)
    t3 = clamped_cubic_spline([kp_C, kp_D],             pts_per_seg)

    traj = t1 + t2[1:] + t3[1:]
    return traj


def spline_arc_length(
    keypoints: List[CartesianWaypoint],
    pts_per_seg: int,
    n_quad: int = 20,
) -> float:
    """
    Analytic arc length ∫‖p'(t)‖ dt via Gauss-Legendre quadrature on each
    spline segment. Does NOT require IK — computed purely from coefficients.
    """
    nk = len(keypoints)
    px = [kp.position[0] for kp in keypoints]
    py = [kp.position[1] for kp in keypoints]
    pz = [kp.position[2] for kp in keypoints]

    cx = _compute_clamped_coeffs(px)
    cy = _compute_clamped_coeffs(py)
    cz = _compute_clamped_coeffs(pz)

    xi, wi = np.polynomial.legendre.leggauss(n_quad)
    total = 0.0

    for seg in range(nk - 1):
        dt_seg = keypoints[seg + 1].timestamp - keypoints[seg].timestamp
        # s = (xi + 1) / 2  maps GL nodes from [-1,1] to [0,1]
        s  = (xi + 1.0) * 0.5
        s2 = s * s

        _, b_x, c_x, d_x = cx[seg]
        _, b_y, c_y, d_y = cy[seg]
        _, b_z, c_z, d_z = cz[seg]

        # dp/ds = b + 2c*s + 3d*s²  →  dp/dt = (dp/ds) / dt_seg
        dp_x = (b_x + 2 * c_x * s + 3 * d_x * s2) / dt_seg
        dp_y = (b_y + 2 * c_y * s + 3 * d_y * s2) / dt_seg
        dp_z = (b_z + 2 * c_z * s + 3 * d_z * s2) / dt_seg

        speed = np.sqrt(dp_x ** 2 + dp_y ** 2 + dp_z ** 2)
        # ∫₀¹ speed * dt_seg * ds  (Jacobian of s→t: dt_seg, GL weight factor: 0.5)
        total += dt_seg * 0.5 * np.dot(wi, speed)

    return total
