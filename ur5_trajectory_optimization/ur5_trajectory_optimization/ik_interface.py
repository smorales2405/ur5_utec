"""
Damped-least-squares IK for the UR5 + gripper_tcp frame.

Mirrors the C++ IKWrapper: registers 'gripper_tcp' as a fixed operational
frame in the Pinocchio model at TCP_OFFSET_Z along tool0's local Z-axis.
The IK is solved directly for gripper_tcp — no analytical offset required.

Intended for offline batch evaluation during trajectory optimization; not
connected to ROS 2 or Gazebo.
"""

from __future__ import annotations
import numpy as np
import pinocchio as pin
from typing import Tuple


def rpy_to_matrix(r: float, p: float, y: float) -> np.ndarray:
    """ZYX convention: R = Rz(y) @ Ry(p) @ Rx(r)  (same as C++ rpy_to_matrix)."""
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0,  0 ], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0,  1,  0 ], [-sp, 0, cp]])
    Rz = np.array([[cy,-sy, 0], [sy, cy,  0], [0,   0,  1]])
    return Rz @ Ry @ Rx


def _log_rotation(R: np.ndarray) -> np.ndarray:
    """SO(3) logarithm map: R → ω (axis-angle, 3-vector)."""
    return pin.log3(R)


class IKInterface:
    """
    Damped least-squares IK solver wrapping Pinocchio.

    At construction, 'gripper_tcp' is registered in the Pinocchio model as
    a fixed operational frame at TCP_OFFSET_Z along tool0's local Z-axis.
    This mirrors UR5Kinematics::registerFixedFrame() / setTargetFrame() in C++.

    Combined offset from ur5_robotiq_2f85.urdf.xacro:
      ur_to_robotiq_joint   0.000 m
      gripper_side_joint    0.011 m  (ur_to_robotiq_adapter.urdf.xacro:36)
      robotiq_85_base_joint 0.000 m
      gripper_tcp_joint     0.130 m
      ──────────────────────────────
      TCP_OFFSET_Z          0.141 m

    Uses the Levenberg-Marquardt update:
        Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ err
    with a warm-start chain along the trajectory for fast convergence.
    """

    TCP_OFFSET_Z = 0.141  # metres along tool0 z-axis (adapter 0.011 + gripper 0.130)

    def __init__(self, urdf_path: str) -> None:
        self._model = pin.buildModelFromUrdf(urdf_path)

        # Verify tool0 exists
        tool0_id = self._model.getFrameId('tool0')
        if tool0_id >= self._model.nframes:
            raise RuntimeError("Frame 'tool0' not found in URDF")

        # Register gripper_tcp as a fixed operational frame at TCP_OFFSET_Z from tool0
        tool0_frame = self._model.frames[tool0_id]
        offset      = pin.SE3(np.eye(3), np.array([0.0, 0.0, self.TCP_OFFSET_Z]))

        tcp_frame = pin.Frame(
            'gripper_tcp',
            tool0_frame.parentJoint,                # same parent joint as tool0
            tool0_id,                               # parent frame
            tool0_frame.placement * offset,         # SE3 in parent joint frame
            pin.FrameType.OP_FRAME,
        )
        self._fid  = self._model.addFrame(tcp_frame)
        self._data = self._model.createData()

    # ------------------------------------------------------------------
    def tcp_pose(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Returns (tcp_pos, tcp_R) of gripper_tcp in world frame for joint config q.
        Pinocchio evaluates the full chain — no analytical offset needed.
        """
        pin.forwardKinematics(self._model, self._data, q)
        pin.updateFramePlacements(self._model, self._data)
        oMf = self._data.oMf[self._fid]
        return oMf.translation.copy(), oMf.rotation.copy()

    # ------------------------------------------------------------------
    def _frame_jacobian(self, q: np.ndarray) -> np.ndarray:
        """6×nv Jacobian at gripper_tcp, world-aligned."""
        pin.computeJointJacobians(self._model, self._data, q)
        pin.updateFramePlacements(self._model, self._data)
        return pin.getFrameJacobian(
            self._model, self._data, self._fid,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

    # ------------------------------------------------------------------
    def solve(
        self,
        q_init: np.ndarray,
        target_pos: np.ndarray,
        target_R:   np.ndarray,
        max_iter: int   = 80,
        tol:      float = 1e-4,
        lam:      float = 0.05,
        alpha:    float = 0.8,
        w_pos:    float = 1.0,
        w_orient: float = 1.0,
    ) -> Tuple[np.ndarray, bool]:
        """
        Solve IK for gripper_tcp reaching (target_pos, target_R).

        Returns:
            q        — joint angles (6,), best-effort even if not converged
            success  — True if ‖error‖ < tol before max_iter
        """
        q  = q_init.copy()
        I6 = np.eye(6)

        for _ in range(max_iter):
            tcp_pos, tcp_R = self.tcp_pose(q)

            err_pos    = w_pos    * (target_pos - tcp_pos)
            err_orient = w_orient * _log_rotation(target_R @ tcp_R.T)
            err        = np.hstack([err_pos, err_orient])

            if np.linalg.norm(err) < tol:
                return q, True

            # Full 6×nv Jacobian at gripper_tcp — Pinocchio accounts for the
            # fixed offset from tool0, so no manual correction needed.
            J      = self._frame_jacobian(q)
            J_full = np.vstack([w_pos * J[:3, :], w_orient * J[3:, :]])

            JJt = J_full @ J_full.T
            dq  = J_full.T @ np.linalg.solve(JJt + lam * lam * I6, err)
            q   = q + alpha * dq

        return q, False

    # ------------------------------------------------------------------
    def solve_trajectory(
        self,
        waypoints,          # list of CartesianWaypoint
        q_home: np.ndarray,
        **ik_kwargs,
    ):
        """
        Solve IK for a sequence of waypoints with warm-start chaining.
        Returns (list of q arrays, list of bool success flags).
        """
        q_seed = q_home.copy()
        qs, ok = [], []
        for wp in waypoints:
            q, success = self.solve(
                q_seed, wp.position, wp.orientation, **ik_kwargs)
            qs.append(q)
            ok.append(success)
            q_seed = q
        return qs, ok
