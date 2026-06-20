#pragma once

#include "ur5_kinematics/kinematics.hpp"
#include <Eigen/Dense>
#include <string>

namespace ur5_pick_place {

// Thin wrapper around UR5Kinematics that corrects for the gripper_tcp offset.
//
// UR5Kinematics solves IK for the 'tool0' frame using Pinocchio + OsqpEigen.
// This class adjusts the desired pose so that the resolved tool0 position
// places 'gripper_tcp' at the requested Cartesian target.
//
// Fixed offset (tool0 → gripper_tcp, along tool0 Z-axis):
//   0.000 m  ur_to_robotiq_joint  (no Z offset in joint origin)
//   0.011 m  gripper_side_joint   (ur_to_robotiq_adapter.urdf.xacro:36)
//   0.000 m  robotiq_85_base_joint (origin 0 0 0 in ur5_robotiq_2f85.urdf.xacro)
//   0.130 m  gripper_tcp_joint    (added in ur5_robotiq_2f85.urdf.xacro)
//   ──────
//   0.141 m  kTcpOffsetZ
class IKWrapper {
public:
    // urdf_path: absolute path to the UR5-only URDF (no gripper) used by Pinocchio.
    // The gripper is NOT part of the IK model; the TCP offset is applied analytically.
    explicit IKWrapper(const std::string& urdf_path);

    // Solve IK so that gripper_tcp reaches (tcp_pos, tcp_orient) in the world frame.
    // q_initial: seed joint configuration [6] (rad).
    // Returns joint angles [6] (rad) or the best-effort result if not converged.
    Eigen::VectorXd solve(
        const Eigen::VectorXd& q_initial,
        const Eigen::Vector3d& tcp_pos,
        const Eigen::Matrix3d& tcp_orient,
        int max_iter = 100,
        double alpha = 0.5,
        double weight_pos = 1.0,
        double weight_orient = 1.0);

private:
    UR5Kinematics kin_;
    // Total Z offset from tool0 origin to gripper_tcp origin (in tool0 local frame).
    static constexpr double kTcpOffsetZ = 0.141;
};

}  // namespace ur5_pick_place
