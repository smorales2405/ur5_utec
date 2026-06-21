#pragma once

#include "ur5_kinematics/kinematics.hpp"
#include <Eigen/Dense>
#include <string>

namespace ur5_pick_place {

// Wrapper around UR5Kinematics that targets the 'gripper_tcp' frame directly.
//
// At construction the gripper_tcp operational frame is registered in the
// Pinocchio model at kGripperTcpOffsetZ along tool0's local Z-axis.
// The offset matches the chain defined in ur5_robotiq_2f85.urdf.xacro:
//   ur_to_robotiq_joint   0.000 m
//   gripper_side_joint    0.011 m  (ur_to_robotiq_adapter.urdf.xacro:36)
//   robotiq_85_base_joint 0.000 m
//   gripper_tcp_joint     0.130 m
//   ──────────────────────────────
//   total                 0.141 m  → kGripperTcpOffsetZ
//
// IK is solved so that gripper_tcp reaches the requested (pos, orient).
class IKWrapper {
public:
    // urdf_path: absolute path to the UR5-only URDF used by Pinocchio.
    // gripper_tcp is added as a fixed operational frame at build time.
    explicit IKWrapper(const std::string& urdf_path);

    // Solve IK so that gripper_tcp reaches (tcp_pos, tcp_orient) in base_link frame.
    // q_initial: seed joint configuration [6] (rad).
    // Returns joint angles [6] (rad) or best-effort result if not converged.
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
    // Combined Z offset from tool0 to gripper_tcp (tool0 local frame, metres).
    static constexpr double kGripperTcpOffsetZ = 0.141;
};

}  // namespace ur5_pick_place
