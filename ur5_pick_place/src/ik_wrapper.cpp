#include "ur5_pick_place/ik_wrapper.hpp"

namespace ur5_pick_place {

IKWrapper::IKWrapper(const std::string& urdf_path)
: kin_(urdf_path)
{
    // Register gripper_tcp as a fixed operational frame in the Pinocchio model.
    // Offset: 0.141 m along tool0's local Z-axis (combined chain from xacro).
    pinocchio::SE3 offset = pinocchio::SE3::Identity();
    offset.translation() << 0.0, 0.0, kGripperTcpOffsetZ;
    kin_.registerFixedFrame("gripper_tcp", offset);
    kin_.setTargetFrame("gripper_tcp");
}

Eigen::VectorXd IKWrapper::solve(
    const Eigen::VectorXd& q_initial,
    const Eigen::Vector3d& tcp_pos,
    const Eigen::Matrix3d& tcp_orient,
    int max_iter,
    double alpha,
    double weight_pos,
    double weight_orient)
{
    // Pinocchio targets gripper_tcp directly — tcp_pos is used as-is.
    const int kRoundCap = 150;
    const int rounds = std::max(1, (max_iter + kRoundCap - 1) / kRoundCap);

    Eigen::VectorXd q = q_initial;
    for (int r = 0; r < rounds; ++r) {
        q = kin_.inverseKinematicsQP2(
            q, tcp_pos, tcp_orient,
            kRoundCap, alpha, weight_pos, weight_orient);
    }
    return q;
}

}  // namespace ur5_pick_place
