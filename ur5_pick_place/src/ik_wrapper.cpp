#include "ur5_pick_place/ik_wrapper.hpp"

namespace ur5_pick_place {

IKWrapper::IKWrapper(const std::string& urdf_path)
: kin_(urdf_path)
{}

Eigen::VectorXd IKWrapper::solve(
    const Eigen::VectorXd& q_initial,
    const Eigen::Vector3d& tcp_pos,
    const Eigen::Matrix3d& tcp_orient,
    int max_iter,
    double alpha,
    double weight_pos,
    double weight_orient)
{
    // Transform the desired TCP pose into the equivalent tool0 pose.
    // The gripper_tcp frame is offset by kTcpOffsetZ along the tool0 Z-axis.
    // When the tool0 has orientation R, the offset in world coords is R * [0, 0, kTcpOffsetZ].
    Eigen::Vector3d tool0_pos = tcp_pos - tcp_orient * Eigen::Vector3d(0.0, 0.0, kTcpOffsetZ);

    // inverseKinematicsQP2 caps internally at 150 iters. Chain multiple rounds
    // so the caller can request more total iterations via max_iter.
    const int kRoundCap = 150;
    const int rounds = std::max(1, (max_iter + kRoundCap - 1) / kRoundCap);

    Eigen::VectorXd q = q_initial;
    for (int r = 0; r < rounds; ++r) {
        q = kin_.inverseKinematicsQP2(
            q, tool0_pos, tcp_orient,
            kRoundCap, alpha, weight_pos, weight_orient);
    }
    return q;
}

}  // namespace ur5_pick_place
