#include "ur5_dyn_control/joint_reference_generator.hpp"

#include <pinocchio/spatial/se3.hpp>

#include <cmath>
#include <sstream>

#include <ur5_kinematics/kinematics.hpp>

namespace ur5_dyn_control
{

namespace
{

// Pseudo-inverso amortiguado (DLS): x = Jᵀ(JJᵀ + λ²I)⁻¹ b.
// Para el UR5e J es 6x6: esto es una inversion robusta cerca de singularidades.
Vector6d dlsSolve(const Matrix6d & J, const Vector6d & b, double lambda)
{
  const Matrix6d JJt = J * J.transpose() +
    lambda * lambda * Matrix6d::Identity();
  return J.transpose() * JJt.ldlt().solve(b);
}

}  // namespace

JointReferenceGenerator::JointReferenceGenerator(
  std::shared_ptr<CartesianSplineTrajectory> traj,
  std::shared_ptr<Ur5Dynamics> dyn,
  const std::string & urdf_path,
  const IkParams & ik)
: traj_(std::move(traj)), dyn_(std::move(dyn)), ik_(ik)
{
  kin_ = std::make_unique<UR5Kinematics>(urdf_path);
  // Mismo frame TCP que la dinamica: gripper_tcp a 0.141 m de tool0.
  kin_->registerFixedFrame(
    "gripper_tcp",
    pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, 0.141)));
  kin_->setTargetFrame("gripper_tcp");
}

JointReferenceGenerator::~JointReferenceGenerator() = default;

bool JointReferenceGenerator::build(double dt, std::string & error_msg)
{
  dt_ = dt;
  table_.clear();

  const double t0 = traj_->startTime();
  const double tN = traj_->endTime();
  const std::size_t n_samples =
    static_cast<std::size_t>(std::ceil((tN - t0) / dt)) + 1;
  table_.reserve(n_samples);

  Eigen::VectorXd q_prev(6);
  q_prev = ik_.seed;

  for (std::size_t k = 0; k < n_samples; ++k) {
    const double t = std::min(t0 + static_cast<double>(k) * dt, tN);

    const Eigen::Vector3d p = traj_->position(t);
    const Eigen::VectorXd q = kin_->inverseKinematicsQP2(
      q_prev, p, traj_->orientation(),
      ik_.max_iterations, ik_.alpha, ik_.weight_pos, ik_.weight_orient);

    if (k > 0) {
      const double jump = (q - q_prev).cwiseAbs().maxCoeff();
      if (jump > ik_.q_jump_tol) {
        std::ostringstream oss;
        oss << "IK salto de rama en t=" << t << " s (|dq|max=" << jump
            << " rad > " << ik_.q_jump_tol << ")";
        error_msg = oss.str();
        table_.clear();
        return false;
      }
    }

    // Verificacion de convergencia de la IK: error cartesiano del resultado.
    const Eigen::Vector3d p_fk = dyn_->fk(q).translation();
    if ((p_fk - p).norm() > 5e-3) {
      std::ostringstream oss;
      oss << "IK no convergio en t=" << t << " s (error pos="
          << (p_fk - p).norm() * 1e3 << " mm)";
      error_msg = oss.str();
      table_.clear();
      return false;
    }

    JointRef ref;
    ref.q = q;

    const Matrix6d J = dyn_->frameJacobian(ref.q);
    Vector6d xdot;
    xdot.head<3>() = traj_->velocity(t);
    xdot.tail<3>().setZero();          // orientacion constante
    ref.dq = dlsSolve(J, xdot, ik_.dls_lambda);

    Vector6d xddot;
    xddot.head<3>() = traj_->acceleration(t);
    xddot.tail<3>().setZero();
    ref.ddq = dlsSolve(J, xddot - dyn_->frameJdotQdot(ref.q, ref.dq),
                       ik_.dls_lambda);

    table_.push_back(ref);
    q_prev = ref.q;
  }

  error_msg.clear();
  return true;
}

const JointRef & JointReferenceGenerator::at(std::size_t k) const
{
  if (k >= table_.size()) {
    return table_.back();
  }
  return table_[k];
}

}  // namespace ur5_dyn_control
