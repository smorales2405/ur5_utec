#include "ur5_dyn_control/ur5_dynamics.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <stdexcept>

namespace ur5_dyn_control
{

Ur5Dynamics::Ur5Dynamics(const std::string & urdf_path,
                         double gravity_z,
                         double tcp_offset_z)
{
  pinocchio::urdf::buildModel(urdf_path, model_);

  if (model_.nv != 6) {
    throw std::runtime_error(
      "Ur5Dynamics: se esperaba un modelo de 6 DOF (brazo solo), nv=" +
      std::to_string(model_.nv) + " en " + urdf_path);
  }

  model_.gravity.linear(Eigen::Vector3d(0.0, 0.0, -gravity_z));

  if (model_.existFrame(tcp_frame_name_)) {
    tcp_frame_id_ = model_.getFrameId(tcp_frame_name_);
  } else {
    // Igual que UR5Kinematics::registerFixedFrame: frame operacional fijo a
    // tcp_offset_z de tool0 en Z local.
    const pinocchio::FrameIndex tool0_id = model_.getFrameId("tool0");
    const pinocchio::Frame & tool0 = model_.frames[tool0_id];
    const pinocchio::SE3 offset(Eigen::Matrix3d::Identity(),
                                Eigen::Vector3d(0.0, 0.0, tcp_offset_z));
    tcp_frame_id_ = model_.addFrame(
      pinocchio::Frame(tcp_frame_name_, tool0.parentJoint, tool0_id,
                       tool0.placement * offset, pinocchio::OP_FRAME));
  }

  data_ = std::make_unique<pinocchio::Data>(model_);
}

Matrix6d Ur5Dynamics::M(const Vector6d & q)
{
  pinocchio::crba(model_, *data_, q);
  data_->M.triangularView<Eigen::StrictlyLower>() =
    data_->M.triangularView<Eigen::StrictlyUpper>().transpose();
  return data_->M;
}

Vector6d Ur5Dynamics::nle(const Vector6d & q, const Vector6d & dq)
{
  return pinocchio::nonLinearEffects(model_, *data_, q, dq);
}

Vector6d Ur5Dynamics::gravity(const Vector6d & q)
{
  return pinocchio::computeGeneralizedGravity(model_, *data_, q);
}

Matrix6d Ur5Dynamics::frameJacobian(const Vector6d & q)
{
  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model_.nv);
  J.setZero();
  pinocchio::computeFrameJacobian(
    model_, *data_, q, tcp_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J);
  return J;
}

Vector6d Ur5Dynamics::frameJdotQdot(const Vector6d & q, const Vector6d & dq)
{
  // Con q̈ = 0, la aceleracion clasica del frame es exactamente J̇·q̇.
  pinocchio::forwardKinematics(model_, *data_, q, dq, Vector6d::Zero().eval());
  const pinocchio::Motion a = pinocchio::getFrameClassicalAcceleration(
    model_, *data_, tcp_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED);
  Vector6d out;
  out.head<3>() = a.linear();
  out.tail<3>() = a.angular();
  return out;
}

pinocchio::SE3 Ur5Dynamics::fk(const Vector6d & q)
{
  pinocchio::forwardKinematics(model_, *data_, q);
  pinocchio::updateFramePlacements(model_, *data_);
  return data_->oMf[tcp_frame_id_];
}

}  // namespace ur5_dyn_control
