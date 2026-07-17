#ifndef UR5_DYN_CONTROL_UR5_DYNAMICS_HPP
#define UR5_DYN_CONTROL_UR5_DYNAMICS_HPP

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <memory>
#include <string>

#include <Eigen/Dense>

#include "ur5_dyn_control/common.hpp"

namespace ur5_dyn_control
{

/**
 * Dinamica de cuerpo rigido del UR5e via Pinocchio, para leyes de control
 * por torque: M(q), n(q,dq) = C·dq + g, g(q), J y J̇·q̇ del frame TCP.
 *
 * - Carga el URDF de brazo solo (sin masa de gripper) — coincide con la
 *   planta de Gazebo sin Robotiq.
 * - gravity_z configurable (mundo Gazebo usa 9.8; Pinocchio default 9.81).
 * - Registra el frame 'gripper_tcp' (tool0 + tcp_offset_z en Z local) si el
 *   URDF no lo trae, igual que IKWrapper/UR5Kinematics.
 */
class Ur5Dynamics
{
public:
  explicit Ur5Dynamics(const std::string & urdf_path,
                       double gravity_z = 9.8,
                       double tcp_offset_z = 0.141);

  /// Matriz de inercia M(q) (CRBA, simetrizada), 6x6.
  Matrix6d M(const Vector6d & q);

  /// Efectos no lineales n(q, dq) = C(q,dq)·dq + g(q).
  Vector6d nle(const Vector6d & q, const Vector6d & dq);

  /// Vector de gravedad g(q).
  Vector6d gravity(const Vector6d & q);

  /// Jacobiano 6x6 del frame TCP en LOCAL_WORLD_ALIGNED.
  Matrix6d frameJacobian(const Vector6d & q);

  /// J̇(q,q̇)·q̇ del frame TCP (aceleracion clasica con q̈ = 0), 6x1 [lin; ang].
  Vector6d frameJdotQdot(const Vector6d & q, const Vector6d & dq);

  /// Pose actual del frame TCP.
  pinocchio::SE3 fk(const Vector6d & q);

  const std::string & tcpFrameName() const { return tcp_frame_name_; }
  const pinocchio::Model & model() const { return model_; }

private:
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  pinocchio::FrameIndex tcp_frame_id_ = 0;
  std::string tcp_frame_name_ = "gripper_tcp";
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_UR5_DYNAMICS_HPP
