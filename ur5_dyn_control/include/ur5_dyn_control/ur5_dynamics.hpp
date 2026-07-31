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
 * Inercia de la herramienta montada en el TCP (supuesto A1 de
 * docs/00_assumptions.md).
 *
 * Con mass = 0 (default) no se modifica el modelo: es exactamente el brazo
 * solo, que es el supuesto vigente. Este es el HOOK para levantar A1 cuando se
 * midan las propiedades del acople del bisturi; los valores NO se inventan.
 */
struct ToolInertia
{
  double mass = 0.0;                                    ///< [kg]
  Eigen::Vector3d com = Eigen::Vector3d::Zero();        ///< [m], en el frame TCP
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();    ///< [kg·m²], respecto al CoM

  bool isNegligible() const { return mass <= 0.0; }
};

/**
 * Dinamica de cuerpo rigido del UR5e via Pinocchio, para leyes de control
 * por torque: M(q), n(q,dq) = C·dq + g, g(q), J y J̇·q̇ del frame TCP.
 *
 * - Carga el URDF de brazo solo (sin masa de gripper) — coincide con la
 *   planta de Gazebo sin Robotiq.
 * - gravity_z configurable (mundo Gazebo usa 9.8; Pinocchio default 9.81).
 * - Registra el frame 'gripper_tcp' (tool0 + tcp_offset_z en Z local) si el
 *   URDF no lo trae, igual que IKWrapper/UR5Kinematics.
 * - tcp_offset_z se expone (tcpOffsetZ()) para que la cinematica de
 *   JointReferenceGenerator use exactamente el mismo frame: no puede haber
 *   divergencia entre el TCP de la IK y el de la dinamica (supuesto A2).
 */
class Ur5Dynamics
{
public:
  explicit Ur5Dynamics(const std::string & urdf_path,
                       double gravity_z = 9.8,
                       double tcp_offset_z = 0.141,
                       const ToolInertia & tool = ToolInertia());

  /// Matriz de inercia M(q) (CRBA, simetrizada), 6x6.
  Matrix6d M(const Vector6d & q);

  /// Efectos no lineales n(q, dq) = C(q,dq)·dq + g(q).
  Vector6d nle(const Vector6d & q, const Vector6d & dq);

  /// Vector de gravedad g(q).
  Vector6d gravity(const Vector6d & q);

  /// Matriz de Coriolis C(q,q̇) tal que n(q,q̇) = C(q,q̇)·q̇ + g(q).
  Matrix6d coriolis(const Vector6d & q, const Vector6d & dq);

  /// Derivada temporal de la matriz de inercia, Ṁ(q,q̇) = C + Cᵀ.
  ///
  /// Se apoya en la propiedad de antisimetria de Ṁ − 2C: con la C construida a
  /// partir de los simbolos de Christoffel (la que devuelve Pinocchio), esa
  /// propiedad equivale exactamente a Ṁ = C + Cᵀ. Hace falta para la cota de
  /// ganancia del SMC, que incluye un termino (1−α)·Ṁ·q̇_r.
  Matrix6d dM(const Vector6d & q, const Vector6d & dq);

  /// Jacobiano 6x6 del frame TCP en LOCAL_WORLD_ALIGNED.
  Matrix6d frameJacobian(const Vector6d & q);

  /// J̇(q,q̇)·q̇ del frame TCP (aceleracion clasica con q̈ = 0), 6x1 [lin; ang].
  Vector6d frameJdotQdot(const Vector6d & q, const Vector6d & dq);

  /// Pose actual del frame TCP.
  pinocchio::SE3 fk(const Vector6d & q);

  const std::string & tcpFrameName() const { return tcp_frame_name_; }
  double tcpOffsetZ() const { return tcp_offset_z_; }
  const pinocchio::Model & model() const { return model_; }

private:
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  pinocchio::FrameIndex tcp_frame_id_ = 0;
  std::string tcp_frame_name_ = "gripper_tcp";
  double tcp_offset_z_ = 0.141;
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_UR5_DYNAMICS_HPP
