#ifndef UR5_DYN_CONTROL_COMMON_HPP
#define UR5_DYN_CONTROL_COMMON_HPP

#include <array>
#include <string>

#include <Eigen/Core>

namespace ur5_dyn_control
{

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// Orden canonico de joints en TODO el paquete: lectura de /joint_states
// (por nombre), tabla de referencias, y orden del Float64MultiArray de
// comandos (= orden de 'joints' en ur5e_effort_controllers.yaml).
inline const std::array<std::string, 6> kJointNames = {
  "shoulder_pan_joint",
  "shoulder_lift_joint",
  "elbow_joint",
  "wrist_1_joint",
  "wrist_2_joint",
  "wrist_3_joint",
};

// Limites de torque UR5e [N·m] (ur_description/config/ur5e/joint_limits.yaml).
inline const Vector6d kTauMax = (Vector6d() << 150.0, 150.0, 150.0, 28.0, 28.0, 28.0).finished();

// Referencia articular en un instante de la trayectoria.
struct JointRef
{
  Vector6d q   = Vector6d::Zero();
  Vector6d dq  = Vector6d::Zero();
  Vector6d ddq = Vector6d::Zero();
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_COMMON_HPP
