#ifndef UR5_DYN_CONTROL_JOINT_REFERENCE_GENERATOR_HPP
#define UR5_DYN_CONTROL_JOINT_REFERENCE_GENERATOR_HPP

#include <memory>
#include <string>
#include <vector>

#include "ur5_dyn_control/cartesian_spline_trajectory.hpp"
#include "ur5_dyn_control/common.hpp"
#include "ur5_dyn_control/ur5_dynamics.hpp"

class UR5Kinematics;  // fwd (ur5_kinematics/kinematics.hpp)

namespace ur5_dyn_control
{

struct IkParams
{
  Vector6d seed = Vector6d::Zero();
  int max_iterations = 450;
  double alpha = 0.5;
  double weight_pos = 1.0;
  double weight_orient = 1.0;
  double q_jump_tol = 0.15;   // [rad] guard de continuidad entre muestras
  double dls_lambda = 0.01;   // amortiguacion del pseudo-inverso
};

/**
 * Convierte la trayectoria cartesiana (spline con derivadas analiticas y
 * orientacion constante) en una tabla de referencias articulares
 * {q, dq, ddq} muestreada al paso del lazo de control:
 *
 *   q_k   = IK_QP(p(t_k), R_const)          (seed encadenado, frame gripper_tcp)
 *   dq_k  = DLS(J(q_k)) · [ṗ(t_k); 0]
 *   ddq_k = DLS(J(q_k)) · ([p̈(t_k); 0] − J̇(q_k, dq_k)·dq_k)
 *
 * build() corre UNA vez antes de aplicar torque (con el robot sostenido /
 * la sim pausada); si la IK salta de rama (|Δq| > q_jump_tol) aborta y
 * devuelve false SIN que se haya comandado nada.
 */
class JointReferenceGenerator
{
public:
  JointReferenceGenerator(std::shared_ptr<CartesianSplineTrajectory> traj,
                          std::shared_ptr<Ur5Dynamics> dyn,
                          const std::string & urdf_path,
                          const IkParams & ik);
  ~JointReferenceGenerator();

  /// Muestrea [t0, tN] con paso dt. Devuelve false si la IK falla/salta.
  bool build(double dt, std::string & error_msg);

  const JointRef & at(std::size_t k) const;
  std::size_t size() const { return table_.size(); }
  double dt() const { return dt_; }

  const CartesianSplineTrajectory & cartesian() const { return *traj_; }

private:
  std::shared_ptr<CartesianSplineTrajectory> traj_;
  std::shared_ptr<Ur5Dynamics> dyn_;
  std::unique_ptr<UR5Kinematics> kin_;
  IkParams ik_;
  std::vector<JointRef> table_;
  double dt_ = 0.0;
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_JOINT_REFERENCE_GENERATOR_HPP
