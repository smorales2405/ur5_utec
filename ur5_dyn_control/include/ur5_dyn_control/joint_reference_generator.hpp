#ifndef UR5_DYN_CONTROL_JOINT_REFERENCE_GENERATOR_HPP
#define UR5_DYN_CONTROL_JOINT_REFERENCE_GENERATOR_HPP

#include <memory>
#include <string>
#include <vector>

#include "ur5_dyn_control/cartesian_trajectory.hpp"
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
  double fk_tol = 5e-3;       // [m] tolerancia de aceptacion de la IK

  // Refinamiento de Newton posterior al QP. UR5Kinematics para en
  // ||error|| < 1e-4 (criterio fijo suyo), que a 10 mm/s equivale a 5 muestras
  // sin moverse: la referencia cartesiana avanzaba a escalones de 100 um.
  // Este refinamiento la deja continua a nivel de 1e-12 m.
  double refine_tol = 1e-11;
  int refine_iters = 20;
};

/**
 * Limites de validacion de la tabla de referencias (FASE 1).
 *
 * dq_max: limite REAL del UR5e, 180 deg/s = pi rad/s en las 6 juntas
 *   (ur_description/config/ur5e/joint_limits.yaml).
 *
 * ddq_max: el robot NO tiene limite de aceleracion articular. Cita literal de
 *   ur_moveit_config/config/joint_limits.yaml: "While the robot does not
 *   inherently have any limits on joint accelerations (only on torques), MoveIt
 *   needs them for time parametrization. They were chosen conservatively to
 *   work in most use cases." El valor por defecto de 5.0 rad/s^2 es ESE valor
 *   conservador de MoveIt, adoptado aqui como cota declarada — no es un dato
 *   de hardware y se reporta como tal.
 *
 * sigma_min / manipulabilidad: umbral de cercania a singularidad a lo largo del
 *   trazo. Con 0 se desactiva el chequeo.
 */
struct TrajectoryLimits
{
  Vector6d dq_max = (Vector6d() << M_PI, M_PI, M_PI, M_PI, M_PI, M_PI).finished();
  Vector6d ddq_max = (Vector6d() << 5.0, 5.0, 5.0, 5.0, 5.0, 5.0).finished();
  double sigma_min_threshold = 0.05;   ///< [~] menor valor singular de J
  double manipulability_threshold = 0.0;  ///< w = sqrt(det(J·Jᵀ)); 0 = desactivado
};

/// Diagnostico de la tabla construida (va a la cabecera de trazabilidad y a las
/// tablas del paper).
struct TrajectoryDiagnostics
{
  double sigma_min = 0.0;        ///< minimo de sigma_min(J) en todo el trazo
  double manipulability_min = 0.0;
  double sigma_min_t = 0.0;      ///< instante donde ocurre
  Vector6d dq_peak = Vector6d::Zero();
  Vector6d ddq_peak = Vector6d::Zero();
  double dq_margin = 1.0;        ///< min(1 - |dq|/dq_max) sobre toda la tabla
  double ddq_margin = 1.0;
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
 * la sim pausada) y ABORTA sin haber comandado nada si:
 *   - la IK salta de rama (|Δq| > q_jump_tol) o no converge;
 *   - sigma_min(J) o la manipulabilidad cruzan su umbral (FASE 1);
 *   - dq o ddq exceden los limites del UR5e (FASE 1).
 */
class JointReferenceGenerator
{
public:
  JointReferenceGenerator(std::shared_ptr<CartesianTrajectory> traj,
                          std::shared_ptr<Ur5Dynamics> dyn,
                          const std::string & urdf_path,
                          const IkParams & ik,
                          const TrajectoryLimits & limits = TrajectoryLimits());
  ~JointReferenceGenerator();

  /// Muestrea [t0, tN] con paso dt. Devuelve false (y llena error_msg) si algun
  /// chequeo falla; en ese caso la tabla queda vacia.
  bool build(double dt, std::string & error_msg);

  const JointRef & at(std::size_t k) const;
  std::size_t size() const { return table_.size(); }
  double dt() const { return dt_; }

  const CartesianTrajectory & cartesian() const { return *traj_; }
  const TrajectoryDiagnostics & diagnostics() const { return diag_; }

private:
  /// Newton amortiguado sobre el error de pose 6D, partiendo de la solucion del
  /// QP. Necesario porque la tolerancia del QP (1e-4 m) es 5x el avance por
  /// muestra en el tramo de corte.
  Vector6d refineIk(const Vector6d & q_seed,
                    const Eigen::Vector3d & p_des,
                    const Eigen::Matrix3d & R_des) const;

  std::shared_ptr<CartesianTrajectory> traj_;
  std::shared_ptr<Ur5Dynamics> dyn_;
  std::unique_ptr<UR5Kinematics> kin_;
  IkParams ik_;
  TrajectoryLimits limits_;
  TrajectoryDiagnostics diag_;
  std::vector<JointRef> table_;
  double dt_ = 0.0;
  double refine_tol_ = 1e-11;
  int refine_iters_ = 20;
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_JOINT_REFERENCE_GENERATOR_HPP
