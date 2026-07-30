#include "ur5_dyn_control/joint_reference_generator.hpp"

#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
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
  std::shared_ptr<CartesianTrajectory> traj,
  std::shared_ptr<Ur5Dynamics> dyn,
  const std::string & urdf_path,
  const IkParams & ik,
  const TrajectoryLimits & limits)
: traj_(std::move(traj)), dyn_(std::move(dyn)), ik_(ik), limits_(limits),
  refine_tol_(ik.refine_tol), refine_iters_(ik.refine_iters)
{
  kin_ = std::make_unique<UR5Kinematics>(urdf_path);
  // Mismo frame TCP que la dinamica, tomado de ella (supuesto A2): el offset
  // no se repite aqui para que la IK y la dinamica no puedan divergir.
  kin_->registerFixedFrame(
    dyn_->tcpFrameName(),
    pinocchio::SE3(Eigen::Matrix3d::Identity(),
                   Eigen::Vector3d(0.0, 0.0, dyn_->tcpOffsetZ())));
  kin_->setTargetFrame(dyn_->tcpFrameName());
}

JointReferenceGenerator::~JointReferenceGenerator() = default;

Vector6d JointReferenceGenerator::refineIk(const Vector6d & q_seed,
                                           const Eigen::Vector3d & p_des,
                                           const Eigen::Matrix3d & R_des) const
{
  // MOTIVO: UR5Kinematics::inverseKinematicsQP2 para cuando ||error|| < 1e-4
  // (criterio fijo dentro de ur5_kinematics, que no se toca). A 10 mm/s y
  // 500 Hz el avance por muestra es de 20 um, muy por debajo de esa tolerancia:
  // el solver devolvia LA MISMA q durante 4-5 muestras seguidas y la referencia
  // cartesiana avanzaba a saltos de 100 um. Medido en Gazebo, eso metia un
  // rizado de +-25 % en la velocidad de avance ejecutada — inaceptable cuando
  // el criterio del corte es +-2 % y la repetibilidad del UR5e son 30 um.
  //
  // Este refinamiento de Newton amortiguado (con el Jacobiano que ya se calcula
  // para dq/ddq) baja el error a ~1e-12 m sin tocar ur5_kinematics.
  Vector6d q = q_seed;
  for (int it = 0; it < refine_iters_; ++it) {
    const pinocchio::SE3 T = dyn_->fk(q);
    Vector6d err;
    err.head<3>() = p_des - T.translation();
    // frameJacobian es LOCAL_WORLD_ALIGNED: el twist esta en ejes del mundo,
    // asi que el error de orientacion es log3(R_des * R_actual^T).
    err.tail<3>() = pinocchio::log3(R_des * T.rotation().transpose());
    if (err.norm() < refine_tol_) {break;}
    q += dlsSolve(dyn_->frameJacobian(q), err, 1e-8);
  }
  return q;
}

bool JointReferenceGenerator::build(double dt, std::string & error_msg)
{
  dt_ = dt;
  table_.clear();
  diag_ = TrajectoryDiagnostics();
  diag_.sigma_min = std::numeric_limits<double>::infinity();
  diag_.manipulability_min = std::numeric_limits<double>::infinity();

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
    if ((p_fk - p).norm() > ik_.fk_tol) {
      std::ostringstream oss;
      oss << "IK no convergio en t=" << t << " s (error pos="
          << (p_fk - p).norm() * 1e3 << " mm)";
      error_msg = oss.str();
      table_.clear();
      return false;
    }

    JointRef ref;
    ref.q = refineIk(q, p, traj_->orientation());

    const Matrix6d J = dyn_->frameJacobian(ref.q);

    // ── Chequeo de manipulabilidad (FASE 1) ─────────────────────────────────
    // sigma_min(J): distancia a la singularidad en el sentido del peor eje
    // cartesiano. w = sqrt(det(J·Jᵀ)) = producto de los valores singulares:
    // volumen del elipsoide de manipulabilidad.
    const Eigen::JacobiSVD<Matrix6d> svd(J);
    const double sigma_min = svd.singularValues()(5);
    const double w = svd.singularValues().prod();
    if (sigma_min < diag_.sigma_min) {
      diag_.sigma_min = sigma_min;
      diag_.sigma_min_t = t;
    }
    diag_.manipulability_min = std::min(diag_.manipulability_min, w);

    if (limits_.sigma_min_threshold > 0.0 && sigma_min < limits_.sigma_min_threshold) {
      std::ostringstream oss;
      oss << "cerca de singularidad en t=" << t << " s: sigma_min(J)=" << sigma_min
          << " < " << limits_.sigma_min_threshold
          << " (TCP = [" << p.transpose() << "])";
      error_msg = oss.str();
      table_.clear();
      return false;
    }
    if (limits_.manipulability_threshold > 0.0 && w < limits_.manipulability_threshold) {
      std::ostringstream oss;
      oss << "manipulabilidad baja en t=" << t << " s: w=" << w
          << " < " << limits_.manipulability_threshold;
      error_msg = oss.str();
      table_.clear();
      return false;
    }

    Vector6d xdot;
    xdot.head<3>() = traj_->velocity(t);
    xdot.tail<3>().setZero();          // orientacion constante (A4)
    ref.dq = dlsSolve(J, xdot, ik_.dls_lambda);

    Vector6d xddot;
    xddot.head<3>() = traj_->acceleration(t);
    xddot.tail<3>().setZero();
    ref.ddq = dlsSolve(J, xddot - dyn_->frameJdotQdot(ref.q, ref.dq),
                       ik_.dls_lambda);

    // ── Limites articulares del UR5e (FASE 1) ───────────────────────────────
    diag_.dq_peak = diag_.dq_peak.cwiseMax(ref.dq.cwiseAbs());
    diag_.ddq_peak = diag_.ddq_peak.cwiseMax(ref.ddq.cwiseAbs());
    for (int j = 0; j < 6; ++j) {
      const double rv = std::abs(ref.dq[j]) / limits_.dq_max[j];
      const double ra = std::abs(ref.ddq[j]) / limits_.ddq_max[j];
      diag_.dq_margin = std::min(diag_.dq_margin, 1.0 - rv);
      diag_.ddq_margin = std::min(diag_.ddq_margin, 1.0 - ra);
      if (rv > 1.0) {
        std::ostringstream oss;
        oss << "dq fuera de limite en t=" << t << " s, junta " << (j + 1) << ": |"
            << ref.dq[j] << "| > " << limits_.dq_max[j] << " rad/s";
        error_msg = oss.str();
        table_.clear();
        return false;
      }
      if (ra > 1.0) {
        std::ostringstream oss;
        oss << "ddq fuera de limite en t=" << t << " s, junta " << (j + 1) << ": |"
            << ref.ddq[j] << "| > " << limits_.ddq_max[j] << " rad/s^2";
        error_msg = oss.str();
        table_.clear();
        return false;
      }
    }

    table_.push_back(ref);
    q_prev = ref.q;
  }

  error_msg.clear();
  return true;
}

}  // namespace ur5_dyn_control
