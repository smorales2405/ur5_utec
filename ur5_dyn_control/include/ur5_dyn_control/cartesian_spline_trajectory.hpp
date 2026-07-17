#ifndef UR5_DYN_CONTROL_CARTESIAN_SPLINE_TRAJECTORY_HPP
#define UR5_DYN_CONTROL_CARTESIAN_SPLINE_TRAJECTORY_HPP

#include <vector>

#include <Eigen/Dense>

namespace ur5_dyn_control
{

/**
 * Spline cubico CLAMPED por waypoints cartesianos con derivadas ANALITICAS.
 *
 * - Cada eje (x, y, z) se interpola de forma independiente con un spline
 *   cubico en forma de momentos (segundas derivadas M_i en los knots),
 *   con knots no uniformes y condiciones de borde v(t0) = v(tN) = 0.
 * - El sistema tridiagonal se resuelve con el algoritmo de Thomas (misma
 *   familia que TrajectoryGenerator::computeClampedCoeffs de ur5_pick_place,
 *   generalizado a espaciado no uniforme y con evaluacion de derivadas).
 * - La orientacion del TCP es CONSTANTE (R_const): omega_des = alpha_des = 0.
 *
 * position/velocity/acceleration son continuas (C2) y evaluables en
 * cualquier t; fuera de [t0, tN] se satura al extremo (con v = a = 0 en los
 * extremos por la condicion clamped).
 */
class CartesianSplineTrajectory
{
public:
  CartesianSplineTrajectory(const std::vector<Eigen::Vector3d> & waypoints,
                            const std::vector<double> & times,
                            const Eigen::Matrix3d & R_const);

  double startTime() const { return t_.front(); }
  double endTime() const { return t_.back(); }
  double duration() const { return t_.back() - t_.front(); }

  Eigen::Vector3d position(double t) const;
  Eigen::Vector3d velocity(double t) const;
  Eigen::Vector3d acceleration(double t) const;
  const Eigen::Matrix3d & orientation() const { return R_const_; }

private:
  // Indice del segmento que contiene t (t ya saturado a [t0, tN]).
  int segmentIndex(double t) const;

  // Momentos M_i por eje: moments_[axis][i], i = 0..N.
  std::vector<double> t_;                     // knots (N+1)
  std::array<std::vector<double>, 3> y_;      // valores por eje (N+1)
  std::array<std::vector<double>, 3> moments_;
  Eigen::Matrix3d R_const_;
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_CARTESIAN_SPLINE_TRAJECTORY_HPP
