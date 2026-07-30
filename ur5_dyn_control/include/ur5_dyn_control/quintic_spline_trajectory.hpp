#ifndef UR5_DYN_CONTROL_QUINTIC_SPLINE_TRAJECTORY_HPP
#define UR5_DYN_CONTROL_QUINTIC_SPLINE_TRAJECTORY_HPP

#include <memory>
#include <vector>

#include "ur5_dyn_control/cartesian_trajectory.hpp"
#include "ur5_dyn_control/quintic_spline.hpp"

namespace ur5_dyn_control
{

/**
 * Trayectoria cartesiana por spline QUINTICO parametrizado en TIEMPO.
 *
 * Sustituto directo de CartesianSplineTrajectory (mismos argumentos de
 * construccion, misma interfaz), con dos mejoras:
 *  - jerk CONTINUO en los nudos (el cubico lo tiene discontinuo);
 *  - aceleracion nula en los extremos ademas de velocidad nula, asi que el
 *    arranque y la parada no piden un escalon de par.
 *
 * La orientacion del TCP es constante (supuesto A4).
 */
class QuinticSplineTrajectory : public CartesianTrajectory
{
public:
  QuinticSplineTrajectory(const std::vector<Eigen::Vector3d> & waypoints,
                          const std::vector<double> & times,
                          const Eigen::Matrix3d & R_const);

  double startTime() const override { return spline_.startKnot(); }
  double endTime() const override { return spline_.endKnot(); }

  Eigen::Vector3d position(double t) const override { return spline_.eval(t, 0); }
  Eigen::Vector3d velocity(double t) const override { return spline_.eval(t, 1); }
  Eigen::Vector3d acceleration(double t) const override { return spline_.eval(t, 2); }
  Eigen::Vector3d jerk(double t) const override { return spline_.eval(t, 3); }

  const Eigen::Matrix3d & orientation() const override { return R_const_; }

  const QuinticSpline3d & spline() const { return spline_; }

private:
  QuinticSpline3d spline_;
  Eigen::Matrix3d R_const_;
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_QUINTIC_SPLINE_TRAJECTORY_HPP
