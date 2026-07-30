#include "ur5_dyn_control/quintic_spline_trajectory.hpp"

namespace ur5_dyn_control
{

QuinticSplineTrajectory::QuinticSplineTrajectory(
  const std::vector<Eigen::Vector3d> & waypoints,
  const std::vector<double> & times,
  const Eigen::Matrix3d & R_const)
: spline_(waypoints, times), R_const_(R_const)
{
}

}  // namespace ur5_dyn_control
