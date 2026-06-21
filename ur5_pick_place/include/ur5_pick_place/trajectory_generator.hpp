#pragma once

#include <Eigen/Dense>
#include <vector>

namespace ur5_pick_place {

struct CartesianWaypoint {
    Eigen::Vector3d position;
    Eigen::Matrix3d orientation;
    double timestamp;                              // seconds from trajectory start
    Eigen::Vector3d jerk{0.0, 0.0, 0.0};          // TCP jerk [m/s³]: analytic for spline, 0 for linear
};

class TrajectoryGenerator {
public:
    // Clamped cubic spline in position (velocity = 0 at first and last keypoint).
    // Orientation is slerp-interpolated between consecutive keypoints.
    static std::vector<CartesianWaypoint> clampedCubicSpline(
        const std::vector<CartesianWaypoint>& keypoints,
        int points_per_segment);

    // Piecewise linear interpolation in position and orientation (slerp).
    static std::vector<CartesianWaypoint> piecewiseLinear(
        const std::vector<CartesianWaypoint>& keypoints,
        int points_per_segment);

private:
    // Compute clamped cubic spline coefficients for one scalar axis.
    // Returns coefficients [a, b, c, d] per segment: p(t) = a + b*t + c*t^2 + d*t^3
    // where t ∈ [0,1] within each segment.
    static std::vector<Eigen::Vector4d> computeClampedCoeffs(
        const std::vector<double>& y);

    // Spherical linear interpolation between two rotation matrices.
    static Eigen::Matrix3d slerp(
        const Eigen::Matrix3d& R0,
        const Eigen::Matrix3d& R1,
        double t);
};

}  // namespace ur5_pick_place
