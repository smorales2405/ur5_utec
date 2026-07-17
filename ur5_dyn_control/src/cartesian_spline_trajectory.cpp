#include "ur5_dyn_control/cartesian_spline_trajectory.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>

namespace ur5_dyn_control
{

namespace
{

// Resuelve el sistema tridiagonal (Thomas) del spline clamped con knots no
// uniformes para un eje. Devuelve los momentos M_0..M_N (segundas derivadas).
//
//   fila 0:  2h0·M0 + h0·M1                     = 6((y1-y0)/h0 - v0)
//   fila i:  h_{i-1}·M_{i-1} + 2(h_{i-1}+h_i)·M_i + h_i·M_{i+1}
//            = 6((y_{i+1}-y_i)/h_i - (y_i-y_{i-1})/h_{i-1})
//   fila N:  h_{N-1}·M_{N-1} + 2h_{N-1}·M_N     = 6(vN - (yN-y_{N-1})/h_{N-1})
//
// con v0 = vN = 0 (clamped en reposo).
std::vector<double> solveClampedMoments(const std::vector<double> & t,
                                        const std::vector<double> & y)
{
  const int n = static_cast<int>(t.size()) - 1;  // numero de segmentos
  std::vector<double> h(n);
  for (int i = 0; i < n; ++i) {
    h[i] = t[i + 1] - t[i];
  }

  std::vector<double> a(n + 1), b(n + 1), c(n + 1), d(n + 1);
  b[0] = 2.0 * h[0];
  c[0] = h[0];
  d[0] = 6.0 * ((y[1] - y[0]) / h[0]);
  for (int i = 1; i < n; ++i) {
    a[i] = h[i - 1];
    b[i] = 2.0 * (h[i - 1] + h[i]);
    c[i] = h[i];
    d[i] = 6.0 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1]);
  }
  a[n] = h[n - 1];
  b[n] = 2.0 * h[n - 1];
  d[n] = 6.0 * (-(y[n] - y[n - 1]) / h[n - 1]);

  // Thomas: eliminacion hacia adelante + sustitucion hacia atras.
  std::vector<double> cp(n + 1), dp(n + 1);
  cp[0] = c[0] / b[0];
  dp[0] = d[0] / b[0];
  for (int i = 1; i <= n; ++i) {
    const double m = b[i] - a[i] * cp[i - 1];
    cp[i] = (i < n) ? c[i] / m : 0.0;
    dp[i] = (d[i] - a[i] * dp[i - 1]) / m;
  }
  std::vector<double> M(n + 1);
  M[n] = dp[n];
  for (int i = n - 1; i >= 0; --i) {
    M[i] = dp[i] - cp[i] * M[i + 1];
  }
  return M;
}

}  // namespace

CartesianSplineTrajectory::CartesianSplineTrajectory(
  const std::vector<Eigen::Vector3d> & waypoints,
  const std::vector<double> & times,
  const Eigen::Matrix3d & R_const)
: t_(times), R_const_(R_const)
{
  if (waypoints.size() < 2 || waypoints.size() != times.size()) {
    throw std::invalid_argument(
      "CartesianSplineTrajectory: se requieren >= 2 waypoints y times del mismo tamano");
  }
  for (std::size_t i = 1; i < times.size(); ++i) {
    if (times[i] <= times[i - 1]) {
      throw std::invalid_argument(
        "CartesianSplineTrajectory: los tiempos deben ser estrictamente crecientes");
    }
  }

  for (int axis = 0; axis < 3; ++axis) {
    y_[axis].resize(waypoints.size());
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
      y_[axis][i] = waypoints[i][axis];
    }
    moments_[axis] = solveClampedMoments(t_, y_[axis]);
  }
}

int CartesianSplineTrajectory::segmentIndex(double t) const
{
  // Ultimo knot <= t (t ya en [t0, tN]); segmento i cubre [t_i, t_{i+1}].
  const auto it = std::upper_bound(t_.begin(), t_.end(), t);
  int i = static_cast<int>(it - t_.begin()) - 1;
  i = std::clamp(i, 0, static_cast<int>(t_.size()) - 2);
  return i;
}

Eigen::Vector3d CartesianSplineTrajectory::position(double t) const
{
  t = std::clamp(t, t_.front(), t_.back());
  const int i = segmentIndex(t);
  const double h = t_[i + 1] - t_[i];
  const double sa = t_[i + 1] - t;   // (t_{i+1} - t)
  const double sb = t - t_[i];       // (t - t_i)

  Eigen::Vector3d p;
  for (int axis = 0; axis < 3; ++axis) {
    const double Mi = moments_[axis][i], Mj = moments_[axis][i + 1];
    const double yi = y_[axis][i], yj = y_[axis][i + 1];
    p[axis] = Mi * sa * sa * sa / (6.0 * h) + Mj * sb * sb * sb / (6.0 * h) +
              (yi / h - Mi * h / 6.0) * sa + (yj / h - Mj * h / 6.0) * sb;
  }
  return p;
}

Eigen::Vector3d CartesianSplineTrajectory::velocity(double t) const
{
  if (t <= t_.front() || t >= t_.back()) {
    return Eigen::Vector3d::Zero();  // clamped: v = 0 en los extremos
  }
  const int i = segmentIndex(t);
  const double h = t_[i + 1] - t_[i];
  const double sa = t_[i + 1] - t;
  const double sb = t - t_[i];

  Eigen::Vector3d v;
  for (int axis = 0; axis < 3; ++axis) {
    const double Mi = moments_[axis][i], Mj = moments_[axis][i + 1];
    const double yi = y_[axis][i], yj = y_[axis][i + 1];
    v[axis] = -Mi * sa * sa / (2.0 * h) + Mj * sb * sb / (2.0 * h) +
              (yj - yi) / h - (Mj - Mi) * h / 6.0;
  }
  return v;
}

Eigen::Vector3d CartesianSplineTrajectory::acceleration(double t) const
{
  if (t < t_.front() || t > t_.back()) {
    return Eigen::Vector3d::Zero();
  }
  const int i = segmentIndex(std::clamp(t, t_.front(), t_.back()));
  const double h = t_[i + 1] - t_[i];
  const double sa = t_[i + 1] - t;
  const double sb = t - t_[i];

  Eigen::Vector3d a;
  for (int axis = 0; axis < 3; ++axis) {
    const double Mi = moments_[axis][i], Mj = moments_[axis][i + 1];
    a[axis] = Mi * sa / h + Mj * sb / h;
  }
  return a;
}

}  // namespace ur5_dyn_control
