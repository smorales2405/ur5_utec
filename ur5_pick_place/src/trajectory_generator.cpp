#include "ur5_pick_place/trajectory_generator.hpp"

#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <stdexcept>

namespace ur5_pick_place {

// ── Slerp helper ──────────────────────────────────────────────────────────────
Eigen::Matrix3d TrajectoryGenerator::slerp(
    const Eigen::Matrix3d& R0,
    const Eigen::Matrix3d& R1,
    double t)
{
    Eigen::Quaterniond q0(R0), q1(R1);
    q0.normalize();
    q1.normalize();
    return q0.slerp(t, q1).toRotationMatrix();
}

// ── Clamped cubic spline coefficients ────────────────────────────────────────
// Given n+1 values y[0..n] at uniform t ∈ {0,1,..,n},
// solve the tridiagonal system for clamped BC: y'(0)=0, y'(n)=0.
// Returns n sets of [a, b, c, d] with p_i(s) = a + b*s + c*s^2 + d*s^3, s∈[0,1].
std::vector<Eigen::Vector4d> TrajectoryGenerator::computeClampedCoeffs(
    const std::vector<double>& y)
{
    const int n = static_cast<int>(y.size()) - 1;
    if (n < 1) throw std::invalid_argument("Need at least 2 keypoints");

    // h = 1 for every segment (uniform parameterisation)
    // Build tridiagonal system for second derivatives M[0..n]
    // Clamped BC: M[0] and M[n] are determined by zero first-derivative condition.
    std::vector<double> M(n + 1, 0.0);
    std::vector<double> rhs(n + 1, 0.0);

    // Interior equations: standard natural/clamped spline tridiagonal
    // Clamped: f'(t0)=0 → M[0] + M[1]/2 = 3*(y[1]-y[0]) but with clamp → simplified below
    // We use the "not-a-knot" variant replaced by a proper clamped setup:
    //   Lower diagonal: 1, Main: 4, Upper: 1  for interior nodes
    //   End rows modified for clamped BC.
    std::vector<double> diag(n + 1, 4.0);
    std::vector<double> upper(n, 1.0);
    std::vector<double> lower(n, 1.0);

    // Clamped BC: d'(0) = 0  →  2*M[0] + M[1] = 3*(y[1]-y[0])
    diag[0]  = 2.0;
    rhs[0]   = 3.0 * (y[1] - y[0]);

    for (int i = 1; i < n; ++i) {
        rhs[i] = 3.0 * (y[i + 1] - 2.0 * y[i] + y[i - 1]);
    }

    // Clamped BC: d'(n) = 0  →  M[n-1] + 2*M[n] = 3*(y[n-1]-y[n])
    diag[n]  = 2.0;
    rhs[n]   = 3.0 * (y[n - 1] - y[n]);

    // Thomas algorithm (forward sweep)
    std::vector<double> c_prime(n + 1, 0.0);
    std::vector<double> d_prime(n + 1, 0.0);
    c_prime[0] = upper[0] / diag[0];
    d_prime[0] = rhs[0]   / diag[0];
    for (int i = 1; i <= n; ++i) {
        double denom = diag[i] - (i < n ? lower[i - 1] : lower[n - 1]) * c_prime[i - 1];
        c_prime[i] = (i < n ? upper[i] / denom : 0.0);
        d_prime[i] = (rhs[i] - (i > 0 ? lower[i - 1] : 0.0) * d_prime[i - 1]) / denom;
    }
    // Back substitution
    M[n] = d_prime[n];
    for (int i = n - 1; i >= 0; --i) {
        M[i] = d_prime[i] - c_prime[i] * M[i + 1];
    }

    // Convert M (second derivatives) to [a,b,c,d] per segment
    std::vector<Eigen::Vector4d> coeffs(n);
    for (int i = 0; i < n; ++i) {
        double a = y[i];
        double b = (y[i + 1] - y[i]) - (2.0 * M[i] + M[i + 1]) / 3.0;
        double c = M[i];
        double d = (M[i + 1] - M[i]) / 3.0;
        coeffs[i] = Eigen::Vector4d(a, b, c, d);
    }
    return coeffs;
}

// ── Clamped cubic spline trajectory ──────────────────────────────────────────
std::vector<CartesianWaypoint> TrajectoryGenerator::clampedCubicSpline(
    const std::vector<CartesianWaypoint>& kp,
    int pts_per_seg)
{
    const int nk = static_cast<int>(kp.size());
    if (nk < 2) throw std::invalid_argument("Need at least 2 keypoints");

    // Extract x, y, z sequences
    std::vector<double> px(nk), py(nk), pz(nk);
    for (int i = 0; i < nk; ++i) {
        px[i] = kp[i].position.x();
        py[i] = kp[i].position.y();
        pz[i] = kp[i].position.z();
    }

    auto cx = computeClampedCoeffs(px);
    auto cy = computeClampedCoeffs(py);
    auto cz = computeClampedCoeffs(pz);

    const int n_segs = nk - 1;

    std::vector<CartesianWaypoint> out;
    out.reserve(n_segs * pts_per_seg + 1);

    for (int seg = 0; seg < n_segs; ++seg) {
        double dt_seg = kp[seg + 1].timestamp - kp[seg].timestamp;
        int steps = (seg == n_segs - 1) ? pts_per_seg + 1 : pts_per_seg;
        for (int j = 0; j < steps; ++j) {
            double s = static_cast<double>(j) / pts_per_seg;  // ∈ [0,1]
            double s2 = s * s, s3 = s2 * s;

            auto eval = [&](const Eigen::Vector4d& co) {
                return co[0] + co[1] * s + co[2] * s2 + co[3] * s3;
            };

            CartesianWaypoint wp;
            wp.position   = {eval(cx[seg]), eval(cy[seg]), eval(cz[seg])};
            wp.orientation = slerp(kp[seg].orientation, kp[seg + 1].orientation, s);
            wp.timestamp   = kp[seg].timestamp + s * dt_seg;
            out.push_back(wp);
        }
    }
    return out;
}

// ── Piecewise linear trajectory ───────────────────────────────────────────────
std::vector<CartesianWaypoint> TrajectoryGenerator::piecewiseLinear(
    const std::vector<CartesianWaypoint>& kp,
    int pts_per_seg)
{
    const int nk = static_cast<int>(kp.size());
    if (nk < 2) throw std::invalid_argument("Need at least 2 keypoints");

    const int n_segs = nk - 1;

    std::vector<CartesianWaypoint> out;
    out.reserve(n_segs * pts_per_seg + 1);

    for (int seg = 0; seg < n_segs; ++seg) {
        double dt_seg = kp[seg + 1].timestamp - kp[seg].timestamp;
        int steps = (seg == n_segs - 1) ? pts_per_seg + 1 : pts_per_seg;
        for (int j = 0; j < steps; ++j) {
            double s = static_cast<double>(j) / pts_per_seg;

            CartesianWaypoint wp;
            wp.position    = (1.0 - s) * kp[seg].position + s * kp[seg + 1].position;
            wp.orientation = slerp(kp[seg].orientation, kp[seg + 1].orientation, s);
            wp.timestamp   = kp[seg].timestamp + s * dt_seg;
            out.push_back(wp);
        }
    }
    return out;
}

}  // namespace ur5_pick_place
