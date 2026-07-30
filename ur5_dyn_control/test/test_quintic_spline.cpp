// ============================================================================
//  Tests del spline QUINTICO (FASE 1) — criterios de aceptacion del plan:
//
//   * "Derivadas analiticas vs. diferencias finitas: error < 1e-6"
//   * "Continuidad de jerk verificada en los nudos (el cubico falla este test,
//      el quintico lo pasa)"
//
//  El segundo test se ejecuta sobre AMBOS splines con los MISMOS waypoints:
//  demuestra la mejora, no solo que el quintico funciona.
// ============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "ur5_dyn_control/cartesian_spline_trajectory.hpp"
#include "ur5_dyn_control/quintic_spline.hpp"
#include "ur5_dyn_control/quintic_spline_trajectory.hpp"

using ur5_dyn_control::CartesianSplineTrajectory;
using ur5_dyn_control::QuinticSpline3d;
using ur5_dyn_control::QuinticSplineTrajectory;

namespace
{

// Waypoints y tiempos NO uniformes (el plan pide nudos no uniformes).
const std::vector<Eigen::Vector3d> kWaypoints = {
  {0.49,  0.13, 0.35},
  {0.55,  0.35, 0.28},
  {0.55, -0.10, 0.25},
  {0.49, -0.35, 0.32},
  {0.45, -0.20, 0.40},
};
const std::vector<double> kTimes = {0.0, 2.5, 5.0, 7.5, 9.0};   // pasos 2.5/2.5/2.5/1.5

Eigen::Matrix3d constOrientation()
{
  return (Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

/// Diferencia central de orden 4 (error O(h^4)): con h=1e-3 el error de
/// truncamiento es ~1e-12 y el de redondeo ~1e-13, muy por debajo del 1e-6
/// exigido, asi que el test mide de verdad la derivada analitica.
template<typename F>
Eigen::Vector3d fdDerivative(F && f, double t, double h)
{
  return (-f(t + 2 * h) + 8.0 * f(t + h) - 8.0 * f(t - h) + f(t - 2 * h)) / (12.0 * h);
}

}  // namespace

// ── Interpolacion y condiciones de contorno ─────────────────────────────────
TEST(QuinticSpline, InterpolatesWaypointsExactly)
{
  QuinticSpline3d s(kWaypoints, kTimes);
  for (std::size_t i = 0; i < kWaypoints.size(); ++i) {
    EXPECT_LT((s.eval(kTimes[i], 0) - kWaypoints[i]).cwiseAbs().maxCoeff(), 1e-12)
      << "waypoint " << i;
  }
}

TEST(QuinticSpline, BoundaryVelocityAndAccelerationAreZero)
{
  QuinticSpline3d s(kWaypoints, kTimes);
  for (double t : {kTimes.front(), kTimes.back()}) {
    EXPECT_LT(s.eval(t, 1).cwiseAbs().maxCoeff(), 1e-12) << "v en t=" << t;
    EXPECT_LT(s.eval(t, 2).cwiseAbs().maxCoeff(), 1e-12) << "a en t=" << t;
  }
}

// ── Criterio 1: derivadas analiticas vs diferencias finitas < 1e-6 ──────────
TEST(QuinticSpline, AnalyticDerivativesMatchFiniteDifferences)
{
  QuinticSplineTrajectory traj(kWaypoints, kTimes, constOrientation());
  const double h = 1e-3;
  double max_v = 0.0, max_a = 0.0, max_j = 0.0;

  // Se evalua dentro del dominio, evitando una franja de +-2h en los extremos
  // (la diferencia central saldria del intervalo, donde la referencia se satura).
  for (double t = kTimes.front() + 3 * h; t < kTimes.back() - 3 * h; t += 0.01) {
    const auto fd_v = fdDerivative([&](double x) {return traj.position(x);}, t, h);
    const auto fd_a = fdDerivative([&](double x) {return traj.velocity(x);}, t, h);
    const auto fd_j = fdDerivative([&](double x) {return traj.acceleration(x);}, t, h);
    max_v = std::max(max_v, (traj.velocity(t) - fd_v).cwiseAbs().maxCoeff());
    max_a = std::max(max_a, (traj.acceleration(t) - fd_a).cwiseAbs().maxCoeff());
    max_j = std::max(max_j, (traj.jerk(t) - fd_j).cwiseAbs().maxCoeff());
  }
  EXPECT_LT(max_v, 1e-6) << "velocidad";
  EXPECT_LT(max_a, 1e-6) << "aceleracion";
  EXPECT_LT(max_j, 1e-6) << "jerk";
}

// ── Criterio 2: continuidad de jerk en los nudos ─────────────────────────────
TEST(QuinticSpline, JerkIsContinuousAtInteriorKnots)
{
  QuinticSplineTrajectory traj(kWaypoints, kTimes, constOrientation());
  const double eps = 1e-7;
  for (std::size_t i = 1; i + 1 < kTimes.size(); ++i) {
    const double t = kTimes[i];
    const double gap = (traj.jerk(t + eps) - traj.jerk(t - eps)).cwiseAbs().maxCoeff();
    EXPECT_LT(gap, 1e-6) << "salto de jerk en el nudo interior " << i
                         << " (t=" << t << "): " << gap;
  }
}

// El mismo test sobre el spline CUBICO debe FALLAR: es lo que justifica el
// cambio a quintico y es el dato de la figura "cubico vs quintico" del paper.
TEST(CubicSpline, JerkIsDiscontinuousAtInteriorKnots)
{
  CartesianSplineTrajectory cubic(kWaypoints, kTimes, constOrientation());
  const double eps = 1e-7;
  double max_gap = 0.0;
  for (std::size_t i = 1; i + 1 < kTimes.size(); ++i) {
    const double t = kTimes[i];
    max_gap = std::max(
      max_gap, (cubic.jerk(t + eps) - cubic.jerk(t - eps)).cwiseAbs().maxCoeff());
  }
  // El cubico salta varios ordenes de magnitud por encima de la tolerancia que
  // el quintico cumple (1e-6).
  EXPECT_GT(max_gap, 1e-3)
    << "el spline cubico deberia tener jerk discontinuo; salto maximo=" << max_gap;
}

// El cubico tampoco anula la aceleracion en los extremos (el quintico si).
TEST(CubicSpline, BoundaryAccelerationIsNonZero)
{
  CartesianSplineTrajectory cubic(kWaypoints, kTimes, constOrientation());
  const double a0 = cubic.acceleration(kTimes.front() + 1e-9).cwiseAbs().maxCoeff();
  EXPECT_GT(a0, 1e-3) << "el cubico clamped arranca con aceleracion no nula";
}

// ── Caso degenerado: dos waypoints => segmento RECTO exacto ─────────────────
// Es la propiedad en la que se apoyan las fases rectas de la incision.
TEST(QuinticSpline, TwoWaypointsGiveExactStraightSegment)
{
  const Eigen::Vector3d p0(0.50, -0.04, 0.015), p1(0.50, 0.04, 0.015);
  QuinticSpline3d s({p0, p1});
  const Eigen::Vector3d dir = (p1 - p0).normalized();

  for (double u = 0.0; u <= 1.0; u += 0.01) {
    const Eigen::Vector3d p = s.eval(u, 0);
    // Distancia perpendicular a la recta p0 + t*dir.
    const Eigen::Vector3d d = p - p0;
    const double perp = (d - d.dot(dir) * dir).norm();
    EXPECT_LT(perp, 1e-15) << "u=" << u;
    // La velocidad de parametro tambien es paralela a la recta.
    const Eigen::Vector3d v = s.eval(u, 1);
    if (v.norm() > 1e-12) {
      EXPECT_LT((v.normalized() - dir).cwiseAbs().maxCoeff(), 1e-12) << "u=" << u;
    }
  }
}

// ── Base de Hermite quintico: valores en los extremos ──────────────────────
TEST(QuinticSpline, HermiteBasisHasCorrectEndpointValues)
{
  // tau = 0: solo h00 vale 1; h10' = 1; h20'' = 1; el resto se anula.
  const auto B0 = QuinticSpline3d::hermiteBasis(0.0, 0);
  const auto D0 = QuinticSpline3d::hermiteBasis(0.0, 1);
  const auto S0 = QuinticSpline3d::hermiteBasis(0.0, 2);
  EXPECT_DOUBLE_EQ(B0[0], 1.0);
  EXPECT_DOUBLE_EQ(D0[1], 1.0);
  EXPECT_DOUBLE_EQ(S0[2], 1.0);
  for (int b : {1, 2, 3, 4, 5}) {EXPECT_DOUBLE_EQ(B0[b], 0.0) << "B0[" << b << "]";}
  for (int b : {0, 2, 3, 4, 5}) {EXPECT_DOUBLE_EQ(D0[b], 0.0) << "D0[" << b << "]";}
  for (int b : {0, 1, 3, 4, 5}) {EXPECT_DOUBLE_EQ(S0[b], 0.0) << "S0[" << b << "]";}

  // tau = 1: h01 = 1; h11' = 1; h21'' = 1.
  const auto B1 = QuinticSpline3d::hermiteBasis(1.0, 0);
  const auto D1 = QuinticSpline3d::hermiteBasis(1.0, 1);
  const auto S1 = QuinticSpline3d::hermiteBasis(1.0, 2);
  EXPECT_NEAR(B1[3], 1.0, 1e-15);
  EXPECT_NEAR(D1[4], 1.0, 1e-14);
  EXPECT_NEAR(S1[5], 1.0, 1e-14);
  for (int b : {0, 1, 2, 4, 5}) {EXPECT_NEAR(B1[b], 0.0, 1e-15) << "B1[" << b << "]";}
  for (int b : {0, 1, 2, 3, 5}) {EXPECT_NEAR(D1[b], 0.0, 1e-14) << "D1[" << b << "]";}
  for (int b : {0, 1, 2, 3, 4}) {EXPECT_NEAR(S1[b], 0.0, 1e-13) << "S1[" << b << "]";}
}

// Continuidad C4 explicita (es la condicion que impone el sistema lineal).
TEST(QuinticSpline, FourthDerivativeIsContinuousAtInteriorKnots)
{
  QuinticSpline3d s(kWaypoints, kTimes);
  const double eps = 1e-7;
  for (std::size_t i = 1; i + 1 < kTimes.size(); ++i) {
    const double t = kTimes[i];
    const double gap = (s.eval(t + eps, 4) - s.eval(t - eps, 4)).cwiseAbs().maxCoeff();
    EXPECT_LT(gap, 1e-5) << "salto de la 4a derivada en el nudo " << i;
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
