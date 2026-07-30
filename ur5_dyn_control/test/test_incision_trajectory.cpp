// ============================================================================
//  Tests de la separacion geometria/temporizacion y de la incision (FASE 1):
//
//   * cuadratura de Gauss-Legendre y reparametrizacion por longitud de arco
//   * perfil S-curve con jerk acotado
//   * "Feed rate constante dentro de +-2 % en el tramo cut"
//   * derivadas analiticas de la trayectoria compuesta p(t)=P(u(s(t)))
//     contra diferencias finitas
//   * geometria de las 5 fases (profundidad constante, rectitud del trazo)
// ============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "ur5_dyn_control/arc_length.hpp"
#include "ur5_dyn_control/gauss_legendre.hpp"
#include "ur5_dyn_control/incision_trajectory.hpp"
#include "ur5_dyn_control/quintic_spline.hpp"
#include "ur5_dyn_control/time_profile.hpp"

using ur5_dyn_control::ArcLength;
using ur5_dyn_control::GaussLegendre;
using ur5_dyn_control::IncisionParams;
using ur5_dyn_control::IncisionPhaseId;
using ur5_dyn_control::IncisionTrajectory;
using ur5_dyn_control::QuinticSpline3d;
using ur5_dyn_control::ScurveProfile;

namespace
{

/// Parametros de la campana (los mismos que config/incision_params.yaml).
IncisionParams defaultParams()
{
  IncisionParams p;
  p.start_pose = {0.49, 0.13, 0.35};
  p.surface_z = 0.02;
  p.cut_x = 0.50;
  p.cut_center_y = 0.0;
  p.cut_length = 0.08;
  p.cut_depth = 0.005;
  p.approach_height = 0.03;
  p.cut_axis = 'y';
  p.v_cut = 0.010;
  return p;
}

template<typename F>
Eigen::Vector3d fdDerivative(F && f, double t, double h)
{
  return (-f(t + 2 * h) + 8.0 * f(t + h) - 8.0 * f(t - h) + f(t - 2 * h)) / (12.0 * h);
}

}  // namespace

// ── Gauss-Legendre (port del CU3) ────────────────────────────────────────────
TEST(GaussLegendre, IsExactForPolynomialsUpToDegree2nMinus1)
{
  // Con n nodos la regla integra exactamente polinomios de grado <= 2n-1.
  for (int n = 2; n <= 8; ++n) {
    const GaussLegendre gl(n);
    const int deg = 2 * n - 1;
    // INT_0^1 x^deg dx = 1/(deg+1)
    const double num = gl.integrate([deg](double x) {return std::pow(x, deg);}, 0.0, 1.0);
    EXPECT_NEAR(num, 1.0 / (deg + 1), 1e-13) << "n=" << n << " grado=" << deg;
  }
}

TEST(GaussLegendre, MatchesKnownIntegrals)
{
  const GaussLegendre gl(12);
  EXPECT_NEAR(gl.integrate([](double x) {return std::sin(x);}, 0.0, M_PI), 2.0, 1e-12);
  EXPECT_NEAR(gl.integrate([](double x) {return std::exp(x);}, 0.0, 1.0),
              std::exp(1.0) - 1.0, 1e-13);
  EXPECT_NEAR(gl.integrate([](double x) {return 1.0 / x;}, 1.0, 2.0),
              std::log(2.0), 1e-12);
  // Pesos: suman la longitud del intervalo.
  EXPECT_NEAR(gl.integrate([](double) {return 1.0;}, -3.0, 5.0), 8.0, 1e-13);
}

// ── Longitud de arco ────────────────────────────────────────────────────────
TEST(ArcLength, StraightSegmentHasExactLength)
{
  const Eigen::Vector3d p0(0.50, -0.04, 0.015), p1(0.50, 0.04, 0.015);
  QuinticSpline3d s({p0, p1});
  ArcLength arc(s);
  EXPECT_NEAR(arc.totalLength(), (p1 - p0).norm(), 1e-12);

  // En una recta s(u) es la longitud recorrida; u(s) debe ser su inversa exacta.
  for (double frac = 0.0; frac <= 1.0; frac += 0.05) {
    const double s_target = frac * arc.totalLength();
    const double u = arc.uOfS(s_target);
    EXPECT_NEAR(arc.sOfU(u), s_target, 1e-12) << "frac=" << frac;
  }
}

TEST(ArcLength, InverseIsConsistentOnCurvedPath)
{
  QuinticSpline3d s({{0.49, 0.13, 0.35}, {0.55, 0.35, 0.28},
                     {0.55, -0.10, 0.25}, {0.49, -0.35, 0.32}});
  ArcLength arc(s);
  ASSERT_GT(arc.totalLength(), 0.0);

  for (double frac = 0.0; frac <= 1.0; frac += 0.02) {
    const double s_t = frac * arc.totalLength();
    const double u = arc.uOfS(s_t);
    EXPECT_NEAR(arc.sOfU(u), s_t, 1e-9) << "frac=" << frac;
  }
}

// La tabla s(u) se compara contra una cuadratura INDEPENDIENTE de alto orden
// hecha aqui mismo: valida a la vez el port de Gauss-Legendre y la
// construccion de la tabla. El VALOR de s(u) es lo unico que se usa despues
// (para invertir s -> u); todas las derivadas de la regla de la cadena salen
// de formulas analiticas, no de derivar la tabla.
TEST(ArcLength, TableMatchesIndependentQuadrature)
{
  QuinticSpline3d s({{0.49, 0.13, 0.35}, {0.55, 0.35, 0.28}, {0.55, -0.10, 0.25}});
  ArcLength arc(s);
  const GaussLegendre fine(32);
  const auto speed = [&s](double u) {return s.eval(u, 1).norm();};

  double worst = 0.0;
  for (double u = 0.05; u <= 1.0; u += 0.05) {
    // Referencia: integrar por sub-tramos finos con 32 nodos cada uno.
    double ref = 0.0;
    const int n_sub = 200;
    for (int i = 0; i < n_sub; ++i) {
      const double a = u * i / n_sub, b = u * (i + 1) / n_sub;
      ref += fine.integrate(speed, a, b);
    }
    worst = std::max(worst, std::abs(arc.sOfU(u) - ref));
  }
  EXPECT_LT(worst, 1e-9) << "error absoluto maximo de s(u): " << worst << " m";
}

// La tabla se interpola con Hermite cubico usando las pendientes EXACTAS
// |p'(u)| en los nodos, asi que su derivada coincide con la rapidez de la
// curva salvo el error de interpolacion entre nodos. Se documenta la cota:
// no afecta a la trayectoria porque la regla de la cadena usa 1/|p'(u)|
// analitico, nunca la pendiente de la tabla.
TEST(ArcLength, TableSlopeApproximatesCurveSpeed)
{
  QuinticSpline3d s({{0.49, 0.13, 0.35}, {0.55, 0.35, 0.28}, {0.55, -0.10, 0.25}});
  ArcLength arc(s);
  const double h = 1e-5;
  double worst_rel = 0.0;

  for (double u = 0.05; u <= 0.95; u += 0.01) {
    const double fd = (-arc.sOfU(u + 2 * h) + 8 * arc.sOfU(u + h) -
                       8 * arc.sOfU(u - h) + arc.sOfU(u - 2 * h)) / (12.0 * h);
    worst_rel = std::max(worst_rel, std::abs(fd - s.eval(u, 1).norm()) /
                         s.eval(u, 1).norm());
  }
  EXPECT_LT(worst_rel, 1e-3) << "error relativo de la pendiente de la tabla: "
                             << worst_rel;
}

// g = |p'(u)| y sus derivadas: es la unica matematica no trivial de
// uDerivatives(). Se comprueba contra diferencias finitas EN u (sin pasar por
// la tabla ni por la inversion).
TEST(ArcLength, SpeedDerivativesMatchFiniteDifferences)
{
  QuinticSpline3d s({{0.49, 0.13, 0.35}, {0.55, 0.35, 0.28}, {0.55, -0.10, 0.25}});
  ArcLength arc(s);
  const double h = 1e-4;
  const auto G = [&](double u) {return s.eval(u, 1).norm();};

  for (double u = 0.05; u <= 0.95; u += 0.01) {
    double g, g_u, g_uu;
    arc.speedDerivatives(u, g, g_u, g_uu);
    EXPECT_NEAR(g, G(u), 1e-14) << "u=" << u;

    const double fd1 = (-G(u + 2 * h) + 8 * G(u + h) - 8 * G(u - h) + G(u - 2 * h)) /
      (12.0 * h);
    const double fd2 = (-G(u + 2 * h) + 16 * G(u + h) - 30 * G(u) +
                        16 * G(u - h) - G(u - 2 * h)) / (12.0 * h * h);
    EXPECT_NEAR(g_u, fd1, 1e-6 * std::max(1.0, std::abs(g_u))) << "u=" << u;
    EXPECT_NEAR(g_uu, fd2, 1e-4 * std::max(1.0, std::abs(g_uu))) << "u=" << u;
  }
}

// Las derivadas de u respecto de s son las identidades algebraicas
//   du/ds = 1/g,  d2u/ds2 = -g_u/g^3,  d3u/ds3 = (3 g_u^2 - g g_uu)/g^5.
TEST(ArcLength, ParameterDerivativesFollowTheirDefinition)
{
  QuinticSpline3d s({{0.49, 0.13, 0.35}, {0.55, 0.35, 0.28}, {0.55, -0.10, 0.25}});
  ArcLength arc(s);
  for (double u = 0.02; u <= 0.98; u += 0.01) {
    double g, g_u, g_uu, du, d2u, d3u;
    arc.speedDerivatives(u, g, g_u, g_uu);
    arc.uDerivatives(u, du, d2u, d3u);
    EXPECT_NEAR(du, 1.0 / g, 1e-14 * std::max(1.0, std::abs(du))) << "u=" << u;
    EXPECT_NEAR(d2u, -g_u / (g * g * g), 1e-12 * std::max(1.0, std::abs(d2u))) << "u=" << u;
    EXPECT_NEAR(d3u, (3 * g_u * g_u - g * g_uu) / std::pow(g, 5),
                1e-10 * std::max(1.0, std::abs(d3u))) << "u=" << u;
  }
}

// Caso con solucion cerrada: en una RECTA con parametrizacion regular,
// g = L constante, s(u) = L u, u(s) = s/L, y las derivadas superiores se anulan.
TEST(ArcLength, StraightSegmentHasClosedFormParameterDerivatives)
{
  const Eigen::Vector3d p0(0.50, -0.04, 0.015), p1(0.50, 0.04, 0.015);
  QuinticSpline3d s({p0, p1});
  ArcLength arc(s);
  const double L = (p1 - p0).norm();

  // d2u/ds2 y d3u/ds3 tienen unidades 1/L^2 y 1/L^3: la tolerancia se escala
  // con esa magnitud (para L = 80 mm, 1/L^5 vale 3e5 y amplifica el redondeo).
  const double scale2 = 1.0 / (L * L), scale3 = 1.0 / (L * L * L);
  for (double u = 0.0; u <= 1.0; u += 0.05) {
    double du, d2u, d3u;
    arc.uDerivatives(u, du, d2u, d3u);
    EXPECT_NEAR(du, 1.0 / L, 1e-12 / L) << "u=" << u;
    EXPECT_NEAR(d2u, 0.0, 1e-9 * scale2) << "u=" << u;
    EXPECT_NEAR(d3u, 0.0, 1e-9 * scale3) << "u=" << u;
    EXPECT_NEAR(arc.sOfU(u), L * u, 1e-14) << "u=" << u;
  }
}

// Una geometria con contorno CLAMPED_REST tiene |p'| = 0 en los extremos y NO
// es una parametrizacion valida para el arco: debe rechazarse en construccion
// en vez de propagar 1/0.
TEST(ArcLength, RejectsNonRegularParametrization)
{
  QuinticSpline3d bad({{0.0, 0.0, 0.0}, {0.1, 0.0, 0.0}}, {0.0, 1.0},
                      ur5_dyn_control::QuinticBoundary::CLAMPED_REST);
  EXPECT_THROW(ArcLength{bad}, std::runtime_error);
}

// ── Perfil S-curve ──────────────────────────────────────────────────────────
TEST(ScurveProfile, TravelsExactLengthWithZeroBoundaryDerivatives)
{
  const ScurveProfile p(0.08, 0.010, 0.10);
  EXPECT_NEAR(p.s(0.0), 0.0, 1e-15);
  EXPECT_NEAR(p.s(p.duration()), 0.08, 1e-12);
  for (double t : {0.0, p.duration()}) {
    EXPECT_NEAR(p.sd(t), 0.0, 1e-12) << "v en t=" << t;
    EXPECT_NEAR(p.sdd(t), 0.0, 1e-12) << "a en t=" << t;
    EXPECT_NEAR(p.sddd(t), 0.0, 1e-12) << "jerk en t=" << t;
  }
  // Duracion: T = (1 + 2*phi) * L / v
  EXPECT_NEAR(p.duration(), 1.2 * 0.08 / 0.010, 1e-12);
}

TEST(ScurveProfile, DerivativesMatchFiniteDifferences)
{
  const ScurveProfile p(0.08, 0.010, 0.10);
  const double h = 1e-4;
  for (double t = 4 * h; t < p.duration() - 4 * h; t += 0.01) {
    const double fd_v = (-p.s(t + 2 * h) + 8 * p.s(t + h) - 8 * p.s(t - h) +
                         p.s(t - 2 * h)) / (12 * h);
    const double fd_a = (-p.sd(t + 2 * h) + 8 * p.sd(t + h) - 8 * p.sd(t - h) +
                         p.sd(t - 2 * h)) / (12 * h);
    EXPECT_NEAR(p.sd(t), fd_v, 1e-9) << "t=" << t;
    EXPECT_NEAR(p.sdd(t), fd_a, 1e-7) << "t=" << t;
  }
}

TEST(ScurveProfile, PlateauHasExactlyCruiseSpeed)
{
  const ScurveProfile p(0.08, 0.010, 0.10);
  double t0, t1;
  p.plateauInterval(t0, t1);
  ASSERT_GT(t1, t0);
  for (double t = t0; t <= t1; t += (t1 - t0) / 100.0) {
    EXPECT_NEAR(p.sd(t), 0.010, 1e-15) << "t=" << t;
    EXPECT_NEAR(p.sdd(t), 0.0, 1e-15);
  }
  // La meseta cubre (1 - 2*phi) de la longitud: 80 % de 80 mm = 64 mm.
  EXPECT_NEAR((t1 - t0) * p.cruiseSpeed(), 0.8 * 0.08, 1e-12);
}

TEST(ScurveProfile, RampFractionOneHalfHasNoPlateau)
{
  const ScurveProfile p(0.05, 0.02, 0.5);
  double t0, t1;
  p.plateauInterval(t0, t1);
  EXPECT_NEAR(t1 - t0, 0.0, 1e-15);
  EXPECT_NEAR(p.duration(), 2.0 * 0.05 / 0.02, 1e-12);
  EXPECT_NEAR(p.sd(0.5 * p.duration()), 0.02, 1e-12);   // pico = v_max
}

// ── Trayectoria de incision ─────────────────────────────────────────────────
TEST(IncisionTrajectory, PhasesHaveExpectedGeometry)
{
  const IncisionParams p = defaultParams();
  IncisionTrajectory traj(p);
  ASSERT_EQ(traj.phases().size(), 5u);

  const double z_surface = p.surface_z;
  const double z_cut = p.surface_z - p.cut_depth;
  const double z_above = p.surface_z + p.approach_height;

  const auto & contact = traj.phase(IncisionPhaseId::CONTACT);
  EXPECT_NEAR(contact.p_start.z(), z_above, 1e-12);
  EXPECT_NEAR(contact.p_end.z(), z_surface, 1e-12);
  EXPECT_NEAR(contact.length, p.approach_height, 1e-12);

  const auto & pen = traj.phase(IncisionPhaseId::PENETRATION);
  EXPECT_NEAR(pen.p_start.z(), z_surface, 1e-12);
  EXPECT_NEAR(pen.p_end.z(), z_cut, 1e-12);
  EXPECT_NEAR(pen.length, p.cut_depth, 1e-12);

  const auto & cut = traj.phase(IncisionPhaseId::CUT);
  EXPECT_NEAR(cut.length, p.cut_length, 1e-12);
  EXPECT_NEAR(cut.p_start.y(), p.cut_center_y - 0.5 * p.cut_length, 1e-12);
  EXPECT_NEAR(cut.p_end.y(), p.cut_center_y + 0.5 * p.cut_length, 1e-12);
  EXPECT_NEAR(cut.p_start.x(), p.cut_x, 1e-12);
  EXPECT_NEAR(cut.p_end.x(), p.cut_x, 1e-12);

  const auto & wd = traj.phase(IncisionPhaseId::WITHDRAW);
  EXPECT_NEAR(wd.p_end.z(), z_above, 1e-12);
}

// CRITERIO DE ACEPTACION: "Feed rate constante dentro de +-2 % en el tramo cut".
TEST(IncisionTrajectory, FeedIsConstantWithinTwoPercentInCutPlateau)
{
  const IncisionParams p = defaultParams();
  IncisionTrajectory traj(p);
  const auto & cut = traj.phase(IncisionPhaseId::CUT);

  ASSERT_GT(cut.plateau_t1, cut.plateau_t0);
  double worst = 0.0;
  const int n = 500;
  for (int i = 0; i <= n; ++i) {
    const double t = cut.plateau_t0 +
      (cut.plateau_t1 - cut.plateau_t0) * static_cast<double>(i) / n;
    const double feed = traj.velocity(t).norm();
    worst = std::max(worst, std::abs(feed - p.v_cut) / p.v_cut);
  }
  EXPECT_LT(worst, 0.02) << "desviacion relativa maxima del feed: " << worst * 100 << " %";
  // De hecho la meseta es exacta salvo redondeo, no solo dentro del 2 %.
  EXPECT_LT(worst, 1e-9);
}

TEST(IncisionTrajectory, CutIsStraightAndAtConstantDepth)
{
  const IncisionParams p = defaultParams();
  IncisionTrajectory traj(p);
  const auto & cut = traj.phase(IncisionPhaseId::CUT);
  const double z_cut = p.surface_z - p.cut_depth;

  for (int i = 0; i <= 200; ++i) {
    const double t = cut.t_start + (cut.t_end - cut.t_start) * i / 200.0;
    const Eigen::Vector3d q = traj.position(t);
    EXPECT_NEAR(q.z(), z_cut, 1e-12) << "profundidad no constante en t=" << t;
    EXPECT_NEAR(q.x(), p.cut_x, 1e-12) << "el trazo se desvia en x en t=" << t;
    EXPECT_GE(q.y(), cut.p_start.y() - 1e-12);
    EXPECT_LE(q.y(), cut.p_end.y() + 1e-12);
  }
}

TEST(IncisionTrajectory, AnalyticDerivativesMatchFiniteDifferences)
{
  IncisionTrajectory traj(defaultParams());
  const double h = 1e-4;
  double max_v = 0.0, max_a = 0.0;

  // Se excluye una franja alrededor de las uniones de fase: alli la referencia
  // es continua C3 pero la diferencia central cruzaria un dwell.
  std::vector<double> boundaries;
  for (const auto & ph : traj.phases()) {
    boundaries.push_back(ph.t_start);
    boundaries.push_back(ph.t_end);
  }
  auto nearBoundary = [&](double t) {
      for (double b : boundaries) {
        if (std::abs(t - b) < 10 * h) {return true;}
      }
      return false;
    };

  for (double t = 10 * h; t < traj.endTime() - 10 * h; t += 0.005) {
    if (nearBoundary(t)) {continue;}
    const auto fd_v = fdDerivative([&](double x) {return traj.position(x);}, t, h);
    const auto fd_a = fdDerivative([&](double x) {return traj.velocity(x);}, t, h);
    max_v = std::max(max_v, (traj.velocity(t) - fd_v).cwiseAbs().maxCoeff());
    max_a = std::max(max_a, (traj.acceleration(t) - fd_a).cwiseAbs().maxCoeff());
  }
  EXPECT_LT(max_v, 1e-6) << "velocidad";
  EXPECT_LT(max_a, 1e-6) << "aceleracion";
}

// Todas las fases arrancan y terminan en reposo con a = jerk = 0: es lo que
// hace que la concatenacion mantenga continuidad C3.
TEST(IncisionTrajectory, PhaseBoundariesAreAtRest)
{
  IncisionTrajectory traj(defaultParams());
  for (const auto & ph : traj.phases()) {
    for (double t : {ph.t_start, ph.t_end}) {
      EXPECT_LT(traj.velocity(t).norm(), 1e-12) << toString(ph.id) << " t=" << t;
      EXPECT_LT(traj.acceleration(t).norm(), 1e-12) << toString(ph.id) << " t=" << t;
      EXPECT_LT(traj.jerk(t).norm(), 1e-12) << toString(ph.id) << " t=" << t;
    }
  }
}

TEST(IncisionTrajectory, PositionIsContinuousAcrossPhases)
{
  IncisionTrajectory traj(defaultParams());
  const auto & ph = traj.phases();
  for (std::size_t i = 0; i + 1 < ph.size(); ++i) {
    EXPECT_LT((ph[i].p_end - ph[i + 1].p_start).cwiseAbs().maxCoeff(), 1e-12)
      << "discontinuidad entre " << toString(ph[i].id) << " y "
      << toString(ph[i + 1].id);
  }
  // Y la trayectoria arranca exactamente en la pose de partida declarada.
  EXPECT_LT((traj.position(0.0) - defaultParams().start_pose).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(IncisionTrajectory, CutAxisXIsSupported)
{
  IncisionParams p = defaultParams();
  p.cut_axis = 'x';
  IncisionTrajectory traj(p);
  const auto & cut = traj.phase(IncisionPhaseId::CUT);
  EXPECT_NEAR(cut.p_start.x(), p.cut_x - 0.5 * p.cut_length, 1e-12);
  EXPECT_NEAR(cut.p_end.x(), p.cut_x + 0.5 * p.cut_length, 1e-12);
  EXPECT_NEAR(cut.p_start.y(), p.cut_center_y, 1e-12);
  EXPECT_NEAR(cut.length, p.cut_length, 1e-12);
}

TEST(IncisionTrajectory, RejectsInvalidParameters)
{
  IncisionParams p = defaultParams();
  p.cut_depth = 0.0;
  EXPECT_THROW(IncisionTrajectory{p}, std::invalid_argument);

  p = defaultParams();
  p.cut_axis = 'z';
  EXPECT_THROW(IncisionTrajectory{p}, std::invalid_argument);

  p = defaultParams();
  p.cut_length = -0.01;
  EXPECT_THROW(IncisionTrajectory{p}, std::invalid_argument);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
