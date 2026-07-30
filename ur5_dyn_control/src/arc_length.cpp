#include "ur5_dyn_control/arc_length.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "ur5_dyn_control/gauss_legendre.hpp"

namespace ur5_dyn_control
{

ArcLength::ArcLength(const QuinticSpline3d & spline, int n_gauss, int n_samples)
: spline_(&spline)
{
  if (n_samples < 2) {
    throw std::invalid_argument("ArcLength: n_samples debe ser >= 2");
  }
  const GaussLegendre gl(n_gauss);
  const auto & knots = spline.knots();

  // Integrando: rapidez respecto del parametro.
  auto speed = [this](double u) {return spline_->eval(u, 1).norm();};

  u_.clear();
  s_.clear();
  u_.push_back(knots.front());
  s_.push_back(0.0);

  // Se integra tramo a tramo del spline: dentro de cada tramo p(u) es un
  // polinomio, asi que |p'(u)| es suave y Gauss-Legendre converge rapido. En
  // los nudos la 5a derivada salta, y subdividir ahi evita ese error.
  for (std::size_t k = 0; k + 1 < knots.size(); ++k) {
    const double du = (knots[k + 1] - knots[k]) / static_cast<double>(n_samples);
    for (int i = 0; i < n_samples; ++i) {
      const double a = knots[k] + static_cast<double>(i) * du;
      const double b = a + du;
      s_.push_back(s_.back() + gl.integrate(speed, a, b));
      u_.push_back(b);
    }
  }

  if (!(s_.back() > 0.0)) {
    throw std::runtime_error("ArcLength: la curva tiene longitud nula");
  }

  // La reparametrizacion exige |p'(u)| > 0 en TODO el dominio: du/ds = 1/|p'|.
  // Un spline con contorno CLAMPED_REST tiene |p'| = 0 en los extremos y NO
  // sirve como geometria; se detecta aqui en vez de propagar 1/0 silencioso.
  const double L = s_.back();
  const double du_span = knots.back() - knots.front();
  const double g_tol = 1e-6 * L / du_span;
  for (const double u : {knots.front(), 0.5 * (knots.front() + knots.back()), knots.back()}) {
    if (spline.eval(u, 1).norm() < g_tol) {
      throw std::runtime_error(
              "ArcLength: parametrizacion no regular (|p'(u)| ~ 0 en u=" +
              std::to_string(u) +
              "). Use QuinticBoundary::CHORD_TANGENT para la geometria.");
    }
  }
}

double ArcLength::sOfU(double u) const
{
  if (u <= u_.front()) {return 0.0;}
  if (u >= u_.back()) {return s_.back();}

  const auto it = std::upper_bound(u_.begin(), u_.end(), u);
  const std::size_t i = static_cast<std::size_t>(it - u_.begin()) - 1;
  // Interpolacion de Hermite cubica en el sub-intervalo usando s'(u) = |p'(u)|,
  // que se conoce exactamente: mucho mas preciso que interpolar linealmente.
  const double h = u_[i + 1] - u_[i];
  const double tau = (u - u_[i]) / h;
  const double m0 = spline_->eval(u_[i], 1).norm() * h;
  const double m1 = spline_->eval(u_[i + 1], 1).norm() * h;
  const double y0 = s_[i], y1 = s_[i + 1];
  const double t2 = tau * tau, t3 = t2 * tau;
  return (2 * t3 - 3 * t2 + 1) * y0 + (t3 - 2 * t2 + tau) * m0 +
         (-2 * t3 + 3 * t2) * y1 + (t3 - t2) * m1;
}

double ArcLength::uOfS(double s) const
{
  const double L = s_.back();
  if (s <= 0.0) {return u_.front();}
  if (s >= L) {return u_.back();}

  // 1) Localizar el sub-intervalo por biseccion sobre la tabla (s_ es creciente).
  const auto it = std::upper_bound(s_.begin(), s_.end(), s);
  const std::size_t i = static_cast<std::size_t>(it - s_.begin()) - 1;

  // 2) Semilla lineal dentro del sub-intervalo.
  const double ds = s_[i + 1] - s_[i];
  double u = (ds > 0.0)
    ? u_[i] + (s - s_[i]) / ds * (u_[i + 1] - u_[i])
    : u_[i];

  // 3) Refinamiento de Newton sobre  F(u) = sOfU(u) - s,  F'(u) = |p'(u)| > 0.
  //    Con salvaguarda: si Newton se sale del sub-intervalo, se bisecta.
  double lo = u_[i], hi = u_[i + 1];
  for (int it_n = 0; it_n < 50; ++it_n) {
    const double F = sOfU(u) - s;
    if (std::abs(F) < 1e-14 * std::max(1.0, L)) {break;}
    if (F > 0.0) {hi = u;} else {lo = u;}
    const double g = spline_->eval(u, 1).norm();
    double u_next = (g > 1e-12) ? (u - F / g) : (0.5 * (lo + hi));
    if (!(u_next > lo && u_next < hi)) {
      u_next = 0.5 * (lo + hi);
    }
    if (std::abs(u_next - u) < 1e-16) {
      u = u_next;
      break;
    }
    u = u_next;
  }
  return u;
}

void ArcLength::speedDerivatives(double u, double & g, double & g_u, double & g_uu) const
{
  const Eigen::Vector3d p1 = spline_->eval(u, 1);
  const Eigen::Vector3d p2 = spline_->eval(u, 2);
  const Eigen::Vector3d p3 = spline_->eval(u, 3);

  g = p1.norm();
  if (g < 1e-12) {
    // No deberia ocurrir: el constructor rechaza las parametrizaciones no
    // regulares. Se deja la salvaguarda para no propagar NaN si aparece un
    // punto de retroceso interior con una geometria futura.
    g = 1e-12;
    g_u = 0.0;
    g_uu = 0.0;
    return;
  }
  //   g   = sqrt(p1.p1)
  //   g_u = (p1.p2)/g
  //   g_uu= (p2.p2 + p1.p3)/g - (p1.p2)^2/g^3
  const double p1p2 = p1.dot(p2);
  g_u = p1p2 / g;
  g_uu = (p2.squaredNorm() + p1.dot(p3)) / g - (p1p2 * p1p2) / (g * g * g);
}

void ArcLength::uDerivatives(double u, double & du_ds, double & d2u_ds2,
                             double & d3u_ds3) const
{
  double g, g_u, g_uu;
  speedDerivatives(u, g, g_u, g_uu);
  const double g2 = g * g, g3 = g2 * g, g5 = g3 * g2;
  du_ds = 1.0 / g;
  d2u_ds2 = -g_u / g3;
  d3u_ds3 = (3.0 * g_u * g_u - g * g_uu) / g5;
}

}  // namespace ur5_dyn_control
