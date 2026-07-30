#include "ur5_dyn_control/quintic_spline.hpp"

#include <algorithm>
#include <stdexcept>

namespace ur5_dyn_control
{

namespace
{

// Coeficientes polinomicos de las 6 funciones base de Hermite quintico:
// fila = funcion base (h00, h10, h20, h01, h11, h21), columna = potencia de tau.
//
//   h00 = 1 - 10t^3 + 15t^4 -  6t^5      h01 =      10t^3 - 15t^4 + 6t^5
//   h10 = t -  6t^3 +  8t^4 -  3t^5      h11 =     - 4t^3 +  7t^4 - 3t^5
//   h20 = t^2/2 - 3t^3/2 + 3t^4/2 - t^5/2   h21 = t^3/2 - t^4 + t^5/2
//
// Verificacion (valores y derivadas en los extremos):
//   tau=0: h00=1, h10'=1, h20''=1, el resto 0 en valor/1a/2a derivada
//   tau=1: h01=1, h11'=1, h21''=1, idem
constexpr double kBasis[6][6] = {
  {1.0, 0.0, 0.0, -10.0,  15.0, -6.0},   // h00
  {0.0, 1.0, 0.0,  -6.0,   8.0, -3.0},   // h10
  {0.0, 0.0, 0.5,  -1.5,   1.5, -0.5},   // h20
  {0.0, 0.0, 0.0,  10.0, -15.0,  6.0},   // h01
  {0.0, 0.0, 0.0,  -4.0,   7.0, -3.0},   // h11
  {0.0, 0.0, 0.0,   0.5,  -1.0,  0.5},   // h21
};

}  // namespace

std::array<double, 6> QuinticSpline3d::hermiteBasis(double tau, int deriv)
{
  std::array<double, 6> out{};
  if (deriv < 0 || deriv > kMaxDeriv) {
    return out;   // derivadas de orden > 5 de un quintico son nulas
  }
  for (int b = 0; b < 6; ++b) {
    double acc = 0.0;
    // d^deriv/dtau^deriv de  sum_k c_k tau^k  =  sum_{k>=deriv} c_k k!/(k-deriv)! tau^(k-deriv)
    for (int k = deriv; k <= 5; ++k) {
      double falling = 1.0;
      for (int j = 0; j < deriv; ++j) {
        falling *= static_cast<double>(k - j);
      }
      double power = 1.0;
      for (int j = 0; j < k - deriv; ++j) {
        power *= tau;
      }
      acc += kBasis[b][k] * falling * power;
    }
    out[b] = acc;
  }
  return out;
}

QuinticSpline3d::QuinticSpline3d(const std::vector<Eigen::Vector3d> & waypoints,
                                 const std::vector<double> & knots,
                                 QuinticBoundary bc)
: u_(knots)
{
  const std::size_t np = waypoints.size();
  if (np < 2 || np != knots.size()) {
    throw std::invalid_argument(
      "QuinticSpline3d: se requieren >= 2 waypoints y knots del mismo tamano");
  }
  for (std::size_t i = 1; i < knots.size(); ++i) {
    if (knots[i] <= knots[i - 1]) {
      throw std::invalid_argument(
        "QuinticSpline3d: los knots deben ser estrictamente crecientes");
    }
  }

  const int N = static_cast<int>(np) - 1;          // numero de segmentos
  std::vector<double> h(N);
  for (int i = 0; i < N; ++i) {
    h[i] = u_[i + 1] - u_[i];
  }

  for (int axis = 0; axis < 3; ++axis) {
    y_[axis].resize(np);
    for (std::size_t i = 0; i < np; ++i) {
      y_[axis][i] = waypoints[i][axis];
    }
    v_[axis].assign(np, 0.0);
    a_[axis].assign(np, 0.0);
  }

  // ── Condiciones de contorno ───────────────────────────────────────────────
  // CLAMPED_REST deja v = a = 0 (ya inicializados).
  // CHORD_TANGENT fija la 1a derivada a la pendiente de la cuerda extrema, con
  // lo que |p'| no se anula y la parametrizacion es regular.
  if (bc == QuinticBoundary::CHORD_TANGENT) {
    for (int axis = 0; axis < 3; ++axis) {
      v_[axis][0] = (y_[axis][1] - y_[axis][0]) / h[0];
      v_[axis][N] = (y_[axis][N] - y_[axis][N - 1]) / h[N - 1];
    }
  }

  if (N == 1) {
    // Un solo tramo, sin nudos interiores: el Hermite quintico ya esta
    // determinado por las condiciones de contorno.
    //  - CLAMPED_REST : p = y0 + (y1-y0)(10t^3-15t^4+6t^5)  -> recta con perfil
    //                   suave (los tres ejes comparten tau).
    //  - CHORD_TANGENT: p = y0 + (y1-y0) t                  -> recta LINEAL,
    //                   de rapidez constante: la geometria de las fases rectas
    //                   de la incision (contact, penetration, cut, withdraw).
    return;
  }

  // ── Sistema lineal: continuidad C3 y C4 en los N-1 nudos interiores ────────
  // Incognitas: v_1..v_{N-1}, a_1..a_{N-1}  ->  2(N-1).
  // Indice de v_i -> (i-1);  indice de a_i -> (N-1) + (i-1).
  const int n_unk = 2 * (N - 1);
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_unk, n_unk);
  Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(n_unk, 3);

  // Aportacion de un extremo de segmento a la derivada d-esima respecto de u:
  //   d^d p/du^d = (1/h^d) * sum_b  basis_b^{(d)}(tau) * coef_b
  // con coef = [y_i, h*v_i, h^2*a_i, y_{i+1}, h*v_{i+1}, h^2*a_{i+1}].
  auto addSegmentTerm =
    [&](int row, int seg, double tau, int deriv, double sign) {
      const auto B = hermiteBasis(tau, deriv);
      double inv_hd = 1.0;
      for (int j = 0; j < deriv; ++j) {inv_hd /= h[seg];}

      const int i0 = seg, i1 = seg + 1;
      // Terminos en v/a: incognitas si el nudo es interior; si es un nudo
      // extremo, v y a ya estan fijados por las condiciones de contorno y
      // pasan al lado derecho.
      const struct { int node; int slot; int order; } terms[] = {
        {i0, 1, 1}, {i0, 2, 2}, {i1, 4, 1}, {i1, 5, 2},
      };
      for (const auto & tm : terms) {
        double hp = 1.0;
        for (int j = 0; j < tm.order; ++j) {hp *= h[seg];}
        const double coef = sign * inv_hd * B[tm.slot] * hp;
        if (tm.node == 0 || tm.node == N) {
          for (int axis = 0; axis < 3; ++axis) {
            const double known =
              (tm.order == 1) ? v_[axis][tm.node] : a_[axis][tm.node];
            rhs(row, axis) -= coef * known;
          }
        } else {
          const int col = (tm.order == 1) ? (tm.node - 1) : ((N - 1) + tm.node - 1);
          A(row, col) += coef;
        }
      }
      // Terminos en y (conocidos) -> lado derecho, con signo cambiado.
      for (int axis = 0; axis < 3; ++axis) {
        rhs(row, axis) -= sign * inv_hd * (B[0] * y_[axis][i0] + B[3] * y_[axis][i1]);
      }
    };

  for (int i = 1; i < N; ++i) {
    // Nudo interior i: fin del segmento i-1 (tau=1) y comienzo del segmento i (tau=0).
    // C3:  p'''_{i-1}(1) - p'''_i(0) = 0        -> fila (i-1)
    // C4:  p''''_{i-1}(1) - p''''_i(0) = 0      -> fila (N-1)+(i-1)
    addSegmentTerm(i - 1,           i - 1, 1.0, 3, +1.0);
    addSegmentTerm(i - 1,           i,     0.0, 3, -1.0);
    addSegmentTerm((N - 1) + i - 1, i - 1, 1.0, 4, +1.0);
    addSegmentTerm((N - 1) + i - 1, i,     0.0, 4, -1.0);
  }

  const Eigen::MatrixXd sol = A.fullPivLu().solve(rhs);
  if (!sol.allFinite()) {
    throw std::runtime_error("QuinticSpline3d: el sistema de continuidad C3/C4 es singular");
  }
  for (int axis = 0; axis < 3; ++axis) {
    for (int i = 1; i < N; ++i) {
      v_[axis][i] = sol(i - 1, axis);
      a_[axis][i] = sol((N - 1) + i - 1, axis);
    }
  }
}

QuinticSpline3d::QuinticSpline3d(const std::vector<Eigen::Vector3d> & waypoints)
: QuinticSpline3d(waypoints,
                  [&waypoints] {
                    // Nudos por longitud de cuerda acumulada, normalizados a
                    // [0,1]: la parametrizacion resultante es ~arco, de modo
                    // que |p'(u)| se mantiene lejos de cero.
                    if (waypoints.size() < 2) {
                      throw std::invalid_argument(
                        "QuinticSpline3d: se requieren >= 2 waypoints");
                    }
                    std::vector<double> u(waypoints.size(), 0.0);
                    for (std::size_t i = 1; i < waypoints.size(); ++i) {
                      const double d = (waypoints[i] - waypoints[i - 1]).norm();
                      if (!(d > 0.0)) {
                        throw std::invalid_argument(
                          "QuinticSpline3d: waypoints consecutivos coincidentes");
                      }
                      u[i] = u[i - 1] + d;
                    }
                    const double total = u.back();
                    for (auto & ui : u) {ui /= total;}
                    return u;
                  }(),
                  QuinticBoundary::CHORD_TANGENT)
{
}

int QuinticSpline3d::segmentIndex(double u) const
{
  const auto it = std::upper_bound(u_.begin(), u_.end(), u);
  int i = static_cast<int>(it - u_.begin()) - 1;
  return std::clamp(i, 0, static_cast<int>(u_.size()) - 2);
}

Eigen::Vector3d QuinticSpline3d::eval(double u, int deriv) const
{
  if (deriv < 0) {
    throw std::invalid_argument("QuinticSpline3d::eval: deriv debe ser >= 0");
  }
  if (deriv > kMaxDeriv) {
    return Eigen::Vector3d::Zero();
  }
  // ESTRICTAMENTE fuera del dominio la referencia se congela en el extremo.
  // En los extremos mismos se evalua el polinomio: con CLAMPED_REST eso da
  // v = a = 0 por construccion, y con CHORD_TANGENT da la tangente de la
  // cuerda — que es justo lo que la reparametrizacion por arco necesita.
  if (u < u_.front() || u > u_.back()) {
    if (deriv > 0) {
      return Eigen::Vector3d::Zero();
    }
    const std::size_t k = (u < u_.front()) ? 0 : (u_.size() - 1);
    return Eigen::Vector3d(y_[0][k], y_[1][k], y_[2][k]);
  }

  const int i = segmentIndex(u);
  const double h = u_[i + 1] - u_[i];
  const double tau = (u - u_[i]) / h;
  const auto B = hermiteBasis(tau, deriv);

  double inv_hd = 1.0;
  for (int j = 0; j < deriv; ++j) {inv_hd /= h;}

  Eigen::Vector3d out;
  for (int axis = 0; axis < 3; ++axis) {
    const double val =
      B[0] * y_[axis][i]     + B[1] * h * v_[axis][i]     + B[2] * h * h * a_[axis][i] +
      B[3] * y_[axis][i + 1] + B[4] * h * v_[axis][i + 1] + B[5] * h * h * a_[axis][i + 1];
    out[axis] = inv_hd * val;
  }
  return out;
}

}  // namespace ur5_dyn_control
