#ifndef UR5_DYN_CONTROL_GAUSS_LEGENDRE_HPP
#define UR5_DYN_CONTROL_GAUSS_LEGENDRE_HPP

#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ur5_dyn_control
{

/**
 * Cuadratura de Gauss-Legendre — port a C++ de
 * ur5_trajectory_optimization/numerical_integration.py::gauss_legendre (CU3).
 *
 * La version de Python obtiene nodos y pesos con
 * numpy.polynomial.legendre.leggauss; aqui se calculan con el algoritmo
 * clasico: los nodos son las raices de P_n, halladas por Newton-Raphson
 * partiendo de la aproximacion asintotica de Chebyshev, y los pesos salen de
 *
 *     w_i = 2 / ((1 - x_i^2) [P_n'(x_i)]^2).
 *
 * P_n y P_n' se evaluan con la recurrencia de Bonnet
 *     (n+1) P_{n+1}(x) = (2n+1) x P_n(x) - n P_{n-1}(x),
 *     (x^2 - 1) P_n'(x) = n (x P_n(x) - P_{n-1}(x)).
 *
 * Con n nodos la regla es exacta para polinomios de grado <= 2n-1, asi que
 * unos pocos nodos bastan para integrar |p'(u)| de un spline quintico tramo a
 * tramo (el integrando es una raiz cuadrada, no un polinomio, pero es suave y
 * la convergencia es espectral).
 */
class GaussLegendre
{
public:
  explicit GaussLegendre(int n)
  {
    if (n < 1) {
      throw std::invalid_argument("GaussLegendre: n debe ser >= 1");
    }
    x_.resize(n);
    w_.resize(n);

    for (int i = 0; i < n; ++i) {
      // Aproximacion inicial de la i-esima raiz de P_n (Chebyshev).
      double x = std::cos(M_PI * (static_cast<double>(i) + 0.75) /
                          (static_cast<double>(n) + 0.5));
      double dp = 0.0;
      for (int it = 0; it < 100; ++it) {
        // Recurrencia de Bonnet: p1 = P_n(x), p2 = P_{n-1}(x).
        double p1 = 1.0, p2 = 0.0;
        for (int k = 0; k < n; ++k) {
          const double p3 = p2;
          p2 = p1;
          p1 = ((2.0 * k + 1.0) * x * p2 - static_cast<double>(k) * p3) /
               static_cast<double>(k + 1);
        }
        dp = static_cast<double>(n) * (x * p1 - p2) / (x * x - 1.0);
        const double dx = p1 / dp;
        x -= dx;
        if (std::abs(dx) < 1e-15) {break;}
      }
      x_[i] = x;
      w_[i] = 2.0 / ((1.0 - x * x) * dp * dp);
    }
  }

  /// Integra f en [a, b] con n nodos: exactamente n evaluaciones del integrando.
  template<typename F>
  double integrate(F && f, double a, double b) const
  {
    const double half = 0.5 * (b - a);
    const double mid = 0.5 * (a + b);
    double acc = 0.0;
    for (std::size_t i = 0; i < x_.size(); ++i) {
      acc += w_[i] * f(half * x_[i] + mid);
    }
    return half * acc;
  }

  int numNodes() const { return static_cast<int>(x_.size()); }
  const std::vector<double> & nodes() const { return x_; }
  const std::vector<double> & weights() const { return w_; }

private:
  std::vector<double> x_;   // nodos en [-1, 1]
  std::vector<double> w_;   // pesos
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_GAUSS_LEGENDRE_HPP
