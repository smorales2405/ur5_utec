#ifndef UR5_DYN_CONTROL_QUINTIC_SPLINE_HPP
#define UR5_DYN_CONTROL_QUINTIC_SPLINE_HPP

#include <array>
#include <vector>

#include <Eigen/Dense>

namespace ur5_dyn_control
{

/**
 * Spline QUINTICO 3D sobre un parametro generico u (tiempo o parametro de
 * geometria), con nudos NO uniformes y derivadas ANALITICAS.
 *
 * Formulacion (Hermite quintico por tramo). En el segmento [u_i, u_{i+1}] con
 * h = u_{i+1} - u_i y tau = (u - u_i)/h:
 *
 *   p(tau) = h00 y_i + h10 h v_i + h20 h^2 a_i
 *          + h01 y_{i+1} + h11 h v_{i+1} + h21 h^2 a_{i+1}
 *
 * Las incognitas son las derivadas primera (v_i) y segunda (a_i) en los nudos.
 * Se determinan imponiendo:
 *   - continuidad C3 y C4 en los N-1 nudos INTERIORES  -> 2(N-1) ecuaciones
 *   - condiciones de contorno v_0 = v_N = a_0 = a_N = 0 -> el resto
 * Como el Hermite ya garantiza C0, C1 y C2 por construccion, el spline
 * resultante es C4 (por tanto tambien C3: jerk CONTINUO, que es el criterio
 * que el spline cubico NO cumple).
 *
 * El sistema lineal es de tamano 2(N-1); con el numero de waypoints que maneja
 * este paquete (unidades) se resuelve con una LU densa de Eigen — no merece la
 * pena un solver bandeado.
 *
 * Fuera de [u_0, u_N] se satura al extremo; alli v = a = jerk = 0 por las
 * condiciones de contorno.
 */
/**
 * Condiciones de contorno del spline quintico.
 *
 * La eleccion NO es cosmetica: determina si la parametrizacion es REGULAR
 * (|p'| > 0 en todo el dominio), que es lo que exige la reparametrizacion por
 * longitud de arco (du/ds = 1/|p'|).
 */
enum class QuinticBoundary
{
  /// v = a = 0 en ambos extremos. Es lo que pide el plan para una trayectoria
  /// parametrizada en TIEMPO: arranca y para en reposo, sin escalon de par.
  /// Como efecto colateral |p'| = 0 en los extremos, asi que NO sirve como
  /// parametrizacion de geometria para el calculo de arco.
  CLAMPED_REST,

  /// v = tangente de la cuerda en cada extremo, a = 0. Parametrizacion
  /// REGULAR, apta para geometria p(u). Con dos waypoints degenera en la
  /// interpolacion LINEAL p(u) = p0 + u (p1 - p0), de rapidez constante.
  CHORD_TANGENT,
};

class QuinticSpline3d
{
public:
  static constexpr int kMaxDeriv = 5;

  /// @param waypoints  >= 2 puntos.
  /// @param knots      mismo tamano, estrictamente crecientes.
  /// @param bc         condiciones de contorno (ver QuinticBoundary).
  QuinticSpline3d(const std::vector<Eigen::Vector3d> & waypoints,
                  const std::vector<double> & knots,
                  QuinticBoundary bc = QuinticBoundary::CLAMPED_REST);

  /// Constructor de conveniencia para GEOMETRIA: nudos por longitud de cuerda
  /// acumulada normalizada a [0, 1] y contorno CHORD_TANGENT.
  explicit QuinticSpline3d(const std::vector<Eigen::Vector3d> & waypoints);

  double startKnot() const { return u_.front(); }
  double endKnot() const { return u_.back(); }

  /// Derivada de orden `deriv` (0..5) respecto de u, evaluada en u.
  Eigen::Vector3d eval(double u, int deriv = 0) const;

  const std::vector<double> & knots() const { return u_; }
  std::size_t numSegments() const { return u_.size() - 1; }

  /// Valores de las 6 funciones base de Hermite quintico (o su derivada
  /// `deriv`-esima respecto de tau) en tau. Publico para los tests.
  static std::array<double, 6> hermiteBasis(double tau, int deriv);

private:
  int segmentIndex(double u) const;

  std::vector<double> u_;                       // nudos (N+1)
  std::array<std::vector<double>, 3> y_;        // valores por eje (N+1)
  std::array<std::vector<double>, 3> v_;        // 1a derivada en los nudos
  std::array<std::vector<double>, 3> a_;        // 2a derivada en los nudos
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_QUINTIC_SPLINE_HPP
