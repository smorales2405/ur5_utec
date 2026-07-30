#ifndef UR5_DYN_CONTROL_ARC_LENGTH_HPP
#define UR5_DYN_CONTROL_ARC_LENGTH_HPP

#include <vector>

#include "ur5_dyn_control/quintic_spline.hpp"

namespace ur5_dyn_control
{

/**
 * Reparametrizacion por LONGITUD DE ARCO de una curva p(u) (FASE 1).
 *
 * Separa GEOMETRIA de TEMPORIZACION: la geometria es p(u); esta clase da la
 * correspondencia s <-> u, y un perfil temporal independiente aporta s(t). El
 * resultado p(t) = p(u(s(t))) tiene velocidad de avance |dp/dt| = ds/dt, es
 * decir CONSTANTE cuando el perfil temporal esta en su meseta — que es el
 * requisito del tramo de corte.
 *
 *   s(u) = INT_{u0}^{u} |p'(w)| dw       (Gauss-Legendre, subdividiendo por
 *                                         tramos del spline: el integrando es
 *                                         suave dentro de cada tramo)
 *   u(s) : inversa, por biseccion sobre la tabla + refinamiento de Newton
 *          (s'(u) = |p'(u)| > 0 => s(u) es estrictamente creciente)
 *
 * Ademas expone las derivadas de u respecto de s, necesarias para propagar
 * velocidad, aceleracion y jerk por la regla de la cadena:
 *
 *   g(u)  = |p'(u)|
 *   du/ds       =  1/g
 *   d2u/ds2     = -g_u / g^3
 *   d3u/ds3     = (3 g_u^2 - g g_uu) / g^5
 */
class ArcLength
{
public:
  /// @param spline    geometria p(u)
  /// @param n_gauss   nodos de Gauss-Legendre por sub-tramo
  /// @param n_samples muestras de la tabla s(u) por tramo del spline
  ///
  /// Con los valores por defecto el error absoluto de s(u) frente a una
  /// cuadratura de referencia queda por debajo de 1e-9 m — cuatro ordenes de
  /// magnitud por debajo de la repetibilidad nominal del UR5e (+-0.03 mm). En
  /// una recta el error es exactamente cero (s(u) es lineal). Ademas, el error
  /// de la tabla NO afecta a la velocidad de avance: |dp/dt| = |p'|/|p'| * ds/dt
  /// = ds/dt exactamente, porque la regla de la cadena usa 1/|p'(u)| analitico.
  ArcLength(const QuinticSpline3d & spline, int n_gauss = 8, int n_samples = 256);

  double totalLength() const { return s_.back(); }

  /// Longitud de arco acumulada desde el knot inicial hasta u.
  double sOfU(double u) const;

  /// Inversa: parametro u tal que sOfU(u) = s (s se satura a [0, L]).
  double uOfS(double s) const;

  /// |p'(u)|, su 1a y 2a derivada respecto de u.  (g, g_u, g_uu)
  void speedDerivatives(double u, double & g, double & g_u, double & g_uu) const;

  /// du/ds, d2u/ds2, d3u/ds3 evaluadas en el u dado.
  void uDerivatives(double u, double & du_ds, double & d2u_ds2, double & d3u_ds3) const;

private:
  const QuinticSpline3d * spline_;
  std::vector<double> u_;   // malla de parametros (creciente)
  std::vector<double> s_;   // longitud acumulada en cada u_[i]
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_ARC_LENGTH_HPP
