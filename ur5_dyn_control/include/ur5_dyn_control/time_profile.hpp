#ifndef UR5_DYN_CONTROL_TIME_PROFILE_HPP
#define UR5_DYN_CONTROL_TIME_PROFILE_HPP

namespace ur5_dyn_control
{

/**
 * Perfil temporal s(t) de tipo S-CURVE con JERK ACOTADO (FASE 1).
 *
 * Recorre una longitud de arco L con velocidad de crucero v_max, en tres
 * tramos:
 *
 *   v(t)
 *   v_max |      ,---------------------.
 *         |     /                       \
 *       0 +----'-------------------------`----->  t
 *          <-Tr->      <---Tp--->     <-Tr->
 *          rampa        MESETA         rampa
 *
 * La rampa usa el quintico "smoothstep" S(x) = 10x^3 - 15x^4 + 6x^5 aplicado a
 * la VELOCIDAD, con lo que en los extremos de cada rampa se anulan a la vez la
 * aceleracion (S'(0)=S'(1)=0) y el jerk (S''(0)=S''(1)=0). Por tanto:
 *
 *  - el perfil arranca y termina con v = a = jerk = 0  ->  al concatenar fases
 *    la trayectoria completa mantiene continuidad C3 en las uniones;
 *  - en la MESETA la velocidad de avance es EXACTAMENTE v_max, que es el
 *    criterio de "feed constante" del tramo de corte.
 *
 * Reparto de distancias: cada rampa cubre 0.5*v_max*Tr (igual que una rampa
 * lineal). Con `ramp_fraction` phi in (0, 0.5]:
 *
 *   distancia de rampa = phi * L        (cada lado)
 *   Tr = 2*phi*L / v_max
 *   Tp = (1 - 2*phi) * L / v_max        (0 cuando phi = 0.5: sin meseta)
 *   T  = 2*Tr + Tp = (1 + 2*phi) * L / v_max
 */
class ScurveProfile
{
public:
  /// @param length        longitud de arco a recorrer (> 0)
  /// @param v_max         velocidad de crucero (> 0)
  /// @param ramp_fraction fraccion de L en cada rampa, en (0, 0.5]
  ScurveProfile(double length, double v_max, double ramp_fraction);

  double duration() const { return T_; }
  double length() const { return L_; }
  double cruiseSpeed() const { return v_; }

  /// Aceleracion maxima del perfil: v_max * max(S') / Tr = 1.875 * v_max / Tr.
  double peakAcceleration() const;

  double s(double t) const;      ///< longitud de arco recorrida
  double sd(double t) const;     ///< ds/dt   (velocidad de avance)
  double sdd(double t) const;    ///< d2s/dt2
  double sddd(double t) const;   ///< d3s/dt3

  /// Intervalo temporal de MESETA [t0, t1], donde sd(t) == v_max exactamente.
  /// Vacio (t0 == t1) si ramp_fraction == 0.5.
  void plateauInterval(double & t0, double & t1) const;

private:
  double L_ = 0.0;    // longitud total
  double v_ = 0.0;    // velocidad de crucero
  double Tr_ = 0.0;   // duracion de cada rampa
  double Tp_ = 0.0;   // duracion de la meseta
  double T_ = 0.0;    // duracion total
  double Lr_ = 0.0;   // longitud de cada rampa (= 0.5 * v * Tr)
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_TIME_PROFILE_HPP
