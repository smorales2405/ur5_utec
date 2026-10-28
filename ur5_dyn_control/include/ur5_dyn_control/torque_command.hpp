#ifndef UR5_DYN_CONTROL_TORQUE_COMMAND_HPP
#define UR5_DYN_CONTROL_TORQUE_COMMAND_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <Eigen/Eigenvalues>

#include "ur5_dyn_control/common.hpp"

namespace ur5_dyn_control
{

/**
 * Conversion LEY DE CONTROL -> COMANDO DE HARDWARE (compuerta G3).
 *
 * Funciones puras (sin ROS, sin estado) para que la regla sea testeable
 * unitariamente: son la unica ruta por la que TorqueControlNodeBase publica
 * torque, de modo que el test cubre exactamente el codigo de produccion.
 *
 * MOTIVO (G3). Las leyes implementadas devuelven el torque FISICO completo de
 * la articulacion, con la gravedad incluida (p.ej. FL: M(q)v + n(q,q̇), donde
 * n = C q̇ + g). En Gazebo eso es correcto: el plugin aplica el torque tal cual
 * y la gravedad del mundo actua sobre el modelo.
 *
 * En el UR5e REAL no: el `forward_effort_controller` del driver comanda via la
 * funcion URScript `direct_torque(...)`, y el robot **compensa la gravedad
 * internamente**, asi que el torque comandado NO debe incluirla (de lo
 * contrario se compensa dos veces y el brazo se va hacia arriba).
 *   Fuente: Universal_Robots_ROS2_Driver,
 *   ur_robot_driver/doc/usage/force_torque_control.rst (seccion
 *   forward_effort_controller): "The robot automatically compensates for
 *   gravity, so the provided target torques should not include gravity
 *   compensation."
 *
 * Por eso la ley de control NO se bifurca por entorno (esta prohibido): lo
 * unico que cambia entre simulacion y robot real es el parametro
 * `gravity_in_command`.
 */

/// Politica de gravedad (G3).
///   gravity_in_command = true  (Gazebo)    -> tau_cmd = tau_ley
///   gravity_in_command = false (UR5e real) -> tau_cmd = tau_ley - g(q)
inline Vector6d applyGravityPolicy(const Vector6d & tau_law,
                                   const Vector6d & g_q,
                                   bool gravity_in_command)
{
  return gravity_in_command ? tau_law : Vector6d(tau_law - g_q);
}

/**
 * INVERSA de la politica de gravedad: par FISICO que entrega la junta a partir
 * del comandado.
 *
 *   gravity_in_command = true  (Gazebo)    -> tau_phys = tau_cmd
 *   gravity_in_command = false (UR5e real) -> tau_phys = tau_cmd + g(q)
 *
 * Existe porque la IDENTIFICACION DE FRICCION compara el par medido contra
 * `rnea(q,dq,ddq)`, que incluye gravedad. En el robot real el par comandado NO
 * la incluye (la pone el robot), asi que restarle el modelo dejaria un sesgo de
 * exactamente `g(q)` que el ajuste atribuiria a friccion de Coulomb: buen R² y
 * coeficientes inventados. Se aplica sobre el par YA saturado, porque si el
 * actuador quedo recortado el par entregado es el recortado.
 */
inline Vector6d physicalTorque(const Vector6d & tau_cmd,
                               const Vector6d & g_q,
                               bool gravity_in_command)
{
  return gravity_in_command ? tau_cmd : Vector6d(tau_cmd + g_q);
}

/// Saturacion simetrica componente a componente: tau_i -> [-tau_max_i, tau_max_i].
inline Vector6d saturate(const Vector6d & tau, const Vector6d & tau_max)
{
  return tau.cwiseMax(-tau_max).cwiseMin(tau_max);
}

/**
 * Compensacion de friccion articular (FASE 2).
 *
 * La planta cumple  M q̈ + C q̇ + g + tau_f(q̇) = tau_cmd, con la friccion
 * OPONIENDOSE al movimiento. Para que la ley de control vea una planta sin
 * friccion hay que SUMAR tau_f al comando:
 *
 *     tau_f(q̇) = F_v · q̇ + F_c · sgn(q̇)
 *
 * El signo se implementa con tanh(q̇/eps), no con sgn: un escalon discontinuo
 * en q̇ = 0 provoca ciclos limite (el comando salta ±F_c cada vez que el ruido
 * de velocidad cruza cero). Con eps pequeno frente a las velocidades de trabajo
 * la aproximacion es indistinguible en regimen y bien portada en el cruce.
 *
 * Consecuencia fisica: cerca de q̇ = 0 la compensacion tiende a cero, asi que
 * NO cancela la friccion estatica. Eso es correcto — un modelo dependiente de
 * la velocidad no puede hacerlo — y hay que tenerlo en cuenta al interpretar el
 * error de regulacion en reposo.
 *
 * En el UR5e real el robot ya aplica su propia compensacion interna
 * (friction_model_controller, compuerta G4), asi que lo que se identifica y se
 * compensa aqui es la friccion RESIDUAL a ese ajuste.
 */
enum class FrictionCompensation
{
  NONE,              ///< sin compensacion
  VISCOUS,           ///< solo F_v · q̇
  VISCOUS_COULOMB,   ///< F_v · q̇ + F_c · tanh(q̇/eps)
};

inline Vector6d frictionFeedforward(const Vector6d & dq,
                                    const Vector6d & f_v,
                                    const Vector6d & f_c,
                                    FrictionCompensation mode,
                                    double dq_eps)
{
  switch (mode) {
    case FrictionCompensation::NONE:
      return Vector6d::Zero();
    case FrictionCompensation::VISCOUS:
      return f_v.cwiseProduct(dq);
    case FrictionCompensation::VISCOUS_COULOMB: {
        Vector6d smooth_sign;
        for (int i = 0; i < 6; ++i) {
          smooth_sign[i] = std::tanh(dq[i] / dq_eps);
        }
        return f_v.cwiseProduct(dq) + f_c.cwiseProduct(smooth_sign);
      }
  }
  return Vector6d::Zero();
}

/**
 * Limite de TASA de variacion del comando (FASE 3).
 *
 * La saturacion acota |tau|, pero no cuan rapido puede cambiar. Un salto
 * instantaneo de decenas de N·m entre dos ciclos excita la estructura y, en el
 * robot real, dispara las protecciones del controlador. El limite de tasa acota
 * |dtau/dt| a `rate_max` [N·m/s] por junta.
 *
 * Importa especialmente para el SMC de la FASE 5: el termino discontinuo
 * sgn(s) conmuta a la frecuencia del lazo, y este limite es lo que separa el
 * chattering "del papel" del que realmente llega al actuador. Se registra en el
 * CSV para poder reportar cuando actua.
 *
 * rate_max <= 0 desactiva el limite.
 */
inline Vector6d rateLimit(const Vector6d & tau, const Vector6d & tau_prev,
                          const Vector6d & rate_max, double dt)
{
  if (!(dt > 0.0)) {return tau;}
  Vector6d out = tau;
  for (int i = 0; i < 6; ++i) {
    if (!(rate_max[i] > 0.0)) {continue;}
    const double max_step = rate_max[i] * dt;
    const double delta = tau[i] - tau_prev[i];
    if (delta > max_step) {
      out[i] = tau_prev[i] + max_step;
    } else if (delta < -max_step) {
      out[i] = tau_prev[i] - max_step;
    }
  }
  return out;
}

/// Marca por junta de que el comando quedo recortado (por saturacion o por
/// limite de tasa). Es la senal que consume el hook onSaturation() para
/// congelar integradores (anti-windup) en las subclases.
struct SaturationFlags
{
  std::array<bool, 6> saturated{};   ///< |tau| llego a tau_max
  std::array<bool, 6> rate_limited{};

  bool any() const
  {
    for (int i = 0; i < 6; ++i) {
      if (saturated[i] || rate_limited[i]) {return true;}
    }
    return false;
  }
};

/// Composicion completa: lo que realmente se publica al controlador.
/// El ORDEN importa: primero la politica de gravedad, despues la saturacion,
/// porque tau_max limita lo que se ENVIA al hardware. En el robot real el
/// comando es un feedforward que se suma a la compensacion interna, asi que
/// saturar antes de restar g(q) daria un limite sobre una magnitud que nunca
/// se transmite.
inline Vector6d torqueCommand(const Vector6d & tau_law,
                              const Vector6d & g_q,
                              bool gravity_in_command,
                              const Vector6d & tau_max)
{
  return saturate(applyGravityPolicy(tau_law, g_q, gravity_in_command), tau_max);
}

/**
 * B4.1: radio espectral del lazo PD de UNA junta del CT, discretizado.
 *
 * Con el modelo exacto el CT desacopla el brazo en seis dobles integradores,
 * e'' = u, u = -kp e - kd e'. El lazo real es discreto: retencion de orden cero
 * a dt = 1/control_rate y `delay_steps` pasos de retardo entre medir y aplicar
 * (1 en Gazebo; 5 en el UR5e real, medido en B7.0 con los escalones del
 * feedforward de Coulomb: analyze_pipeline_delay.py). Estado
 * [e_k, e'_k, u_{k-1}, ..., u_{k-d}]. rho < 1 <=> estable.
 *
 * Esto es el limite IDEAL. El que manda en el UR5e real es otro, mas bajo: la
 * G <= 120 que fijaron la cuantizacion del encoder y el modo estructural de
 * 25-35 Hz (docs/09_real_bringup.md). Por eso el banner da los dos, con el
 * retardo del parametro `pipeline_delay_steps` (1 en Gazebo, 5 en el real: el
 * margen del CT nominal baja de x25 a x7; plan V15 y R10).
 */
inline double pdDiscreteSpectralRadius(double kp, double kd, double dt,
                                       int delay_steps = 1);

/**
 * R10: lo mismo con accion integral, e'' = u, u = -ki*sigma - kp e - kd e',
 * sigma' = e (el LQI). Estado [sigma, e, e', u_{k-1}, ..., u_{k-d}]. Con
 * ki = 0 se quita sigma (seria un modo desacoplado en 1) y coincide con
 * pdDiscreteSpectralRadius. El evaluador de Python (closed_loop.py) tiene el
 * mismo calculo, y los dos tests fijan los mismos numeros.
 */
inline double pidDiscreteSpectralRadius(double ki, double kp, double kd, double dt,
                                        int delay_steps = 1)
{
  const int d = std::max(0, delay_steps);
  const int o = (ki != 0.0) ? 1 : 0;     // desplazamiento por sigma
  const int n = o + 2 + d;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  const int e = o, de = o + 1;
  // e, e' integran el mando que se aplica AHORA, que es el de hace d pasos.
  const int u_col = (d == 0) ? -1 : o + 2 + d - 1;
  auto add_u = [&](int row, double gain) {
      if (u_col >= 0) {
        A(row, u_col) += gain;
      } else {          // sin retardo: u_k = -ki sigma_k - kp e_k - kd e'_k
        if (o) {A(row, 0) += -gain * ki;}
        A(row, e) += -gain * kp;
        A(row, de) += -gain * kd;
      }
    };
  if (o) {
    A(0, 0) = 1.0;
    A(0, e) = dt;       // sigma += e dt
  }
  A(e, e) = 1.0;
  A(e, de) = dt;
  add_u(e, 0.5 * dt * dt);
  A(de, de) = 1.0;
  add_u(de, dt);
  if (d > 0) {
    const int u0 = o + 2;
    if (o) {A(u0, 0) = -ki;}  // u_k, calculado con la medida de ahora
    A(u0, e) = -kp;
    A(u0, de) = -kd;
    for (int j = u0 + 1; j < n; ++j) {A(j, j - 1) = 1.0;}   // la cola del retardo
  }
  return Eigen::EigenSolver<Eigen::MatrixXd>(A, false).eigenvalues().cwiseAbs().maxCoeff();
}

inline double pdDiscreteSpectralRadius(double kp, double kd, double dt, int delay_steps)
{
  return pidDiscreteSpectralRadius(0.0, kp, kd, dt, delay_steps);
}

/**
 * B4.1 / R10: margen de ganancia discreto: el factor s por el que se pueden
 * multiplicar (ki, kp, kd) a la vez antes de que el lazo discreto se haga
 * inestable (rho = 1). Biseccion sobre [1, s_max]; 0 si ya es inestable,
 * infinito si sigue estable en s_max.
 */
inline double pidDiscreteGainMargin(double ki, double kp, double kd, double dt,
                                    int delay_steps = 1, double s_max = 1e4)
{
  auto rho = [&](double sc) {
      return pidDiscreteSpectralRadius(sc * ki, sc * kp, sc * kd, dt, delay_steps);
    };
  if (rho(1.0) >= 1.0) {return 0.0;}
  if (rho(s_max) < 1.0) {return std::numeric_limits<double>::infinity();}
  double lo = 1.0, hi = s_max;
  for (int it = 0; it < 60; ++it) {
    const double mid = std::sqrt(lo * hi);
    (rho(mid) < 1.0 ? lo : hi) = mid;
  }
  return lo;
}

inline double pdDiscreteGainMargin(double kp, double kd, double dt,
                                   int delay_steps = 1, double s_max = 1e4)
{
  return pidDiscreteGainMargin(0.0, kp, kd, dt, delay_steps, s_max);
}

/**
 * R10: el lazo COMPLETO de una ley lineal en el error, para el chequeo MIMO.
 *
 *   tau = -(K_I sigma + K_P e + K_D e'),   e'' = M^-1 tau,   sigma' = e
 *
 * con las K en par (N.m por rad, por rad/s, por rad.s) en un punto de la
 * referencia. Por junta (pid*) solo vale si M^-1 K es diagonal: es exacto en
 * el CT (K = M diag(k)) y en la semilla del LQI, pero no en el LQI con
 * `Q_mode: fixed` lejos de q_init (M^-1 K con acoplos dominantes y diagonales
 * negativas) ni del todo en el SMC (diag(K/phi) frente a M llena).
 */
struct LoopGains
{
  Matrix6d M = Matrix6d::Identity();
  Matrix6d KI = Matrix6d::Zero();
  Matrix6d KP = Matrix6d::Zero();
  Matrix6d KD = Matrix6d::Zero();
  bool integral = false;
};

/// Modo dominante del lazo discreto: modulo, frecuencia [Hz] y la junta con
/// mas peso en el error de posicion del autovector.
struct DiscreteMode
{
  double rho = 0.0;
  double freq_hz = 0.0;
  int joint = -1;
};

/**
 * R10: ZOH exacto de la cadena de integradores (sin el termino de Coriolis,
 * despreciable a las velocidades de la incision) a dt, y `delay_steps` pasos
 * entre medir y aplicar. Estado [sigma?, e, e', u_{k-1}, ..., u_{k-d}].
 * `scale` multiplica las tres K a la vez (para el margen de ganancia).
 */
inline DiscreteMode mimoDiscreteMode(const LoopGains & g, double dt, int delay_steps,
                                     double scale = 1.0)
{
  const int d = std::max(0, delay_steps);
  const int nb = g.integral ? 3 : 2;
  const int nx = 6 * nb;
  const int n = nx + 6 * d;
  const Matrix6d I = Matrix6d::Identity();
  const Matrix6d Minv = g.M.llt().solve(I);
  Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(nx, nx);
  Eigen::MatrixXd Gam = Eigen::MatrixXd::Zero(nx, 6);
  Eigen::MatrixXd K(6, nx);
  const int e = g.integral ? 6 : 0;          // bloque del error de posicion
  if (g.integral) {
    Phi.block(0, 0, 6, 6) = I;
    Phi.block(0, 6, 6, 6) = dt * I;
    Phi.block(0, 12, 6, 6) = 0.5 * dt * dt * I;
    Gam.block(0, 0, 6, 6) = dt * dt * dt / 6.0 * Minv;
    K << scale * g.KI, scale * g.KP, scale * g.KD;
  } else {
    K << scale * g.KP, scale * g.KD;
  }
  Phi.block(e, e, 6, 6) = I;
  Phi.block(e, e + 6, 6, 6) = dt * I;
  Phi.block(e + 6, e + 6, 6, 6) = I;
  Gam.block(e, 0, 6, 6) = 0.5 * dt * dt * Minv;
  Gam.block(e + 6, 0, 6, 6) = dt * Minv;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  if (d == 0) {
    A = Phi - Gam * K;
  } else {
    A.topLeftCorner(nx, nx) = Phi;
    A.block(0, n - 6, nx, 6) = Gam;          // se aplica u_{k-d}
    A.block(nx, 0, 6, nx) = -K;              // u_k, con la medida de ahora
    for (int j = 1; j < d; ++j) {A.block(nx + 6 * j, nx + 6 * (j - 1), 6, 6) = I;}
  }
  Eigen::EigenSolver<Eigen::MatrixXd> es(A, true);
  const Eigen::VectorXcd lam = es.eigenvalues();
  int k = 0;
  for (int i = 1; i < lam.size(); ++i) {
    if (std::abs(lam[i]) > std::abs(lam[k])) {k = i;}
  }
  DiscreteMode out;
  out.rho = std::abs(lam[k]);
  out.freq_hz = std::abs(std::arg(lam[k])) / (2.0 * M_PI * dt);
  const Eigen::VectorXcd v = es.eigenvectors().col(k).segment(e, 6);
  v.cwiseAbs().maxCoeff(&out.joint);
  return out;
}

/// R10: margen de ganancia MIMO: factor comun de (K_I, K_P, K_D) hasta rho = 1.
inline double mimoDiscreteGainMargin(const LoopGains & g, double dt, int delay_steps,
                                     double s_max = 1e4)
{
  auto rho = [&](double sc) {return mimoDiscreteMode(g, dt, delay_steps, sc).rho;};
  if (rho(1.0) >= 1.0) {return 0.0;}
  if (rho(s_max) < 1.0) {return std::numeric_limits<double>::infinity();}
  double lo = 1.0, hi = s_max;
  for (int it = 0; it < 50; ++it) {
    const double mid = std::sqrt(lo * hi);
    (rho(mid) < 1.0 ? lo : hi) = mid;
  }
  return lo;
}

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_TORQUE_COMMAND_HPP
