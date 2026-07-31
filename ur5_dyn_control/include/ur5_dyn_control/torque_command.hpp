#ifndef UR5_DYN_CONTROL_TORQUE_COMMAND_HPP
#define UR5_DYN_CONTROL_TORQUE_COMMAND_HPP

#include <cmath>

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

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_TORQUE_COMMAND_HPP
