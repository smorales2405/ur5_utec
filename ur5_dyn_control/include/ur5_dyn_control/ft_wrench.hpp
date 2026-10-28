#ifndef UR5_DYN_CONTROL_FT_WRENCH_HPP
#define UR5_DYN_CONTROL_FT_WRENCH_HPP

#include <Eigen/Dense>

#include "ur5_dyn_control/common.hpp"

namespace ur5_dyn_control
{

/**
 * B1: wrench del sensor F/T integrado del UR5e, llevado a la PUNTA de la hoja y
 * expresado en `base_link` (supuesto A3: todas las metricas en base_link).
 *
 * El driver publica el wrench en `tool0_controller`: ejes del TCP configurado en
 * el pendant y momento respecto a ese punto. Con el TCP del pendant sin rotacion
 * (como se usa en este proyecto), sus ejes son los de tool0, que son los del
 * frame `gripper_tcp` del modelo (solo trasladado), asi que la rotacion es la de
 * la FK que el nodo ya calcula para el CSV.
 *
 *   f_b   = R f
 *   m_b   = R m                         (respecto al punto del pendant)
 *   r_b   = R [0, 0, ref_to_tip_z]      (de ese punto a la punta)
 *   m_tip = m_b - r_b x f_b
 *
 * `ref_to_tip_z` = tcp_offset_z (punta) - ft.ref_offset_z (punto del pendant, 0 si
 * el TCP del pendant esta en la brida). El SIGNO es el del driver, sin tocar: que
 * sea la fuerza que el entorno ejerce sobre la herramienta o la contraria se fija
 * en el cierre de B1 con un peso conocido colgado de la punta.
 *
 * @param R_base_tool  rotacion de tool0 (= gripper_tcp) en base_link
 * @param w_tool       [f; m] tal como llega del driver
 */
inline Vector6d wrenchAtTipInBase(const Eigen::Matrix3d & R_base_tool,
                                  const Vector6d & w_tool,
                                  double ref_to_tip_z)
{
  const Eigen::Vector3d f = R_base_tool * w_tool.head<3>();
  const Eigen::Vector3d m = R_base_tool * w_tool.tail<3>();
  const Eigen::Vector3d r = R_base_tool * Eigen::Vector3d(0.0, 0.0, ref_to_tip_z);
  Vector6d out;
  out.head<3>() = f;
  out.tail<3>() = m - r.cross(f);
  return out;
}

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_FT_WRENCH_HPP
