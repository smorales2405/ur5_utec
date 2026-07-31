#ifndef UR5_DYN_CONTROL_TORQUE_CONTROL_NODE_BASE_HPP
#define UR5_DYN_CONTROL_TORQUE_CONTROL_NODE_BASE_HPP

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "ur5_dyn_control/cartesian_spline_trajectory.hpp"
#include "ur5_dyn_control/cartesian_trajectory.hpp"
#include "ur5_dyn_control/common.hpp"
#include "ur5_dyn_control/controller_switcher.hpp"
#include "ur5_dyn_control/csv_logger.hpp"
#include "ur5_dyn_control/incision_trajectory.hpp"
#include "ur5_dyn_control/joint_reference_generator.hpp"
#include "ur5_dyn_control/joint_reference_table.hpp"
#include "ur5_dyn_control/joint_sweep_generator.hpp"
#include "ur5_dyn_control/quintic_spline_trajectory.hpp"
#include "ur5_dyn_control/torque_command.hpp"
#include "ur5_dyn_control/ur5_dynamics.hpp"

namespace ur5_dyn_control
{

/**
 * Clase base de los nodos de control por torque del UR5e (FL, SMC, MRAC...).
 * Las subclases solo implementan computeTau() y declaran sus ganancias.
 *
 * Secuencia de arranque en simulacion (mundo PAUSADO, validada
 * empiricamente — el robot NUNCA cae):
 *  1. Constructor: parametros, modelo Pinocchio, tabla de referencias
 *     (IK offline; la sim esta pausada, sin prisa).
 *  2. PRE_HOLD: publica tau = g(q_init) "a ciegas" en cada tick (el
 *     controlador de esfuerzo tendra comando desde su primer update);
 *     pide switch_controller (activar [jsb, forward_effort_controller]) —
 *     la activacion queda PENDIENTE hasta el primer paso de fisica — y
 *     despausa el mundo.
 *  3. WAIT_STATE: espera /joint_states completo; captura q0 (~ q_init).
 *  4. HOLD_START: regulacion cerrada en q0 durante hold_start_duration.
 *  5. RAMP: transicion quintica q0 -> tabla[0].
 *  6. TRACK: sigue la tabla (indexada por tiempo de SIMULACION); CSV.
 *  7. HOLD_END: regulacion en el punto final (t_sim opcional).
 *
 * Reloj: timer de PARED a control_rate (funciona con la sim pausada y con
 * RTF < 1); las fases y la tabla usan tiempo de SIMULACION (use_sim_time).
 * En el robot real: use_sim_time=false y perform_unpause=false; el mismo
 * codigo corre con tiempo de pared y el switch activa/desactiva los
 * controladores del driver UR (forward_effort_controller vs JTC).
 *
 * Gravedad (compuerta G3): computeTau() devuelve SIEMPRE el torque fisico
 * completo (con gravedad). El parametro `gravity_in_command` decide que se
 * comanda: true en Gazebo (tal cual), false en el UR5e real (se resta g(q),
 * que el robot compensa internamente). Ver torque_command.hpp.
 */
class TorqueControlNodeBase : public rclcpp::Node
{
public:
  explicit TorqueControlNodeBase(const std::string & node_name);
  ~TorqueControlNodeBase() override;

protected:
  /// Ley de control de la subclase (dt: delta de tiempo de sim, >= 0).
  virtual Vector6d computeTau(const Vector6d & q, const Vector6d & dq,
                              const JointRef & ref, double dt) = 0;

  /// Prefijo del CSV (p.ej. "fl", "gravity_comp").
  virtual std::string csvPrefix() const = 0;

  /// Llamar al final del constructor de la subclase.
  void start();

  Ur5Dynamics & dyn() { return *dyn_; }
  const Vector6d & qInit() const { return q_init_; }

  /**
   * Torque que se COMANDA al hardware a partir del torque de la ley (G3):
   * politica de gravedad + saturacion. Es la unica ruta hacia publishTau(),
   * y es la que ejercita el test unitario de la compuerta G3.
   */
  Vector6d commandFromLaw(const Vector6d & tau_law, const Vector6d & q,
                          const Vector6d & dq);

  bool gravityInCommand() const { return gravity_in_command_; }

private:
  enum class State { PRE_HOLD, WAIT_STATE, HOLD_START, RAMP, TRACK, HOLD_END, DONE };

  void tick();
  /// Declara los parametros `incision.*` y construye la trayectoria de 5 fases.
  std::shared_ptr<CartesianTrajectory> buildIncisionTrajectory(
    const std::vector<double> & rpy, std::string & description);
  /// Declara los parametros `sweep.*` y construye la campana de excitacion.
  std::unique_ptr<JointReferenceTable> buildJointSweep(std::string & description);
  bool readJointStates(Vector6d & q, Vector6d & dq);
  JointRef rampReference(double t_ramp) const;
  /// Aplica commandFromLaw() y publica. Devuelve el torque comandado (para el CSV).
  Vector6d publishTau(const Vector6d & tau_law, const Vector6d & q,
                      const Vector6d & dq);
  void enterState(State s);
  const char * stateName(State s) const;

  // -- parametros --
  double control_rate_ = 500.0;
  Vector6d q_init_ = Vector6d::Zero();
  Vector6d tau_max_ = kTauMax;
  // G3: true en Gazebo (el torque comandado incluye g), false en el UR5e real
  // (el robot compensa la gravedad internamente -> hay que restarla).
  bool gravity_in_command_ = true;
  // FASE 2 — compensacion de friccion identificada (feedforward en el comando).
  FrictionCompensation friction_mode_ = FrictionCompensation::NONE;
  Vector6d friction_f_v_ = Vector6d::Zero();
  Vector6d friction_f_c_ = Vector6d::Zero();
  double friction_dq_eps_ = 1e-3;
  std::string command_topic_;
  std::string controller_manager_ns_;
  std::vector<std::string> activate_controllers_;
  std::vector<std::string> deactivate_controllers_;
  bool perform_switch_ = true;
  bool perform_unpause_ = true;
  std::string world_name_ = "default";
  double hold_start_duration_ = 1.0;
  double transition_duration_ = 3.0;
  double t_sim_limit_ = 0.0;      // 0 = ilimitado (medido desde TRACK)
  bool skip_trajectory_ = false;  // true: quedarse en HOLD_START (regulacion)
  double q_init_check_tol_ = 0.15;
  std::string csv_dir_;
  int test_num_ = 1;
  int csv_hold_decimation_ = 10;
  int hold_log_counter_ = 0;

  // -- infraestructura --
  std::shared_ptr<Ur5Dynamics> dyn_;
  std::unique_ptr<JointReferenceTable> ref_gen_;
  std::unique_ptr<ControllerSwitcher> switcher_;
  CsvLogger csv_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tau_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState::SharedPtr last_js_;

  // -- estado --
  State state_ = State::PRE_HOLD;
  bool switch_requested_ = false;
  bool check_pending_ = false;
  bool unpause_done_ = false;
  int pre_hold_ticks_ = 0;
  int request_tick_ = 0;
  Vector6d q0_ = Vector6d::Zero();          // pose medida al iniciar HOLD_START
  Vector6d tau_hold_blind_ = Vector6d::Zero();
  double t_state_start_ = 0.0;              // tiempo sim al entrar al estado
  double t_prev_ = -1.0;                    // tiempo sim del tick anterior
  std::size_t track_index_ = 0;
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_TORQUE_CONTROL_NODE_BASE_HPP
