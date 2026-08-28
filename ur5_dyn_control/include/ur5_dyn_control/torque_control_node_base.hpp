#ifndef UR5_DYN_CONTROL_TORQUE_CONTROL_NODE_BASE_HPP
#define UR5_DYN_CONTROL_TORQUE_CONTROL_NODE_BASE_HPP

#include <limits>
#include <map>
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
   *
   * `dq_ref` es la velocidad DESEADA, y solo se usa si
   * `friction.dq_source` = "desired". Es opcional para no romper a ningun
   * llamante: donde no se pasa (los holds a ciegas) la velocidad deseada es
   * cero de todos modos, que es lo mismo que ve la ruta por velocidad medida.
   */
  Vector6d commandFromLaw(const Vector6d & tau_law, const Vector6d & q,
                          const Vector6d & dq,
                          const Vector6d * dq_ref = nullptr);

  bool gravityInCommand() const { return gravity_in_command_; }

  /**
   * Hook de ANTI-WINDUP (FASE 3). Se llama en cada tick DESPUES de calcular el
   * comando efectivo, con las marcas de que juntas quedaron recortadas por
   * saturacion o por limite de tasa.
   *
   * Las subclases con estado interno que integra (el termino v del
   * super-twisting en ASTSMC, o cualquier accion integral) deben CONGELAR esa
   * integracion en las juntas marcadas: si el actuador ya no puede entregar mas
   * par, seguir integrando el error solo acumula un windup que hay que
   * "descargar" despues, con sobre-disparo garantizado.
   *
   * Implementacion por defecto: nada (las leyes sin estado no la necesitan).
   */
  virtual void onSaturation(const SaturationFlags & /*flags*/) {}

  /// Variable de deslizamiento del controlador, para la columna `s` del CSV.
  /// Las leyes que no la tienen (FL, LQR) dejan la implementacion por defecto.
  virtual Vector6d slidingVariable() const { return Vector6d::Zero(); }

  /// Ultimo comando efectivamente publicado (post-saturacion y post-limite de
  /// tasa). Lo necesitan las leyes que razonan sobre el par realmente aplicado.
  const Vector6d & lastCommand() const { return tau_prev_cmd_; }

  /**
   * PARADA SEGURA pedida por la ley de control (FASE 4).
   *
   * El watchdog de la FASE 3 solo vigila la INFRAESTRUCTURA del lazo (ritmo de
   * ciclo, llegada de /joint_states). Hay fallos que solo la ley puede ver: el
   * LQR-SDRE exige que el par (A(q,q̇), B(q)) sea estabilizable y que la CARE
   * devuelva una K con max Re(eig(A - B K)) < 0 en cada actualizacion, y si eso
   * deja de cumplirse seguir comandando esa K es exactamente lo que no hay que
   * hacer. El plan lo pide de forma explicita ("si falla, HOLD seguro").
   *
   * Efecto: se entra en SAFE_HOLD (estado TERMINAL, no se sale de el) y el par
   * de la ley de ESTE ciclo NO se publica. A partir del siguiente tick se
   * sostiene la ultima pose conocida con el PD modesto de la clase base.
   *
   * Llamable desde computeTau(). Idempotente.
   */
  void requestSafeHold(const std::string & reason);

  /// Metadatos de trazabilidad: git SHA, hash de los parametros efectivos.
  /// Protegido para que las subclases con ficheros de log propios (el CSV de
  /// diagnostico del LQR-SDRE) pongan la MISMA cabecera que el CSV unificado.
  std::map<std::string, std::string> traceMetadata() const;

  /// Directorio de salida de los CSV, ya resuelto ("" = $HOME/.ros/...).
  const std::string & csvDir() const { return csv_dir_; }
  int testNum() const { return test_num_; }

private:
  // SAFE_HOLD (FASE 3): estado terminal de seguridad al que se entra si el
  // watchdog detecta que el lazo dejo de ser fiable. No se sale de el.
  enum class State
  {
    PRE_HOLD, WAIT_STATE, HOLD_START, RAMP, TRACK, HOLD_END, SAFE_HOLD, DONE
  };

  void tick();
  /// Declara los parametros `incision.*` y construye la trayectoria de 5 fases.
  std::shared_ptr<CartesianTrajectory> buildIncisionTrajectory(
    const std::vector<double> & rpy, std::string & description);
  /// Declara los parametros `sweep.*` y construye la campana de excitacion.
  std::unique_ptr<JointReferenceTable> buildJointSweep(std::string & description);
  bool readJointStates(Vector6d & q, Vector6d & dq);
  /// Comprueba el ritmo del lazo y la llegada de /joint_states. Devuelve false
  /// (y entra en SAFE_HOLD) si el lazo dejo de ser fiable.
  bool watchdogOk(double dt_sim);
  /// Rellena y escribe una fila del CSV unificado (FASE 3).
  void logRow(double t_sim, const Vector6d & q, const Vector6d & dq,
              const JointRef & ref, const Vector6d & tau_cmd,
              const std::string & state);
  JointRef rampReference(double t_ramp) const;
  /// Aplica commandFromLaw() y publica. Devuelve el torque comandado (para el CSV).
  Vector6d publishTau(const Vector6d & tau_law, const Vector6d & q,
                      const Vector6d & dq,
                      const Vector6d * dq_ref = nullptr);
  void enterState(State s);
  const char * stateName(State s) const;

  // -- parametros --
  double control_rate_ = 500.0;
  Vector6d q_init_ = Vector6d::Zero();
  Vector6d tau_max_ = kTauMax;
  // G3: true en Gazebo (el torque comandado incluye g), false en el UR5e real
  // (el robot compensa la gravedad internamente -> hay que restarla).
  bool gravity_in_command_ = true;
  /// Si q0 excede q_init_check_tol: true = PARAR (SAFE_HOLD), false = avisar.
  /// Default: estricto en el robot real, permisivo en Gazebo.
  bool q_init_check_strict_ = false;
  // FASE 5 — error inicial deliberado (ensayo de tiempo de alcance). Desplaza
  // el destino de la rampa, de modo que TRACK arranca con s(0) = Lambda*offset.
  Vector6d initial_offset_ = Vector6d::Zero();
  // FASE 2 — compensacion de friccion identificada (feedforward en el comando).
  FrictionCompensation friction_mode_ = FrictionCompensation::NONE;
  Vector6d friction_f_v_ = Vector6d::Zero();
  Vector6d friction_f_c_ = Vector6d::Zero();
  double friction_dq_eps_ = 1e-3;
  /// `friction.dq_source` = "desired": alimentar el feedforward con la
  /// velocidad DESEADA en vez de la medida. Ver commandFromLaw().
  bool friction_use_ref_dq_ = false;
  /**
   * LIMITE DE TASA del feedforward de friccion, en incremento de velocidad por
   * ciclo [rad/s]. El limite en par sale de la inercia: `dv_max * M_ii / dt`.
   *
   * Existe por un incidente en el robot real (2026-08-27, corrida smc_710):
   * `wrist_2` se fugo 162 grados durante un barrido bajo SMC. La causa no fue el
   * SMC — las otras cinco juntas sostuvieron dentro de 0.01 grados — sino que
   * `f_c·tanh(q̇_d/eps)` con eps = 1e-5 es practicamente un ESCALON: saltaba
   * 0.98 N·m entre dos ciclos, que sobre M_55 = 0.00535 kg m2 son 184 rad/s2.
   * El feedforward valia 1.973 N·m RMS frente a 0.342 de la ley de control —
   * 5.8x— asi que mandaba el termino en LAZO ABIERTO, que se calcula con la
   * velocidad de REFERENCIA y no sabe que la junta se ha ido: agravaba el error
   * en el 45 % de los ciclos.
   *
   * Se limita en INCREMENTO DE VELOCIDAD y no en N·m/s porque es lo que
   * significa lo mismo en las seis juntas: la inercia recorre cuatro ordenes de
   * magnitud, asi que un limite en par comun seria irrelevante en el hombro y
   * brutal en la muñeca. Con 0.01 rad/s por ciclo, el hombro admite 6475 N·m/s
   * (sin efecto practico) y `wrist_2` 13.4 N·m/s.
   */
  double friction_ff_dv_max_ = 0.0;      // 0 = sin limite
  /**
   * Limite de ERROR DE SEGUIMIENTO por junta [rad]. Al superarlo se entra en
   * SAFE_HOLD. 0 = desactivado.
   *
   * El watchdog de la FASE 3 vigila el RITMO del lazo y la llegada de
   * /joint_states — infraestructura— pero no si el robot esta haciendo lo que
   * se le pide. Por eso no se entero de que `wrist_2` se fugaba 162 grados en
   * la corrida smc_710: el lazo corria a 500 Hz y los estados llegaban
   * puntuales todo el rato. Se paro cuando termino el barrido, no antes.
   *
   * El default (1.0 rad = 57 grados) esta puesto para distinguir una FUGA de un
   * seguimiento malo: en Gazebo, `wrist_2` acumula 37.9 grados de error en el
   * barrido lento sin que eso sea un fallo de seguridad, y no debe abortar.
   */
  double watchdog_q_err_max_ = 0.0;
  /// Ultimo error de seguimiento, para que el watchdog pueda mirarlo.
  Vector6d q_err_ = Vector6d::Zero();
  /// diag M(q_init), congelada al arrancar: escala del limite de tasa.
  Vector6d friction_ff_inertia_ = Vector6d::Ones();
  /// Feedforward del ciclo anterior, para poder limitar su tasa.
  Vector6d friction_ff_prev_ = Vector6d::Zero();
  bool friction_ff_prev_valid_ = false;
  /// Ultimo `effort` de /joint_states: CORRIENTE [A] en el UR5e real,
  /// esfuerzo [N·m] en Gazebo (G5). NaN si el driver no lo publica.
  Vector6d cur_ = Vector6d::Constant(std::numeric_limits<double>::quiet_NaN());

  // FASE 3 — limite de tasa del comando y watchdog del lazo.
  Vector6d tau_rate_max_ = Vector6d::Zero();   // 0 = desactivado
  bool watchdog_enabled_ = true;
  /// Se dispara si el dt real supera k veces el periodo nominal del lazo.
  double watchdog_dt_factor_ = 5.0;
  /// Se dispara si /joint_states deja de llegar durante este tiempo [s].
  double watchdog_js_timeout_ = 0.2;
  /// Ciclos consecutivos malos antes de disparar: un unico dt largo (arranque,
  /// GC del sistema) no debe abortar un ensayo de media hora.
  int watchdog_strikes_ = 5;
  bool dry_run_ = false;
  std::string reference_table_out_;   // FASE 7: volcado para el evaluador offline
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

  // FASE 3
  Vector6d tau_prev_cmd_ = Vector6d::Zero();  // ultimo comando publicado
  SaturationFlags sat_flags_;
  int watchdog_bad_ticks_ = 0;
  double t_last_js_wall_ = -1.0;             // reloj de PARED del ultimo /joint_states
  std::size_t js_seq_ = 0;                   // cuenta de mensajes recibidos
  std::size_t js_seq_prev_ = 0;
  Vector6d q_last_ = Vector6d::Zero();       // ultima pose medida valida
  Vector6d q_safe_ = Vector6d::Zero();       // pose de sosten al entrar en SAFE_HOLD
  Vector6d last_wrench_ = Vector6d::Zero();  // ft_data del robot real; 0 en sim
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_TORQUE_CONTROL_NODE_BASE_HPP
