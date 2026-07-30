#include "ur5_dyn_control/torque_control_node_base.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <cmath>

namespace ur5_dyn_control
{

namespace
{

Vector6d paramToVec6(const std::vector<double> & v, const char * name)
{
  if (v.size() != 6) {
    throw std::runtime_error(std::string("El parametro '") + name +
                             "' debe tener 6 elementos");
  }
  Vector6d out;
  for (int i = 0; i < 6; ++i) {out[i] = v[i];}
  return out;
}

}  // namespace

std::shared_ptr<CartesianTrajectory>
TorqueControlNodeBase::buildIncisionTrajectory(const std::vector<double> & rpy,
                                               std::string & description)
{
  IncisionParams ip;
  ip.tcp_rpy = Eigen::Vector3d(rpy[0], rpy[1], rpy[2]);

  const auto start = declare_parameter<std::vector<double>>(
    "incision.start_pose", std::vector<double>{0.49, 0.13, 0.35});
  if (start.size() != 3) {
    throw std::runtime_error("incision.start_pose debe tener 3 elementos");
  }
  ip.start_pose = Eigen::Vector3d(start[0], start[1], start[2]);

  ip.surface_z       = declare_parameter<double>("incision.surface_z", 0.02);
  ip.cut_x           = declare_parameter<double>("incision.cut_x", 0.50);
  ip.cut_center_y    = declare_parameter<double>("incision.cut_center_y", 0.0);
  ip.cut_length      = declare_parameter<double>("incision.cut_length", 0.08);
  ip.cut_depth       = declare_parameter<double>("incision.cut_depth", 0.005);
  ip.approach_height = declare_parameter<double>("incision.approach_height", 0.03);

  const std::string axis = declare_parameter<std::string>("incision.cut_axis", "y");
  if (axis != "x" && axis != "y") {
    throw std::runtime_error("incision.cut_axis debe ser 'x' o 'y'");
  }
  ip.cut_axis = axis[0];

  ip.v_approach    = declare_parameter<double>("incision.v_approach", 0.10);
  ip.v_contact     = declare_parameter<double>("incision.v_contact", 0.015);
  ip.v_penetration = declare_parameter<double>("incision.v_penetration", 0.005);
  ip.v_cut         = declare_parameter<double>("incision.v_cut", 0.010);
  ip.v_withdraw    = declare_parameter<double>("incision.v_withdraw", 0.05);

  ip.ramp_fraction_cut  = declare_parameter<double>("incision.ramp_fraction_cut", 0.10);
  ip.ramp_fraction_move = declare_parameter<double>("incision.ramp_fraction_move", 0.5);
  ip.dwell              = declare_parameter<double>("incision.dwell", 0.3);

  auto traj = std::make_shared<IncisionTrajectory>(ip);

  RCLCPP_INFO(get_logger(),
              "Incision: corte de %.1f mm a %.1f mm/s, profundidad %.1f mm, "
              "eje '%c', superficie z=%.3f m",
              ip.cut_length * 1e3, ip.v_cut * 1e3, ip.cut_depth * 1e3,
              ip.cut_axis, ip.surface_z);
  for (const auto & ph : traj->phases()) {
    RCLCPP_INFO(get_logger(),
                "  %-11s t=[%6.2f, %6.2f] s  L=%6.1f mm  v=%5.1f mm/s"
                "  meseta=[%6.2f, %6.2f] s",
                toString(ph.id), ph.t_start, ph.t_end, ph.length * 1e3,
                ph.v_max * 1e3, ph.plateau_t0, ph.plateau_t1);
  }

  const auto & cut = traj->phase(IncisionPhaseId::CUT);
  const double plateau_len = (cut.plateau_t1 - cut.plateau_t0) * cut.v_max;
  std::ostringstream oss;
  oss << "5 fases, corte " << ip.cut_length * 1e3 << " mm, meseta de feed constante "
      << plateau_len * 1e3 << " mm";
  description = oss.str();
  return traj;
}

TorqueControlNodeBase::TorqueControlNodeBase(const std::string & node_name)
: rclcpp::Node(node_name)
{
  // ── Parametros generales ───────────────────────────────────────────────────
  control_rate_ = declare_parameter<double>("control_rate", 500.0);
  q_init_ = paramToVec6(declare_parameter<std::vector<double>>(
      "q_init", {0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0}), "q_init");
  tau_max_ = paramToVec6(declare_parameter<std::vector<double>>(
      "tau_max", {150.0, 150.0, 150.0, 28.0, 28.0, 28.0}), "tau_max");
  // G3 — politica de gravedad. Default true = Gazebo (comportamiento historico).
  // En el UR5e real DEBE ser false: direct_torque() compensa la gravedad dentro
  // del robot y comandarla otra vez la duplica. Ver torque_command.hpp.
  gravity_in_command_ = declare_parameter<bool>("gravity_in_command", true);
  command_topic_ = declare_parameter<std::string>(
    "command_topic", "/forward_effort_controller/commands");
  controller_manager_ns_ = declare_parameter<std::string>(
    "controller_manager", "/controller_manager");
  activate_controllers_ = declare_parameter<std::vector<std::string>>(
    "activate_controllers",
    std::vector<std::string>{"joint_state_broadcaster", "forward_effort_controller"});
  deactivate_controllers_ = declare_parameter<std::vector<std::string>>(
    "deactivate_controllers", std::vector<std::string>{});
  // Tolerar [""] en YAML (una lista vacia no es representable en rclcpp).
  auto drop_empty = [](std::vector<std::string> & v) {
      v.erase(std::remove_if(v.begin(), v.end(),
                             [](const std::string & s) {return s.empty();}),
              v.end());
    };
  drop_empty(activate_controllers_);
  drop_empty(deactivate_controllers_);
  perform_switch_ = declare_parameter<bool>("perform_switch", true);
  perform_unpause_ = declare_parameter<bool>("perform_unpause", true);
  world_name_ = declare_parameter<std::string>("world_name", "default");
  hold_start_duration_ = declare_parameter<double>("hold_start_duration", 1.0);
  transition_duration_ = declare_parameter<double>("transition_duration", 3.0);
  t_sim_limit_ = declare_parameter<double>("t_sim", 0.0);
  skip_trajectory_ = declare_parameter<bool>("skip_trajectory", false);
  q_init_check_tol_ = declare_parameter<double>("q_init_check_tol", 0.15);
  // En HOLD_START/HOLD_END se registra 1 de cada N muestras (los holds pueden
  // durar indefinidamente y a 500 Hz el CSV crece a GB/hora).
  csv_hold_decimation_ = std::max<int>(
    1, static_cast<int>(declare_parameter<int>("csv_hold_decimation", 10)));

  // ── Dinamica (Pinocchio) ───────────────────────────────────────────────────
  std::string urdf_path = declare_parameter<std::string>("urdf_path", "");
  if (urdf_path.empty()) {
    urdf_path = ament_index_cpp::get_package_share_directory("ur5_kinematics") +
      "/ur5e.urdf";
  }
  const double gravity = declare_parameter<double>("gravity", 9.8);
  // A2 — el offset del TCP es configurable; 0.141 m no se escribe en ningun
  // otro sitio (JointReferenceGenerator lo toma de Ur5Dynamics).
  const double tcp_offset_z = declare_parameter<double>("tcp_offset_z", 0.141);
  // A1 — hook de la herramienta (acople del bisturi). mass = 0 -> brazo solo,
  // que es el supuesto vigente. Ver docs/00_assumptions.md.
  ToolInertia tool;
  tool.mass = declare_parameter<double>("tool_mass", 0.0);
  {
    const auto com = declare_parameter<std::vector<double>>(
      "tool_com", std::vector<double>{0.0, 0.0, 0.0});
    // [Ixx, Iyy, Izz, Ixy, Ixz, Iyz] respecto al CoM, en el frame TCP.
    const auto inertia = declare_parameter<std::vector<double>>(
      "tool_inertia", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    if (com.size() != 3 || inertia.size() != 6) {
      throw std::runtime_error(
        "tool_com debe tener 3 elementos y tool_inertia 6 [Ixx,Iyy,Izz,Ixy,Ixz,Iyz]");
    }
    tool.com = Eigen::Vector3d(com[0], com[1], com[2]);
    tool.inertia << inertia[0], inertia[3], inertia[4],
                    inertia[3], inertia[1], inertia[5],
                    inertia[4], inertia[5], inertia[2];
  }
  dyn_ = std::make_shared<Ur5Dynamics>(urdf_path, gravity, tcp_offset_z, tool);
  RCLCPP_INFO(get_logger(), "Modelo Pinocchio: %s (g=%.2f, tcp_offset_z=%.4f m)",
              urdf_path.c_str(), gravity, tcp_offset_z);
  if (!tool.isNegligible()) {
    RCLCPP_WARN(get_logger(),
                "A1 LEVANTADO: herramienta de %.4f kg anadida al TCP "
                "(el supuesto de brazo solo ya no aplica; documentarlo)",
                tool.mass);
  }
  RCLCPP_INFO(get_logger(),
              "G3 gravity_in_command=%s -> tau_cmd = tau_ley%s",
              gravity_in_command_ ? "true (Gazebo)" : "false (UR5e real)",
              gravity_in_command_ ? "" : " - g(q)");

  // Torque de sosten "a ciegas" para PRE_HOLD (antes de /joint_states).
  tau_hold_blind_ = dyn_->gravity(q_init_);

  // ── Trayectoria cartesiana -> tabla articular ─────────────────────────────
  if (!skip_trajectory_) {
    // cubic_spline  : CartesianSplineTrajectory (historico; jerk discontinuo)
    // quintic_spline: QuinticSplineTrajectory   (FASE 1; jerk continuo)
    // incision      : IncisionTrajectory        (FASE 1; 5 fases, feed constante)
    const std::string traj_type =
      declare_parameter<std::string>("trajectory_type", "cubic_spline");
    const auto rpy = declare_parameter<std::vector<double>>(
      "tcp_orientation_rpy", {3.14159265, 0.0, -1.57079633});
    if (rpy.size() != 3) {
      throw std::runtime_error("tcp_orientation_rpy debe tener 3 elementos");
    }
    const Eigen::Matrix3d R_const =
      (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();

    std::shared_ptr<CartesianTrajectory> traj;
    std::string traj_desc;

    if (traj_type == "incision") {
      traj = buildIncisionTrajectory(rpy, traj_desc);
    } else {
      const auto wp_flat = declare_parameter<std::vector<double>>(
        "waypoints_xyz", std::vector<double>{});
      const auto wp_times = declare_parameter<std::vector<double>>(
        "waypoint_times", std::vector<double>{});
      if (wp_flat.size() < 6 || wp_flat.size() % 3 != 0 ||
          wp_times.size() != wp_flat.size() / 3)
      {
        throw std::runtime_error(
          "waypoints_xyz (3N valores) y waypoint_times (N valores, N>=2) invalidos");
      }
      std::vector<Eigen::Vector3d> waypoints;
      for (std::size_t i = 0; i < wp_flat.size(); i += 3) {
        waypoints.emplace_back(wp_flat[i], wp_flat[i + 1], wp_flat[i + 2]);
      }
      if (traj_type == "quintic_spline") {
        traj = std::make_shared<QuinticSplineTrajectory>(waypoints, wp_times, R_const);
      } else if (traj_type == "cubic_spline") {
        traj = std::make_shared<CartesianSplineTrajectory>(waypoints, wp_times, R_const);
      } else {
        throw std::runtime_error(
          "trajectory_type desconocido: '" + traj_type +
          "' (validos: cubic_spline, quintic_spline, incision)");
      }
      std::ostringstream oss;
      oss << waypoints.size() << " waypoints";
      traj_desc = oss.str();
    }

    IkParams ik;
    ik.seed = paramToVec6(declare_parameter<std::vector<double>>(
        "ik_seed", {0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0}), "ik_seed");
    ik.max_iterations = static_cast<int>(declare_parameter<int>("ik_max_iterations", 450));
    ik.alpha = declare_parameter<double>("ik_alpha", 0.5);
    ik.weight_pos = declare_parameter<double>("ik_weight_pos", 1.0);
    ik.weight_orient = declare_parameter<double>("ik_weight_orient", 1.0);
    ik.q_jump_tol = declare_parameter<double>("q_jump_tol", 0.15);
    ik.fk_tol = declare_parameter<double>("ik_fk_tol", 5e-3);

    // Limites y umbrales de validacion de la tabla (FASE 1).
    TrajectoryLimits lim;
    lim.dq_max = paramToVec6(declare_parameter<std::vector<double>>(
        "dq_max", {M_PI, M_PI, M_PI, M_PI, M_PI, M_PI}), "dq_max");
    lim.ddq_max = paramToVec6(declare_parameter<std::vector<double>>(
        "ddq_max", {5.0, 5.0, 5.0, 5.0, 5.0, 5.0}), "ddq_max");
    lim.sigma_min_threshold = declare_parameter<double>("sigma_min_threshold", 0.05);
    lim.manipulability_threshold =
      declare_parameter<double>("manipulability_threshold", 0.0);

    ref_gen_ = std::make_unique<JointReferenceGenerator>(traj, dyn_, urdf_path, ik, lim);
    RCLCPP_INFO(get_logger(),
                "Trayectoria '%s' (%s, %.2f s). Construyendo tabla (IK offline)...",
                traj_type.c_str(), traj_desc.c_str(), traj->duration());
    std::string err;
    if (!ref_gen_->build(1.0 / control_rate_, err)) {
      throw std::runtime_error("JointReferenceGenerator: " + err);
    }
    const auto & d = ref_gen_->diagnostics();
    RCLCPP_INFO(get_logger(), "Tabla lista: %zu muestras a %.0f Hz.",
                ref_gen_->size(), control_rate_);
    RCLCPP_INFO(get_logger(),
                "  sigma_min(J) = %.4f (min en t=%.2f s, umbral %.4f) | w_min = %.4f",
                d.sigma_min, d.sigma_min_t, lim.sigma_min_threshold,
                d.manipulability_min);
    RCLCPP_INFO(get_logger(),
                "  |dq|max  = [%.3f %.3f %.3f %.3f %.3f %.3f] rad/s   (margen %.1f %%)",
                d.dq_peak[0], d.dq_peak[1], d.dq_peak[2],
                d.dq_peak[3], d.dq_peak[4], d.dq_peak[5], 100.0 * d.dq_margin);
    RCLCPP_INFO(get_logger(),
                "  |ddq|max = [%.3f %.3f %.3f %.3f %.3f %.3f] rad/s^2 (margen %.1f %%)",
                d.ddq_peak[0], d.ddq_peak[1], d.ddq_peak[2],
                d.ddq_peak[3], d.ddq_peak[4], d.ddq_peak[5], 100.0 * d.ddq_margin);
  }

  // ── Logging (el CSV se abre en start(): csvPrefix() es virtual) ───────────
  csv_dir_ = declare_parameter<std::string>("csv_output_dir", "");
  test_num_ = static_cast<int>(declare_parameter<int>("test_num", 1));

  // ── ROS I/O ────────────────────────────────────────────────────────────────
  tau_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 10);
  js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [this](sensor_msgs::msg::JointState::SharedPtr msg) {last_js_ = std::move(msg);});
  switcher_ = std::make_unique<ControllerSwitcher>(this, controller_manager_ns_);
}

TorqueControlNodeBase::~TorqueControlNodeBase()
{
  csv_.close();
}

void TorqueControlNodeBase::start()
{
  if (!csv_.open(csv_dir_, csvPrefix(), test_num_)) {
    RCLCPP_WARN(get_logger(), "No se pudo abrir el CSV (se continua sin log)");
  } else {
    RCLCPP_INFO(get_logger(), "CSV: %s", csv_.path().c_str());
  }

  // Timer de PARED: corre aunque la sim este pausada o con RTF < 1.
  const auto period = std::chrono::duration<double>(1.0 / control_rate_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() {tick();});
  RCLCPP_INFO(get_logger(), "Lazo de control a %.0f Hz (pared). Estado: PRE_HOLD",
              control_rate_);
}

bool TorqueControlNodeBase::readJointStates(Vector6d & q, Vector6d & dq)
{
  if (!last_js_) {return false;}
  const auto & js = *last_js_;
  int found = 0;
  for (int j = 0; j < 6; ++j) {
    for (std::size_t i = 0; i < js.name.size(); ++i) {
      if (js.name[i] == kJointNames[j]) {
        if (i < js.position.size() && i < js.velocity.size()) {
          q[j] = js.position[i];
          dq[j] = js.velocity[i];
          ++found;
        }
        break;
      }
    }
  }
  return found == 6;
}

JointRef TorqueControlNodeBase::rampReference(double t_ramp) const
{
  // Transicion quintica q0 -> tabla[0] con v = a = 0 en ambos extremos
  // (la tabla arranca con dq = ddq = 0 por el spline clamped).
  const double T = transition_duration_;
  const Vector6d qf = ref_gen_->at(0).q;
  const double s = std::clamp(t_ramp / T, 0.0, 1.0);
  const double s3 = s * s * s;
  const double p = 10.0 * s3 - 15.0 * s3 * s + 6.0 * s3 * s * s;
  const double dp = (30.0 * s * s - 60.0 * s3 + 30.0 * s3 * s) / T;
  const double ddp = (60.0 * s - 180.0 * s * s + 120.0 * s3) / (T * T);

  JointRef ref;
  ref.q = q0_ + p * (qf - q0_);
  ref.dq = dp * (qf - q0_);
  ref.ddq = ddp * (qf - q0_);
  return ref;
}

Vector6d TorqueControlNodeBase::commandFromLaw(const Vector6d & tau_law, const Vector6d & q)
{
  // g(q) solo se evalua cuando hace falta restarla (ahorra una RNEA por tick
  // en el caso de Gazebo, que es el default).
  const Vector6d g_q = gravity_in_command_ ? Vector6d::Zero().eval() : dyn_->gravity(q);
  return torqueCommand(tau_law, g_q, gravity_in_command_, tau_max_);
}

Vector6d TorqueControlNodeBase::publishTau(const Vector6d & tau_law, const Vector6d & q)
{
  const Vector6d tau_cmd = commandFromLaw(tau_law, q);
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data.assign(tau_cmd.data(), tau_cmd.data() + 6);
  tau_pub_->publish(cmd);
  return tau_cmd;
}

void TorqueControlNodeBase::enterState(State s)
{
  state_ = s;
  t_state_start_ = now().seconds();
  RCLCPP_INFO(get_logger(), "Estado -> %s (t_sim=%.2f)", stateName(s), t_state_start_);
}

const char * TorqueControlNodeBase::stateName(State s) const
{
  switch (s) {
    case State::PRE_HOLD: return "PRE_HOLD";
    case State::WAIT_STATE: return "WAIT_STATE";
    case State::HOLD_START: return "HOLD_START";
    case State::RAMP: return "RAMP";
    case State::TRACK: return "TRACK";
    case State::HOLD_END: return "HOLD_END";
    case State::DONE: return "DONE";
  }
  return "?";
}

void TorqueControlNodeBase::tick()
{
  const double t_sim = now().seconds();
  double dt = (t_prev_ >= 0.0) ? (t_sim - t_prev_) : 0.0;
  t_prev_ = t_sim;
  dt = std::clamp(dt, 0.0, 0.05);

  Vector6d q, dq;
  const bool have_state = readJointStates(q, dq);

  switch (state_) {
    case State::PRE_HOLD: {
        // Publicar sosten a ciegas desde el primer tick: cuando el controlador
        // de esfuerzo se active (primer paso de fisica) ya tendra comando.
        // Aun no hay /joint_states: se evalua g en q_init (con
        // gravity_in_command=false esto da exactamente 0, que es el comando
        // correcto para el robot real en reposo).
        publishTau(tau_hold_blind_, q_init_);
        ++pre_hold_ticks_;

        // 1) Esperar a que los controladores esten CARGADOS (los spawners
        //    --inactive corren en paralelo); 2) pedir el switch STRICT solo
        //    con los que falte activar.
        if (perform_switch_ && !switch_requested_ && !check_pending_) {
          if (pre_hold_ticks_ % static_cast<int>(control_rate_ * 1.0) == 1) {
            check_pending_ = true;
            switcher_->checkControllersLoaded(
              activate_controllers_,
              [this](bool ready, std::vector<std::string> to_activate) {
                check_pending_ = false;
                if (!ready || switch_requested_) {return;}
                switch_requested_ = true;
                request_tick_ = pre_hold_ticks_;
                if (to_activate.empty() && deactivate_controllers_.empty()) {
                  RCLCPP_INFO(get_logger(), "Controladores ya activos.");
                  return;
                }
                RCLCPP_INFO(get_logger(), "Solicitando activacion de controladores...");
                switcher_->requestSwitch(
                  to_activate, deactivate_controllers_,
                  [this](bool ok) {
                    if (!ok) {
                      RCLCPP_FATAL(get_logger(), "Fallo el switch de controladores");
                      enterState(State::DONE);
                      return;
                    }
                    RCLCPP_INFO(get_logger(), "Controladores activados.");
                  });
              });
          }
        }

        // Con la peticion de switch armada, despausar el mundo: la activacion
        // se consuma en el primer paso y nuestro torque llega de inmediato.
        if (switch_requested_ && !unpause_done_) {
          // pequeno margen para que la peticion llegue al controller_manager
          if (pre_hold_ticks_ - request_tick_ > static_cast<int>(control_rate_ * 0.5)) {
            if (perform_unpause_) {
              setIgnWorldPaused(world_name_, false, get_logger());
            }
            unpause_done_ = true;
            enterState(State::WAIT_STATE);
          }
        }
        if (!perform_switch_ && !unpause_done_) {
          // Sin switch (controladores ya activos): despausar y continuar.
          if (perform_unpause_) {
            setIgnWorldPaused(world_name_, false, get_logger());
          }
          unpause_done_ = true;
          enterState(State::WAIT_STATE);
        }
        break;
      }

    case State::WAIT_STATE: {
        publishTau(tau_hold_blind_, have_state ? q : q_init_);
        if (!have_state) {break;}
        q0_ = q;
        const double err0 = (q0_ - q_init_).cwiseAbs().maxCoeff();
        if (err0 > q_init_check_tol_) {
          RCLCPP_WARN(get_logger(),
                      "q0 difiere de q_init en %.3f rad (> %.3f). Se regula en q0.",
                      err0, q_init_check_tol_);
        }
        RCLCPP_INFO(get_logger(),
                    "q0 = [%.3f %.3f %.3f %.3f %.3f %.3f]",
                    q0_[0], q0_[1], q0_[2], q0_[3], q0_[4], q0_[5]);
        enterState(State::HOLD_START);
        break;
      }

    case State::HOLD_START: {
        if (!have_state) {break;}
        JointRef ref;
        ref.q = q0_;
        const Vector6d tau = publishTau(computeTau(q, dq, ref, dt), q);
        if (++hold_log_counter_ % csv_hold_decimation_ == 0) {
          csv_.log(t_sim, q, dq, ref, tau, dyn_->fk(q).translation(),
                   dyn_->fk(ref.q).translation(), "HOLD_START");
        }
        if (skip_trajectory_) {
          if (t_sim_limit_ > 0.0 && t_sim - t_state_start_ >= t_sim_limit_) {
            enterState(State::DONE);
            timer_->cancel();
            csv_.close();
          }
          break;
        }
        if (t_sim - t_state_start_ >= hold_start_duration_) {
          enterState(State::RAMP);
        }
        break;
      }

    case State::RAMP: {
        if (!have_state) {break;}
        const JointRef ref = rampReference(t_sim - t_state_start_);
        const Vector6d tau = publishTau(computeTau(q, dq, ref, dt), q);
        csv_.log(t_sim, q, dq, ref, tau, dyn_->fk(q).translation(),
                 dyn_->fk(ref.q).translation(), "RAMP");
        if (t_sim - t_state_start_ >= transition_duration_) {
          track_index_ = 0;
          enterState(State::TRACK);
        }
        break;
      }

    case State::TRACK: {
        if (!have_state) {break;}
        track_index_ = static_cast<std::size_t>(
          std::max(0.0, (t_sim - t_state_start_) / ref_gen_->dt()));
        const JointRef & ref = ref_gen_->at(track_index_);
        const Vector6d tau = publishTau(computeTau(q, dq, ref, dt), q);
        csv_.log(t_sim, q, dq, ref, tau, dyn_->fk(q).translation(),
                 dyn_->fk(ref.q).translation(), "TRACK");

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "TRACK t=%.2f  |e|max=%.4f rad",
                             t_sim - t_state_start_,
                             (ref.q - q).cwiseAbs().maxCoeff());

        if (track_index_ >= ref_gen_->size() - 1) {
          enterState(State::HOLD_END);
        }
        break;
      }

    case State::HOLD_END: {
        if (!have_state) {break;}
        JointRef ref;
        ref.q = ref_gen_->at(ref_gen_->size() - 1).q;
        const Vector6d tau = publishTau(computeTau(q, dq, ref, dt), q);
        if (++hold_log_counter_ % csv_hold_decimation_ == 0) {
          csv_.log(t_sim, q, dq, ref, tau, dyn_->fk(q).translation(),
                   dyn_->fk(ref.q).translation(), "HOLD_END");
        }
        if (t_sim_limit_ > 0.0 && t_sim - t_state_start_ >= t_sim_limit_) {
          RCLCPP_INFO(get_logger(), "t_sim agotado; finalizando (hold se detiene).");
          enterState(State::DONE);
          timer_->cancel();
          csv_.close();
        }
        break;
      }

    case State::DONE:
    default:
      break;
  }
}

}  // namespace ur5_dyn_control
