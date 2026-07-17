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

TorqueControlNodeBase::TorqueControlNodeBase(const std::string & node_name)
: rclcpp::Node(node_name)
{
  // ── Parametros generales ───────────────────────────────────────────────────
  control_rate_ = declare_parameter<double>("control_rate", 500.0);
  q_init_ = paramToVec6(declare_parameter<std::vector<double>>(
      "q_init", {0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0}), "q_init");
  tau_max_ = paramToVec6(declare_parameter<std::vector<double>>(
      "tau_max", {150.0, 150.0, 150.0, 28.0, 28.0, 28.0}), "tau_max");
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
  dyn_ = std::make_shared<Ur5Dynamics>(urdf_path, gravity);
  RCLCPP_INFO(get_logger(), "Modelo Pinocchio: %s (g=%.2f)", urdf_path.c_str(), gravity);

  // Torque de sosten "a ciegas" para PRE_HOLD (antes de /joint_states).
  tau_hold_blind_ = dyn_->gravity(q_init_);

  // ── Trayectoria cartesiana -> tabla articular ─────────────────────────────
  if (!skip_trajectory_) {
    const auto wp_flat = declare_parameter<std::vector<double>>(
      "waypoints_xyz", std::vector<double>{});
    const auto wp_times = declare_parameter<std::vector<double>>(
      "waypoint_times", std::vector<double>{});
    const auto rpy = declare_parameter<std::vector<double>>(
      "tcp_orientation_rpy", {3.14159265, 0.0, -1.57079633});
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
    const Eigen::Matrix3d R_const =
      (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();

    auto traj = std::make_shared<CartesianSplineTrajectory>(
      waypoints, wp_times, R_const);

    IkParams ik;
    ik.seed = paramToVec6(declare_parameter<std::vector<double>>(
        "ik_seed", {0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0}), "ik_seed");
    ik.max_iterations = static_cast<int>(declare_parameter<int>("ik_max_iterations", 450));
    ik.alpha = declare_parameter<double>("ik_alpha", 0.5);
    ik.weight_pos = declare_parameter<double>("ik_weight_pos", 1.0);
    ik.weight_orient = declare_parameter<double>("ik_weight_orient", 1.0);
    ik.q_jump_tol = declare_parameter<double>("q_jump_tol", 0.15);

    ref_gen_ = std::make_unique<JointReferenceGenerator>(traj, dyn_, urdf_path, ik);
    RCLCPP_INFO(get_logger(),
                "Construyendo tabla de referencias (%zu waypoints, %.1f s, IK offline)...",
                waypoints.size(), traj->duration());
    std::string err;
    if (!ref_gen_->build(1.0 / control_rate_, err)) {
      throw std::runtime_error("JointReferenceGenerator: " + err);
    }
    RCLCPP_INFO(get_logger(), "Tabla lista: %zu muestras a %.0f Hz.",
                ref_gen_->size(), control_rate_);
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

void TorqueControlNodeBase::publishTau(const Vector6d & tau)
{
  const Vector6d tau_sat = tau.cwiseMin(tau_max_).cwiseMax(-tau_max_);
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data.assign(tau_sat.data(), tau_sat.data() + 6);
  tau_pub_->publish(cmd);
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
        publishTau(tau_hold_blind_);
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
        publishTau(tau_hold_blind_);
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
        const Vector6d tau = computeTau(q, dq, ref, dt);
        publishTau(tau);
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
        const Vector6d tau = computeTau(q, dq, ref, dt);
        publishTau(tau);
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
        const Vector6d tau = computeTau(q, dq, ref, dt);
        publishTau(tau);
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
        const Vector6d tau = computeTau(q, dq, ref, dt);
        publishTau(tau);
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
