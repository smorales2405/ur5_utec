#include "ur5_dyn_control/torque_control_node_base.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pinocchio/spatial/explog.hpp>

#include <chrono>
#include <cmath>
#include <map>

namespace ur5_dyn_control
{

namespace
{

// Ganancias del PD de SAFE_HOLD (FASE 3). Deliberadamente MODESTAS y
// escaladas a la inercia tipica de cada junta: el objetivo no es seguir una
// referencia sino frenar y sostener sin excitar nada. Las munecas van bajas
// porque su inercia es de 1e-3 kg·m² y un PD agresivo alli chatea (medido en
// la puesta a punto de gravity_comp).
const Vector6d kSafeHoldKp =
  (Vector6d() << 60.0, 60.0, 60.0, 10.0, 5.0, 2.0).finished();
const Vector6d kSafeHoldKd =
  (Vector6d() << 10.0, 10.0, 10.0, 1.0, 0.5, 0.2).finished();

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
  ip.cut_lead        = declare_parameter<double>("incision.cut_lead", 0.010);
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

  ip.ramp_fraction_move = declare_parameter<double>("incision.ramp_fraction_move", 0.5);
  ip.dwell              = declare_parameter<double>("incision.dwell", 0.3);

  auto traj = std::make_shared<IncisionTrajectory>(ip);

  RCLCPP_INFO(get_logger(),
              "Incision: %.1f mm MEDIDOS a feed constante de %.1f mm/s "
              "(+ %.1f mm de entrada y salida -> %.1f mm de trazo), "
              "profundidad %.1f mm, eje '%c', superficie z=%.3f m",
              ip.cut_length * 1e3, ip.v_cut * 1e3, ip.cut_lead * 1e3,
              ip.cutStroke() * 1e3, ip.cut_depth * 1e3, ip.cut_axis, ip.surface_z);
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
  oss << "5 fases, trazo " << ip.cutStroke() * 1e3 << " mm con "
      << plateau_len * 1e3 << " mm medidos a feed constante";
  description = oss.str();
  return traj;
}

std::unique_ptr<JointReferenceTable>
TorqueControlNodeBase::buildJointSweep(std::string & description)
{
  JointSweepParams sp;
  sp.q_fixed = paramToVec6(declare_parameter<std::vector<double>>(
      "sweep.q_fixed", {0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0}),
      "sweep.q_fixed");
  sp.joint = static_cast<int>(declare_parameter<int>("sweep.joint", 0));
  sp.amplitude = declare_parameter<double>("sweep.amplitude", M_PI / 4);
  sp.velocities = declare_parameter<std::vector<double>>(
    "sweep.velocities", std::vector<double>{0.05, 0.10, 0.20, 0.35, 0.50, 0.75, 1.00});
  sp.ramp_fraction = declare_parameter<double>("sweep.ramp_fraction", 0.15);
  sp.max_sweep_duration = declare_parameter<double>("sweep.max_sweep_duration", 40.0);
  sp.dwell = declare_parameter<double>("sweep.dwell", 1.0);
  sp.approach_duration = declare_parameter<double>("sweep.approach_duration", 3.0);
  sp.ddq_max = declare_parameter<double>("sweep.ddq_max", 5.0);

  auto gen = std::make_unique<JointSweepGenerator>(sp);
  std::string err;
  if (!gen->build(1.0 / control_rate_, err)) {
    throw std::runtime_error("JointSweepGenerator: " + err);
  }

  RCLCPP_INFO(get_logger(),
              "Barrido de excitacion: junta %d (%s), %zu niveles de velocidad "
              "x 2 sentidos, %.1f s",
              sp.joint, kJointNames[sp.joint].c_str(), sp.velocities.size(),
              gen->duration());
  for (const auto & sg : gen->segments()) {
    if (!sg.useful) {continue;}
    RCLCPP_INFO(get_logger(),
                "  v=%+6.3f rad/s  amplitud +-%.3f rad (%.1f deg)  "
                "t=[%6.2f, %6.2f] s  meseta=[%6.2f, %6.2f] s (%.2f s utiles)",
                sg.velocity, sg.amplitude, sg.amplitude * 180.0 / M_PI,
                sg.t_start, sg.t_end, sg.plateau_t0, sg.plateau_t1,
                sg.plateau_t1 - sg.plateau_t0);
  }

  std::ostringstream oss;
  oss << "junta " << sp.joint << ", " << sp.velocities.size()
      << " niveles x 2 sentidos";
  description = oss.str();
  return gen;
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

  // FASE 2 — compensacion de friccion identificada. Default "none": el
  // comportamiento de las FASES 0-1 no cambia. Los coeficientes salen del YAML
  // que produce `ros2 run ur5_identification run_identification`.
  {
    const std::string mode =
      declare_parameter<std::string>("friction_compensation", "none");
    if (mode == "none") {
      friction_mode_ = FrictionCompensation::NONE;
    } else if (mode == "viscous") {
      friction_mode_ = FrictionCompensation::VISCOUS;
    } else if (mode == "viscous_coulomb") {
      friction_mode_ = FrictionCompensation::VISCOUS_COULOMB;
    } else {
      throw std::runtime_error(
        "friction_compensation desconocido: '" + mode +
        "' (validos: none, viscous, viscous_coulomb)");
    }
    friction_f_v_ = paramToVec6(declare_parameter<std::vector<double>>(
        "friction.f_v", std::vector<double>(6, 0.0)), "friction.f_v");
    friction_f_c_ = paramToVec6(declare_parameter<std::vector<double>>(
        "friction.f_c", std::vector<double>(6, 0.0)), "friction.f_c");
    friction_dq_eps_ = declare_parameter<double>("friction.dq_eps", 1e-3);
    if (!(friction_dq_eps_ > 0.0)) {
      throw std::runtime_error("friction.dq_eps debe ser > 0");
    }
    if (friction_mode_ != FrictionCompensation::NONE) {
      RCLCPP_INFO(get_logger(),
                  "Compensacion de friccion '%s': F_v=[%.3f %.3f %.3f %.3f %.3f %.3f] "
                  "F_c=[%.3f %.3f %.3f %.3f %.3f %.3f] (tanh eps=%.4f rad/s)",
                  mode.c_str(),
                  friction_f_v_[0], friction_f_v_[1], friction_f_v_[2],
                  friction_f_v_[3], friction_f_v_[4], friction_f_v_[5],
                  friction_f_c_[0], friction_f_c_[1], friction_f_c_[2],
                  friction_f_c_[3], friction_f_c_[4], friction_f_c_[5],
                  friction_dq_eps_);
    }
  }

  // FASE 3 — limite de tasa, watchdog y dry-run.
  tau_rate_max_ = paramToVec6(declare_parameter<std::vector<double>>(
      "tau_rate_max", std::vector<double>(6, 0.0)), "tau_rate_max");
  watchdog_enabled_ = declare_parameter<bool>("watchdog.enabled", true);
  watchdog_dt_factor_ = declare_parameter<double>("watchdog.dt_factor", 5.0);
  watchdog_js_timeout_ = declare_parameter<double>("watchdog.joint_state_timeout", 0.2);
  watchdog_strikes_ = static_cast<int>(declare_parameter<int>("watchdog.strikes", 5));
  dry_run_ = declare_parameter<bool>("dry_run", false);
  if (dry_run_) {
    RCLCPP_WARN(get_logger(),
                "DRY-RUN: se calcula todo pero NO se publica ningun torque.");
  }
  if (tau_rate_max_.maxCoeff() > 0.0) {
    RCLCPP_INFO(get_logger(),
                "Limite de tasa: [%.0f %.0f %.0f %.0f %.0f %.0f] N.m/s",
                tau_rate_max_[0], tau_rate_max_[1], tau_rate_max_[2],
                tau_rate_max_[3], tau_rate_max_[4], tau_rate_max_[5]);
  }
  RCLCPP_INFO(get_logger(),
              "Watchdog %s (dt > %.1fx nominal o /joint_states mudo > %.2f s, "
              "%d ciclos seguidos)",
              watchdog_enabled_ ? "activo" : "DESACTIVADO",
              watchdog_dt_factor_, watchdog_js_timeout_, watchdog_strikes_);

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
    // joint_sweep    : barrido articular de excitacion (FASE 2), sin IK
    const std::string traj_type =
      declare_parameter<std::string>("trajectory_type", "cubic_spline");
    const bool is_sweep = (traj_type == "joint_sweep");
    if (is_sweep) {
      // El barrido es articular puro: no pasa por IK ni por una trayectoria
      // cartesiana, asi que se salta todo el bloque de abajo.
      std::string desc;
      ref_gen_ = buildJointSweep(desc);
      const auto & d = ref_gen_->diagnostics();
      RCLCPP_INFO(get_logger(),
                  "Tabla lista: %zu muestras a %.0f Hz (%s). "
                  "|dq|max=%.3f rad/s  |ddq|max=%.3f rad/s^2",
                  ref_gen_->size(), control_rate_, desc.c_str(),
                  d.dq_peak.maxCoeff(), d.ddq_peak.maxCoeff());
    }
    if (!is_sweep) {
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

    auto gen = std::make_unique<JointReferenceGenerator>(traj, dyn_, urdf_path, ik, lim);
    RCLCPP_INFO(get_logger(),
                "Trayectoria '%s' (%s, %.2f s). Construyendo tabla (IK offline)...",
                traj_type.c_str(), traj_desc.c_str(), traj->duration());
    std::string err;
    if (!gen->build(1.0 / control_rate_, err)) {
      throw std::runtime_error("JointReferenceGenerator: " + err);
    }
    const auto d = gen->diagnostics();
    ref_gen_ = std::move(gen);
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
    }  // !is_sweep
  }

  // ── Logging (el CSV se abre en start(): csvPrefix() es virtual) ───────────
  csv_dir_ = declare_parameter<std::string>("csv_output_dir", "");
  test_num_ = static_cast<int>(declare_parameter<int>("test_num", 1));

  // ── ROS I/O ────────────────────────────────────────────────────────────────
  tau_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 10);
  js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [this](sensor_msgs::msg::JointState::SharedPtr msg) {
      last_js_ = std::move(msg);
      ++js_seq_;
    });
  switcher_ = std::make_unique<ControllerSwitcher>(this, controller_manager_ns_);
}

TorqueControlNodeBase::~TorqueControlNodeBase()
{
  csv_.close();
}

void TorqueControlNodeBase::start()
{
  if (!csv_.open(csv_dir_, csvPrefix(), test_num_, traceMetadata())) {
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

std::map<std::string, std::string> TorqueControlNodeBase::traceMetadata() const
{
  std::map<std::string, std::string> params;
  // Conjunto EFECTIVO de parametros: recoge tambien los overrides de linea de
  // comandos, no solo lo que diga el YAML. Dos corridas con distinto
  // sweep.joint tienen que dar hashes distintos.
  for (const auto & name : const_cast<TorqueControlNodeBase *>(this)->
       list_parameters({}, 0).names)
  {
    params[name] = get_parameter(name).value_to_string();
  }
  return {
    {"git_sha", UR5_DYN_CONTROL_GIT_SHA},
    {"params_hash", CsvLogger::hashParameters(params)},
    {"n_params", std::to_string(params.size())},
  };
}

void TorqueControlNodeBase::logRow(double t_sim, const Vector6d & q,
                                   const Vector6d & dq, const JointRef & ref,
                                   const Vector6d & tau_cmd,
                                   const std::string & state)
{
  LogSample d;
  d.t_wall = std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  d.t_sim = t_sim;
  d.q = q;
  d.dq = dq;
  d.q_des = ref.q;
  d.dq_des = ref.dq;
  d.ddq_des = ref.ddq;
  d.tau_cmd = tau_cmd;
  for (int i = 0; i < 6; ++i) {
    d.tau_sat_flag[i] =
      (sat_flags_.saturated[i] || sat_flags_.rate_limited[i]) ? 1 : 0;
  }
  d.s = slidingVariable();

  const pinocchio::SE3 T = dyn_->fk(q);
  const pinocchio::SE3 T_des = dyn_->fk(ref.q);
  d.xyz = T.translation();
  d.xyz_des = T_des.translation();
  // theta_err = ||log(R_des^T R)||: angulo del error de orientacion. NUNCA
  // angulos de Euler — el wrap los hace inservibles para una metrica.
  d.theta_err = pinocchio::log3(
    Eigen::Matrix3d(T_des.rotation().transpose() * T.rotation())).norm();

  d.wrench = last_wrench_;   // 0 en simulacion; ft_data en el robot real
  d.state = state;
  csv_.log(d);
}

bool TorqueControlNodeBase::watchdogOk(double dt_sim)
{
  if (!watchdog_enabled_) {return true;}
  // Estados en los que el lazo aun no esta en regimen: no se vigila.
  if (state_ == State::PRE_HOLD || state_ == State::WAIT_STATE ||
      state_ == State::SAFE_HOLD || state_ == State::DONE)
  {
    return true;
  }

  const double t_wall = std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();

  std::string reason;
  // 1) Ritmo del lazo: dt de SIMULACION muy por encima del nominal significa
  //    que el lazo perdio ciclos y la referencia va a saltar.
  const double dt_max = watchdog_dt_factor_ / control_rate_;
  if (dt_sim > dt_max) {
    reason = "dt de simulacion " + std::to_string(dt_sim) + " s > " +
      std::to_string(dt_max) + " s (" + std::to_string(watchdog_dt_factor_) +
      "x el periodo nominal)";
  }
  // 2) /joint_states: se mide con el reloj de PARED, porque si la fuente de
  //    estado muere el reloj de simulacion tambien puede congelarse y un
  //    timeout en tiempo de sim nunca se dispararia.
  if (reason.empty() && t_last_js_wall_ > 0.0 && js_seq_ == js_seq_prev_ &&
      (t_wall - t_last_js_wall_) > watchdog_js_timeout_)
  {
    reason = "sin /joint_states nuevos durante " +
      std::to_string(t_wall - t_last_js_wall_) + " s";
  }
  if (js_seq_ != js_seq_prev_) {
    js_seq_prev_ = js_seq_;
    t_last_js_wall_ = t_wall;
  }

  if (reason.empty()) {
    watchdog_bad_ticks_ = 0;
    return true;
  }
  // Se exigen varios ciclos malos seguidos: un dt largo aislado (arranque, GC
  // del sistema, hipo del planificador) no debe abortar un ensayo largo.
  if (++watchdog_bad_ticks_ < watchdog_strikes_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "watchdog: %s (%d/%d)", reason.c_str(),
                         watchdog_bad_ticks_, watchdog_strikes_);
    return true;
  }

  RCLCPP_FATAL(get_logger(),
               "WATCHDOG DISPARADO tras %d ciclos: %s. Entrando en SAFE_HOLD "
               "(se mantiene el par de sosten en la ultima pose conocida).",
               watchdog_bad_ticks_, reason.c_str());
  q_safe_ = q_last_;
  enterState(State::SAFE_HOLD);
  return false;
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

Vector6d TorqueControlNodeBase::commandFromLaw(const Vector6d & tau_law,
                                               const Vector6d & q,
                                               const Vector6d & dq)
{
  // FASE 2: la friccion se SUMA al comando (se opone al movimiento en la
  // planta), antes de la politica de gravedad y de la saturacion, porque es un
  // termino mas del par fisico que la articulacion debe entregar.
  const Vector6d tau_total = tau_law + frictionFeedforward(
    dq, friction_f_v_, friction_f_c_, friction_mode_, friction_dq_eps_);
  // g(q) solo se evalua cuando hace falta restarla (ahorra una RNEA por tick
  // en el caso de Gazebo, que es el default).
  const Vector6d g_q = gravity_in_command_ ? Vector6d::Zero().eval() : dyn_->gravity(q);
  const Vector6d tau_sat = torqueCommand(tau_total, g_q, gravity_in_command_, tau_max_);

  // FASE 3 — limite de tasa sobre el comando ya saturado, y marcas de recorte
  // para el hook de anti-windup.
  const double dt_nom = 1.0 / control_rate_;
  const Vector6d tau_cmd = rateLimit(tau_sat, tau_prev_cmd_, tau_rate_max_, dt_nom);

  const Vector6d tau_unsat = applyGravityPolicy(tau_total, g_q, gravity_in_command_);
  for (int i = 0; i < 6; ++i) {
    sat_flags_.saturated[i] = std::abs(tau_unsat[i]) > tau_max_[i];
    sat_flags_.rate_limited[i] = std::abs(tau_cmd[i] - tau_sat[i]) > 1e-12;
  }
  onSaturation(sat_flags_);

  return tau_cmd;
}

Vector6d TorqueControlNodeBase::publishTau(const Vector6d & tau_law,
                                           const Vector6d & q,
                                           const Vector6d & dq)
{
  const Vector6d tau_cmd = commandFromLaw(tau_law, q, dq);
  tau_prev_cmd_ = tau_cmd;
  // FASE 3 — dry-run: se calcula todo (referencias, ley, limites) pero NO se
  // publica. Sirve para validar una configuracion contra el robot real sin
  // moverlo.
  if (!dry_run_) {
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(tau_cmd.data(), tau_cmd.data() + 6);
    tau_pub_->publish(cmd);
  }
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
    case State::SAFE_HOLD: return "SAFE_HOLD";
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
  if (have_state) {q_last_ = q;}

  // FASE 3 — el watchdog vigila el ritmo del lazo y la llegada de estado.
  // Si dispara, entra en SAFE_HOLD y este tick ya cae en ese caso.
  watchdogOk(dt);

  switch (state_) {
    case State::PRE_HOLD: {
        // Publicar sosten a ciegas desde el primer tick: cuando el controlador
        // de esfuerzo se active (primer paso de fisica) ya tendra comando.
        // Aun no hay /joint_states: se evalua g en q_init (con
        // gravity_in_command=false esto da exactamente 0, que es el comando
        // correcto para el robot real en reposo).
        publishTau(tau_hold_blind_, q_init_, Vector6d::Zero());
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
        publishTau(tau_hold_blind_, have_state ? q : q_init_,
                   have_state ? dq : Vector6d::Zero().eval());
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
        const Vector6d tau = publishTau(computeTau(q, dq, ref, dt), q, dq);
        if (++hold_log_counter_ % csv_hold_decimation_ == 0) {
          logRow(t_sim, q, dq, ref, tau, "HOLD_START");
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
        const Vector6d tau = publishTau(computeTau(q, dq, ref, dt), q, dq);
        logRow(t_sim, q, dq, ref, tau, "RAMP");
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
        const Vector6d tau = publishTau(computeTau(q, dq, ref, dt), q, dq);
        // La etiqueta la puede refinar el generador: el barrido de excitacion
        // marca la MESETA de velocidad constante, que es la ventana util para
        // identificar friccion (alli ddq = 0 y el residuo es friccion pura).
        std::string label = ref_gen_->phaseLabel(track_index_);
        if (label.empty()) {label = "TRACK";}
        logRow(t_sim, q, dq, ref, tau, label);

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
        const Vector6d tau = publishTau(computeTau(q, dq, ref, dt), q, dq);
        if (++hold_log_counter_ % csv_hold_decimation_ == 0) {
          logRow(t_sim, q, dq, ref, tau, "HOLD_END");
        }
        if (t_sim_limit_ > 0.0 && t_sim - t_state_start_ >= t_sim_limit_) {
          RCLCPP_INFO(get_logger(), "t_sim agotado; finalizando (hold se detiene).");
          enterState(State::DONE);
          timer_->cancel();
          csv_.close();
        }
        break;
      }

    case State::SAFE_HOLD: {
        // Estado terminal: se sostiene la ultima pose conocida con un PD sobre
        // la gravedad. No se usa computeTau() a proposito — si el lazo dejo de
        // ser fiable, la ley del controlador es justo lo que no hay que seguir
        // ejecutando. Con dq desconocido se usa 0, que es conservador.
        const Vector6d q_now = have_state ? q : q_safe_;
        const Vector6d dq_now = have_state ? dq : Vector6d::Zero();
        const Vector6d tau_hold = dyn_->gravity(q_now) +
          kSafeHoldKp.asDiagonal() * (q_safe_ - q_now) -
          kSafeHoldKd.asDiagonal() * dq_now;
        publishTau(tau_hold, q_now, dq_now);
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                              "SAFE_HOLD: sosteniendo la ultima pose conocida.");
        break;
      }

    case State::DONE:
    default:
      break;
  }
}

}  // namespace ur5_dyn_control
