#include "ur5_pick_place/ik_wrapper.hpp"
#include "ur5_pick_place/trajectory_generator.hpp"

#include <pinocchio/algorithm/rnea.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

using FollowJT = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJT>;
using namespace ur5_pick_place;

// ─────────────────────────────────────────────────────────────────────────────
static Eigen::Matrix3d rpy_to_matrix(double r, double p, double y)
{
    return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()))
               .toRotationMatrix();
}

// ─────────────────────────────────────────────────────────────────────────────
class PickPlaceNode : public rclcpp::Node
{
public:
    PickPlaceNode()
    : Node("pick_place_node")
    {
        // ── Declare & read parameters ─────────────────────────────────────
        declare_parameter("method",             "clamped_spline");
        declare_parameter("points_per_segment", 20);
        declare_parameter("total_duration",     8.0);
        declare_parameter("start_delay",        5.0);
        declare_parameter("tcp_orientation_rpy", std::vector<double>{M_PI, 0.0, 0.0});
        declare_parameter("point_A_pre", std::vector<double>{0.45,  0.35, 0.45});
        declare_parameter("point_A",     std::vector<double>{0.45,  0.35, 0.30});
        declare_parameter("point_via",   std::vector<double>{0.55,  0.00, 0.55});
        declare_parameter("point_B",     std::vector<double>{0.45, -0.35, 0.30});
        declare_parameter("point_B_post",std::vector<double>{0.45, -0.35, 0.45});
        declare_parameter("pre_post_duration", 0.5);
        declare_parameter("home_joint_angles",
            std::vector<double>{0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0});
        declare_parameter("ik_max_iterations", 100);
        declare_parameter("ik_alpha",          0.5);
        declare_parameter("ik_weight_pos",     1.0);
        declare_parameter("ik_weight_orient",  1.0);
        declare_parameter("csv_output_dir",    std::string(""));

        method_          = get_parameter("method").as_string();
        pts_per_seg_     = get_parameter("points_per_segment").as_int();
        total_duration_  = get_parameter("total_duration").as_double();
        start_delay_     = get_parameter("start_delay").as_double();
        auto rpy         = get_parameter("tcp_orientation_rpy").as_double_array();
        auto pAp         = get_parameter("point_A_pre").as_double_array();
        auto pA          = get_parameter("point_A").as_double_array();
        auto pV          = get_parameter("point_via").as_double_array();
        auto pB          = get_parameter("point_B").as_double_array();
        auto pBp         = get_parameter("point_B_post").as_double_array();
        pre_post_dur_    = get_parameter("pre_post_duration").as_double();
        auto home_arr    = get_parameter("home_joint_angles").as_double_array();
        home_q_          = Eigen::Map<const Eigen::VectorXd>(home_arr.data(), 6);
        ik_max_iter_     = get_parameter("ik_max_iterations").as_int();
        ik_alpha_        = get_parameter("ik_alpha").as_double();
        ik_weight_pos_   = get_parameter("ik_weight_pos").as_double();
        ik_weight_orient_= get_parameter("ik_weight_orient").as_double();

        csv_output_dir_ = get_parameter("csv_output_dir").as_string();
        if (csv_output_dir_.empty()) {
            const char* home_env = std::getenv("HOME");
            std::string home = home_env ? home_env : "/tmp";
            csv_output_dir_ = home + "/ur5_ws/src/ur5_utec/ur5_pick_place/data";
        }

        tcp_orient_   = rpy_to_matrix(rpy[0], rpy[1], rpy[2]);
        point_A_pre_  = {pAp[0], pAp[1], pAp[2]};
        point_A_      = {pA[0],  pA[1],  pA[2]};
        point_via_    = {pV[0],  pV[1],  pV[2]};
        point_B_      = {pB[0],  pB[1],  pB[2]};
        point_B_post_ = {pBp[0], pBp[1], pBp[2]};

        // ── IK wrapper ───────────────────────────────────────────────────
        std::string urdf_path =
            ament_index_cpp::get_package_share_directory("ur5_kinematics") +
            "/ur5.urdf";
        ik_ = std::make_unique<IKWrapper>(urdf_path);

        // ── Pinocchio model for RNEA (torque logging) ─────────────────────
        // Uses the same UR5-only URDF as the IK (no gripper; TCP offset is
        // handled analytically by IKWrapper). Gravity is set automatically
        // to (0,0,-9.81) by buildModel.
        pinocchio::urdf::buildModel(urdf_path, pin_model_);
        pin_data_ = std::make_unique<pinocchio::Data>(pin_model_);

        // ── Action client ─────────────────────────────────────────────────
        ac_ = rclcpp_action::create_client<FollowJT>(
            this, "/joint_trajectory_controller/follow_joint_trajectory");

        // ── Joint state subscriber (to seed IK with current robot state) ─
        js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                last_js_ = msg;
            });

        // Run once after spin starts
        timer_ = create_wall_timer(
            std::chrono::seconds(2),
            [this]() {
                timer_->cancel();
                runPickPlace();
            });

        RCLCPP_INFO(get_logger(), "pick_place_node ready — method: %s", method_.c_str());
        RCLCPP_INFO(get_logger(), "CSV output dir: %s", csv_output_dir_.c_str());
    }

private:
    // ── Members ───────────────────────────────────────────────────────────────
    std::unique_ptr<IKWrapper>  ik_;
    pinocchio::Model                    pin_model_;
    std::unique_ptr<pinocchio::Data>    pin_data_;
    rclcpp_action::Client<FollowJT>::SharedPtr ac_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState::SharedPtr last_js_;

    std::string      method_;
    int              pts_per_seg_;
    double           total_duration_;
    double           start_delay_;
    double           pre_post_dur_;
    Eigen::Matrix3d  tcp_orient_;
    Eigen::Vector3d  point_A_pre_, point_A_, point_via_, point_B_, point_B_post_;
    Eigen::VectorXd  home_q_;
    int              ik_max_iter_;
    double           ik_alpha_, ik_weight_pos_, ik_weight_orient_;
    std::string      csv_output_dir_;

    static constexpr const char* kJointNames[] = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};

    // ── Core logic ────────────────────────────────────────────────────────────
    void runPickPlace()
    {
        RCLCPP_INFO(get_logger(), "Waiting for action server (up to 60 s)...");
        if (!ac_->wait_for_action_server(std::chrono::seconds(60))) {
            RCLCPP_ERROR(get_logger(), "Action server not available after 60 s — is the simulation running?");
            return;
        }

        // Build keypoints: pre_A → A → via → B → post_B
        // pre_post_dur_ controls the approach (pre_A→A) and retreat (B→post_B) segments.
        // total_duration_ controls the central arc A→via→B (split evenly in two).
        const double t0    = start_delay_;
        const double t_pp  = pre_post_dur_;
        const double t_mid = total_duration_ / 2.0;
        std::vector<CartesianWaypoint> keypoints = {
            {point_A_pre_, tcp_orient_, t0},
            {point_A_,     tcp_orient_, t0 + t_pp},
            {point_via_,   tcp_orient_, t0 + t_pp + t_mid},
            {point_B_,     tcp_orient_, t0 + t_pp + total_duration_},
            {point_B_post_,tcp_orient_, t0 + t_pp + total_duration_ + t_pp},
        };

        // Generate dense cartesian trajectory
        std::vector<CartesianWaypoint> cart_traj;
        if (method_ == "piecewise_linear") {
            cart_traj = TrajectoryGenerator::piecewiseLinear(keypoints, pts_per_seg_);
            RCLCPP_INFO(get_logger(), "Using piecewise linear — %zu waypoints",
                        cart_traj.size());
        } else {
            // 3 independent clamped-spline segments → v=0 at A_pre, A, B, and B_post.
            // Seg 1: A_pre→A  (2 kp → v=0 at both ends)
            // Seg 2: A→via→B  (3 kp → v=0 at both ends)
            // Seg 3: B→B_post (2 kp → v=0 at both ends)
            std::vector<CartesianWaypoint> kp1 = {keypoints[0], keypoints[1]};
            std::vector<CartesianWaypoint> kp2 = {keypoints[1], keypoints[2], keypoints[3]};
            std::vector<CartesianWaypoint> kp3 = {keypoints[3], keypoints[4]};
            auto t1 = TrajectoryGenerator::clampedCubicSpline(kp1, pts_per_seg_);
            auto t2 = TrajectoryGenerator::clampedCubicSpline(kp2, pts_per_seg_);
            auto t3 = TrajectoryGenerator::clampedCubicSpline(kp3, pts_per_seg_);
            cart_traj = t1;
            cart_traj.insert(cart_traj.end(), t2.begin() + 1, t2.end());
            cart_traj.insert(cart_traj.end(), t3.begin() + 1, t3.end());
            RCLCPP_INFO(get_logger(), "Using clamped cubic spline — %zu waypoints",
                        cart_traj.size());
        }

        // Solve IK for each cartesian waypoint
        std::vector<Eigen::VectorXd> joint_traj;
        joint_traj.reserve(cart_traj.size());

        Eigen::VectorXd q_seed = home_q_;
        int ik_failures = 0;

        for (size_t i = 0; i < cart_traj.size(); ++i) {
            const auto& wp = cart_traj[i];
            Eigen::VectorXd q = ik_->solve(
                q_seed, wp.position, wp.orientation,
                ik_max_iter_, ik_alpha_, ik_weight_pos_, ik_weight_orient_);

            joint_traj.push_back(q);
            q_seed = q;

            if (i % 10 == 0) {
                RCLCPP_INFO(get_logger(), "IK %zu/%zu  q=[%.3f %.3f %.3f %.3f %.3f %.3f]",
                    i, cart_traj.size(),
                    q[0], q[1], q[2], q[3], q[4], q[5]);
            }
        }

        RCLCPP_INFO(get_logger(), "IK done — %zu joint waypoints, %d failures",
                    joint_traj.size(), ik_failures);

        // Export trajectory data to CSV
        exportToCsv(cart_traj, joint_traj);

        // Build and send JointTrajectory
        sendTrajectory(joint_traj, cart_traj);
    }

    // ── CSV export ────────────────────────────────────────────────────────────
    // Columns: time_s, tcp_x/y/z, waypoint(0=interp/1=A/2=via/3=B),
    //          vel_x/y/z, acc_x/y/z, q0..q5
    // Velocities and accelerations computed by central finite differences.
    void exportToCsv(
        const std::vector<CartesianWaypoint>& cart_traj,
        const std::vector<Eigen::VectorXd>&   joint_traj)
    {
        namespace fs = std::filesystem;

        // Create output directory
        std::error_code ec;
        fs::create_directories(csv_output_dir_, ec);
        if (ec) {
            RCLCPP_ERROR(get_logger(), "Cannot create CSV dir '%s': %s",
                         csv_output_dir_.c_str(), ec.message().c_str());
            return;
        }

        // Timestamped filename
        std::time_t now = std::time(nullptr);
        std::tm tm_buf{};
        localtime_r(&now, &tm_buf);
        char ts[32];
        std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm_buf);

        std::string filepath = csv_output_dir_ + "/trajectory_" +
                               std::string(ts) + "_" + method_ + ".csv";

        std::ofstream f(filepath);
        if (!f.is_open()) {
            RCLCPP_ERROR(get_logger(), "Cannot open '%s' for writing", filepath.c_str());
            return;
        }

        const int N = static_cast<int>(cart_traj.size());

        // ── Cartesian velocities (central differences, m/s) ──────────────
        std::vector<Eigen::Vector3d> vel(N);
        for (int i = 0; i < N; ++i) {
            if (i == 0) {
                double dt = cart_traj[1].timestamp - cart_traj[0].timestamp;
                vel[i] = (cart_traj[1].position - cart_traj[0].position) / dt;
            } else if (i == N - 1) {
                double dt = cart_traj[N-1].timestamp - cart_traj[N-2].timestamp;
                vel[i] = (cart_traj[N-1].position - cart_traj[N-2].position) / dt;
            } else {
                double dt = cart_traj[i+1].timestamp - cart_traj[i-1].timestamp;
                vel[i] = (cart_traj[i+1].position - cart_traj[i-1].position) / dt;
            }
        }

        // ── Cartesian accelerations (central differences on vel, m/s²) ───
        std::vector<Eigen::Vector3d> acc(N);
        for (int i = 0; i < N; ++i) {
            if (i == 0) {
                double dt = cart_traj[1].timestamp - cart_traj[0].timestamp;
                acc[i] = (vel[1] - vel[0]) / dt;
            } else if (i == N - 1) {
                double dt = cart_traj[N-1].timestamp - cart_traj[N-2].timestamp;
                acc[i] = (vel[N-1] - vel[N-2]) / dt;
            } else {
                double dt = cart_traj[i+1].timestamp - cart_traj[i-1].timestamp;
                acc[i] = (vel[i+1] - vel[i-1]) / dt;
            }
        }

        // ── Joint velocities (central differences, rad/s) ────────────────
        std::vector<Eigen::VectorXd> dq(N, Eigen::VectorXd::Zero(6));
        for (int i = 0; i < N; ++i) {
            if (i == 0) {
                double dt = cart_traj[1].timestamp - cart_traj[0].timestamp;
                dq[i] = (joint_traj[1] - joint_traj[0]) / dt;
            } else if (i == N - 1) {
                double dt = cart_traj[N-1].timestamp - cart_traj[N-2].timestamp;
                dq[i] = (joint_traj[N-1] - joint_traj[N-2]) / dt;
            } else {
                double dt = cart_traj[i+1].timestamp - cart_traj[i-1].timestamp;
                dq[i] = (joint_traj[i+1] - joint_traj[i-1]) / dt;
            }
        }

        // ── Joint accelerations (central differences on dq, rad/s²) ─────
        std::vector<Eigen::VectorXd> ddq(N, Eigen::VectorXd::Zero(6));
        for (int i = 0; i < N; ++i) {
            if (i == 0) {
                double dt = cart_traj[1].timestamp - cart_traj[0].timestamp;
                ddq[i] = (dq[1] - dq[0]) / dt;
            } else if (i == N - 1) {
                double dt = cart_traj[N-1].timestamp - cart_traj[N-2].timestamp;
                ddq[i] = (dq[N-1] - dq[N-2]) / dt;
            } else {
                double dt = cart_traj[i+1].timestamp - cart_traj[i-1].timestamp;
                ddq[i] = (dq[i+1] - dq[i-1]) / dt;
            }
        }

        // ── Joint torques via RNEA (Nm) ──────────────────────────────────
        // τ = M(q)·q̈ + C(q,q̇)·q̇ + g(q), computed with Pinocchio.
        // dq and ddq come from finite differences on the IK solution, so
        // torque values are approximate but consistent with the trajectory.
        std::vector<Eigen::VectorXd> tau(N);
        for (int i = 0; i < N; ++i) {
            tau[i] = pinocchio::rnea(
                pin_model_, *pin_data_,
                joint_traj[i], dq[i], ddq[i]);
        }

        // ── Keypoint tags ─────────────────────────────────────────────────
        // 4 segments × pts_per_seg_ steps each → N = 4*pts_per_seg_ + 1
        // Index 0                → pre_A  (tag=1)
        // Index   pts_per_seg_   → A      (tag=2)
        // Index 2*pts_per_seg_   → via    (tag=3)
        // Index 3*pts_per_seg_   → B      (tag=4)
        // Index N-1              → post_B (tag=5)
        std::vector<int> kp_tag(N, 0);
        kp_tag[0]                = 1;
        kp_tag[pts_per_seg_]     = 2;
        kp_tag[2 * pts_per_seg_] = 3;
        kp_tag[3 * pts_per_seg_] = 4;
        kp_tag[N - 1]            = 5;

        // ── Header ───────────────────────────────────────────────────────
        f << "time_s,"
          << "tcp_x,tcp_y,tcp_z,"
          << "waypoint,"
          << "vel_x,vel_y,vel_z,"
          << "acc_x,acc_y,acc_z,"
          << "q0,q1,q2,q3,q4,q5,"
          << "dq0,dq1,dq2,dq3,dq4,dq5,"
          << "tau0,tau1,tau2,tau3,tau4,tau5,"
          << "jerk_x,jerk_y,jerk_z\n";

        f << std::fixed << std::setprecision(6);

        for (int i = 0; i < N; ++i) {
            const auto& p    = cart_traj[i].position;
            const auto& jk   = cart_traj[i].jerk;
            const auto& v    = vel[i];
            const auto& a    = acc[i];
            const auto& q    = joint_traj[i];
            const auto& dqi  = dq[i];
            const auto& taui = tau[i];

            f << cart_traj[i].timestamp << ","
              << p.x() << "," << p.y() << "," << p.z() << ","
              << kp_tag[i] << ","
              << v.x() << "," << v.y() << "," << v.z() << ","
              << a.x() << "," << a.y() << "," << a.z() << ","
              << q[0]    << "," << q[1]    << "," << q[2]    << ","
              << q[3]    << "," << q[4]    << "," << q[5]    << ","
              << dqi[0]  << "," << dqi[1]  << "," << dqi[2]  << ","
              << dqi[3]  << "," << dqi[4]  << "," << dqi[5]  << ","
              << taui[0] << "," << taui[1] << "," << taui[2] << ","
              << taui[3] << "," << taui[4] << "," << taui[5] << ","
              << jk.x()  << "," << jk.y()  << "," << jk.z()  << "\n";
        }

        f.close();
        RCLCPP_INFO(get_logger(), "CSV exported → %s  (%d rows)", filepath.c_str(), N);
    }

    // ── Send trajectory action ────────────────────────────────────────────────
    void sendTrajectory(
        const std::vector<Eigen::VectorXd>& joint_traj,
        const std::vector<CartesianWaypoint>& cart_traj)
    {
        trajectory_msgs::msg::JointTrajectory jt;
        jt.joint_names = {
            kJointNames[0], kJointNames[1], kJointNames[2],
            kJointNames[3], kJointNames[4], kJointNames[5]};

        for (size_t i = 0; i < joint_traj.size(); ++i) {
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions.resize(6);
            for (int j = 0; j < 6; ++j) pt.positions[j] = joint_traj[i][j];
            pt.time_from_start = rclcpp::Duration::from_seconds(cart_traj[i].timestamp);
            jt.points.push_back(pt);
        }

        auto goal = FollowJT::Goal();
        goal.trajectory = jt;

        auto send_opts = rclcpp_action::Client<FollowJT>::SendGoalOptions();
        send_opts.goal_response_callback = [this](const GoalHandleFJT::SharedPtr& gh) {
            if (!gh) {
                RCLCPP_ERROR(get_logger(), "Goal REJECTED by action server.");
            } else {
                RCLCPP_INFO(get_logger(), "Goal ACCEPTED — robot is moving.");
            }
        };
        send_opts.result_callback = [this](const GoalHandleFJT::WrappedResult& res) {
            if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(get_logger(), "Trajectory executed successfully.");
            } else {
                RCLCPP_ERROR(get_logger(), "Trajectory failed (code %d)",
                             static_cast<int>(res.code));
            }
        };

        RCLCPP_INFO(get_logger(), "Sending %zu waypoints to follow_joint_trajectory",
                    jt.points.size());
        ac_->async_send_goal(goal, send_opts);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PickPlaceNode>());
    rclcpp::shutdown();
    return 0;
}
