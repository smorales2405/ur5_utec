// ============================================================================
//  gz_gravity_comp_node — Compensacion de gravedad + amortiguamiento (UR5e)
//
//  Ley:  tau = g(q) + Kp·(q0 − q) − Kd·dq
//
//  Smoke test de toda la cadena de control por torque: interfaz effort,
//  activacion con la sim pausada, despausado sin caida, y consistencia del
//  modelo Pinocchio con la planta de Gazebo. Con kp=0 queda la compensacion
//  de gravedad pura (el brazo deriva lentamente: la planta no tiene friccion).
//
//  Por defecto skip_trajectory=true: regulacion indefinida en q_init.
// ============================================================================

#include "ur5_dyn_control/torque_control_node_base.hpp"

namespace ur5_dyn_control
{

class GravityCompNode : public TorqueControlNodeBase
{
public:
  GravityCompNode()
  : TorqueControlNodeBase("gz_gravity_comp_node")
  {
    auto vec6 = [this](const char * name, std::vector<double> def) {
        const auto v = declare_parameter<std::vector<double>>(name, def);
        Vector6d out;
        for (int i = 0; i < 6; ++i) {out[i] = v.at(i);}
        return out;
      };
    kp_ = vec6("kp", {60.0, 60.0, 60.0, 10.0, 5.0, 2.0});
    kd_ = vec6("kd", {10.0, 10.0, 10.0, 1.0, 0.5, 0.2});
    RCLCPP_INFO(get_logger(), "GravityComp kp=[%.1f...] kd=[%.1f...]", kp_[0], kd_[0]);
    start();
  }

protected:
  Vector6d computeTau(const Vector6d & q, const Vector6d & dq,
                      const JointRef & ref, double /*dt*/) override
  {
    return dyn().gravity(q) + kp_.asDiagonal() * (ref.q - q) - kd_.asDiagonal() * dq;
  }

  std::string csvPrefix() const override {return "gravity_comp";}

private:
  Vector6d kp_, kd_;
};

}  // namespace ur5_dyn_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ur5_dyn_control::GravityCompNode>());
  rclcpp::shutdown();
  return 0;
}
