// ============================================================================
//  gz_fl_control_node — Feedback Linearization / Computed Torque (UR5e)
//
//  Ley de control (espacio articular):
//    v   = ddq_des + Kp·(q_des − q) + Kd·(dq_des − dq)
//    tau = M(q)·v + n(q, dq)            [n = C·dq + g, via Pinocchio]
//
//  Referencias: trayectoria cartesiana (spline cubico clamped, orientacion
//  TCP fija rpy=[pi,0,-pi/2]) convertida offline a {q, dq, ddq} por
//  JointReferenceGenerator (IK QP + Jacobiano). Ver fl_control_params.yaml.
//
//  Fases: PRE_HOLD (g(q_init) a ciegas + activacion + despausado) ->
//  HOLD_START -> RAMP (quintica) -> TRACK -> HOLD_END.  CSV para MATLAB.
// ============================================================================

#include <cstdio>
#include <string>

#include "ur5_dyn_control/torque_control_node_base.hpp"

namespace ur5_dyn_control
{

class FlControlNode : public TorqueControlNodeBase
{
public:
  FlControlNode()
  : TorqueControlNodeBase("gz_fl_control_node")
  {
    auto vec6 = [this](const char * name, std::vector<double> def) {
        const auto v = declare_parameter<std::vector<double>>(name, def);
        Vector6d out;
        for (int i = 0; i < 6; ++i) {out[i] = v.at(i);}
        return out;
      };
    // Criticamente amortiguado con wn = 10 rad/s: kp = wn^2, kd = 2*wn.
    kp_ = vec6("kp", {100.0, 100.0, 100.0, 100.0, 100.0, 100.0});
    kd_ = vec6("kd", {20.0, 20.0, 20.0, 20.0, 20.0, 20.0});
    RCLCPP_INFO(get_logger(),
                "FL kp=[%.0f %.0f %.0f %.0f %.0f %.0f] kd=[%.0f %.0f %.0f %.0f %.0f %.0f]",
                kp_[0], kp_[1], kp_[2], kp_[3], kp_[4], kp_[5],
                kd_[0], kd_[1], kd_[2], kd_[3], kd_[4], kd_[5]);
    // B4.1 / plan 4.5: G = d(tau)/d(dq) = M*Kd sobre toda la referencia, la
    // misma magnitud y la misma cota que en el SMC y el LQR-SDRE.
    reportLoopGain("M_ii*kd_i", [this](const Vector6d & q, const Vector6d &,
      const Vector6d &) {return Matrix6d(dyn().M(q) * kd_.asDiagonal());});
    // B4.1: estabilidad del lazo DISCRETO ideal (500 Hz, un paso de retardo):
    // radio espectral por junta y cuanto se pueden escalar kp y kd juntos antes
    // de inestabilizarlo. El limite real del UR5e es la G de arriba, mas bajo.
    {
      const double dt = 1.0 / controlRate();
      std::string txt;
      char b[64];
      for (int i = 0; i < 6; ++i) {
        std::snprintf(b, sizeof(b), "%s%.4f/x%.0f", i ? "  " : "",
                      pdDiscreteSpectralRadius(kp_[i], kd_[i], dt, 1),
                      pdDiscreteGainMargin(kp_[i], kd_[i], dt, 1));
        txt += b;
      }
      RCLCPP_INFO(get_logger(), "  lazo discreto (rho / margen de ganancia) por junta: %s",
                  txt.c_str());
    }
    start();
  }

protected:
  Vector6d computeTau(const Vector6d & q, const Vector6d & dq,
                      const JointRef & ref, double /*dt*/) override
  {
    const Vector6d e = ref.q - q;
    const Vector6d de = ref.dq - dq;
    const Vector6d v = ref.ddq + kp_.asDiagonal() * e + kd_.asDiagonal() * de;
    return dyn().M(q) * v + dyn().nle(q, dq);
  }

  std::string csvPrefix() const override {return "fl";}

private:
  Vector6d kp_, kd_;
};

}  // namespace ur5_dyn_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ur5_dyn_control::FlControlNode>());
  rclcpp::shutdown();
  return 0;
}
