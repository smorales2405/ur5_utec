// ============================================================================
//  Test unitario de la COMPUERTA G3 — gravedad fuera del comando de torque
//
//  El UR5e real compensa la gravedad internamente cuando se comanda por
//  direct_torque() (forward_effort_controller del driver UR). Comandar g(q)
//  otra vez la DUPLICA: el brazo se acelera hacia arriba. Este test es el
//  requisito bloqueante del plan: "sin este test, prohibido tocar el robot".
//
//  Se ejercita la MISMA ruta de codigo que usa TorqueControlNodeBase para
//  publicar (torque_command.hpp), con el modelo Pinocchio real del UR5e.
// ============================================================================

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cmath>
#include <string>
#include <vector>

#include "ur5_dyn_control/common.hpp"
#include "ur5_dyn_control/torque_command.hpp"
#include "ur5_dyn_control/ur5_dynamics.hpp"

using ur5_dyn_control::Vector6d;
using ur5_dyn_control::applyGravityPolicy;
using ur5_dyn_control::saturate;
using ur5_dyn_control::torqueCommand;

namespace
{

constexpr double kGravity = 9.8;      // igual que <gravity> del mundo Gazebo
constexpr double kTolNm = 1e-9;       // cancelacion exacta salvo redondeo

std::string urdfPath()
{
  return ament_index_cpp::get_package_share_directory("ur5_kinematics") + "/ur5e.urdf";
}

Vector6d vec6(double a, double b, double c, double d, double e, double f)
{
  return (Vector6d() << a, b, c, d, e, f).finished();
}

/// Poses de prueba: q_init del paquete + configuraciones con brazo extendido
/// (donde g(q) es grande y un error de signo seria evidente).
std::vector<Vector6d> testConfigurations()
{
  const double pi = M_PI;
  return {
    vec6(0.0, -pi / 2, pi / 2, -pi / 2, -pi / 2, 0.0),   // q_init del paquete
    vec6(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),                  // brazo horizontal extendido
    vec6(0.3, -1.2, 0.9, -1.9, -1.4, 0.7),
    vec6(-1.1, -0.4, 1.8, -2.6, 1.2, -0.9),
    vec6(pi / 2, -pi / 4, pi / 4, 0.0, pi / 3, pi),
  };
}

const Vector6d kTauMax = vec6(150.0, 150.0, 150.0, 28.0, 28.0, 28.0);

}  // namespace

// ── G3, caso obligatorio del plan ────────────────────────────────────────────
// Ley de compensacion de gravedad pura (gz_gravity_comp_node con e = 0, dq = 0):
// tau_ley = g(q). Con gravity_in_command=false el comando debe ser ~0.
TEST(GravityPolicy, PureGravityLawCommandsZeroOnRealRobot)
{
  ur5_dyn_control::Ur5Dynamics dyn(urdfPath(), kGravity);

  for (const auto & q : testConfigurations()) {
    const Vector6d g_q = dyn.gravity(q);
    const Vector6d tau_law = g_q;   // kp*(q_ref - q) = 0, kd*dq = 0

    const Vector6d tau_cmd = torqueCommand(tau_law, g_q, /*gravity_in_command=*/false, kTauMax);

    EXPECT_LT(tau_cmd.cwiseAbs().maxCoeff(), kTolNm)
      << "q = " << q.transpose() << "\ntau_cmd = " << tau_cmd.transpose();
  }
}

// Misma comprobacion para Feedback Linearization en reposo sobre una
// referencia estatica: tau_ley = M(q)*0 + n(q, 0) = g(q)  ->  comando ~0.
// Es la prueba de que gz_fl_control_node tampoco compensa dos veces.
TEST(GravityPolicy, FeedbackLinearizationAtRestCommandsZeroOnRealRobot)
{
  ur5_dyn_control::Ur5Dynamics dyn(urdfPath(), kGravity);
  const Vector6d zero = Vector6d::Zero();

  for (const auto & q : testConfigurations()) {
    // q_des = q, dq_des = dq = 0, ddq_des = 0  =>  v = 0
    const Vector6d tau_law = dyn.M(q) * zero + dyn.nle(q, zero);
    const Vector6d g_q = dyn.gravity(q);

    // n(q, 0) tiene que ser exactamente g(q) (sin terminos de Coriolis).
    ASSERT_LT((tau_law - g_q).cwiseAbs().maxCoeff(), kTolNm) << "q = " << q.transpose();

    const Vector6d tau_cmd = torqueCommand(tau_law, g_q, /*gravity_in_command=*/false, kTauMax);
    EXPECT_LT(tau_cmd.cwiseAbs().maxCoeff(), kTolNm) << "q = " << q.transpose();
  }
}

// La politica solo quita la gravedad: la accion de control debe sobrevivir
// intacta (si se restara de mas, el lazo perderia ganancia en el robot real).
TEST(GravityPolicy, KeepsNonGravityTermsUntouched)
{
  ur5_dyn_control::Ur5Dynamics dyn(urdfPath(), kGravity);
  const Vector6d pd = vec6(1.5, -2.5, 3.5, -0.5, 0.25, -0.125);

  for (const auto & q : testConfigurations()) {
    const Vector6d g_q = dyn.gravity(q);
    const Vector6d tau_cmd =
      torqueCommand(g_q + pd, g_q, /*gravity_in_command=*/false, kTauMax);
    EXPECT_LT((tau_cmd - pd).cwiseAbs().maxCoeff(), kTolNm) << "q = " << q.transpose();
  }
}

// Regresion de Gazebo: con gravity_in_command=true el comando es la ley tal
// cual (bit a bit), asi que las corridas ya validadas no cambian.
TEST(GravityPolicy, GazeboPolicyIsPassThrough)
{
  ur5_dyn_control::Ur5Dynamics dyn(urdfPath(), kGravity);
  const Vector6d extra = vec6(3.0, -7.0, 11.0, -0.4, 0.9, -1.3);

  for (const auto & q : testConfigurations()) {
    const Vector6d g_q = dyn.gravity(q);
    const Vector6d tau_law = g_q + extra;
    const Vector6d tau_cmd =
      torqueCommand(tau_law, g_q, /*gravity_in_command=*/true, kTauMax);
    EXPECT_TRUE(tau_cmd == tau_law) << "q = " << q.transpose();
  }
}

// El modelo debe honrar la gravedad configurada: si el mundo no tuviera
// gravedad, g(q) = 0 y ambas politicas coincidirian. Guarda contra un
// mismatch silencioso modelo <-> planta (R6 del plan original).
TEST(GravityPolicy, ModelHonoursConfiguredGravity)
{
  ur5_dyn_control::Ur5Dynamics dyn_zero(urdfPath(), 0.0);
  ur5_dyn_control::Ur5Dynamics dyn_g(urdfPath(), kGravity);

  for (const auto & q : testConfigurations()) {
    EXPECT_LT(dyn_zero.gravity(q).cwiseAbs().maxCoeff(), kTolNm);
  }
  // Con el brazo horizontal extendido la gravedad tiene que ser apreciable.
  const Vector6d q_horizontal = vec6(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  EXPECT_GT(dyn_g.gravity(q_horizontal).cwiseAbs().maxCoeff(), 10.0);
}

// ── Saturacion ───────────────────────────────────────────────────────────────
TEST(TorqueSaturation, IsSymmetricAndComponentWise)
{
  const Vector6d tau = vec6(1000.0, -1000.0, 10.0, 100.0, -100.0, 0.0);
  const Vector6d sat = saturate(tau, kTauMax);
  EXPECT_DOUBLE_EQ(sat[0], 150.0);
  EXPECT_DOUBLE_EQ(sat[1], -150.0);
  EXPECT_DOUBLE_EQ(sat[2], 10.0);     // dentro de limite: sin tocar
  EXPECT_DOUBLE_EQ(sat[3], 28.0);
  EXPECT_DOUBLE_EQ(sat[4], -28.0);
  EXPECT_DOUBLE_EQ(sat[5], 0.0);
}

// El ORDEN importa: primero se resta g(q), despues se satura. Con tau_max
// conservador (§7 del plan: 30 % del nominal en el primer ensayo real) el
// limite tiene que aplicarse al comando, no a la ley.
TEST(TorqueSaturation, AppliesToCommandNotToLaw)
{
  const Vector6d tau_max_conservador = 0.3 * kTauMax;
  const Vector6d g_q = vec6(0.0, 40.0, 20.0, 1.0, 0.0, 0.0);
  const Vector6d tau_law = g_q + vec6(0.0, 10.0, 5.0, 0.5, 0.0, 0.0);

  const Vector6d cmd = torqueCommand(tau_law, g_q, false, tau_max_conservador);
  // tau_ley - g = [0, 10, 5, 0.5, 0, 0]: por debajo de 0.3*tau_max -> sin saturar.
  EXPECT_LT((cmd - vec6(0.0, 10.0, 5.0, 0.5, 0.0, 0.0)).cwiseAbs().maxCoeff(), kTolNm);

  // Saturar la LEY antes de restar g habria recortado a 45 N·m y dado -(-)...
  // aqui se comprueba que ese camino NO es el implementado.
  const Vector6d cmd_wrong_order = saturate(tau_law, tau_max_conservador) - g_q;
  EXPECT_GT((cmd - cmd_wrong_order).cwiseAbs().maxCoeff(), 1.0);
}

TEST(GravityPolicy, ApplyGravityPolicyMatchesDefinition)
{
  const Vector6d tau_law = vec6(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  const Vector6d g_q = vec6(0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
  EXPECT_TRUE(applyGravityPolicy(tau_law, g_q, true) == tau_law);
  EXPECT_TRUE(applyGravityPolicy(tau_law, g_q, false) == Vector6d(tau_law - g_q));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
