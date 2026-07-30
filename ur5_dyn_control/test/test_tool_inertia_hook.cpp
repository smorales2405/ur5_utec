// ============================================================================
//  Test del HOOK de herramienta (supuesto A1) y del offset del TCP (A2)
//
//  A1 se declara como supuesto: se desprecia la masa del acople del bisturi y
//  el modelo sigue siendo el de brazo solo. Este test verifica que
//   (a) el hook es NEUTRO con mass = 0 -> el supuesto vigente no cambia nada;
//   (b) el hook es CORRECTO con mass > 0 -> cuando se midan las propiedades
//       reales del acople, basta rellenar el YAML y el modelo queda bien.
//
//  Sin (b), el hook seria una declaracion sin respaldo: no se sabria si el
//  dia que se levante A1 el resultado es valido.
// ============================================================================

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cmath>
#include <string>
#include <vector>

#include "ur5_dyn_control/common.hpp"
#include "ur5_dyn_control/ur5_dynamics.hpp"

using ur5_dyn_control::ToolInertia;
using ur5_dyn_control::Ur5Dynamics;
using ur5_dyn_control::Vector6d;

namespace
{

constexpr double kGravity = 9.8;
constexpr double kTcpOffsetZ = 0.141;

std::string urdfPath()
{
  return ament_index_cpp::get_package_share_directory("ur5_kinematics") + "/ur5e.urdf";
}

Vector6d vec6(double a, double b, double c, double d, double e, double f)
{
  return (Vector6d() << a, b, c, d, e, f).finished();
}

std::vector<Vector6d> testConfigurations()
{
  const double pi = M_PI;
  return {
    vec6(0.0, -pi / 2, pi / 2, -pi / 2, -pi / 2, 0.0),
    vec6(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    vec6(0.3, -1.2, 0.9, -1.9, -1.4, 0.7),
    vec6(-1.1, -0.4, 1.8, -2.6, 1.2, -0.9),
  };
}

}  // namespace

// (a) El supuesto A1 vigente: sin herramienta el modelo es identico al de antes.
TEST(ToolInertiaHook, ZeroMassLeavesModelUntouched)
{
  Ur5Dynamics plain(urdfPath(), kGravity, kTcpOffsetZ);
  Ur5Dynamics with_zero_tool(urdfPath(), kGravity, kTcpOffsetZ, ToolInertia{});

  for (const auto & q : testConfigurations()) {
    EXPECT_LT((plain.gravity(q) - with_zero_tool.gravity(q)).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LT((plain.M(q) - with_zero_tool.M(q)).cwiseAbs().maxCoeff(), 1e-12);
  }
}

// (b) El hook es correcto: para una masa PUNTUAL m anclada en el TCP, la
// energia potencial anadida es U = m·g·z_tcp(q), luego
//     Δg(q) = ∂U/∂q = m·g·[fila z del Jacobiano lineal del TCP]ᵀ
// Se compara contra el Jacobiano que ya calcula Ur5Dynamics: si el hook
// anclara la masa en el frame equivocado, esta identidad fallaria.
TEST(ToolInertiaHook, PointMassMatchesJacobianDerivedGravity)
{
  const double m = 0.35;   // valor de PRUEBA del test, no un dato del acople real

  ToolInertia tool;
  tool.mass = m;                                    // CoM en el origen del TCP
  Ur5Dynamics plain(urdfPath(), kGravity, kTcpOffsetZ);
  Ur5Dynamics with_tool(urdfPath(), kGravity, kTcpOffsetZ, tool);

  for (const auto & q : testConfigurations()) {
    const Vector6d delta_g = with_tool.gravity(q) - plain.gravity(q);
    // frameJacobian es LOCAL_WORLD_ALIGNED: la fila 2 es ∂z_tcp/∂q.
    const Vector6d expected = m * kGravity * plain.frameJacobian(q).row(2).transpose();
    EXPECT_LT((delta_g - expected).cwiseAbs().maxCoeff(), 1e-9)
      << "q = " << q.transpose()
      << "\ndelta_g = " << delta_g.transpose()
      << "\nesperado = " << expected.transpose();
  }
}

// La masa de la herramienta tiene que aumentar la inercia articular: guarda
// contra un signo invertido en appendBodyToJoint.
TEST(ToolInertiaHook, PointMassIncreasesInertia)
{
  ToolInertia tool;
  tool.mass = 0.35;

  Ur5Dynamics plain(urdfPath(), kGravity, kTcpOffsetZ);
  Ur5Dynamics with_tool(urdfPath(), kGravity, kTcpOffsetZ, tool);

  for (const auto & q : testConfigurations()) {
    EXPECT_GT(with_tool.M(q).trace(), plain.M(q).trace()) << "q = " << q.transpose();
  }
}

// (A2) El offset del TCP es un parametro efectivo, no un valor cosido: al
// cambiarlo, la FK del TCP se desplaza exactamente esa distancia a lo largo
// del eje Z de tool0.
TEST(TcpOffset, IsHonouredByForwardKinematics)
{
  const double extra = 0.05;
  Ur5Dynamics base(urdfPath(), kGravity, kTcpOffsetZ);
  Ur5Dynamics shifted(urdfPath(), kGravity, kTcpOffsetZ + extra);

  EXPECT_DOUBLE_EQ(base.tcpOffsetZ(), kTcpOffsetZ);
  EXPECT_DOUBLE_EQ(shifted.tcpOffsetZ(), kTcpOffsetZ + extra);

  for (const auto & q : testConfigurations()) {
    const auto T_base = base.fk(q);
    const auto T_shift = shifted.fk(q);
    // Misma orientacion, desplazado 'extra' a lo largo del Z local del TCP.
    EXPECT_LT((T_base.rotation() - T_shift.rotation()).cwiseAbs().maxCoeff(), 1e-12);
    const Eigen::Vector3d expected =
      T_base.translation() + T_base.rotation() * Eigen::Vector3d(0.0, 0.0, extra);
    EXPECT_LT((T_shift.translation() - expected).cwiseAbs().maxCoeff(), 1e-12)
      << "q = " << q.transpose();
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
