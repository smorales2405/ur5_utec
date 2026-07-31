// ============================================================================
//  Tests de las piezas puras de la FASE 3: límite de tasa del comando,
//  marcas de saturación y hash de trazabilidad del CSV.
//
//  El watchdog y la máquina de estados viven dentro del nodo (necesitan ROS) y
//  se validan con la corrida de aceptación en Gazebo; aquí se cubre lo que se
//  puede aislar, que es donde está la aritmética con la que uno se equivoca.
// ============================================================================

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <map>
#include <string>

#include "ur5_dyn_control/common.hpp"
#include "ur5_dyn_control/csv_logger.hpp"
#include "ur5_dyn_control/torque_command.hpp"

using ur5_dyn_control::CsvLogger;
using ur5_dyn_control::SaturationFlags;
using ur5_dyn_control::Vector6d;
using ur5_dyn_control::rateLimit;

namespace
{
Vector6d vec6(double a, double b, double c, double d, double e, double f)
{
  return (Vector6d() << a, b, c, d, e, f).finished();
}
}  // namespace

// ── Límite de tasa ───────────────────────────────────────────────────────────
TEST(RateLimit, ClampsTheStepToRateTimesDt)
{
  const Vector6d prev = Vector6d::Zero();
  const Vector6d rate = Vector6d::Constant(100.0);   // 100 N·m/s
  const double dt = 0.002;                           // -> paso máximo 0.2 N·m

  // Salto grande en ambos sentidos: se recorta a ±0.2.
  const Vector6d big = vec6(50.0, -50.0, 5.0, -5.0, 0.3, -0.3);
  const Vector6d out = rateLimit(big, prev, rate, dt);
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(std::abs(out[i]), 0.2, 1e-12) << "junta " << i;
    EXPECT_EQ(out[i] > 0, big[i] > 0) << "el recorte cambió el signo, junta " << i;
  }
}

TEST(RateLimit, LeavesSmallChangesUntouched)
{
  const Vector6d prev = vec6(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  const Vector6d rate = Vector6d::Constant(100.0);
  const double dt = 0.002;                           // paso máximo 0.2
  const Vector6d small = prev + vec6(0.1, -0.1, 0.05, -0.05, 0.19, -0.19);
  EXPECT_TRUE(rateLimit(small, prev, rate, dt) == small);
}

TEST(RateLimit, ZeroRateDisablesTheLimit)
{
  const Vector6d prev = Vector6d::Zero();
  const Vector6d huge = Vector6d::Constant(1000.0);
  EXPECT_TRUE(rateLimit(huge, prev, Vector6d::Zero(), 0.002) == huge);
}

TEST(RateLimit, IsPerJoint)
{
  const Vector6d prev = Vector6d::Zero();
  // Solo la junta 0 está limitada; el resto pasa libre.
  const Vector6d rate = vec6(100.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  const Vector6d tau = Vector6d::Constant(10.0);
  const Vector6d out = rateLimit(tau, prev, rate, 0.002);
  EXPECT_NEAR(out[0], 0.2, 1e-12);
  for (int i = 1; i < 6; ++i) {EXPECT_DOUBLE_EQ(out[i], 10.0) << "junta " << i;}
}

TEST(RateLimit, NonPositiveDtIsANoOp)
{
  const Vector6d prev = Vector6d::Zero();
  const Vector6d tau = Vector6d::Constant(10.0);
  EXPECT_TRUE(rateLimit(tau, prev, Vector6d::Constant(1.0), 0.0) == tau);
}

// Aplicado repetidamente, converge al objetivo a la tasa pedida: es lo que hace
// que el comando sea continuo en vez de escalonado.
TEST(RateLimit, ConvergesToTargetAtTheRequestedRate)
{
  const Vector6d rate = Vector6d::Constant(100.0);
  const Vector6d target = Vector6d::Constant(1.0);
  const double dt = 0.002;
  Vector6d tau = Vector6d::Zero();
  // 1.0 N·m a 100 N·m/s = 10 ms = 5 pasos de 2 ms.
  for (int k = 0; k < 5; ++k) {
    tau = rateLimit(target, tau, rate, dt);
  }
  EXPECT_NEAR(tau[0], 1.0, 1e-12);
}

// ── Marcas de saturación ─────────────────────────────────────────────────────
TEST(SaturationFlagsTest, AnyReportsEitherCause)
{
  SaturationFlags f;
  EXPECT_FALSE(f.any());
  f.saturated[3] = true;
  EXPECT_TRUE(f.any());
  f.saturated[3] = false;
  f.rate_limited[0] = true;
  EXPECT_TRUE(f.any());
}

// ── Hash de trazabilidad ─────────────────────────────────────────────────────
TEST(TraceHash, IsDeterministic)
{
  const std::map<std::string, std::string> p{{"kp", "100.0"}, {"control_rate", "500.0"}};
  EXPECT_EQ(CsvLogger::hashParameters(p), CsvLogger::hashParameters(p));
}

// Lo que motiva hashear los parámetros EFECTIVOS y no el fichero YAML: dos
// corridas con el mismo YAML pero distinto override tienen que distinguirse.
TEST(TraceHash, DistinguishesRunsThatDifferOnlyInAnOverride)
{
  std::map<std::string, std::string> a{{"sweep.joint", "1"}, {"control_rate", "500.0"}};
  std::map<std::string, std::string> b{{"sweep.joint", "3"}, {"control_rate", "500.0"}};
  EXPECT_NE(CsvLogger::hashParameters(a), CsvLogger::hashParameters(b));
}

TEST(TraceHash, DetectsAnyValueChange)
{
  std::map<std::string, std::string> a{{"kp", "100.0"}};
  std::map<std::string, std::string> b{{"kp", "100.1"}};
  EXPECT_NE(CsvLogger::hashParameters(a), CsvLogger::hashParameters(b));
}

TEST(TraceHash, DetectsAddedParameters)
{
  std::map<std::string, std::string> a{{"kp", "100.0"}};
  std::map<std::string, std::string> b{{"kp", "100.0"}, {"kd", "20.0"}};
  EXPECT_NE(CsvLogger::hashParameters(a), CsvLogger::hashParameters(b));
}

TEST(TraceHash, IsFixedWidthHex)
{
  const std::string h = CsvLogger::hashParameters({{"a", "1"}});
  EXPECT_EQ(h.size(), 16u);
  EXPECT_EQ(h.find_first_not_of("0123456789abcdef"), std::string::npos);
}

// ── El CSV sigue siendo legible por nombre de columna ────────────────────────
TEST(CsvSchema, HeaderKeepsTheLegacyColumnNames)
{
  CsvLogger log;
  const std::string dir = "/tmp/ur5_dyn_control_test";
  ASSERT_TRUE(log.open(dir, "schema_test", 1, {{"git_sha", "abc123"}}));
  const std::string path = log.path();
  log.close();

  std::ifstream fh(path);
  ASSERT_TRUE(fh.is_open());
  std::string line, header;
  int n_comments = 0;
  while (std::getline(fh, line)) {
    if (!line.empty() && line[0] == '#') {
      ++n_comments;
      continue;
    }
    header = line;
    break;
  }
  EXPECT_GE(n_comments, 4) << "faltan lineas de trazabilidad";

  // Los nombres que ya consumian los scripts existentes deben seguir ahi, para
  // que los lectores por nombre (genfromtxt(names=True), DictReader) no rompan.
  // Se busca el nombre EXACTO entre comas, para no confundir "x" con "x_des".
  const std::string padded = "," + header + ",";
  for (const char * needed : {"q1", "q6", "dq1", "q1_des", "ddq6_des", "tau1",
                              "x", "y", "z", "x_des", "z_des", "state"})
  {
    EXPECT_NE(padded.find(std::string(",") + needed + ","), std::string::npos)
      << "falta la columna heredada '" << needed << "'";
  }
  // Y las nuevas de la FASE 3.
  for (const char * added : {"t_wall", "t_sim", "tau1_sat", "s1", "theta_err",
                             "wrench1"})
  {
    EXPECT_NE(header.find(added), std::string::npos)
      << "falta la columna nueva '" << added << "'";
  }
  std::remove(path.c_str());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
