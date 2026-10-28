// B4.2: LQI-SDRE (lqr_sdre_law.hpp) — sintesis polinomica, pesos fisicos,
// anti-windup y coste de la CARE de 18 estados.
#include <gtest/gtest.h>

#include <chrono>
#include <complex>
#include <random>

#include <Eigen/Eigenvalues>

#include "ur5_dyn_control/care_solver.hpp"
#include "ur5_dyn_control/lqr_sdre_law.hpp"

using namespace ur5_dyn_control;  // NOLINT

namespace
{

/// M simetrica definida positiva con la dispersion de inercias del UR5e
/// (de 2.6 a 2.6e-4 kg m^2) y acoplos.
Matrix6d spdMass(unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  Matrix6d A;
  for (int i = 0; i < 6; ++i) {for (int j = 0; j < 6; ++j) {A(i, j) = u(rng);}}
  Vector6d d;
  d << 2.6, 2.6, 0.9, 0.03, 0.006, 3e-4;
  const Matrix6d S = d.cwiseSqrt().asDiagonal();
  return S * (0.3 * A * A.transpose() + Matrix6d::Identity()) * S;
}

}  // namespace

TEST(Lqi, PolynomialSeedPlacesThe18PolesAtTheRoots)
{
  // a(s) = (s + wn)(s^2 + 2 zeta wn s + wn^2), con zeta = 0.8: raices
  // distintas (con zeta = 1 la raiz triple es defectiva y el autovalor solo se
  // resuelve a ~eps^(1/3)).
  const double wn = 10.0, zeta = 0.8, r = 1.0;
  const Matrix6d M = spdMass(1);
  Eigen::MatrixXd A, B;
  buildSdc(M, Matrix6d::Zero(), true, A, B);
  const CareResult res = solveCare(A, B, polynomialQ(M, wn, zeta, r, true),
                                   Eigen::MatrixXd::Identity(6, 6) * r);
  ASSERT_TRUE(res.ok) << res.reason;
  const std::complex<double> p1(-wn, 0.0);
  const std::complex<double> p2(-zeta * wn, wn * std::sqrt(1 - zeta * zeta));
  const auto ev = Eigen::EigenSolver<Eigen::MatrixXd>(A - B * res.K, false).eigenvalues();
  for (int i = 0; i < ev.size(); ++i) {
    const double d = std::min({std::abs(ev(i) - p1), std::abs(ev(i) - p2),
                               std::abs(ev(i) - std::conj(p2))});
    EXPECT_LT(d, 1e-6) << "polo " << ev(i);
  }
}

TEST(Lqi, PolynomialSeedIsComputedTorqueWithIntegral)
{
  // K = [a0 M, a1 M, a2 M]: la semilla es un CT con integral, escalado por M.
  // Lo que distingue al LQI son los pesos FISICOS (B5), no esta semilla.
  const double wn = 10.0, zeta = 1.0;
  const Matrix6d M = spdMass(2);
  Eigen::MatrixXd A, B;
  buildSdc(M, Matrix6d::Zero(), true, A, B);
  const CareResult res = solveCare(A, B, polynomialQ(M, wn, zeta, 1.0, true),
                                   Eigen::MatrixXd::Identity(6, 6));
  ASSERT_TRUE(res.ok) << res.reason;
  const double a2 = wn * (1 + 2 * zeta), a1 = wn * wn * (1 + 2 * zeta), a0 = wn * wn * wn;
  EXPECT_LT((gainBlock(res.K, true, 0) - a0 * M).norm() / (a0 * M.norm()), 1e-8);
  EXPECT_LT((gainBlock(res.K, true, 1) - a1 * M).norm() / (a1 * M.norm()), 1e-8);
  EXPECT_LT((gainBlock(res.K, true, 2) - a2 * M).norm() / (a2 * M.norm()), 1e-8);
}

TEST(Lqi, PhysicalWeightsGiveAStableLoopNotTiedToInertia)
{
  const Matrix6d M = spdMass(3);
  Matrix6d C = Matrix6d::Zero();
  C(1, 2) = 0.05;
  C(2, 1) = -0.03;
  Vector6d qs, qp, qv, rj;
  qs << 1e4, 1e4, 1e4, 1e4, 1e4, 1e4;          // 1/(10 mrad s)^2
  qp << 1e6, 1e6, 1e6, 1e6, 1e6, 1e6;          // 1/(1 mrad)^2
  qv << 1e3, 1e3, 1e3, 1e3, 1e3, 1e3;          // 1/(30 mrad/s)^2
  rj << 1e-2, 1e-2, 1e-2, 1.0, 1.0, 1.0;
  Eigen::MatrixXd A, B;
  buildSdc(M, C, true, A, B);
  const CareResult res = solveCare(A, B, physicalQ(qs, qp, qv, true),
                                   Eigen::MatrixXd(rj.asDiagonal()));
  ASSERT_TRUE(res.ok) << res.reason;
  EXPECT_LT(res.max_real_eig, 0.0);
  // Con Q y R constantes K_D NO es proporcional a M: el cociente K_D,ii/M_ii
  // cambia de una junta a otra (en la semilla seria a2 en las seis).
  const Matrix6d KD = gainBlock(res.K, true, 2);
  const Vector6d ratio = KD.diagonal().cwiseQuotient(M.diagonal());
  EXPECT_GT(ratio.maxCoeff() / ratio.minCoeff(), 2.0);
}

TEST(Lqi, IntegralFreezesSaturatedJointsAndIsClamped)
{
  IntegralState st;
  const Vector6d smax = Vector6d::Constant(0.01);
  st.frozen[1] = true;                         // la base marco la junta 1
  for (int k = 0; k < 100; ++k) {st.update(Vector6d::Constant(0.05), 0.002, smax);}
  EXPECT_DOUBLE_EQ(st.sigma[1], 0.0);          // congelada: no integra
  EXPECT_DOUBLE_EQ(st.sigma[0], 0.01);         // 0.05*0.2 = 0.01: llega a la cota
  for (int k = 0; k < 100; ++k) {st.update(Vector6d::Constant(0.05), 0.002, smax);}
  EXPECT_DOUBLE_EQ(st.sigma[0], 0.01);         // y no la pasa
  st.reset();
  EXPECT_EQ(st.sigma.norm(), 0.0);
}

TEST(Lqi, AtZeroErrorTheLawIsTheFeedforward)
{
  const Matrix6d M = spdMass(4);
  Matrix6d C = Matrix6d::Identity() * 0.1;
  Vector6d g;
  g << 0.0, -20.0, -7.0, -1.0, 0.0, 0.0;
  JointRef ref;
  ref.dq << 0.1, 0.0, -0.1, 0.0, 0.0, 0.0;
  ref.ddq << 0.0, 0.5, 0.0, 0.0, 0.0, 0.0;
  const Eigen::MatrixXd K = Eigen::MatrixXd::Random(6, 18);
  const Vector6d tau = lqrTorque(M, C, g, ref, K, Vector6d::Zero(), Vector6d::Zero(),
                                 Vector6d::Zero(), true);
  EXPECT_LT((tau - (M * ref.ddq + C * ref.dq + g)).norm(), 1e-12);
}

TEST(Lqi, Care18FitsTheControlPeriod)
{
  // Presupuesto: un ciclo a 500 Hz = 2000 us. En banco (sin Gazebo cargando la
  // CPU, que en la FASE 4 multiplico la media x6) tiene que sobrar mucho.
  const Matrix6d M = spdMass(5);
  Eigen::MatrixXd A, B;
  buildSdc(M, Matrix6d::Identity() * 0.01, true, A, B);
  const Eigen::MatrixXd Q = polynomialQ(M, 10.0, 1.0, 1.0, true);
  const Eigen::MatrixXd R = Eigen::MatrixXd::Identity(6, 6);
  const int n = 50;
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < n; ++i) {ASSERT_TRUE(solveCare(A, B, Q, R).ok);}
  const double us = std::chrono::duration<double, std::micro>(
    std::chrono::steady_clock::now() - t0).count() / n;
  RecordProperty("care18_us", static_cast<int>(us));
  std::cout << "CARE 18x18: " << us << " us por resolucion\n";
  EXPECT_LT(us, 2000.0);
}
