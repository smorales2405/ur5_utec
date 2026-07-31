// ============================================================================
//  FASE 4 — solver CARE por funcion signo del hamiltoniano, estabilizabilidad
//  del par SDC y escalado por inercia de la Q del LQR-SDRE.
//
//  El caso analitico de referencia es el doble integrador con ganancia de
//  entrada b = 1/m:
//
//      A = [0 1; 0 0]   B = [0; b]   Q = diag(qp, qv)   R = r
//
//      p12 = sqrt(qp r) / b
//      p22 = sqrt(r (2 p12 + qv)) / b
//      Kp  = b p12 / r = sqrt(qp / r)          <-- NO depende de m
//      Kv  = b p22 / r = sqrt((2 p12 + qv)/r)
//
//  De ahi sale el resultado de diseno que gobierna el YAML: en lazo cerrado
//  m q̈ + Kv q̇ + Kp q = 0, luego wn = sqrt(Kp/m) y para tener el MISMO wn en
//  las seis juntas hace falta Kp ∝ m, es decir qp ∝ m^2. Con una Q uniforme el
//  wn de la muneca sale 62 veces el del hombro.
// ============================================================================

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "ur5_dyn_control/care_solver.hpp"
#include "ur5_dyn_control/ur5_dynamics.hpp"

using ur5_dyn_control::CareOptions;
using ur5_dyn_control::CareResult;
using ur5_dyn_control::Matrix6d;
using ur5_dyn_control::Vector6d;
using ur5_dyn_control::controllabilityMargin;
using ur5_dyn_control::solveCare;
using Eigen::MatrixXd;

namespace
{

/// Doble integrador de masa m: A = [0 1; 0 0], B = [0; 1/m].
void doubleIntegrator(double m, MatrixXd & A, MatrixXd & B)
{
  A = MatrixXd::Zero(2, 2);
  A(0, 1) = 1.0;
  B = MatrixXd::Zero(2, 1);
  B(1, 0) = 1.0 / m;
}

/// Residuo relativo de la CARE, recalculado por fuera del solver.
double careResidual(const MatrixXd & A, const MatrixXd & B, const MatrixXd & Q,
                    const MatrixXd & R, const MatrixXd & P)
{
  const MatrixXd G = B * R.inverse() * B.transpose();
  const MatrixXd res = A.transpose() * P + P * A - P * G * P + Q;
  return res.norm() / std::max(1.0, Q.norm());
}

/// A(q,q̇) y B(q) de la parametrizacion SDC de la FASE 4.
void buildSdc(const Matrix6d & M, const Matrix6d & C, MatrixXd & A, MatrixXd & B)
{
  const Matrix6d Minv = M.llt().solve(Matrix6d::Identity());
  A = MatrixXd::Zero(12, 12);
  A.topRightCorner(6, 6) = Matrix6d::Identity();
  A.bottomRightCorner(6, 6) = -Minv * C;
  B = MatrixXd::Zero(12, 6);
  B.bottomRows(6) = Minv;
}

/**
 * Q de diseno del LQR-SDRE: Qp = r wn^4 M^2, Qv = r wn^2 (4 zeta^2 - 2) M^2,
 * con la M COMPLETA. Replica gz_lqr_sdre_control_node::buildQ().
 */
MatrixXd inertiaQ(const Matrix6d & M, double wn, double zeta, double r)
{
  const Matrix6d M2 = M * M;
  MatrixXd Q = MatrixXd::Zero(12, 12);
  Q.topLeftCorner(6, 6) = r * std::pow(wn, 4) * M2;
  Q.bottomRightCorner(6, 6) = r * wn * wn * (4.0 * zeta * zeta - 2.0) * M2;
  return Q;
}

/// Modulos de los 12 autovalores de A - B K, ordenados.
Eigen::VectorXd closedLoopMagnitudes(const MatrixXd & A, const MatrixXd & B,
                                     const MatrixXd & K)
{
  const Eigen::EigenSolver<MatrixXd> es(A - B * K, false);
  Eigen::VectorXd mag(es.eigenvalues().size());
  for (int i = 0; i < mag.size(); ++i) {mag(i) = std::abs(es.eigenvalues()(i));}
  std::sort(mag.data(), mag.data() + mag.size());
  return mag;
}

std::string urdfPath()
{
  return ament_index_cpp::get_package_share_directory("ur5_kinematics") + "/ur5e.urdf";
}

const Vector6d kQInit =
  (Vector6d() << 0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0).finished();

}  // namespace

// ── Caso analitico ──────────────────────────────────────────────────────────

TEST(CareSolver, DoubleIntegradorCoincideConLaSolucionCerrada)
{
  for (const double m : {0.00026, 0.02324, 1.05823, 2.59146}) {
    MatrixXd A, B;
    doubleIntegrator(m, A, B);
    const double qp = 7.0, qv = 3.0, r = 0.5;
    MatrixXd Q = MatrixXd::Zero(2, 2);
    Q(0, 0) = qp;
    Q(1, 1) = qv;
    const MatrixXd R = MatrixXd::Constant(1, 1, r);

    const CareResult res = solveCare(A, B, Q, R);
    ASSERT_TRUE(res.ok) << "m=" << m << ": " << res.reason;

    const double b = 1.0 / m;
    const double p12 = std::sqrt(qp * r) / b;
    const double p22 = std::sqrt(r * (2.0 * p12 + qv)) / b;
    const double p11 = (b * b / r) * p12 * p22;

    EXPECT_NEAR(res.P(0, 0), p11, 1e-9 * std::max(1.0, std::abs(p11))) << "m=" << m;
    EXPECT_NEAR(res.P(0, 1), p12, 1e-9 * std::max(1.0, std::abs(p12))) << "m=" << m;
    EXPECT_NEAR(res.P(1, 1), p22, 1e-9 * std::max(1.0, std::abs(p22))) << "m=" << m;

    // Kp = sqrt(qp/r) es INDEPENDIENTE de la masa: es el nucleo del problema de
    // escalado por inercia que ataca lqr_sdre_params.yaml.
    EXPECT_NEAR(res.K(0, 0), std::sqrt(qp / r), 1e-9);
    EXPECT_NEAR(res.K(0, 1), std::sqrt((2.0 * p12 + qv) / r), 1e-9);

    EXPECT_LT(res.max_real_eig, 0.0) << "m=" << m;
    EXPECT_LT(res.residual, 1e-10) << "m=" << m;
    EXPECT_LT(res.asymmetry, 1e-10) << "m=" << m;
  }
}

// ── El caso que rompe el metodo de autovectores ─────────────────────────────

TEST(CareSolver, PoloDobleDefectivoSeResuelveIgual)
{
  // zeta = 1 exacto => el lazo cerrado tiene un POLO DOBLE en -wn y A - B K, que
  // esta en forma companera, es DEFECTIVA (un solo autovector). El metodo de
  // autovectores del hamiltoniano falla justo aqui; la funcion signo no.
  //
  // Diseno inverso: Kp = m wn^2, Kv = 2 zeta m wn  =>
  //   qp = r m^2 wn^4,  qv = r m^2 wn^2 (4 zeta^2 - 2)
  const double m = 2.59146, wn = 20.0, zeta = 1.0, r = 1.0;
  MatrixXd A, B;
  doubleIntegrator(m, A, B);
  MatrixXd Q = MatrixXd::Zero(2, 2);
  Q(0, 0) = r * m * m * std::pow(wn, 4);
  Q(1, 1) = r * m * m * wn * wn * (4.0 * zeta * zeta - 2.0);
  const MatrixXd R = MatrixXd::Constant(1, 1, r);

  const CareResult res = solveCare(A, B, Q, R);
  ASSERT_TRUE(res.ok) << res.reason;
  EXPECT_NEAR(res.K(0, 0), m * wn * wn, 1e-7 * m * wn * wn);
  EXPECT_NEAR(res.K(0, 1), 2.0 * zeta * m * wn, 1e-7 * 2.0 * m * wn);

  // Los DOS polos del lazo cerrado caen en -wn (dentro de la precision con que
  // se resuelve un autovalor doble, que es ~sqrt(eps) por su propia naturaleza).
  const Eigen::EigenSolver<MatrixXd> es(A - B * res.K, false);
  for (int i = 0; i < 2; ++i) {
    EXPECT_NEAR(es.eigenvalues()(i).real(), -wn, 1e-4);
  }
  EXPECT_LT(res.max_real_eig, 0.0);
}

// ── MIMO aleatorio ──────────────────────────────────────────────────────────

TEST(CareSolver, MimoAleatorioDaResiduoPequenoYLazoEstable)
{
  std::srand(42);
  for (int trial = 0; trial < 20; ++trial) {
    const int n = 6, m = 3;
    const MatrixXd A = MatrixXd::Random(n, n);
    const MatrixXd B = MatrixXd::Random(n, m);
    const MatrixXd Qh = MatrixXd::Random(n, n);
    // Q semidefinida positiva + un poco de identidad => detectable seguro.
    const MatrixXd Q = Qh.transpose() * Qh + 0.1 * MatrixXd::Identity(n, n);
    const MatrixXd Rh = MatrixXd::Random(m, m);
    const MatrixXd R = Rh.transpose() * Rh + MatrixXd::Identity(m, m);

    const CareResult res = solveCare(A, B, Q, R);
    ASSERT_TRUE(res.ok) << "trial " << trial << ": " << res.reason;
    EXPECT_LT(careResidual(A, B, Q, R, res.P), 1e-8) << "trial " << trial;
    EXPECT_LT(res.max_real_eig, 0.0) << "trial " << trial;
    // P debe ser definida positiva (Q ≻ 0 y (A,B) estabilizable).
    const Eigen::SelfAdjointEigenSolver<MatrixXd> ep(res.P, Eigen::EigenvaluesOnly);
    EXPECT_GT(ep.eigenvalues().minCoeff(), 0.0) << "trial " << trial;
  }
}

TEST(CareSolver, RechazaEntradasInvalidas)
{
  MatrixXd A, B;
  doubleIntegrator(1.0, A, B);
  const MatrixXd Q = MatrixXd::Identity(2, 2);

  // R no definida positiva.
  EXPECT_FALSE(solveCare(A, B, Q, MatrixXd::Zero(1, 1)).ok);
  EXPECT_FALSE(solveCare(A, B, Q, MatrixXd::Constant(1, 1, -1.0)).ok);
  // Dimensiones incoherentes.
  EXPECT_FALSE(solveCare(A, B, MatrixXd::Identity(3, 3), MatrixXd::Identity(1, 1)).ok);

  // Modo inestable NO estabilizable: B no lo alcanza y Q no lo ve.
  MatrixXd A2 = MatrixXd::Zero(2, 2);
  A2(0, 0) = 1.0;              // polo en +1, desacoplado
  A2(1, 1) = -1.0;
  MatrixXd B2 = MatrixXd::Zero(2, 1);
  B2(1, 0) = 1.0;
  MatrixXd Q2 = MatrixXd::Zero(2, 2);
  Q2(1, 1) = 1.0;
  EXPECT_FALSE(solveCare(A2, B2, Q2, MatrixXd::Identity(1, 1)).ok);
}

// ── Estabilizabilidad del par SDC ───────────────────────────────────────────

TEST(CareSolver, MargenDeControlabilidad)
{
  // Par controlable: doble integrador.
  MatrixXd A, B;
  doubleIntegrator(1.0, A, B);
  EXPECT_GT(controllabilityMargin(A, B), 1e-6);

  // Par NO controlable: el modo 0 no se alcanza desde la entrada.
  MatrixXd A2 = MatrixXd::Identity(2, 2);
  MatrixXd B2 = MatrixXd::Zero(2, 1);
  B2(1, 0) = 1.0;
  EXPECT_LT(controllabilityMargin(A2, B2), 1e-12);
}

TEST(CareSolver, ParSdcDelUr5eEsControlableYEstabilizante)
{
  ur5_dyn_control::Ur5Dynamics dyn(urdfPath(), 9.8, 0.141);

  // Q escalada por inercia (misma receta que gz_lqr_sdre_control_node::buildQ).
  const MatrixXd Q = inertiaQ(dyn.M(kQInit), 20.0, 1.0, 1.0);
  const MatrixXd R = MatrixXd::Identity(6, 6);

  // Varias configuraciones y velocidades a lo largo del espacio de trabajo.
  std::srand(7);
  for (int trial = 0; trial < 12; ++trial) {
    const Vector6d q = kQInit + 0.4 * Vector6d::Random();
    const Vector6d dq = 0.5 * Vector6d::Random();
    MatrixXd A, B;
    buildSdc(dyn.M(q), dyn.coriolis(q, dq), A, B);

    EXPECT_GT(controllabilityMargin(A, B), 1e-9) << "trial " << trial;

    const CareResult res = solveCare(A, B, Q, R);
    ASSERT_TRUE(res.ok) << "trial " << trial << ": " << res.reason;
    EXPECT_LT(res.max_real_eig, 0.0) << "trial " << trial;
    EXPECT_LT(res.residual, 1e-7) << "trial " << trial;
  }
}

// ── Escalado por inercia: la regresion que este paquete ya pago tres veces ──

TEST(CareSolver, QProporcionalAMCuadradoPoneLos12PolosEnMenosWn)
{
  ur5_dyn_control::Ur5Dynamics dyn(urdfPath(), 9.8, 0.141);
  const Matrix6d M0 = dyn.M(kQInit);
  const double dt = 1.0 / 500.0;

  MatrixXd A, B;
  buildSdc(M0, Matrix6d::Zero(), A, B);   // q̇ = 0 => C = 0

  for (const double zeta : {1.0, 1.0 / std::sqrt(2.0), 1.5}) {
    for (const double wn : {10.0, 20.0, 50.0}) {
      const CareResult res =
        solveCare(A, B, inertiaQ(M0, wn, zeta, 1.0), MatrixXd::Identity(6, 6));
      ASSERT_TRUE(res.ok) << "wn=" << wn << " zeta=" << zeta << ": " << res.reason;
      EXPECT_LT(res.residual, 1e-10) << "wn=" << wn << " zeta=" << zeta;

      // Los 12 modos normalizados por masa obedecen todos el MISMO polinomio
      // s^2 + 2 zeta wn s + wn^2 = 0. Sus raices son:
      //   zeta <= 1 : par complejo, |lambda| = wn,  Re = -zeta wn
      //   zeta >  1 : dos reales,   wn(-zeta +- sqrt(zeta^2 - 1))
      const double disc = zeta * zeta - 1.0;
      const double lam_slow = (disc <= 0.0) ? wn : wn * (zeta - std::sqrt(disc));
      const double lam_fast = (disc <= 0.0) ? wn : wn * (zeta + std::sqrt(disc));
      const double re_max = -wn * (disc <= 0.0 ? zeta : zeta - std::sqrt(disc));

      const Eigen::VectorXd mag = closedLoopMagnitudes(A, B, res.K);
      EXPECT_NEAR(mag.minCoeff(), lam_slow, 1e-6 * lam_fast)
        << "wn=" << wn << " zeta=" << zeta;
      EXPECT_NEAR(mag.maxCoeff(), lam_fast, 1e-6 * lam_fast)
        << "wn=" << wn << " zeta=" << zeta;
      EXPECT_NEAR(res.max_real_eig, re_max, 1e-5 * lam_fast)
        << "wn=" << wn << " zeta=" << zeta;
      EXPECT_LT(res.max_real_eig, 0.0);
    }
  }

  // Punto de diseno del YAML (wn = 20, zeta = 1): margen amplio frente al
  // limite de estabilidad discreta a 500 Hz. Se comprueba aparte porque con
  // zeta > 1 el polo RAPIDO se aleja y wn deja de ser la escala relevante:
  // wn = 50 con zeta = 1.5 ya da max|lambda|*dt = 0.26.
  const CareResult ref =
    solveCare(A, B, inertiaQ(M0, 20.0, 1.0, 1.0), MatrixXd::Identity(6, 6));
  ASSERT_TRUE(ref.ok) << ref.reason;
  EXPECT_NEAR(closedLoopMagnitudes(A, B, ref.K).maxCoeff() * dt, 0.04, 1e-6);
}

TEST(CareSolver, QConLaDiagonalDeMDispersaLosPolos)
{
  // REGRESION del escalado por inercia (cuarta vez que muerde en este paquete).
  //
  // La receta tentadora — Qp_ii = I_ii^2 wn^4 con I = diag(M) — parece el mismo
  // escalado, pero M(q_init) NO es diagonal: M(1,5) = 2.8e-2 frente a
  // M(5,5) = 5.4e-3, un acoplo CINCO veces la propia diagonal. El resultado
  // MEDIDO es una dispersion de polos de 34x y max|lambda|*dt = 0.58, muy por
  // encima del limite discreto de 0.2. Este test fija ese hallazgo para que
  // nadie "simplifique" buildQ() a la diagonal.
  ur5_dyn_control::Ur5Dynamics dyn(urdfPath(), 9.8, 0.141);
  const Matrix6d M0 = dyn.M(kQInit);
  const double wn = 20.0, zeta = 1.0, dt = 1.0 / 500.0;

  MatrixXd A, B;
  buildSdc(M0, Matrix6d::Zero(), A, B);

  MatrixXd Qd = MatrixXd::Zero(12, 12);
  for (int i = 0; i < 6; ++i) {
    const double I = M0(i, i);
    Qd(i, i) = I * I * std::pow(wn, 4);
    Qd(6 + i, 6 + i) = I * I * wn * wn * (4.0 * zeta * zeta - 2.0);
  }
  const CareResult rd = solveCare(A, B, Qd, MatrixXd::Identity(6, 6));
  ASSERT_TRUE(rd.ok) << rd.reason;   // resuelve bien: el problema es el DISENO

  const Eigen::VectorXd mag = closedLoopMagnitudes(A, B, rd.K);
  EXPECT_GT(mag.maxCoeff() / mag.minCoeff(), 10.0)
    << "|lambda| en [" << mag.minCoeff() << ", " << mag.maxCoeff() << "]";
  EXPECT_GT(mag.maxCoeff() * dt, 0.2)
    << "la Q diagonal deberia violar el limite de estabilidad discreta";

  // Y el contraste directo: la M completa lo arregla en el mismo punto.
  const CareResult rf =
    solveCare(A, B, inertiaQ(M0, wn, zeta, 1.0), MatrixXd::Identity(6, 6));
  ASSERT_TRUE(rf.ok) << rf.reason;
  const Eigen::VectorXd magf = closedLoopMagnitudes(A, B, rf.K);
  EXPECT_NEAR(magf.maxCoeff() / magf.minCoeff(), 1.0, 1e-5);
}

// ── Presupuesto de computo ──────────────────────────────────────────────────

TEST(CareSolver, CosteDeUnaResolucionCabeEnUnCicloDe500Hz)
{
  ur5_dyn_control::Ur5Dynamics dyn(urdfPath(), 9.8, 0.141);
  const Matrix6d M0 = dyn.M(kQInit);
  const MatrixXd Q = inertiaQ(M0, 20.0, 1.0, 1.0);
  MatrixXd A, B;
  buildSdc(M0, dyn.coriolis(kQInit, Vector6d::Constant(0.2)), A, B);

  const int n = 200;
  const auto t0 = std::chrono::steady_clock::now();
  for (int k = 0; k < n; ++k) {
    const CareResult r = solveCare(A, B, Q, MatrixXd::Identity(6, 6));
    ASSERT_TRUE(r.ok) << r.reason;
  }
  const double us =
    std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0)
    .count() / n;

  // Presupuesto del plan: 2 ms (un ciclo a 500 Hz). Se deja un margen holgado
  // porque este test tambien corre en maquinas de CI cargadas; la cifra que se
  // reporta en el paper sale del CSV de diagnostico de una corrida real, no de
  // aqui. Lo que este test protege es que nadie meta un solver 10 veces mas
  // lento sin enterarse.
  EXPECT_LT(us, 2000.0) << "solveCare tardo " << us << " us de media";
  std::cout << "[          ] solveCare 12x6: " << us << " us por resolucion\n";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
