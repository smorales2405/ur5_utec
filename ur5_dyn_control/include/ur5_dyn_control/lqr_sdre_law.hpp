#ifndef UR5_DYN_CONTROL_LQR_SDRE_LAW_HPP
#define UR5_DYN_CONTROL_LQR_SDRE_LAW_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include "ur5_dyn_control/common.hpp"

namespace ur5_dyn_control
{

/**
 * LQR-SDRE y LQI-SDRE (B4.2, 4.3 de docs/Journal_UR5eControl_plan.md). Funciones
 * puras: las usa gz_lqr_sdre_control_node y las prueban test_care_solver y
 * test_lqi.
 *
 * Sin integral (12 estados, la formulacion de la FASE 4 = ablacion del paper):
 *   x = [q_e; q̇_e],         A = [0 I; 0 −M⁻¹C],          B = [0; M⁻¹]
 * Con integral (LQI, 18 estados):
 *   x = [σ; q_e; q̇_e],      A = [0 I 0; 0 0 I; 0 0 −M⁻¹C], B = [0; 0; M⁻¹]
 *   σ̇ = q_e
 * Ley (la misma para los dos, con K = R⁻¹BᵀP de la CARE):
 *   τ = M q̈_d + C q̇_d + g − K x
 */

inline int lqrStates(bool integral) {return integral ? 18 : 12;}

/// A(q,q̇) y B(q) de la parametrizacion SDC.
inline void buildSdc(const Matrix6d & M, const Matrix6d & C, bool integral,
                     Eigen::MatrixXd & A, Eigen::MatrixXd & B)
{
  const int n = lqrStates(integral);
  const int o = integral ? 6 : 0;          // desplazamiento por el bloque de σ
  // M es simetrica definida positiva: Cholesky en vez de inverse().
  const Matrix6d Minv = M.llt().solve(Matrix6d::Identity());
  A = Eigen::MatrixXd::Zero(n, n);
  B = Eigen::MatrixXd::Zero(n, 6);
  if (integral) {A.block(0, 6, 6, 6) = Matrix6d::Identity();}   // σ̇ = q_e
  A.block(o, o + 6, 6, 6) = Matrix6d::Identity();                // d(q_e) = q̇_e
  A.block(o + 6, o + 6, 6, 6) = -Minv * C;
  B.block(o + 6, 0, 6, 6) = Minv;
}

/**
 * Q por SINTESIS POLINOMICA (semilla y test de B4.2, no el diseno final).
 *
 * Con R = r I y Q ∝ M², cada modo normalizado por masa ve una cadena de
 * integradores, y en el limite desacoplado (C = 0) los polos del lazo cerrado
 * son las raices del polinomio deseado a(s):
 *   12 estados: a(s) = s² + 2ζωn s + ωn²
 *       Q_p = r ωn⁴ M²,   Q_v = r ωn² (4ζ² − 2) M²            (formula de la FASE 4)
 *   18 estados: a(s) = (s + ωn)(s² + 2ζωn s + ωn²) = s³ + a₂s² + a₁s + a₀
 *       Q_σ = r a₀² M²,   Q_p = r (a₁² − 2a₀a₂) M²,   Q_v = r (a₂² − 2a₁) M²
 * Q tiene que ser semidefinida: exige a₁² ≥ 2a₀a₂ y a₂² ≥ 2a₁ (ζ ≥ 1/2 en el
 * caso de 18 estados, ζ ≥ 1/√2 en el de 12).
 */
inline Eigen::MatrixXd polynomialQ(const Matrix6d & M, double wn, double zeta, double r,
                                   bool integral)
{
  const Matrix6d M2 = M * M;
  const int n = lqrStates(integral);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n, n);
  if (!integral) {
    const double qv = wn * wn * (4.0 * zeta * zeta - 2.0);
    if (qv < -1e-12) {
      throw std::runtime_error("zeta < 1/sqrt(2): Q_v definida negativa");
    }
    Q.topLeftCorner(6, 6) = r * std::pow(wn, 4) * M2;
    Q.bottomRightCorner(6, 6) = r * std::max(qv, 0.0) * M2;
    return Q;
  }
  const double a2 = wn * (1.0 + 2.0 * zeta);
  const double a1 = wn * wn * (1.0 + 2.0 * zeta);
  const double a0 = wn * wn * wn;
  const double qp = a1 * a1 - 2.0 * a0 * a2;
  const double qv = a2 * a2 - 2.0 * a1;
  if (qp < -1e-9 * a1 * a1 || qv < -1e-9 * a2 * a2) {
    throw std::runtime_error("zeta < 1/2: Q del LQI definida negativa");
  }
  Q.block(0, 0, 6, 6) = r * a0 * a0 * M2;
  Q.block(6, 6, 6, 6) = r * std::max(qp, 0.0) * M2;
  Q.block(12, 12, 6, 6) = r * std::max(qv, 0.0) * M2;
  return Q;
}

/**
 * Q con PESOS FISICOS, diagonales y constantes (el diseno de 4.3; los valores
 * los fija B5). Por junta: Q_σ,ii [1/(rad·s)²], Q_p,ii [1/rad²], Q_v,ii
 * [1/(rad/s)²]; regla de Bryson como punto de partida, Q_ii = 1/e_max,i². La
 * rigidez de cada junta sale del compromiso par/error y NO queda atada a M_ii;
 * K(q) se re-programa por B(q) = M(q)⁻¹, que es lo que el SDRE aporta aqui.
 */
inline Eigen::MatrixXd physicalQ(const Vector6d & q_sigma, const Vector6d & q_p,
                                 const Vector6d & q_v, bool integral)
{
  const int n = lqrStates(integral);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n, n);
  const int o = integral ? 6 : 0;
  if (integral) {Q.block(0, 0, 6, 6) = q_sigma.asDiagonal();}
  Q.block(o, o, 6, 6) = q_p.asDiagonal();
  Q.block(o + 6, o + 6, 6, 6) = q_v.asDiagonal();
  return Q;
}

/// Bloques de K = [K_I K_P K_D] (K_I vacia sin integral).
inline Matrix6d gainBlock(const Eigen::MatrixXd & K, bool integral, int which)
{
  // which: 0 = K_I, 1 = K_P, 2 = K_D
  const int o = integral ? 0 : -6;
  if (!integral && which == 0) {return Matrix6d::Zero();}
  return K.block(0, o + 6 * which, 6, 6);
}

/// Ley: τ = M q̈_d + C q̇_d + g − K x.
inline Vector6d lqrTorque(const Matrix6d & M, const Matrix6d & C, const Vector6d & g,
                          const JointRef & ref, const Eigen::MatrixXd & K,
                          const Vector6d & q_e, const Vector6d & dq_e,
                          const Vector6d & sigma, bool integral)
{
  Eigen::VectorXd x(lqrStates(integral));
  if (integral) {
    x << sigma, q_e, dq_e;
  } else {
    x << q_e, dq_e;
  }
  return M * ref.ddq + C * ref.dq + g - K * x;
}

/**
 * Estado integral σ con ANTI-WINDUP (B4.2).
 *
 * σ += q_e·dt solo en las juntas NO congeladas: la base marca las juntas cuyo
 * comando quedo recortado por saturacion o por limite de tasa (onSaturation), y
 * seguir integrando ahi solo acumula un windup que luego hay que descargar con
 * sobre-disparo. Ademas |σ_i| ≤ σ_max,i: el termino integral nunca puede pedir
 * mas que la fraccion del par que se le reservo.
 */
struct IntegralState
{
  Vector6d sigma = Vector6d::Zero();
  std::array<bool, 6> frozen{};       // ultimas marcas de saturacion

  void update(const Vector6d & q_e, double dt, const Vector6d & sigma_max)
  {
    for (int i = 0; i < 6; ++i) {
      if (frozen[static_cast<std::size_t>(i)]) {continue;}
      sigma[i] = std::clamp(sigma[i] + q_e[i] * dt, -sigma_max[i], sigma_max[i]);
    }
  }
  void reset()
  {
    sigma.setZero();
    frozen.fill(false);
  }
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_LQR_SDRE_LAW_HPP
