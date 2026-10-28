#ifndef UR5_DYN_CONTROL_SMC_LAW_HPP
#define UR5_DYN_CONTROL_SMC_LAW_HPP

#include <algorithm>

#include "ur5_dyn_control/common.hpp"
#include "ur5_dyn_control/joint_reference_table.hpp"
#include "ur5_dyn_control/ur5_dynamics.hpp"

namespace ur5_dyn_control
{

/**
 * Ley SMC (Slotine-Li) con las correcciones R1 y R2 del plan (B4.3, 4.4 de
 * docs/Journal_UR5eControl_plan.md). Funcion pura: la usan el nodo
 * (gz_smc_control_node) y el test dorado contra la `SmcLaw` del evaluador
 * offline (test_smc_law_cross.py), que tiene que dar el mismo par.
 *
 *   s    = q̇_e + Λ q_e,     q̇_r = q̇_d − Λ q_e,     q̈_r = q̈_d − Λ q̇_e
 *   τ    = M̂ q̈_r + Ĉ(q,q̇) q̇_r + ĝ − K ⊙ ρ(s)                          (R1)
 *   K_i  = η_i + α_M |(M̂ q̈_r)_i| + α_C |(Ĉ q̇_r)_i| + α_g |ĝ_i|        (R2)
 *
 * R1 — Slotine-Li EXACTO: `Ĉ q̇_r` y no `b̂ = Ĉ q̇ + ĝ`. Con Ṁ − 2C
 * antisimetrica, V = ½ sᵀ M s da V̇ = −sᵀ K ρ − sᵀ Δ, con Δ solo el error de
 * modelo: la condicion de alcance K_i ≥ |Δ_i| + η_i es GLOBAL. Con `b̂` quedaba
 * un ½ sᵀ Ṁ s que solo se acota localmente; de ahi venia el termino
 * (1−α) Ṁ q̇_r de la cota vieja, que con R1 sobra.
 *
 * R2 — cota ESTRUCTURADA: un α por termino (inercia, Coriolis, gravedad) en vez
 * de un unico α sobre todo `b̂`. La gravedad pone el 56 % de la G del hombro
 * (V2): con α_g medido (B4.4) en vez del 0.3 comun se libera ganancia que el
 * optimizador puede gastar en Λ dentro de G ≤ 120. Hasta que se mida, los tres
 * valen `alpha`.
 *
 * El feedforward de friccion lo suma la base despues (publishTau), igual para
 * las tres leyes.
 */
struct SmcGains
{
  Vector6d lambda = Vector6d::Constant(20.0);
  Vector6d eta = Vector6d::Zero();
  Vector6d phi = Vector6d::Constant(0.05);
  double alpha_M = 0.3;
  double alpha_C = 0.3;
  double alpha_g = 0.3;
  bool use_sat = true;
};

struct SmcOutput
{
  Vector6d tau = Vector6d::Zero();
  Vector6d s = Vector6d::Zero();
  Vector6d K = Vector6d::Zero();
};

/// K de la cota estructurada (R2) a partir de los tres terminos nominales.
inline Vector6d smcGainK(const SmcGains & g, const Vector6d & M_ddq_r,
                         const Vector6d & C_dq_r, const Vector6d & grav)
{
  return g.eta + g.alpha_M * M_ddq_r.cwiseAbs() + g.alpha_C * C_dq_r.cwiseAbs() +
         g.alpha_g * grav.cwiseAbs();
}

inline SmcOutput smcLaw(Ur5Dynamics & dyn, const Vector6d & q, const Vector6d & dq,
                        const JointRef & ref, const SmcGains & g)
{
  SmcOutput out;
  const Vector6d q_e = q - ref.q;              // e = actual − deseado
  const Vector6d dq_e = dq - ref.dq;
  out.s = dq_e + g.lambda.cwiseProduct(q_e);
  const Vector6d dq_r = ref.dq - g.lambda.cwiseProduct(q_e);
  const Vector6d ddq_r = ref.ddq - g.lambda.cwiseProduct(dq_e);

  const Vector6d M_ddq_r = dyn.M(q) * ddq_r;
  const Vector6d C_dq_r = dyn.coriolis(q, dq) * dq_r;
  const Vector6d grav = dyn.gravity(q);
  out.K = smcGainK(g, M_ddq_r, C_dq_r, grav);

  // ρ(s): sat(s/φ) es continua (ganancia K/φ dentro de la capa, sin
  // conmutar); sign conmuta y lleva s a 0 exacto, con chattering.
  Vector6d rho;
  for (int i = 0; i < 6; ++i) {
    rho[i] = g.use_sat
      ? std::clamp(out.s[i] / g.phi[i], -1.0, 1.0)
      : ((out.s[i] > 0.0) ? 1.0 : ((out.s[i] < 0.0) ? -1.0 : 0.0));
  }
  out.tau = M_ddq_r + C_dq_r + grav - out.K.cwiseProduct(rho);
  return out;
}

/// d(tau)/d(dq) dentro de la capa limite, sin la derivada de Coriolis
/// (despreciable a las velocidades de la incision): M Λ + diag(K/φ). Su
/// diagonal es la G = M_ii λ_i + K_i/φ_i del banner (plan 4.5).
inline Matrix6d smcDtauDdq(Ur5Dynamics & dyn, const Vector6d & q, const Vector6d & dq,
                           const Vector6d & ddq, const SmcGains & g)
{
  JointRef ref;
  ref.q = q;
  ref.dq = dq;
  ref.ddq = ddq;
  const Vector6d K = smcLaw(dyn, q, dq, ref, g).K;   // error nulo: q_r = q_d
  Matrix6d D = dyn.M(q) * g.lambda.asDiagonal();
  D.diagonal() += K.cwiseQuotient(g.phi);
  return D;
}

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_SMC_LAW_HPP
