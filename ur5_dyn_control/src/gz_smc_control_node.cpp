// ============================================================================
//  gz_smc_control_node — Sliding Mode Control clásico del UR5e (FASE 5)
//
//  Formulación (va tal cual al paper; Slotine-Li EXACTO, B4.3 del plan):
//
//    q_e   = q − q_d              dq_e  = q̇ − q̇_d
//    s     = dq_e + Λ q_e                              (superficie deslizante)
//    q̇_r   = q̇_d − Λ q_e          q̈_r   = q̈_d − Λ dq_e  (referencia deslizante)
//                                  ⟹  s = q̇ − q̇_r
//
//    tau   = M̂(q) q̈_r + Ĉ(q,q̇) q̇_r + ĝ(q) − K ⊙ ρ(s)              (R1)
//    K_i   = η_i + α_M |(M̂ q̈_r)_i| + α_C |(Ĉ q̇_r)_i| + α_g |ĝ_i|   (R2)
//
//    ρ(s)  = sgn(s)          (switching_function: "sign")
//          = sat(s/φ)        (switching_function: "sat")
//
//  La ley vive en smc_law.hpp (funcion pura): el test dorado la compara con la
//  SmcLaw del evaluador offline.
//
//  Por qué desliza. Con Ṁ − 2C antisimétrica y V = ½ sᵀ M s:
//      M ṡ = −C s − K ρ(s) − Δ      ⟹      V̇ = −sᵀ K ρ(s) − sᵀ Δ
//  (Δ: error de modelo). Basta K_i ≥ |Δ_i| + η_i para que V̇ < 0 fuera de la capa,
//  y la prueba es GLOBAL. Hasta B4.3 la ley usaba b̂ = Ĉ q̇ + ĝ en vez de
//  Ĉ q̇_r + ĝ: quedaba un ½ sᵀ Ṁ s que solo se acotaba localmente, de ahí el
//  término (1−α) Ṁ q̇_r de la cota vieja, y el contraejemplo de V11 del plan.
//
//  OJO: es "≥", no "≤" (la tesis de partida lo tenía invertido). Con "≤" la
//  ganancia podría quedar POR DEBAJO de la incertidumbre y la condición de
//  alcance no se cumpliría: el modo deslizante no se establecería.
//
//  Se calcula K en vez de dejarla como parámetro libre para que la FASE 7
//  optimice sobre [Λ, η, φ] con la condición de alcance satisfecha POR
//  CONSTRUCCIÓN, en vez de tener que comprobarla como una restricción más.
// ============================================================================

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>

#include "ur5_dyn_control/smc_law.hpp"
#include "ur5_dyn_control/torque_control_node_base.hpp"

namespace ur5_dyn_control
{

class SmcControlNode : public TorqueControlNodeBase
{
public:
  SmcControlNode()
  : TorqueControlNodeBase("gz_smc_control_node")
  {
    auto vec6 = [this](const char * name, std::vector<double> def) {
        const auto v = declare_parameter<std::vector<double>>(name, def);
        if (v.size() != 6) {
          throw std::runtime_error(std::string("'") + name + "' debe tener 6 elementos");
        }
        Vector6d out;
        for (int i = 0; i < 6; ++i) {out[i] = v[i];}
        return out;
      };

    // Λ fija el polo del modo deslizante: sobre s = 0 el error obedece
    // dq_e = −Λ q_e, es decir decae con constante de tiempo 1/λ_i.
    lambda_ = vec6("lambda", {20.0, 20.0, 20.0, 20.0, 20.0, 20.0});
    // η es el margen de alcance: cuanto MÁS de la cota de incertidumbre se
    // aplica. Manda el tiempo de alcance y también el chattering.
    eta_ = vec6("eta", {5.0, 5.0, 5.0, 1.0, 1.0, 1.0});

    const double phi_scalar = declare_parameter<double>("phi", 0.05);
    if (!(phi_scalar > 0.0)) {
      throw std::runtime_error("phi debe ser > 0 (es el ancho de la capa límite)");
    }
    // phi POR JUNTA. Vacio = usar el escalar `phi` en las seis, que es el
    // comportamiento historico y lo que escriben los ficheros de ganancias de
    // la FASE 7; por eso `phi` sigue siendo escalar y esto es un anadido
    // opcional, en vez de cambiarle el tipo y romper lo ya generado.
    //
    // Hace falta porque el umbral de estabilidad discreta va por junta:
    // chi_i = (K_i/phi)·dt/M_ii, y la inercia del UR5e abarca cuatro ordenes de
    // magnitud (2.59 kg m2 en shoulder_lift, 2.6e-4 en wrist_3). Un phi comun
    // que sea razonable en el hombro deja chi ~ 515 en wrist_3 (docs/05_smc.md
    // §7.2): la junta no puede recibir la autoridad que necesita para vencer su
    // friccion sin que el lazo discreto se vuelva inestable.
    phi_ = Vector6d::Constant(phi_scalar);
    const auto phi_joint = declare_parameter<std::vector<double>>(
      "phi_joint", std::vector<double>{});
    if (!phi_joint.empty()) {
      if (phi_joint.size() != 6) {
        throw std::runtime_error(
                "phi_joint debe tener 6 valores (o estar vacio para usar `phi`)");
      }
      for (int i = 0; i < 6; ++i) {
        if (!(phi_joint[static_cast<size_t>(i)] > 0.0)) {
          throw std::runtime_error("phi_joint: todos los valores deben ser > 0");
        }
        phi_[i] = phi_joint[static_cast<size_t>(i)];
      }
    }
    // α ∈ (0,1]: fracción de los términos nominales que se asume incierta.
    // El plan barre 0.1 / 0.3 / 0.5 / 1.0 como estudio de sensibilidad.
    alpha_ = declare_parameter<double>("alpha", 0.3);
    if (!(alpha_ > 0.0) || alpha_ > 1.0) {
      throw std::runtime_error("alpha debe estar en (0, 1]");
    }
    // R2 (B4.3): un alpha por termino de la cota (inercia, Coriolis, gravedad).
    // Negativo = el `alpha` de arriba, que es lo que llevan los ficheros de
    // ganancias de la FASE 7. alpha_g se medira con el ensayo de sosten (B4.4).
    auto alpha_term = [this](const char * name) {
        double a = declare_parameter<double>(name, -1.0);
        if (a < 0.0) {a = alpha_;}
        if (a > 1.0) {
          throw std::runtime_error(std::string(name) + " debe estar en [0, 1]");
        }
        return a;
      };
    gains_.alpha_M = alpha_term("alpha_M");
    gains_.alpha_C = alpha_term("alpha_C");
    gains_.alpha_g = alpha_term("alpha_g");

    const std::string sw = declare_parameter<std::string>("switching_function", "sat");
    if (sw == "sign") {
      use_sat_ = false;
    } else if (sw == "sat") {
      use_sat_ = true;
    } else {
      throw std::runtime_error(
        "switching_function desconocida: '" + sw + "' (validas: sign, sat)");
    }

    // El string se materializa en una variable: pasar .c_str() de un temporal
    // a un printf-like es un uso despues de destruir.
    const bool phi_uniforme =
      (phi_.maxCoeff() - phi_.minCoeff()) < 1e-12;
    // Se imprimen las SEIS. Con "[phi_0 ... phi_5]" el mensaje decia
    // "[0.05 ... 0.05]" mientras wrist_1 valia 0.10: la unica junta distinta era
    // justo una de las que no se veian.
    std::string phi_lista;
    for (int i = 0; i < 6; ++i) {
      char b[24];
      std::snprintf(b, sizeof(b), "%s%.4g", i ? " " : "", phi_[i]);
      phi_lista += b;
    }
    const std::string phi_txt =
      use_sat_
      ? (phi_uniforme
        ? (" (phi=" + phi_lista.substr(0, phi_lista.find(' ')) + ")")
        : (" (phi POR JUNTA=[" + phi_lista + "])"))
      : std::string();
    // Las SEIS lambdas y etas, no solo la primera. En una rampa de lambda sobre
    // una junta (docs/09_real_bringup.md §6) el resto no cambia, asi que con
    // "lambda=[20.0 ...]" los cinco escalones salen IDENTICOS en el log y no
    // hay forma de saber cual fue cada corrida.
    auto lista = [](const Vector6d & v) {
        std::string s;
        char b[32];
        for (int i = 0; i < 6; ++i) {
          std::snprintf(b, sizeof(b), "%s%.4g", i ? " " : "", v[i]);
          s += b;
        }
        return s;
      };
    gains_.lambda = lambda_;
    gains_.eta = eta_;
    gains_.phi = phi_;
    gains_.use_sat = use_sat_;
    RCLCPP_INFO(get_logger(),
                "SMC rho=%s%s | lambda=[%s] eta=[%s] alpha M/C/g=%.2f/%.2f/%.2f",
                sw.c_str(), phi_txt.c_str(),
                lista(lambda_).c_str(), lista(eta_).c_str(),
                gains_.alpha_M, gains_.alpha_C, gains_.alpha_g);
    // Ganancia derivativa que esto crea sobre la velocidad MEDIDA (docs/09
    // 2): dentro de la capa limite d(tau)/d(dq) = M*Lambda + diag(K/phi), con
    // la diagonal G_i = M_ii*lambda_i + K_i/phi_i. Es el numero que entro en
    // ciclo limite en smc_712. La evalua la base sobre toda la referencia
    // (reportLoopGain, comun a las tres leyes; g_loop_max vive en el YAML comun).
    reportLoopGain(
      "M_ii*lambda_i + K_i/phi_i",
      [this](const Vector6d & q, const Vector6d & dq, const Vector6d & ddq) {
        return smcDtauDdq(dyn(), q, dq, ddq, gains_);
      });
    RCLCPP_INFO(get_logger(),
                "  K se calcula por ciclo: K_i = eta_i + alpha_M|M*ddq_r|_i + "
                "alpha_C|C*dq_r|_i + alpha_g|g|_i  (R2; condicion de alcance por "
                "construccion)");
    start();
  }

protected:
  Vector6d computeTau(const Vector6d & q, const Vector6d & dq,
                      const JointRef & ref, double /*dt*/) override
  {
    const SmcOutput o = smcLaw(dyn(), q, dq, ref, gains_);
    s_ = o.s;
    k_ = o.K;
    return o.tau;
  }

  /// Va a la columna `s[6]` del CSV unificado (FASE 3).
  Vector6d slidingVariable() const override {return s_;}

  std::string csvPrefix() const override {return "smc";}

private:
  Vector6d lambda_, eta_;
  Vector6d phi_ = Vector6d::Constant(0.05);
  double alpha_ = 0.3;
  bool use_sat_ = true;
  SmcGains gains_;           // lo que usa la ley (smc_law.hpp)
  Vector6d s_ = Vector6d::Zero();
  Vector6d k_ = Vector6d::Zero();
};

}  // namespace ur5_dyn_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ur5_dyn_control::SmcControlNode>());
  rclcpp::shutdown();
  return 0;
}
