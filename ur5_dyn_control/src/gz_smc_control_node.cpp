// ============================================================================
//  gz_smc_control_node — Sliding Mode Control clásico del UR5e (FASE 5)
//
//  Formulación (va tal cual al paper; estructura de Slotine-Li):
//
//    q_e   = q − q_d              dq_e  = q̇ − q̇_d
//    s     = dq_e + Λ q_e                              (superficie deslizante)
//    q̇_r   = q̇_d − Λ q_e          q̈_r   = q̈_d − Λ dq_e  (referencia deslizante)
//                                  ⟹  s = q̇ − q̇_r
//
//    tau   = b̂(q,q̇) + M̂(q) q̈_r − K ⊙ ρ(s)
//
//    ρ(s)  = sgn(s)          (switching_function: "sign")
//          = sat(s/φ)        (switching_function: "sat")
//
//  Condición de alcance, con K calculada EN CADA CICLO a partir de la cota:
//
//    K_i ≥ η_i + | α M̂ q̈_r + α b̂ + (1−α) Ṁ̂ q̇_r |_i
//
//  OJO: es "≥", no "≤" (la tesis de partida lo tenía invertido). Con "≤" la
//  ganancia podría quedar POR DEBAJO de la incertidumbre y la condición de
//  alcance no se cumpliría: el modo deslizante no se establecería.
//
//  Se calcula K en vez de dejarla como parámetro libre para que la FASE 7
//  optimice sobre [Λ, η, φ] con la condición de alcance satisfecha POR
//  CONSTRUCCIÓN (restricción g3 del plan), en vez de tener que comprobarla como
//  una restricción más que el optimizador puede violar.
//
//  Por qué el lazo cerrado desliza (con modelo perfecto, M̂ = M y b̂ = b):
//      M q̈ + b = b + M q̈_r − K ρ(s)
//      M (q̈ − q̈_r) = −K ρ(s)      y      ṡ = q̈ − q̈_r
//   ⟹ ṡ = −M⁻¹ K ρ(s),   con M ≻ 0  ⟹  sᵀ ṡ < 0 fuera de s = 0.
// ============================================================================

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>

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
    // Region de G probada en el UR5e real. Solo AVISA; no cambia la ley. Es la
    // misma cifra que G_LOOP_MAX del optimizador (problem.py): un test del
    // optimizador comprueba que coinciden con el YAML.
    g_loop_max_ = declare_parameter<double>("g_loop_max", 120.0);
    if (!(alpha_ > 0.0) || alpha_ > 1.0) {
      throw std::runtime_error("alpha debe estar en (0, 1]");
    }

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
    RCLCPP_INFO(get_logger(),
                "SMC rho=%s%s | lambda=[%s] eta=[%s] alpha=%.2f",
                sw.c_str(), phi_txt.c_str(),
                lista(lambda_).c_str(), lista(eta_).c_str(), alpha_);
    // Ganancia derivativa que esto crea sobre la velocidad MEDIDA,
    // G_i = M_ii*lambda_i + K_i/phi_i (docs/09_real_bringup.md §2). Es el
    // numero que entro en ciclo limite en smc_712.
    //
    // Se evalua a lo largo de TODA la referencia que se va a ejecutar, con la
    // ley en seguimiento perfecto (e = 0), y se informa del maximo. Antes se
    // evaluaba solo en q_init, y para la incision eso ENGANA: con el brazo
    // extendido M_11 pasa de 1.06 a 3.2 kg m^2, y las ganancias de smc_v4_g5
    // habrian dado G = 528 en la base mientras el banner decia 186.
    {
      const JointReferenceTable * tab = referenceTable();
      const std::size_t n = tab ? tab->size() : 0;
      const std::size_t paso = std::max<std::size_t>(1, n / 2000);
      Vector6d Gmax = Vector6d::Zero();
      std::array<std::size_t, 6> kmax{};
      for (std::size_t k = 0; k < std::max<std::size_t>(n, 1); k += paso) {
        const Vector6d q = tab ? tab->at(k).q : qInit();
        const Vector6d dq = tab ? tab->at(k).dq : Vector6d::Zero();
        const Vector6d ddq = tab ? tab->at(k).ddq : Vector6d::Zero();
        const Matrix6d M = dyn().M(q);
        const Vector6d K = eta_ + (alpha_ * (M * ddq) + alpha_ * dyn().nle(q, dq) +
          (1.0 - alpha_) * (dyn().dM(q, dq) * dq)).cwiseAbs();
        for (int i = 0; i < 6; ++i) {
          const double g = M(i, i) * lambda_[i] + K[i] / phi_[i];
          if (g > Gmax[i]) {Gmax[i] = g; kmax[i] = k;}
        }
      }
      RCLCPP_INFO(get_logger(),
                  "  G = M_ii*lambda_i + K_i/phi_i, MAXIMO sobre la referencia "
                  "(%zu muestras) = [%s] N.m por rad/s",
                  n, lista(Gmax).c_str());
      for (int i = 0; i < 6; ++i) {
        if (Gmax[i] > g_loop_max_) {
          RCLCPP_WARN(get_logger(),
                      "  G de %s = %.1f en t = %.2f s: por encima de g_loop_max = "
                      "%.0f, la cota de DISENO del optimizador. Lo probado limpio y "
                      "el fallo, junta a junta, en docs/09_real_bringup.md §6.9 "
                      "(smc_712 entro en ciclo limite con [179 301 128]). Con el "
                      "bisturi montado G sube un 3-5 %%.",
                      kJointNames[static_cast<std::size_t>(i)].c_str(), Gmax[i],
                      tab ? kmax[i] * tab->dt() : 0.0, g_loop_max_);
        }
      }
    }
    RCLCPP_INFO(get_logger(),
                "  K se calcula por ciclo: K_i = eta_i + |alpha*M*ddq_r + "
                "alpha*b + (1-alpha)*dM*dq_r|_i  (condicion de alcance por "
                "construccion)");
    start();
  }

protected:
  Vector6d computeTau(const Vector6d & q, const Vector6d & dq,
                      const JointRef & ref, double /*dt*/) override
  {
    // Errores con el convenio del plan: e = actual − deseado.
    const Vector6d q_e = q - ref.q;
    const Vector6d dq_e = dq - ref.dq;

    s_ = dq_e + lambda_.asDiagonal() * q_e;

    const Vector6d dq_r = ref.dq - lambda_.asDiagonal() * q_e;
    const Vector6d ddq_r = ref.ddq - lambda_.asDiagonal() * dq_e;

    const Matrix6d M = dyn().M(q);
    const Vector6d b = dyn().nle(q, dq);
    const Matrix6d dM = dyn().dM(q, dq);

    // Cota de incertidumbre y ganancia de conmutacion (por junta).
    const Vector6d bound =
      (alpha_ * (M * ddq_r) + alpha_ * b + (1.0 - alpha_) * (dM * dq_r)).cwiseAbs();
    k_ = eta_ + bound;

    // ρ(s). sat(s/φ) es CONTINUA: dentro de |s| < φ se comporta como una
    // ganancia proporcional alta (K/φ) en vez de conmutar, que es lo que
    // elimina el chattering a costa de dejar |s| acotada por O(φ) en vez de
    // llevarla a cero exactamente.
    Vector6d rho;
    for (int i = 0; i < 6; ++i) {
      rho[i] = use_sat_
        ? std::clamp(s_[i] / phi_[i], -1.0, 1.0)
        : ((s_[i] > 0.0) ? 1.0 : ((s_[i] < 0.0) ? -1.0 : 0.0));
    }

    return b + M * ddq_r - k_.cwiseProduct(rho);
  }

  /// Va a la columna `s[6]` del CSV unificado (FASE 3).
  Vector6d slidingVariable() const override {return s_;}

  std::string csvPrefix() const override {return "smc";}

private:
  Vector6d lambda_, eta_;
  Vector6d phi_ = Vector6d::Constant(0.05);
  double alpha_ = 0.3;
  double g_loop_max_ = 120.0;
  bool use_sat_ = true;
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
