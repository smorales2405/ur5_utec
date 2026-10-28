// ============================================================================
//  gz_lqr_sdre_control_node — LQR-SDRE del UR5e (FASE 4)
//
//  State-Dependent Riccati Equation: la ley es un LQR resuelto EN CADA
//  ACTUALIZACION sobre la parametrizacion dependiente del estado (SDC) del
//  error de seguimiento, con la Riccati CONGELADA en el paso.
//
//  ── Parametrizacion SDC ───────────────────────────────────────────────────
//
//    x_e  = [q_e ; q̇_e] ∈ R^12,     q_e = q − q_d,   q̇_e = q̇ − q̇_d
//
//    A(q,q̇) = [ 0   I ; 0  −M(q)^-1 C(q,q̇) ]        (12x12)
//    B(q)    = [ 0 ; M(q)^-1 ]                       (12x6)
//
//    J = ∫ (x_eᵀ Q x_e + uᵀ R u) dt,   Q = blkdiag(Qp, Qv),   R ≻ 0
//    Aᵀ P + P A − P B R^-1 Bᵀ P + Q = 0,     K = R^-1 Bᵀ P
//
//    tau = M̂(q) q̈_d + Ĉ(q,q̇) q̇_d + g(q) − K x_e
//                                    ^^^^
//  ── B4.2: LQI-SDRE (con accion integral, default) ─────────────────────────
//
//    x = [σ ; q_e ; q̇_e] ∈ R^18,  σ̇ = q_e
//    A = [0 I 0; 0 0 I; 0 0 −M^-1 C],   B = [0; 0; M^-1]
//    tau = M̂ q̈_d + Ĉ q̇_d + g − K_I σ − K_P q_e − K_D q̇_e
//
//  La integral rechaza la parte constante de la fuerza de corte, el residuo de
//  friccion y el error de gravedad. Se integra en TRACK y HOLD_END, se congela
//  por junta cuando la base recorta el par (onSaturation: anti-windup) y se
//  acota |σ_i| ≤ σ_max,i, con σ_max tal que |K_I σ| no pase de
//  `lqr.integral_tau_frac` de tau_max. Pesos (lqr_sdre_law.hpp):
//    polynomial : Q ∝ M² por sintesis polinomica (semilla y test; con integral
//                 da K = [a0 M, a1 M, a2 M], un CT con integral)
//    physical   : Q, R diagonales constantes en unidades fisicas (diseno de
//                 4.3; los valores los fija B5)
//  `lqr.integral: false` es la FASE 4 tal cual (12 estados): la ablacion.
//
//  ── OJO: q̇_d, NO q̇ (discrepancia con el plan, resuelta a favor de A) ─────
//
//  El plan escribe el prealimentado como `Ĉ(q,dq) dq` y a la vez da
//  A = [0 I; 0 −M^-1 C]. Las dos cosas no pueden ser ciertas a la vez:
//
//    · con  tau_ff = M q̈_d + C q̇ + g   (par calculado clasico) el termino de
//      Coriolis se cancela ENTERO contra el de la planta y queda M q̈_e = u,
//      es decir A = [0 I; 0 0]: la dependencia del estado desaparece de A y el
//      esquema deja de ser SDRE en A (solo B(q) varia);
//    · con  tau_ff = M q̈_d + C q̇_d + g  la planta da
//          M q̈ + C q̇ + g = M q̈_d + C q̇_d + g + u
//        ⟹ M q̈_e + C q̇_e = u
//        ⟹ ẋ_e = [0 I ; 0 −M^-1 C] x_e + [0 ; M^-1] u,
//      que es EXACTAMENTE el par (A, B) del plan.
//
//  Se implementa la segunda: hace que el (A,B) documentado sea exacto y no una
//  aproximacion, y es la formulacion SDRE estandar para manipuladores. La C se
//  evalua en el estado REAL (q, q̇) en ambos sitios, que es lo que hace que la
//  diferencia C q̇ − C q̇_d sea exactamente C q̇_e con la MISMA C.
//
//  ── Gravedad (compuerta G3) ───────────────────────────────────────────────
//  computeTau() devuelve el par fisico COMPLETO, g(q) incluida. Es la clase
//  base la que decide si se comanda o se resta (gravity_in_command).
//
//  ── Solver de la CARE ─────────────────────────────────────────────────────
//  Funcion signo de la matriz hamiltoniana (ver care_solver.hpp para por que no
//  Schur ni autovectores). Se puede DECIMAR con `lqr.care_update_rate`: la K se
//  mantiene con ZOH entre actualizaciones. El plan exige reportar la decimacion
//  usada, y el CSV de diagnostico la registra por paso.
//
//  ── Vigilancia por paso (criterios de aceptacion del plan) ────────────────
//  En CADA ciclo, no solo al actualizar, se evalua max Re(eig(A(t) − B(t) K))
//  con la K REALMENTE vigente. Esa es la pregunta que importa: la CARE
//  garantiza estabilidad en el instante en que se resuelve, pero entre
//  actualizaciones A(q,q̇) se mueve y la K congelada puede dejar de estabilizar.
//  Si el par (A,B) pierde la certificacion de estabilizabilidad, o si la CARE
//  falla repetidas veces seguidas, se pide SAFE_HOLD.
// ============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "ur5_dyn_control/care_solver.hpp"
#include "ur5_dyn_control/diag_logger.hpp"
#include "ur5_dyn_control/lqr_sdre_law.hpp"
#include "ur5_dyn_control/torque_control_node_base.hpp"

namespace ur5_dyn_control
{

namespace
{
constexpr int kNu = 6;    // dimension del control (el estado: 12 o 18, n_)

using Clock = std::chrono::steady_clock;

/// Microsegundos entre dos instantes. Se resta ANTES de convertir a double: el
/// tiempo absoluto desde epoch en microsegundos ronda 1e15, y a esa magnitud un
/// double solo resuelve ~0.1 us — justo el orden de lo que hay que medir.
double elapsedMicros(const Clock::time_point & t0, const Clock::time_point & t1)
{
  return std::chrono::duration<double, std::micro>(t1 - t0).count();
}
}  // namespace

class LqrSdreControlNode : public TorqueControlNodeBase
{
public:
  LqrSdreControlNode()
  : TorqueControlNodeBase("gz_lqr_sdre_control_node")
  {
    // ── Sintesis de Q, R ────────────────────────────────────────────────────
    // Q = blkdiag(Qp, Qv) con  Qp = r wn^4 M^2  y  Qv = r wn^2 (4 zeta^2 - 2) M^2,
    // R = r I. Ver lqr_sdre_params.yaml para la derivacion completa; el
    // resultado es que los DOCE polos del lazo cerrado caen en -wn con
    // amortiguamiento zeta, en las coordenadas normalizadas por masa.
    // B4.2: con integral (LQI, 18 estados) por defecto; false = la FASE 4.
    integral_ = declare_parameter<bool>("lqr.integral", true);
    n_ = lqrStates(integral_);
    weights_ = declare_parameter<std::string>("lqr.weights", "polynomial");
    if (weights_ != "polynomial" && weights_ != "physical") {
      throw std::runtime_error(
        "lqr.weights desconocido: '" + weights_ + "' (validos: polynomial, physical)");
    }
    wn_ = declare_parameter<double>("lqr.wn", 20.0);
    zeta_ = declare_parameter<double>("lqr.zeta", 1.0);
    const double r_ctrl = declare_parameter<double>("lqr.r", 1.0);
    if (!(wn_ > 0.0)) {throw std::runtime_error("lqr.wn debe ser > 0");}
    if (!(r_ctrl > 0.0)) {throw std::runtime_error("lqr.r debe ser > 0");}
    // Qv = r wn^2 (4 zeta^2 - 2) M^2 solo es semidefinida positiva si
    // zeta >= 1/sqrt(2). No es una limitacion de esta implementacion: es el
    // resultado clasico de que un LQR sobre un doble integrador no puede dar
    // menos amortiguamiento que el de Butterworth (zeta = 1/sqrt(2), que es
    // justo el caso Qv = 0). Pedir menos exigiria Q indefinida.
    constexpr double kZetaMin = 0.70710678118654746;   // 1/sqrt(2)
    if (!integral_ && weights_ == "polynomial" && zeta_ < kZetaMin) {
      throw std::runtime_error(
        "lqr.zeta = " + std::to_string(zeta_) + " < 1/sqrt(2): con Q = "
        "blkdiag(r wn^4 M^2, r wn^2 (4 zeta^2 - 2) M^2) eso da Qv definida "
        "NEGATIVA. Un LQR sobre este doble integrador no puede amortiguar por "
        "debajo de zeta = 1/sqrt(2) (el caso Qv = 0).");
    }
    R_ = Eigen::MatrixXd::Identity(kNu, kNu) * r_ctrl;
    r_ctrl_ = r_ctrl;
    if (weights_ == "physical") {
      // Pesos por junta en unidades fisicas (Bryson: Q_ii = 1/e_max,i^2).
      auto vec6 = [this](const char * name) {
          const auto v = declare_parameter<std::vector<double>>(name, std::vector<double>{});
          if (v.size() != 6) {
            throw std::runtime_error(std::string(name) +
                    " debe tener 6 valores con lqr.weights = physical");
          }
          Vector6d out;
          for (int i = 0; i < 6; ++i) {out[i] = v[static_cast<std::size_t>(i)];}
          return out;
        };
      q_sigma_ = integral_ ? vec6("lqr.q_sigma") : Vector6d::Zero();
      q_p_ = vec6("lqr.q_p");
      q_v_ = vec6("lqr.q_v");
      const Vector6d r_joint = vec6("lqr.r_joint");
      if ((r_joint.array() <= 0.0).any()) {
        throw std::runtime_error("lqr.r_joint debe ser > 0 en las seis juntas");
      }
      R_ = r_joint.asDiagonal();
    }

    // fixed     : Q constante, sintetizada con M(q_init). Es la formulacion del
    //             plan (Q y R son PESOS DE COSTE, constantes) y la que hace que
    //             la figura de max Re(lambda) vs t tenga algo que ensenar.
    // scheduled : Q(q) = f(M(q)), recalculada en cada actualizacion. El marco
    //             SDRE admite pesos dependientes del estado; asi los polos caen
    //             en -wn en TODA la trayectoria, no solo en q_init.
    q_mode_ = declare_parameter<std::string>("lqr.Q_mode", "fixed");
    if (q_mode_ != "fixed" && q_mode_ != "scheduled") {
      throw std::runtime_error(
        "lqr.Q_mode desconocido: '" + q_mode_ + "' (validos: fixed, scheduled)");
    }
    Q_ = buildQ(dyn().M(qInit()));

    opt_.residual_tol = declare_parameter<double>("lqr.residual_tol", 1e-6);
    opt_.stable_margin = declare_parameter<double>("lqr.stable_margin", 1e-9);
    opt_.max_iterations =
      static_cast<int>(declare_parameter<int>("lqr.care_max_iterations", 60));
    opt_.tol = declare_parameter<double>("lqr.care_tol", 1e-11);
    // El nodo ya evalua eig(A - B K) por su cuenta en cada ciclo; que lo haga
    // tambien el solver seria una descomposicion propia de mas por
    // actualizacion. Se deja activo porque es la comprobacion que decide si la
    // K nueva se acepta, y ese es el punto donde tiene que decidirse.
    opt_.check_closed_loop = true;

    ctrl_tol_ = declare_parameter<double>("lqr.controllability_tol", 1e-9);
    max_fails_ = static_cast<int>(
      declare_parameter<int>("lqr.max_consecutive_failures", 5));

    // Decimacion de la CARE (ZOH sobre K). 0 o negativo -> cada ciclo.
    const double ctrl_rate = get_parameter("control_rate").as_double();
    const double care_rate = declare_parameter<double>("lqr.care_update_rate", 0.0);
    care_decim_ = (care_rate > 0.0)
      ? std::max(1, static_cast<int>(std::lround(ctrl_rate / care_rate)))
      : 1;

    diag_enabled_ = declare_parameter<bool>("lqr.diag_csv", true);
    diag_decim_ = std::max<int>(
      1, static_cast<int>(declare_parameter<int>("lqr.diag_decimation", 1)));

    // ── Solucion de diseno en q_init ────────────────────────────────────────
    // Sirve para tres cosas: (1) validar Q/R antes de mover nada, (2) dar la K
    // de reserva si la primera actualizacion en linea falla, y (3) reportar los
    // polos de diseno, que es donde se ve si el escalado por inercia esta bien.
    {
      Eigen::MatrixXd A, B;
      buildSdcAt(qInit(), Vector6d::Zero(), A, B);
      const CareResult r = solveCare(A, B, Q_, R_, opt_);
      if (!r.ok) {
        throw std::runtime_error(
          "La CARE de diseno en q_init no tiene solucion estabilizante: " + r.reason +
          ". Revisa lqr.wn / lqr.zeta / lqr.r.");
      }
      K_ = r.K;
      RCLCPP_INFO(get_logger(),
                  "CARE de diseno en q_init: %d iteraciones, residuo relativo %.2e, "
                  "max Re(eig(A-BK)) = %.4f rad/s",
                  r.iterations, r.residual, r.max_real_eig);

      // Polos de diseno REALES: los 12 autovalores de A - B K. No se lee la
      // diagonal de K por junta, que solo seria valida si M fuese diagonal — y
      // no lo es ni de lejos (ver buildQ()).
      const Eigen::EigenSolver<Eigen::MatrixXd> es(A - B * K_, false);
      Eigen::VectorXd mag(n_);
      for (int i = 0; i < n_; ++i) {mag(i) = std::abs(es.eigenvalues()(i));}
      const double dt_nom = 1.0 / ctrl_rate;
      const double wn_dt_max = mag.maxCoeff() * dt_nom;
      RCLCPP_INFO(get_logger(),
                  "Polos de diseno: |lambda| en [%.2f, %.2f] rad/s (objetivo wn=%.1f, "
                  "zeta=%.2f), dispersion %.2fx, max|lambda|*dt = %.4f",
                  mag.minCoeff(), mag.maxCoeff(), wn_, zeta_,
                  mag.maxCoeff() / std::max(1e-12, mag.minCoeff()), wn_dt_max);
      if (wn_dt_max > 0.2) {
        RCLCPP_WARN(get_logger(),
                    "max|lambda|*dt = %.3f > 0.2: el lazo esta cerca del limite de "
                    "estabilidad DISCRETA. Baja lqr.wn o sube control_rate.",
                    wn_dt_max);
      }
    }

    // ── Cota del estado integral (anti-windup) ───────────────────────────────
    // |K_I σ| no puede pasar de una fraccion del par del actuador: con la K de
    // diseno, σ_max,i = frac * tau_max,i / |K_I,ii|.
    if (integral_) {
      const double frac = declare_parameter<double>("lqr.integral_tau_frac", 0.2);
      const auto tau_max = get_parameter("tau_max").as_double_array();
      const Matrix6d K_I = gainBlock(K_, true, 0);
      for (int i = 0; i < 6; ++i) {
        const double k = std::abs(K_I(i, i));
        sigma_max_[i] = (k > 1e-12)
          ? frac * tau_max[static_cast<std::size_t>(i)] / k
          : std::numeric_limits<double>::infinity();
      }
      RCLCPP_INFO(get_logger(),
                  "  integral: |K_I sigma| <= %.0f %% de tau_max -> sigma_max = "
                  "[%.3g %.3g %.3g %.3g %.3g %.3g] rad.s", 100.0 * frac,
                  sigma_max_[0], sigma_max_[1], sigma_max_[2], sigma_max_[3],
                  sigma_max_[4], sigma_max_[5]);
    }

    RCLCPP_INFO(get_logger(),
                "%s | pesos=%s Q_mode=%s wn=%.1f rad/s zeta=%.2f r=%.3g | "
                "CARE %dx%d cada %d ciclos (%.0f Hz, ZOH sobre K) | "
                "residual_tol=%.1e stable_margin=%.1e | max fallos seguidos=%d",
                integral_ ? "LQI-SDRE" : "LQR-SDRE (sin integral)",
                weights_.c_str(), q_mode_.c_str(), wn_, zeta_, r_ctrl_, n_, n_,
                care_decim_, ctrl_rate / care_decim_, opt_.residual_tol,
                opt_.stable_margin, max_fails_);

    // B4.1 / plan 4.5: la G comun, la fila i de K_D sobre toda la referencia.
    reportLoopGain(
      "K_D,ii (fila i de K_D)",
      [this](const Vector6d & q, const Vector6d & dq, const Vector6d &) {
        Eigen::MatrixXd A, B;
        const Matrix6d M = dyn().M(q);
        buildSdc(M, dyn().coriolis(q, dq), integral_, A, B);
        const Eigen::MatrixXd Q = (q_mode_ == "scheduled") ? buildQ(M) : Q_;
        const CareResult r = solveCare(A, B, Q, R_, opt_);
        return r.ok ? Matrix6d(gainBlock(r.K, integral_, 2)) : Matrix6d(gainBlock(K_, integral_, 2));
      });
    // R10: el lazo completo con la K de la CARE en cada punto. Por junta no
    // vale: con `Q_mode: fixed`, lejos de q_init M^-1 K tiene acoplos dominantes.
    reportDiscreteMargin(
      integral_ ? "LQI (K de la CARE)" : "LQR (K de la CARE)",
      [this](const Vector6d & q, const Vector6d & dq, const Vector6d &) {
        Eigen::MatrixXd A, B;
        const Matrix6d M = dyn().M(q);
        buildSdc(M, dyn().coriolis(q, dq), integral_, A, B);
        const Eigen::MatrixXd Q = (q_mode_ == "scheduled") ? buildQ(M) : Q_;
        const CareResult r = solveCare(A, B, Q, R_, opt_);
        const Eigen::MatrixXd & K = r.ok ? r.K : K_;
        LoopGains g;
        g.M = M;
        g.integral = integral_;
        if (integral_) {g.KI = gainBlock(K, integral_, 0);}
        g.KP = gainBlock(K, integral_, 1);
        g.KD = gainBlock(K, integral_, 2);
        return g;
      });

    start();

    if (diag_enabled_) {
      const bool ok = diag_.open(
        csvDir(), csvPrefix(), testNum(),
        {"max_re_eig", "cond_M", "ctrl_margin", "care_residual", "care_iters",
         "care_updated", "t_care_us", "t_law_us", "k_max", "care_fails",
         "sigma_abs_max"},
        traceMetadata());
      if (ok) {
        RCLCPP_INFO(get_logger(), "CSV de diagnostico: %s", diag_.path().c_str());
      } else {
        RCLCPP_WARN(get_logger(), "No se pudo abrir el CSV de diagnostico");
      }
    }
  }

protected:
  Vector6d computeTau(const Vector6d & q, const Vector6d & dq,
                      const JointRef & ref, double dt) override
  {
    const Clock::time_point t0 = Clock::now();

    const Matrix6d M = dyn().M(q);
    const Matrix6d C = dyn().coriolis(q, dq);
    const Vector6d g = dyn().gravity(q);

    Eigen::MatrixXd A, B;
    buildSdc(M, C, integral_, A, B);

    // ── Estabilizabilidad del par (A, B) ──────────────────────────────────
    // Se certifica por controlabilidad (controlable => estabilizable). Para
    // este par la certificacion solo se pierde si M(q) se vuelve numericamente
    // singular, asi que cond(M) se registra al lado como el diagnostico que
    // explica el fallo si llega a ocurrir.
    const double ctrl_margin = controllabilityMargin(A, B);
    cond_m_ = conditionNumber(M);
    if (!(ctrl_margin > ctrl_tol_)) {
      requestSafeHold(
        "el par (A,B) no queda certificado como estabilizable: margen de "
        "controlabilidad " + std::to_string(ctrl_margin) + " <= " +
        std::to_string(ctrl_tol_) + " (cond(M) = " + std::to_string(cond_m_) + ")");
      return dyn().nle(q, dq);   // no se publica; solo para no devolver basura
    }

    // ── Actualizacion de la CARE (con decimacion + ZOH) ────────────────────
    bool updated = false;
    double t_care = 0.0;
    if (tick_++ % care_decim_ == 0) {
      // Con Q_mode=scheduled los pesos se resintetizan con la M del instante,
      // asi que los polos caen en -wn en toda la trayectoria y no solo en
      // q_init. Con Q_mode=fixed (default) Q es un peso de coste CONSTANTE, que
      // es la formulacion del plan.
      if (q_mode_ == "scheduled") {Q_ = buildQ(M);}
      const Clock::time_point tc0 = Clock::now();
      const CareResult r = solveCare(A, B, Q_, R_, opt_);
      t_care = elapsedMicros(tc0, Clock::now());
      if (r.ok) {
        K_ = r.K;
        care_residual_ = r.residual;
        care_iters_ = r.iterations;
        consecutive_fails_ = 0;
        updated = true;
      } else {
        ++care_fails_;
        // ZOH sobre la ultima K valida. Un fallo aislado (una configuracion
        // desafortunada, un q̇ con ruido) no debe abortar la corrida; una racha
        // significa que el problema ya no es el paso, sino el diseno.
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "CARE fallida (%s); se mantiene la K anterior (%d/%d)",
                             r.reason.c_str(), consecutive_fails_ + 1, max_fails_);
        if (++consecutive_fails_ >= max_fails_) {
          requestSafeHold("la CARE fallo " + std::to_string(consecutive_fails_) +
                          " veces seguidas: " + r.reason);
          return dyn().nle(q, dq);
        }
      }
    }

    // ── Vigilancia por paso: estabilidad con la K REALMENTE vigente ────────
    // Criterio de aceptacion del plan: max Re(eig(A − B K)) < 0 en el 100 % de
    // los pasos. Con decimacion, entre actualizaciones A(q,q̇) se mueve y la K
    // congelada puede dejar de estabilizar: esto es justo lo que lo detecta.
    {
      const Eigen::EigenSolver<Eigen::MatrixXd> es(A - B * K_, false);
      max_re_eig_ = (es.info() == Eigen::Success)
        ? es.eigenvalues().real().maxCoeff()
        : std::numeric_limits<double>::quiet_NaN();
    }

    // ── Ley de control ────────────────────────────────────────────────────
    // B4.2: la integral solo corre en TRACK y HOLD_END (no en la rampa desde
    // q_init) y en las juntas que la base no esta recortando (onSaturation).
    if (integral_ && trackingOrHoldEnd()) {integ_.update(q - ref.q, dt, sigma_max_);}
    const Vector6d tau =
      lqrTorque(M, C, g, ref, K_, q - ref.q, dq - ref.dq, integ_.sigma, integral_);

    const double t_law = elapsedMicros(t0, Clock::now());
    t_law_max_ = std::max(t_law_max_, t_law);
    if (updated) {t_care_max_ = std::max(t_care_max_, t_care);}

    // Presupuesto declarado: un ciclo entero a control_rate. El maximo es la
    // cifra que importa (un solo ciclo desbordado ya rompe el determinismo del
    // lazo), no la media.
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "computo: ley %.0f us (max %.0f) | CARE %.0f us (max %.0f) "
                         "| presupuesto %.0f us | max Re(eig)=%.3f",
                         t_law, t_law_max_, t_care, t_care_max_,
                         1e6 / get_parameter("control_rate").as_double(), max_re_eig_);

    if (diag_.isOpen() && (diag_tick_++ % diag_decim_ == 0)) {
      diag_.log(now().seconds(),
                {max_re_eig_, cond_m_, ctrl_margin, care_residual_,
                 static_cast<double>(care_iters_), updated ? 1.0 : 0.0,
                 t_care, t_law, K_.cwiseAbs().maxCoeff(),
                 static_cast<double>(care_fails_), integ_.sigma.cwiseAbs().maxCoeff()});
    }
    return tau;
  }

  std::string csvPrefix() const override {return "lqr";}

  /// ANTI-WINDUP (B4.2): congela la integral en las juntas recortadas por
  /// saturacion o por limite de tasa en este tick.
  void onSaturation(const SaturationFlags & flags) override
  {
    for (std::size_t i = 0; i < 6; ++i) {
      integ_.frozen[i] = flags.saturated[i] || flags.rate_limited[i];
    }
  }

private:
  /// Q segun `lqr.weights` (lqr_sdre_law.hpp). Con `polynomial` va Q ∝ M²
  /// con la M COMPLETA, no diag(M)²: M(q_init) esta lejos de ser diagonal
  /// (M(1,5) = 2.8e-2 con M(5,5) = 5.4e-3) y con la receta diagonal los polos
  /// se dispersaban de 8.4 a 290 rad/s (test_care_solver lo fija).
  Eigen::MatrixXd buildQ(const Matrix6d & M) const
  {
    return (weights_ == "physical")
      ? physicalQ(q_sigma_, q_p_, q_v_, integral_)
      : polynomialQ(M, wn_, zeta_, r_ctrl_, integral_);
  }

  /// Version que evalua la dinamica ella misma (solo para el arranque). Nombre
  /// distinto a proposito: Eigen deja construir una Matrix6d a partir de una
  /// Vector6d (falla en tiempo de EJECUCION), asi que dos sobrecargas que solo
  /// se diferencien en eso salen ambiguas.
  void buildSdcAt(const Vector6d & q, const Vector6d & dq,
                  Eigen::MatrixXd & A, Eigen::MatrixXd & B)
  {
    buildSdc(dyn().M(q), dyn().coriolis(q, dq), integral_, A, B);
  }

  /// cond(M) = lambda_max / lambda_min. M es simetrica definida positiva, asi
  /// que sus autovalores SON sus valores singulares y sale mas barato que SVD.
  static double conditionNumber(const Matrix6d & M)
  {
    const Eigen::SelfAdjointEigenSolver<Matrix6d> es(M, Eigen::EigenvaluesOnly);
    if (es.info() != Eigen::Success) {
      return std::numeric_limits<double>::infinity();
    }
    const double lo = es.eigenvalues().minCoeff();
    const double hi = es.eigenvalues().maxCoeff();
    return (lo > 0.0) ? hi / lo : std::numeric_limits<double>::infinity();
  }

  Eigen::MatrixXd Q_, R_;
  Eigen::MatrixXd K_;
  bool integral_ = true;                // B4.2: LQI (18) o la FASE 4 (12)
  int n_ = 18;
  std::string weights_ = "polynomial";
  Vector6d q_sigma_ = Vector6d::Zero(), q_p_ = Vector6d::Zero(), q_v_ = Vector6d::Zero();
  IntegralState integ_;
  Vector6d sigma_max_ = Vector6d::Constant(std::numeric_limits<double>::infinity());
  CareOptions opt_;
  double wn_ = 20.0;
  double zeta_ = 1.0;
  double r_ctrl_ = 1.0;
  std::string q_mode_ = "fixed";

  double ctrl_tol_ = 1e-9;
  int max_fails_ = 5;
  int care_decim_ = 1;
  bool diag_enabled_ = true;
  int diag_decim_ = 1;

  long tick_ = 0;
  long diag_tick_ = 0;
  int consecutive_fails_ = 0;
  long care_fails_ = 0;
  int care_iters_ = 0;
  double care_residual_ = 0.0;
  double max_re_eig_ = 0.0;
  double cond_m_ = 0.0;
  double t_law_max_ = 0.0;
  double t_care_max_ = 0.0;

  DiagLogger diag_;
};

}  // namespace ur5_dyn_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::spin(std::make_shared<ur5_dyn_control::LqrSdreControlNode>());
  } catch (const std::exception & e) {
    // El nodo se nego a arrancar (sin bloque comun, inestable con el retardo
    // real, ...): ya lo dijo en el log. Salir con error, no con abort().
    std::fprintf(stderr, "%s: no arranca: %s\n", argv[0], e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
