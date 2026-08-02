#include "ur5_dyn_control/joint_sweep_generator.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "ur5_dyn_control/time_profile.hpp"

namespace ur5_dyn_control
{

namespace
{

/// Quintico "smoothstep" y derivadas: transiciones punto a punto en reposo.
inline double S(double x) {return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));}
inline double dS(double x) {return 30.0 * x * x * (1.0 - x) * (1.0 - x);}
inline double ddS(double x) {return 60.0 * x * (1.0 + x * (-3.0 + 2.0 * x));}

std::string velocityTag(double v)
{
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(3);
  oss << std::abs(v);
  return oss.str();
}

}  // namespace

JointSweepGenerator::JointSweepGenerator(const JointSweepParams & params)
: params_(params)
{
}

bool JointSweepGenerator::build(double dt, std::string & error_msg)
{
  dt_ = dt;
  table_.clear();
  labels_.clear();
  segments_.clear();
  diag_ = TrajectoryDiagnostics();

  const auto & p = params_;
  if (p.joint < 0 || p.joint >= 6) {
    error_msg = "joint fuera de rango (0..5)";
    return false;
  }
  if (!(p.amplitude > 0.0)) {
    error_msg = "amplitude debe ser > 0";
    return false;
  }
  if (p.velocities.empty()) {
    error_msg = "velocities no puede estar vacio";
    return false;
  }

  const int j = p.joint;
  const double q_center = p.q_fixed[j];

  // ── Planificacion de los tramos ──────────────────────────────────────────
  // Amplitud efectiva por nivel: si el barrido completo no cabe en
  // max_sweep_duration, se reduce la AMPLITUD (nunca la velocidad, que es la
  // variable independiente de la identificacion).
  struct Level
  {
    double v;
    double amp;
  };
  std::vector<Level> levels;
  for (const double v : p.velocities) {
    if (!(v > 0.0)) {
      error_msg = "las velocidades deben ser > 0";
      return false;
    }
    double amp = p.amplitude;
    if (p.max_sweep_duration > 0.0) {
      // T = (1 + 2*phi) * L / v  con L = 2*amp  =>  amp = v*T / (2*(1+2*phi))
      const double amp_max =
        v * p.max_sweep_duration / (2.0 * (1.0 + 2.0 * p.ramp_fraction));
      amp = std::min(amp, amp_max);
    }
    // Aceleracion pico de la rampa: a = 1.875 * v / T_r,  T_r = 2*phi*L/v
    const double a_peak = 1.875 * v * v / (2.0 * p.ramp_fraction * 2.0 * amp);
    if (a_peak > p.ddq_max) {
      std::ostringstream oss;
      oss << "el nivel v=" << v << " rad/s exige a_pico=" << a_peak
          << " rad/s^2 > ddq_max=" << p.ddq_max
          << ". Baje la velocidad, suba ramp_fraction o suba la amplitud.";
      error_msg = oss.str();
      return false;
    }
    levels.push_back({v, amp});
  }

  // Instantes de los tramos.
  //
  // CADA nivel se RECENTRA antes de empezar: se inserta una transicion hasta
  // q_center - amp_del_nivel. Encadenar sin recentrar solo es simetrico si
  // todas las amplitudes coinciden, y `max_sweep_duration` las recorta a baja
  // velocidad, asi que el barrido derivaba: con las velocidades por defecto
  // llegaba a q_center + 72 grados en vez de +45. Ver Segment::q_target.
  double t = 0.0;
  double q_cur = q_center;

  auto push_transition = [&](double target) {
      segments_.push_back({t, t + p.approach_duration, 0.0, 0.0, 0.0, 0.0,
                           false, target});
      t += p.approach_duration + p.dwell;
      q_cur = target;
    };

  for (const auto & lv : levels) {
    const double start = q_center - lv.amp;
    if (std::abs(q_cur - start) > 1e-9) {
      push_transition(start);
    }
    const double L = 2.0 * lv.amp;
    const ScurveProfile prof(L, lv.v, p.ramp_fraction);
    double r0, r1;
    prof.plateauInterval(r0, r1);
    // Sentido positivo, luego negativo, al mismo nivel de velocidad: el par
    // deja la junta de vuelta en `start`.
    for (const double sign : {+1.0, -1.0}) {
      segments_.push_back({t, t + prof.duration(), t + r0, t + r1,
                           sign * lv.v, lv.amp, true, 0.0});
      t += prof.duration() + p.dwell;
      q_cur += sign * L;
    }
  }
  // Vuelta al centro.
  segments_.push_back({t, t + p.approach_duration, 0.0, 0.0, 0.0, 0.0,
                       false, q_center});
  t += p.approach_duration;
  duration_ = t;

  // ── Muestreo ─────────────────────────────────────────────────────────────
  const std::size_t n = static_cast<std::size_t>(std::ceil(duration_ / dt)) + 1;
  table_.reserve(n);
  labels_.reserve(n);

  // Posicion de la junta al final de cada tramo, para encadenar.
  // El primer tramo lleva de q_center a q_center - amp0; el ultimo vuelve.
  double q_seg_start = q_center;
  std::vector<double> seg_start(segments_.size()), seg_end(segments_.size());
  {
    double q = q_center;
    for (std::size_t s = 0; s < segments_.size(); ++s) {
      seg_start[s] = q;
      if (!segments_[s].useful) {
        q = segments_[s].q_target;      // destino explicito de la transicion
      } else {
        q += (segments_[s].velocity > 0.0 ? +1.0 : -1.0) * 2.0 * segments_[s].amplitude;
      }
      seg_end[s] = q;
    }
    q_seg_start = seg_start.front();
  }
  (void)q_seg_start;

  for (std::size_t k = 0; k < n; ++k) {
    const double tk = std::min(static_cast<double>(k) * dt, duration_);

    // Tramo activo: el ultimo cuyo t_start <= tk (durante un dwell, el que
    // acaba de terminar, saturado a su final -> reposo).
    std::size_t s = 0;
    for (std::size_t i = 0; i < segments_.size(); ++i) {
      if (tk >= segments_[i].t_start) {s = i;} else {break;}
    }
    const Segment & sg = segments_[s];
    const double tl = std::clamp(tk - sg.t_start, 0.0, sg.t_end - sg.t_start);

    JointRef ref;
    ref.q = p.q_fixed;
    double qj = seg_end[s], dqj = 0.0, ddqj = 0.0;

    if (!sg.useful) {
      // Transicion quintica entre seg_start y seg_end.
      const double T = sg.t_end - sg.t_start;
      const double x = std::clamp(tl / T, 0.0, 1.0);
      const double d = seg_end[s] - seg_start[s];
      qj = seg_start[s] + d * S(x);
      dqj = d * dS(x) / T;
      ddqj = d * ddS(x) / (T * T);
    } else {
      const ScurveProfile prof(2.0 * sg.amplitude, std::abs(sg.velocity),
                               p.ramp_fraction);
      const double dir = (sg.velocity > 0.0) ? +1.0 : -1.0;
      qj = seg_start[s] + dir * prof.s(tl);
      dqj = dir * prof.sd(tl);
      ddqj = dir * prof.sdd(tl);
    }

    ref.q[j] = qj;
    ref.dq = Vector6d::Zero();
    ref.ddq = Vector6d::Zero();
    ref.dq[j] = dqj;
    ref.ddq[j] = ddqj;

    diag_.dq_peak = diag_.dq_peak.cwiseMax(ref.dq.cwiseAbs());
    diag_.ddq_peak = diag_.ddq_peak.cwiseMax(ref.ddq.cwiseAbs());

    // Etiqueta: solo la MESETA es ventana util de identificacion.
    std::string label;
    if (sg.useful && tk >= sg.plateau_t0 && tk <= sg.plateau_t1) {
      label = "SWEEP_" + velocityTag(sg.velocity) +
        (sg.velocity > 0.0 ? "_POS" : "_NEG");
    } else if (sg.useful) {
      label = "SWEEP_RAMP";
    } else {
      label = "SWEEP_MOVE";
    }

    table_.push_back(ref);
    labels_.push_back(std::move(label));
  }

  // ── Guarda de rango ──────────────────────────────────────────────────────
  // La tabla NO puede salirse de q_fixed +- amplitude. Se comprueba sobre las
  // muestras ya generadas, no sobre la aritmetica de los tramos: si una
  // refactorizacion futura vuelve a romper el encadenado, esto lo para aqui y
  // no en el robot. Un barrido fuera de rango metio el efector final contra la
  // mesa antes de existir esta comprobacion.
  {
    double q_lo = q_center, q_hi = q_center;
    for (const auto & ref : table_) {
      q_lo = std::min(q_lo, ref.q[p.joint]);
      q_hi = std::max(q_hi, ref.q[p.joint]);
    }
    const double tol = 1e-6;
    if (q_lo < q_center - p.amplitude - tol || q_hi > q_center + p.amplitude + tol) {
      std::ostringstream oss;
      oss << "el barrido generado se sale del rango pedido: ["
          << q_lo << ", " << q_hi << "] rad, pero q_fixed +- amplitude es ["
          << q_center - p.amplitude << ", " << q_center + p.amplitude
          << "]. Es un fallo del generador, no de la configuracion.";
      error_msg = oss.str();
      table_.clear();
      labels_.clear();
      return false;
    }
  }

  error_msg.clear();
  return true;
}

std::string JointSweepGenerator::phaseLabel(std::size_t k) const
{
  if (labels_.empty()) {return {};}
  return labels_[std::min(k, labels_.size() - 1)];
}

}  // namespace ur5_dyn_control
