#include "ur5_dyn_control/incision_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ur5_dyn_control
{

const char * toString(IncisionPhaseId id)
{
  switch (id) {
    case IncisionPhaseId::APPROACH:    return "approach";
    case IncisionPhaseId::CONTACT:     return "contact";
    case IncisionPhaseId::PENETRATION: return "penetration";
    case IncisionPhaseId::CUT:         return "cut";
    case IncisionPhaseId::WITHDRAW:    return "withdraw";
  }
  return "?";
}

IncisionTrajectory::IncisionTrajectory(const IncisionParams & p)
: params_(p)
{
  if (p.cut_axis != 'x' && p.cut_axis != 'y') {
    throw std::invalid_argument("IncisionTrajectory: cut_axis debe ser 'x' o 'y'");
  }
  if (!(p.cut_length > 0.0)) {
    throw std::invalid_argument("IncisionTrajectory: cut_length debe ser > 0");
  }
  if (!(p.cut_lead > 0.0)) {
    throw std::invalid_argument(
      "IncisionTrajectory: cut_lead debe ser > 0 (sin entrada/salida no hay "
      "meseta y el feed no puede ser constante en toda la incision medida)");
  }
  if (!(p.cut_depth > 0.0)) {
    throw std::invalid_argument("IncisionTrajectory: cut_depth debe ser > 0");
  }
  if (!(p.approach_height > 0.0)) {
    throw std::invalid_argument("IncisionTrajectory: approach_height debe ser > 0");
  }

  R_const_ = (Eigen::AngleAxisd(p.tcp_rpy[2], Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(p.tcp_rpy[1], Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(p.tcp_rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();

  // ── Waypoints de las 5 fases ─────────────────────────────────────────────
  const double z_surface = p.surface_z;
  const double z_cut = p.surface_z - p.cut_depth;      // punta de la hoja
  const double z_above = p.surface_z + p.approach_height;
  // El trazo total incluye la entrada y la salida; el tramo MEDIDO son los
  // cut_length centrales, que quedan integramente a feed constante.
  const double half_stroke = 0.5 * p.cutStroke();
  const double half_measured = 0.5 * p.cut_length;

  auto planePoint = [&p](double offset) {
      return (p.cut_axis == 'y')
             ? Eigen::Vector2d(p.cut_x, p.cut_center_y + offset)
             : Eigen::Vector2d(p.cut_x + offset, p.cut_center_y);
    };
  const Eigen::Vector2d xy_a = planePoint(-half_stroke);
  const Eigen::Vector2d xy_b = planePoint(+half_stroke);
  const Eigen::Vector2d xy_m0 = planePoint(-half_measured);
  const Eigen::Vector2d xy_m1 = planePoint(+half_measured);

  const Eigen::Vector3d p_above_a(xy_a.x(), xy_a.y(), z_above);
  const Eigen::Vector3d p_surf_a(xy_a.x(), xy_a.y(), z_surface);
  const Eigen::Vector3d p_cut_a(xy_a.x(), xy_a.y(), z_cut);
  const Eigen::Vector3d p_cut_b(xy_b.x(), xy_b.y(), z_cut);
  const Eigen::Vector3d p_above_b(xy_b.x(), xy_b.y(), z_above);
  const Eigen::Vector3d p_meas_0(xy_m0.x(), xy_m0.y(), z_cut);
  const Eigen::Vector3d p_meas_1(xy_m1.x(), xy_m1.y(), z_cut);

  // Fraccion de rampa del corte: derivada, no libre. Con ella la meseta del
  // perfil dura exactamente lo que tarda en recorrerse cut_length.
  const double ramp_fraction_cut = p.cut_lead / p.cutStroke();

  struct Spec
  {
    IncisionPhaseId id;
    std::vector<Eigen::Vector3d> wp;
    double v_max;
    double ramp_fraction;
  };

  // La fase de aproximacion admite waypoints intermedios (spline de varios
  // tramos); las demas son rectas de dos puntos por construccion.
  std::vector<Eigen::Vector3d> approach_wp{p.start_pose};
  approach_wp.push_back(p_above_a);

  const std::vector<Spec> specs = {
    {IncisionPhaseId::APPROACH,    approach_wp,               p.v_approach,     p.ramp_fraction_move},
    {IncisionPhaseId::CONTACT,     {p_above_a, p_surf_a},     p.v_contact,      p.ramp_fraction_move},
    {IncisionPhaseId::PENETRATION, {p_surf_a,  p_cut_a},      p.v_penetration,  p.ramp_fraction_move},
    {IncisionPhaseId::CUT,         {p_cut_a,   p_cut_b},      p.v_cut,          ramp_fraction_cut},
    {IncisionPhaseId::WITHDRAW,    {p_cut_b,   p_above_b},    p.v_withdraw,     p.ramp_fraction_move},
  };

  double t = 0.0;
  for (const auto & sp : specs) {
    Phase ph;
    ph.id = sp.id;
    ph.geom = std::make_unique<QuinticSpline3d>(sp.wp);
    ph.arc = std::make_unique<ArcLength>(*ph.geom);
    ph.prof = std::make_unique<ScurveProfile>(
      ph.arc->totalLength(), sp.v_max, sp.ramp_fraction);
    ph.t_start = t;
    ph.t_end = t + ph.prof->duration();

    double q0, q1;
    ph.prof->plateauInterval(q0, q1);

    PhaseInfo pi;
    pi.id = sp.id;
    pi.t_start = ph.t_start;
    pi.t_end = ph.t_end;
    pi.length = ph.arc->totalLength();
    pi.v_max = sp.v_max;
    pi.plateau_t0 = ph.t_start + q0;
    pi.plateau_t1 = ph.t_start + q1;
    pi.p_start = sp.wp.front();
    pi.p_end = sp.wp.back();
    // Solo el corte tiene tramo medido distinto del recorrido.
    pi.p_measured_start = (sp.id == IncisionPhaseId::CUT) ? p_meas_0 : sp.wp.front();
    pi.p_measured_end = (sp.id == IncisionPhaseId::CUT) ? p_meas_1 : sp.wp.back();

    t = ph.t_end + p.dwell;
    phases_.push_back(std::move(ph));
    info_.push_back(pi);
  }

  // El ultimo dwell no cuenta: la trayectoria termina al acabar `withdraw`.
  t_end_ = phases_.back().t_end;
}

const IncisionTrajectory::PhaseInfo & IncisionTrajectory::phase(IncisionPhaseId id) const
{
  for (const auto & pi : info_) {
    if (pi.id == id) {return pi;}
  }
  throw std::invalid_argument("IncisionTrajectory::phase: fase desconocida");
}

std::size_t IncisionTrajectory::locate(double t, double & t_local) const
{
  // Ultima fase cuyo t_start <= t. Durante un dwell devuelve la fase que acaba
  // de terminar, con t_local saturado a su duracion -> reposo en el punto final.
  std::size_t i = 0;
  for (std::size_t k = 0; k < phases_.size(); ++k) {
    if (t >= phases_[k].t_start) {i = k;} else {break;}
  }
  t_local = std::clamp(t - phases_[i].t_start, 0.0, phases_[i].prof->duration());
  return i;
}

IncisionPhaseId IncisionTrajectory::phaseAt(double t) const
{
  double t_local;
  return phases_[locate(t, t_local)].id;
}

Eigen::Vector3d IncisionTrajectory::position(double t) const
{
  double tl;
  const Phase & ph = phases_[locate(t, tl)];
  return ph.geom->eval(ph.arc->uOfS(ph.prof->s(tl)), 0);
}

Eigen::Vector3d IncisionTrajectory::velocity(double t) const
{
  double tl;
  const Phase & ph = phases_[locate(t, tl)];
  const double u = ph.arc->uOfS(ph.prof->s(tl));

  double du_ds, d2u_ds2, d3u_ds3;
  ph.arc->uDerivatives(u, du_ds, d2u_ds2, d3u_ds3);

  // p'(t) = P_u * (du/dt),  du/dt = (du/ds)*(ds/dt)
  const double du_dt = du_ds * ph.prof->sd(tl);
  return ph.geom->eval(u, 1) * du_dt;
}

Eigen::Vector3d IncisionTrajectory::acceleration(double t) const
{
  double tl;
  const Phase & ph = phases_[locate(t, tl)];
  const double u = ph.arc->uOfS(ph.prof->s(tl));

  double du_ds, d2u_ds2, d3u_ds3;
  ph.arc->uDerivatives(u, du_ds, d2u_ds2, d3u_ds3);
  const double sd = ph.prof->sd(tl);
  const double sdd = ph.prof->sdd(tl);

  const double du_dt = du_ds * sd;
  const double d2u_dt2 = d2u_ds2 * sd * sd + du_ds * sdd;

  return ph.geom->eval(u, 2) * (du_dt * du_dt) + ph.geom->eval(u, 1) * d2u_dt2;
}

Eigen::Vector3d IncisionTrajectory::jerk(double t) const
{
  double tl;
  const Phase & ph = phases_[locate(t, tl)];
  const double u = ph.arc->uOfS(ph.prof->s(tl));

  double du_ds, d2u_ds2, d3u_ds3;
  ph.arc->uDerivatives(u, du_ds, d2u_ds2, d3u_ds3);
  const double sd = ph.prof->sd(tl);
  const double sdd = ph.prof->sdd(tl);
  const double sddd = ph.prof->sddd(tl);

  const double du_dt = du_ds * sd;
  const double d2u_dt2 = d2u_ds2 * sd * sd + du_ds * sdd;
  const double d3u_dt3 = d3u_ds3 * sd * sd * sd + 3.0 * d2u_ds2 * sd * sdd + du_ds * sddd;

  return ph.geom->eval(u, 3) * (du_dt * du_dt * du_dt) +
         ph.geom->eval(u, 2) * (3.0 * du_dt * d2u_dt2) +
         ph.geom->eval(u, 1) * d3u_dt3;
}

}  // namespace ur5_dyn_control
