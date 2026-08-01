#ifndef UR5_DYN_CONTROL_INCISION_TRAJECTORY_HPP
#define UR5_DYN_CONTROL_INCISION_TRAJECTORY_HPP

#include <memory>
#include <string>
#include <vector>

#include "ur5_dyn_control/arc_length.hpp"
#include "ur5_dyn_control/cartesian_trajectory.hpp"
#include "ur5_dyn_control/quintic_spline.hpp"
#include "ur5_dyn_control/time_profile.hpp"

namespace ur5_dyn_control
{

/// Fases de la incision (FASE 1 del plan).
enum class IncisionPhaseId
{
  APPROACH,       ///< desde la pose de partida hasta un punto SOBRE la superficie
  CONTACT,        ///< descenso hasta tocar la superficie
  PENETRATION,    ///< descenso hasta la profundidad de corte d
  CUT,            ///< avance a profundidad constante y FEED CONSTANTE
  WITHDRAW,       ///< retirada vertical
};

const char * toString(IncisionPhaseId id);

/// Parametros geometricos y temporales de la incision (ver incision_params.yaml).
struct IncisionParams
{
  // -- Geometria (frame base_link, metros) ----------------------------------
  Eigen::Vector3d start_pose{0.49, 0.13, 0.35};  ///< TCP al inicio (= FK(q_init))
  double surface_z = 0.02;                       ///< cara superior del tejido
  double cut_x = 0.50;                           ///< coordenada fija del trazo
  /// Longitud de la incision MEDIDA: el tramo que se recorre a feed
  /// EXACTAMENTE constante y sobre el que se calculan las metricas del paper.
  double cut_length = 0.08;
  /// Entrada y salida adicionales a cada lado del tramo medido, donde ocurren
  /// las rampas de aceleracion y frenado. El bisturi corta
  /// cut_length + 2*cut_lead de material, pero solo los cut_length centrales
  /// estan a feed constante. Con cut_lead = 0 no habria meseta posible.
  double cut_lead = 0.010;
  double cut_center_y = 0.0;                     ///< centro del trazo
  double cut_depth = 0.005;                      ///< profundidad de penetracion
  double approach_height = 0.05;                 ///< altura sobre la superficie
  /// Eje del corte en el plano: 'y' (recomendado) o 'x'.
  char cut_axis = 'y';

  /**
   * SENTIDO del recorrido a lo largo de `cut_axis`: +1 o -1.
   *
   *   +1 -> el trazo va de (centro - medio) a (centro + medio): la coordenada
   *         del eje CRECE. Con cut_axis 'x', el TCP se ALEJA de la base.
   *   -1 -> al reves: la coordenada DECRECE; con 'x', se ACERCA a la base.
   *
   * Invierte las CINCO fases, no solo el corte: la aproximacion, el contacto y
   * la penetracion ocurren en el extremo por el que ahora se empieza, y la
   * retirada en el otro. Se implementa multiplicando los desplazamientos
   * respecto del centro, de modo que no hay geometria duplicada que pueda
   * quedar desincronizada.
   */
  int cut_direction = +1;

  /// Longitud total del trazo (lo que se corta de material).
  double cutStroke() const { return cut_length + 2.0 * cut_lead; }

  // -- Velocidades por fase (m/s) -------------------------------------------
  double v_approach = 0.10;
  double v_contact = 0.01;
  double v_penetration = 0.005;
  double v_cut = 0.010;                          ///< FEED del tramo de corte
  double v_withdraw = 0.05;

  // -- Forma del perfil temporal --------------------------------------------
  /// En el corte la fraccion de rampa NO es un parametro libre: se deriva de
  /// cut_lead, de modo que la meseta coincide EXACTAMENTE con cut_length.
  double ramp_fraction_move = 0.5;               ///< fases punto a punto: sin meseta

  /// Pausa en reposo entre fases (s). Deja que el transitorio se extinga y
  /// separa las fases para el analisis del paper.
  double dwell = 0.3;

  // -- Orientacion constante del TCP (supuesto A4) --------------------------
  Eigen::Vector3d tcp_rpy{M_PI, 0.0, -M_PI / 2};
};

/**
 * Trayectoria de INCISION en 5 fases, con separacion geometria/temporizacion.
 *
 * Cada fase es:
 *   geometria   QuinticSpline3d p(u), u in [0,1]
 *   arco        s(u) por Gauss-Legendre  ->  u(s)
 *   tiempo      s(t) S-curve con jerk acotado
 *   resultado   p(t) = p(u(s(t)))
 *
 * Una fase de dos waypoints es EXACTAMENTE un segmento recto (el Hermite
 * quintico con v=a=0 en ambos extremos degenera en la recta), asi que
 * `contact`, `penetration`, `cut` y `withdraw` son rectas por construccion y
 * la profundidad se mantiene constante durante el corte.
 *
 * Todas las fases empiezan y terminan en reposo con a = jerk = 0, de modo que
 * la concatenacion (con o sin dwell) conserva la continuidad C3.
 *
 * Velocidad de avance en el corte: |dp/dt| = ds/dt = v_cut EXACTAMENTE en la
 * meseta del perfil (la recta tiene |p'(u)| constante).
 */
class IncisionTrajectory : public CartesianTrajectory
{
public:
  explicit IncisionTrajectory(const IncisionParams & params);

  double startTime() const override { return 0.0; }
  double endTime() const override { return t_end_; }

  Eigen::Vector3d position(double t) const override;
  Eigen::Vector3d velocity(double t) const override;
  Eigen::Vector3d acceleration(double t) const override;
  Eigen::Vector3d jerk(double t) const override;
  const Eigen::Matrix3d & orientation() const override { return R_const_; }

  // -- Introspeccion (figuras y tablas del paper, y tests) -------------------
  struct PhaseInfo
  {
    IncisionPhaseId id;
    double t_start;        ///< instante en que arranca el movimiento de la fase
    double t_end;          ///< instante en que termina (antes del dwell)
    double length;         ///< longitud de arco recorrida
    double v_max;          ///< velocidad de crucero
    double plateau_t0;     ///< meseta de feed constante (absoluta)
    double plateau_t1;
    Eigen::Vector3d p_start;
    Eigen::Vector3d p_end;
    /// Extremos del tramo MEDIDO (solo la fase CUT): coinciden con la meseta.
    Eigen::Vector3d p_measured_start;
    Eigen::Vector3d p_measured_end;
  };

  const std::vector<PhaseInfo> & phases() const { return info_; }
  const PhaseInfo & phase(IncisionPhaseId id) const;

  /// Fase activa en el instante t (durante un dwell devuelve la fase que acaba
  /// de terminar).
  IncisionPhaseId phaseAt(double t) const;

  const IncisionParams & params() const { return params_; }

private:
  struct Phase
  {
    IncisionPhaseId id;
    std::unique_ptr<QuinticSpline3d> geom;
    std::unique_ptr<ArcLength> arc;
    std::unique_ptr<ScurveProfile> prof;
    double t_start = 0.0;
    double t_end = 0.0;
  };

  /// Devuelve el indice de la fase que contiene t, y t_local dentro de ella.
  std::size_t locate(double t, double & t_local) const;

  IncisionParams params_;
  Eigen::Matrix3d R_const_;
  std::vector<Phase> phases_;
  std::vector<PhaseInfo> info_;
  double t_end_ = 0.0;
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_INCISION_TRAJECTORY_HPP
