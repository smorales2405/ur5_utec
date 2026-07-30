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
  double cut_length = 0.08;                      ///< longitud de la incision
  double cut_center_y = 0.0;                     ///< centro del trazo
  double cut_depth = 0.005;                      ///< profundidad de penetracion
  double approach_height = 0.05;                 ///< altura sobre la superficie
  /// Eje del corte en el plano: 'y' (recomendado) o 'x'.
  char cut_axis = 'y';

  // -- Velocidades por fase (m/s) -------------------------------------------
  double v_approach = 0.10;
  double v_contact = 0.01;
  double v_penetration = 0.005;
  double v_cut = 0.010;                          ///< FEED del tramo de corte
  double v_withdraw = 0.05;

  // -- Forma del perfil temporal --------------------------------------------
  /// Fraccion de cada fase dedicada a rampa, en (0, 0.5]. En el corte controla
  /// que porcion del trazo se recorre a feed exactamente constante:
  /// meseta = (1 - 2*ramp_fraction) * cut_length.
  double ramp_fraction_cut = 0.10;
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
