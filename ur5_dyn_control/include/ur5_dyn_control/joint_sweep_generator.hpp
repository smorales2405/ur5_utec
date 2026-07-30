#ifndef UR5_DYN_CONTROL_JOINT_SWEEP_GENERATOR_HPP
#define UR5_DYN_CONTROL_JOINT_SWEEP_GENERATOR_HPP

#include <string>
#include <vector>

#include "ur5_dyn_control/common.hpp"
#include "ur5_dyn_control/joint_reference_table.hpp"

namespace ur5_dyn_control
{

/// Parametros del barrido de excitacion para identificar friccion (FASE 2).
struct JointSweepParams
{
  /// Configuracion en la que se mantienen las juntas NO barridas.
  Vector6d q_fixed = (Vector6d() << 0.0, -M_PI / 2, M_PI / 2,
                      -M_PI / 2, -M_PI / 2, 0.0).finished();

  int joint = 0;                 ///< junta que se barre (0..5)
  double amplitude = M_PI / 4;   ///< +-45 grados desde q_fixed[joint]

  /// Niveles de |q̇| a barrer [rad/s]. Cada nivel se recorre en AMBOS sentidos.
  std::vector<double> velocities = {0.05, 0.10, 0.20, 0.35, 0.50, 0.75, 1.00};

  /// Fraccion de cada barrido dedicada a rampa, en (0, 0.5]. La MESETA
  /// (velocidad constante, aceleracion nula) es la parte util para identificar:
  /// alli el residuo de par es friccion pura.
  double ramp_fraction = 0.15;

  /// Cota superior de la duracion de un barrido [s]. Si un nivel lento no cabe,
  /// se REDUCE su amplitud (no su velocidad) para no alargar la campana. 0 =
  /// sin limite.
  double max_sweep_duration = 40.0;

  double dwell = 1.0;            ///< reposo entre barridos [s]
  double approach_duration = 3.0;///< transicion inicial q_fixed -> primer extremo

  /// Aceleracion maxima admitida en las rampas [rad/s^2]. Un nivel que la
  /// exceda se rechaza en construccion (no se recorta en silencio).
  double ddq_max = 5.0;
};

/**
 * Tabla de referencias para IDENTIFICACION DE FRICCION (FASE 2).
 *
 * Genera, en espacio articular puro (sin IK), un barrido de UNA junta mientras
 * las demas se mantienen en q_fixed:
 *
 *   1. transicion suave  q_fixed[j]  ->  q_fixed[j] - A
 *   2. por cada velocidad v de `velocities`:
 *        barrido  -A -> +A  a  +v     (meseta de velocidad constante)
 *        reposo
 *        barrido  +A -> -A  a  -v     (mismo nivel, sentido contrario)
 *        reposo
 *   3. transicion suave  ->  q_fixed[j]
 *
 * Cada barrido usa el mismo perfil S-curve con jerk acotado que la incision
 * (ScurveProfile), asi que empieza y termina en reposo con a = jerk = 0 y tiene
 * una MESETA de velocidad exactamente constante. Solo la meseta se marca como
 * util: alli q̈ = 0 y el residuo
 *
 *     tau_residual = tau_cmd - RNEA(q, q̇, q̈)
 *
 * es friccion pura, sin terminos inerciales. Los dos sentidos permiten separar
 * el termino de Coulomb (impar en q̇) del viscoso (tambien impar) del sesgo del
 * modelo (par).
 */
class JointSweepGenerator : public JointReferenceTable
{
public:
  /// Un tramo del barrido, con la ventana util marcada.
  struct Segment
  {
    double t_start;        ///< inicio del movimiento
    double t_end;          ///< fin del movimiento
    double plateau_t0;     ///< ventana UTIL: velocidad constante, q̈ = 0
    double plateau_t1;
    double velocity;       ///< velocidad con signo [rad/s]
    double amplitude;      ///< amplitud efectiva de este tramo [rad]
    bool useful;           ///< false para las transiciones de entrada/salida
  };

  explicit JointSweepGenerator(const JointSweepParams & params);

  /// Muestrea la campana con paso dt. false + error_msg si algun nivel viola
  /// ddq_max o si la amplitud efectiva queda por debajo de min_amplitude.
  bool build(double dt, std::string & error_msg);

  const std::vector<Segment> & segments() const { return segments_; }
  const JointSweepParams & params() const { return params_; }

  /// Etiqueta de estado para el CSV: "SWEEP_<v>_<sentido>" en la meseta,
  /// "RAMP" fuera de ella. Permite al identificador filtrar por ventana util
  /// sin recalcular la temporizacion.
  std::string phaseLabel(std::size_t k) const override;

  /// Duracion total de la campana [s].
  double duration() const { return duration_; }

private:
  JointSweepParams params_;
  std::vector<Segment> segments_;
  std::vector<std::string> labels_;
  double duration_ = 0.0;
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_JOINT_SWEEP_GENERATOR_HPP
