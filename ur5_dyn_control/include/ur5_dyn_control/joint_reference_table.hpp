#ifndef UR5_DYN_CONTROL_JOINT_REFERENCE_TABLE_HPP
#define UR5_DYN_CONTROL_JOINT_REFERENCE_TABLE_HPP

#include <algorithm>
#include <string>
#include <vector>

#include "ur5_dyn_control/common.hpp"

namespace ur5_dyn_control
{

/// Diagnostico de la tabla construida (va al log y a las tablas del paper).
struct TrajectoryDiagnostics
{
  double sigma_min = 0.0;        ///< minimo de sigma_min(J) en todo el trazo
  double manipulability_min = 0.0;
  double sigma_min_t = 0.0;      ///< instante donde ocurre
  Vector6d dq_peak = Vector6d::Zero();
  Vector6d ddq_peak = Vector6d::Zero();
  double dq_margin = 1.0;        ///< min(1 - |dq|/dq_max) sobre toda la tabla
  double ddq_margin = 1.0;
};

/**
 * Tabla de referencias articulares {q, dq, ddq} muestreada al paso del lazo.
 *
 * Es lo unico que TorqueControlNodeBase necesita de un generador de
 * trayectorias, de modo que se puede alimentar indistintamente desde:
 *  - JointReferenceGenerator : trayectoria cartesiana + IK (incision, splines)
 *  - JointSweepGenerator     : barrido articular de excitacion (FASE 2)
 */
class JointReferenceTable
{
public:
  virtual ~JointReferenceTable() = default;

  const JointRef & at(std::size_t k) const
  {
    return (k >= table_.size()) ? table_.back() : table_[k];
  }
  std::size_t size() const { return table_.size(); }
  double dt() const { return dt_; }
  const TrajectoryDiagnostics & diagnostics() const { return diag_; }

  /// Etiqueta de la muestra k para la columna `state` del CSV. Vacia = usar la
  /// etiqueta del estado de la maquina (comportamiento historico).
  virtual std::string phaseLabel(std::size_t /*k*/) const { return {}; }

  /// B0.3: fase GEOMETRICA de la muestra k ("approach", "contact",
  /// "penetration", "cut", "withdraw" en la incision); vacia si el generador
  /// no la conoce. Va a la tabla exportada (reference_table_out), de donde la
  /// leen el evaluador y el analisis en vez de tiempos escritos a mano. NO va a
  /// la columna `state` del CSV, que sigue en TRACK: los analisis la usan para
  /// localizar la fase de seguimiento.
  std::string segmentLabel(std::size_t k) const
  {
    return segment_.empty() ? std::string() : segment_[std::min(k, segment_.size() - 1)];
  }

  /// B0.3: true si la muestra k esta en la MESETA de feed constante del corte,
  /// la ventana donde se miden las metricas del paper.
  bool inCutPlateau(std::size_t k) const
  {
    return !plateau_.empty() && plateau_[std::min(k, plateau_.size() - 1)] != 0;
  }

protected:
  std::vector<JointRef> table_;
  std::vector<std::string> segment_;      // B0.3, vacio si no hay fases
  std::vector<unsigned char> plateau_;    // B0.3, 1 = meseta del corte
  TrajectoryDiagnostics diag_;
  double dt_ = 0.0;
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_JOINT_REFERENCE_TABLE_HPP
