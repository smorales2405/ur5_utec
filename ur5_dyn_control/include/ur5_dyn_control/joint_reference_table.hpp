#ifndef UR5_DYN_CONTROL_JOINT_REFERENCE_TABLE_HPP
#define UR5_DYN_CONTROL_JOINT_REFERENCE_TABLE_HPP

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

protected:
  std::vector<JointRef> table_;
  TrajectoryDiagnostics diag_;
  double dt_ = 0.0;
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_JOINT_REFERENCE_TABLE_HPP
