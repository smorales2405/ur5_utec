#ifndef UR5_DYN_CONTROL_CARTESIAN_TRAJECTORY_HPP
#define UR5_DYN_CONTROL_CARTESIAN_TRAJECTORY_HPP

#include <Eigen/Dense>

namespace ur5_dyn_control
{

/**
 * Interfaz comun de las trayectorias cartesianas de referencia.
 *
 * JointReferenceGenerator trabaja contra esta interfaz, de modo que cambiar de
 * spline cubico a quintico o a la trayectoria de incision no requiere tocarlo
 * (FASE 1 del plan: "misma interfaz para que JointReferenceGenerator no
 * cambie").
 *
 * Convenios:
 *  - t se satura a [startTime(), endTime()]: fuera del intervalo la referencia
 *    es el extremo, en reposo.
 *  - La orientacion del TCP es CONSTANTE (supuesto A4): omega = alpha = 0.
 *  - jerk() existe para poder verificar continuidad C3 y para las metricas del
 *    paper; una trayectoria cubica lo tiene discontinuo en los nudos, que es
 *    justamente lo que el criterio de aceptacion de la FASE 1 comprueba.
 */
class CartesianTrajectory
{
public:
  virtual ~CartesianTrajectory() = default;

  virtual double startTime() const = 0;
  virtual double endTime() const = 0;
  double duration() const { return endTime() - startTime(); }

  virtual Eigen::Vector3d position(double t) const = 0;
  virtual Eigen::Vector3d velocity(double t) const = 0;
  virtual Eigen::Vector3d acceleration(double t) const = 0;
  virtual Eigen::Vector3d jerk(double t) const = 0;

  virtual const Eigen::Matrix3d & orientation() const = 0;
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_CARTESIAN_TRAJECTORY_HPP
