#ifndef UR5_DYN_CONTROL_CSV_LOGGER_HPP
#define UR5_DYN_CONTROL_CSV_LOGGER_HPP

#include <fstream>
#include <string>

#include "ur5_dyn_control/common.hpp"

namespace ur5_dyn_control
{

/**
 * Logger CSV para los nodos de control dinamico (mismo espiritu que los CSV
 * de ur5_pick_place y del paquete OMX, para post-proceso en MATLAB).
 *
 * Columnas: t, q1..q6, dq1..dq6, q1_des..q6_des, dq1_des..dq6_des,
 *           ddq1_des..ddq6_des, tau1..tau6, x, y, z, x_des, y_des, z_des, state
 *
 * tau1..tau6 es el torque COMANDADO (despues de la politica de gravedad G3 y
 * de la saturacion), no la salida cruda de la ley: es la magnitud que
 * realmente llega al hardware y la unica referencia de torque disponible en el
 * robot real (el campo `effort` de /joint_states son corrientes de motor).
 */
class CsvLogger
{
public:
  /// output_dir vacio -> $HOME/.ros/ur5_dyn_control
  CsvLogger() = default;
  ~CsvLogger();

  bool open(const std::string & output_dir,
            const std::string & prefix,
            int test_num);

  void log(double t,
           const Vector6d & q, const Vector6d & dq,
           const JointRef & ref,
           const Vector6d & tau,
           const Eigen::Vector3d & tcp,
           const Eigen::Vector3d & tcp_des,
           const std::string & state);

  void close();
  bool isOpen() const { return csv_.is_open(); }
  const std::string & path() const { return path_; }

private:
  std::ofstream csv_;
  std::string path_;
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_CSV_LOGGER_HPP
