#include "ur5_dyn_control/csv_logger.hpp"

#include <cstdlib>
#include <filesystem>
#include <iomanip>

namespace ur5_dyn_control
{

CsvLogger::~CsvLogger()
{
  close();
}

bool CsvLogger::open(const std::string & output_dir,
                     const std::string & prefix,
                     int test_num)
{
  std::string dir = output_dir;
  if (dir.empty()) {
    const char * home = std::getenv("HOME");
    dir = std::string(home ? home : ".") + "/.ros/ur5_dyn_control";
  }
  std::filesystem::create_directories(dir);
  path_ = dir + "/" + prefix + "_" + std::to_string(test_num) + ".csv";

  csv_.open(path_);
  if (!csv_.is_open()) {
    return false;
  }

  csv_ << "t";
  for (int i = 1; i <= 6; ++i) {csv_ << ",q" << i;}
  for (int i = 1; i <= 6; ++i) {csv_ << ",dq" << i;}
  for (int i = 1; i <= 6; ++i) {csv_ << ",q" << i << "_des";}
  for (int i = 1; i <= 6; ++i) {csv_ << ",dq" << i << "_des";}
  for (int i = 1; i <= 6; ++i) {csv_ << ",ddq" << i << "_des";}
  for (int i = 1; i <= 6; ++i) {csv_ << ",tau" << i;}
  csv_ << ",x,y,z,x_des,y_des,z_des,state\n";
  return true;
}

void CsvLogger::log(double t,
                    const Vector6d & q, const Vector6d & dq,
                    const JointRef & ref,
                    const Vector6d & tau,
                    const Eigen::Vector3d & tcp,
                    const Eigen::Vector3d & tcp_des,
                    const std::string & state)
{
  if (!csv_.is_open()) {return;}
  // 9 decimales = 1 nm en las columnas cartesianas. Con los 6 anteriores el
  // paso de cuantizacion (1 um) era el 2.5 % del RMSE que hay que reportar y
  // el 5 % del avance por muestra a 10 mm/s: al derivar el CSV para medir el
  // feed, el ruido de cuantizacion tapaba la senal. Medido: con 6 decimales la
  // desviacion aparente del feed de REFERENCIA (que es exacto por
  // construccion) era del 10 % en ventanas de 0.1 s.
  csv_ << std::fixed << std::setprecision(9) << t;
  for (int i = 0; i < 6; ++i) {csv_ << ',' << q[i];}
  for (int i = 0; i < 6; ++i) {csv_ << ',' << dq[i];}
  for (int i = 0; i < 6; ++i) {csv_ << ',' << ref.q[i];}
  for (int i = 0; i < 6; ++i) {csv_ << ',' << ref.dq[i];}
  for (int i = 0; i < 6; ++i) {csv_ << ',' << ref.ddq[i];}
  for (int i = 0; i < 6; ++i) {csv_ << ',' << tau[i];}
  csv_ << ',' << tcp.x() << ',' << tcp.y() << ',' << tcp.z()
       << ',' << tcp_des.x() << ',' << tcp_des.y() << ',' << tcp_des.z()
       << ',' << state << '\n';
}

void CsvLogger::close()
{
  if (csv_.is_open()) {
    csv_.close();
  }
}

}  // namespace ur5_dyn_control
