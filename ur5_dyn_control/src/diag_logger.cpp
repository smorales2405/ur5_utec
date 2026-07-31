#include "ur5_dyn_control/diag_logger.hpp"

#include <iomanip>

#include "ur5_dyn_control/log_utils.hpp"

namespace ur5_dyn_control
{

DiagLogger::~DiagLogger()
{
  close();
}

bool DiagLogger::open(const std::string & output_dir,
                      const std::string & prefix,
                      int test_num,
                      const std::vector<std::string> & columns,
                      const std::map<std::string, std::string> & metadata)
{
  close();
  if (columns.empty()) {return false;}

  const std::string dir = log_utils::resolveDir(output_dir);
  log_utils::makeDirs(dir);

  path_ = dir + "/" + prefix + "_diag_" + std::to_string(test_num) + ".csv";
  csv_.open(path_);
  if (!csv_.is_open()) {return false;}

  csv_ << "# controller_id=" << prefix << "\n";
  csv_ << "# test_num=" << test_num << "\n";
  csv_ << "# timestamp=" << log_utils::isoTimestamp() << "\n";
  for (const auto & [k, v] : metadata) {
    csv_ << "# " << k << "=" << v << "\n";
  }

  csv_ << "t_sim";
  for (const auto & c : columns) {csv_ << ',' << c;}
  csv_ << '\n';

  n_cols_ = columns.size();
  dropped_ = 0;
  rows_ = 0;
  return true;
}

void DiagLogger::log(double t_sim, const std::vector<double> & values)
{
  if (!csv_.is_open()) {return;}
  if (values.size() != n_cols_) {
    ++dropped_;
    return;
  }
  // Formato por DEFECTO (no `fixed`) con 10 cifras significativas: en estas
  // columnas conviven el residuo de la CARE (1e-14) y cond(M) (1e4), y con
  // `std::fixed` el residuo se escribia como 0.000000000 — medido, y por poco
  // se cuela como "residuo perfecto" en el analisis de la FASE 4.
  csv_ << std::defaultfloat << std::setprecision(10) << t_sim;
  for (const double v : values) {csv_ << ',' << v;}
  csv_ << '\n';

  // Volcado periodico: el destructor cierra bien en una salida normal, pero una
  // corrida que acaba a golpe de kill (limpieza de huerfanos de Gazebo, timeout
  // de la campana) pierde lo que quede en el bufero. Cada segundo de lazo a
  // 500 Hz acota esa perdida a una fila y el coste es despreciable.
  if (++rows_ % kFlushEvery == 0) {csv_.flush();}
}

void DiagLogger::close()
{
  if (csv_.is_open()) {
    csv_.flush();
    csv_.close();
  }
  n_cols_ = 0;
}

}  // namespace ur5_dyn_control
