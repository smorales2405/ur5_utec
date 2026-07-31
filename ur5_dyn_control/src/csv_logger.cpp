#include "ur5_dyn_control/csv_logger.hpp"

#include <sys/stat.h>
#include <sys/types.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace ur5_dyn_control
{

namespace
{

void makeDirs(const std::string & path)
{
  std::string acc;
  std::istringstream iss(path);
  std::string part;
  if (!path.empty() && path.front() == '/') {acc = "/";}
  while (std::getline(iss, part, '/')) {
    if (part.empty()) {continue;}
    acc += part + "/";
    ::mkdir(acc.c_str(), 0755);
  }
}

std::string isoTimestamp()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t tt = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  ::localtime_r(&tt, &tm);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S%z");
  return oss.str();
}

}  // namespace

CsvLogger::~CsvLogger()
{
  close();
}

std::string CsvLogger::hashParameters(const std::map<std::string, std::string> & params)
{
  // FNV-1a de 64 bits sobre "clave=valor\n" en orden (std::map ya ordena).
  // Sin dependencias externas y suficiente para distinguir configuraciones.
  std::uint64_t h = 1469598103934665603ULL;
  auto feed = [&h](const std::string & s) {
      for (const unsigned char c : s) {
        h ^= c;
        h *= 1099511628211ULL;
      }
    };
  for (const auto & [k, v] : params) {
    feed(k);
    feed("=");
    feed(v);
    feed("\n");
  }
  std::ostringstream oss;
  oss << std::hex << std::setw(16) << std::setfill('0') << h;
  return oss.str();
}

bool CsvLogger::open(const std::string & output_dir,
                     const std::string & prefix,
                     int test_num,
                     const std::map<std::string, std::string> & metadata)
{
  close();

  std::string dir = output_dir;
  if (dir.empty()) {
    const char * home = std::getenv("HOME");
    dir = std::string(home ? home : ".") + "/.ros/ur5_dyn_control";
  }
  makeDirs(dir);

  path_ = dir + "/" + prefix + "_" + std::to_string(test_num) + ".csv";
  csv_.open(path_);
  if (!csv_.is_open()) {
    return false;
  }

  // ── Cabecera de trazabilidad ─────────────────────────────────────────────
  // Lineas de comentario: numpy.genfromtxt y csv las saltan con comments='#'
  // (valor por defecto), asi que los lectores por nombre de columna siguen
  // funcionando sin cambios.
  csv_ << "# controller_id=" << prefix << "\n";
  csv_ << "# test_num=" << test_num << "\n";
  csv_ << "# timestamp=" << isoTimestamp() << "\n";
  for (const auto & [k, v] : metadata) {
    csv_ << "# " << k << "=" << v << "\n";
  }

  // ── Cabecera de columnas ─────────────────────────────────────────────────
  csv_ << "t_wall,t_sim";
  for (const char * g : {"q", "dq"}) {
    for (int i = 1; i <= 6; ++i) {csv_ << ',' << g << i;}
  }
  for (const char * g : {"q", "dq", "ddq"}) {
    for (int i = 1; i <= 6; ++i) {csv_ << ',' << g << i << "_des";}
  }
  for (int i = 1; i <= 6; ++i) {csv_ << ",tau" << i;}
  for (int i = 1; i <= 6; ++i) {csv_ << ",tau" << i << "_sat";}
  for (int i = 1; i <= 6; ++i) {csv_ << ",s" << i;}
  csv_ << ",x,y,z,x_des,y_des,z_des,theta_err";
  for (int i = 1; i <= 6; ++i) {csv_ << ",wrench" << i;}
  csv_ << ",state\n";
  return true;
}

void CsvLogger::log(const LogSample & d)
{
  if (!csv_.is_open()) {return;}
  // 9 decimales = 1 nm en las columnas cartesianas. Con 6 el paso de
  // cuantizacion (1 um) era el 5 % del avance por muestra a 10 mm/s y tapaba la
  // senal al derivar el CSV para medir el feed (medido en la FASE 1).
  csv_ << std::fixed << std::setprecision(9) << d.t_wall << ',' << d.t_sim;
  for (int i = 0; i < 6; ++i) {csv_ << ',' << d.q[i];}
  for (int i = 0; i < 6; ++i) {csv_ << ',' << d.dq[i];}
  for (int i = 0; i < 6; ++i) {csv_ << ',' << d.q_des[i];}
  for (int i = 0; i < 6; ++i) {csv_ << ',' << d.dq_des[i];}
  for (int i = 0; i < 6; ++i) {csv_ << ',' << d.ddq_des[i];}
  for (int i = 0; i < 6; ++i) {csv_ << ',' << d.tau_cmd[i];}
  for (int i = 0; i < 6; ++i) {csv_ << ',' << d.tau_sat_flag[i];}
  for (int i = 0; i < 6; ++i) {csv_ << ',' << d.s[i];}
  csv_ << ',' << d.xyz.x() << ',' << d.xyz.y() << ',' << d.xyz.z()
       << ',' << d.xyz_des.x() << ',' << d.xyz_des.y() << ',' << d.xyz_des.z()
       << ',' << d.theta_err;
  for (int i = 0; i < 6; ++i) {csv_ << ',' << d.wrench[i];}
  csv_ << ',' << d.state << '\n';
}

void CsvLogger::close()
{
  if (csv_.is_open()) {
    csv_.flush();
    csv_.close();
  }
}

}  // namespace ur5_dyn_control
