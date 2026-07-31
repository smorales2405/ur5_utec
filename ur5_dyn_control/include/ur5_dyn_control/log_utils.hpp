#ifndef UR5_DYN_CONTROL_LOG_UTILS_HPP
#define UR5_DYN_CONTROL_LOG_UTILS_HPP

#include <sys/stat.h>
#include <sys/types.h>

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

namespace ur5_dyn_control
{
namespace log_utils
{

/// mkdir -p (sin <filesystem>, que en Humble/GCC 11 arrastra -lstdc++fs en
/// algunas toolchains y aqui no aporta nada).
inline void makeDirs(const std::string & path)
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

inline std::string isoTimestamp()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t tt = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  ::localtime_r(&tt, &tm);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S%z");
  return oss.str();
}

/// Directorio de salida efectivo: vacio -> $HOME/.ros/ur5_dyn_control.
inline std::string resolveDir(const std::string & output_dir)
{
  if (!output_dir.empty()) {return output_dir;}
  const char * home = std::getenv("HOME");
  return std::string(home ? home : ".") + "/.ros/ur5_dyn_control";
}

}  // namespace log_utils
}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_LOG_UTILS_HPP
