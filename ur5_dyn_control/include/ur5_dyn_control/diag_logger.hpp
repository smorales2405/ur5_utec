#ifndef UR5_DYN_CONTROL_DIAG_LOGGER_HPP
#define UR5_DYN_CONTROL_DIAG_LOGGER_HPP

#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace ur5_dyn_control
{

/**
 * CSV de DIAGNOSTICO por paso, con columnas nombradas por el controlador
 * (FASE 4). Fichero aparte: `<prefix>_diag_<test_num>.csv`.
 *
 * POR QUE UN SEGUNDO FICHERO Y NO COLUMNAS NUEVAS EN EL CSV UNIFICADO.
 * El esquema de CsvLogger es deliberadamente el MISMO para los cuatro
 * controladores, para que el analisis de la FASE 10 no se bifurque. Pero cada
 * ley tiene magnitudes internas que solo existen en ella y que el plan exige
 * registrar por paso: en el LQR-SDRE, max Re(eig(A - B K)), cond(M) y el tiempo
 * de computo de la CARE; en el ASTSMC (FASE 6) seran las ganancias adaptativas.
 * Meterlas en el esquema comun lo llenaria de columnas a cero.
 *
 * Misma cabecera de trazabilidad `#` que CsvLogger (git_sha, params_hash,
 * timestamp) y la misma advertencia: `numpy.genfromtxt(names=True)` NO salta
 * esas lineas por si solo, hay que contarlas y pasarlas por `skip_header`.
 */
class DiagLogger
{
public:
  DiagLogger() = default;
  ~DiagLogger();

  /// output_dir vacio -> $HOME/.ros/ur5_dyn_control. `columns` NO incluye
  /// t_sim, que se escribe siempre como primera columna.
  bool open(const std::string & output_dir,
            const std::string & prefix,
            int test_num,
            const std::vector<std::string> & columns,
            const std::map<std::string, std::string> & metadata = {});

  /// `values` debe tener exactamente tantos elementos como columnas se
  /// declararon en open(); si no, la fila se descarta (y se cuenta).
  void log(double t_sim, const std::vector<double> & values);

  void close();
  bool isOpen() const { return csv_.is_open(); }
  const std::string & path() const { return path_; }
  /// Filas descartadas por longitud incorrecta (deberia ser siempre 0).
  std::size_t droppedRows() const { return dropped_; }

private:
  static constexpr std::size_t kFlushEvery = 500;   // 1 s de lazo a 500 Hz

  std::ofstream csv_;
  std::string path_;
  std::size_t n_cols_ = 0;
  std::size_t dropped_ = 0;
  std::size_t rows_ = 0;
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_DIAG_LOGGER_HPP
