#ifndef UR5_DYN_CONTROL_CSV_LOGGER_HPP
#define UR5_DYN_CONTROL_CSV_LOGGER_HPP

#include <array>
#include <fstream>
#include <map>
#include <string>

#include "ur5_dyn_control/common.hpp"

namespace ur5_dyn_control
{

/**
 * Una fila del CSV unificado (FASE 3).
 *
 * El esquema es el MISMO para todos los controladores (FL, LQR-SDRE, SMC,
 * ASTSMC), de modo que los scripts de analisis de la FASE 10 no tengan que
 * bifurcarse por controlador. Los campos que no aplican a una ley concreta se
 * quedan a cero — p.ej. `s` (variable de deslizamiento) solo lo llena el SMC, y
 * `wrench` solo existe en el robot real.
 */
struct LogSample
{
  double t_wall = 0.0;          ///< reloj de PARED [s], para cruzar con ft_data
  double t_sim = 0.0;           ///< reloj de SIMULACION (o de pared en el real)

  Vector6d q = Vector6d::Zero();
  Vector6d dq = Vector6d::Zero();
  Vector6d q_des = Vector6d::Zero();
  Vector6d dq_des = Vector6d::Zero();
  Vector6d ddq_des = Vector6d::Zero();

  /// Par COMANDADO: post-friccion, post-politica de gravedad (G3),
  /// post-saturacion y post-limite de tasa. Es lo que llega al hardware y la
  /// unica referencia de par disponible en el robot real (G5).
  Vector6d tau_cmd = Vector6d::Zero();
  /// 1 si esa junta quedo recortada por saturacion o por limite de tasa.
  std::array<int, 6> tau_sat_flag{};

  /**
   * Par FISICO que entrega la junta:  tau_phys = tau_cmd + (G3 ? 0 : g(q)).
   *
   * NO es redundante con `tau_cmd`, y confundirlos arruina la identificacion de
   * friccion en el robot real. En Gazebo (`gravity_in_command=true`) coinciden.
   * En el UR5e real (`=false`) se comanda `tau_ley - g(q)` porque
   * `direct_torque()` compensa la gravedad DENTRO del robot: el par que la
   * junta entrega de verdad es el comandado MAS esa gravedad interna.
   *
   * El residuo de friccion se calcula contra `rnea(q,dq,ddq)`, que SI incluye
   * gravedad. Restarle `tau_cmd` en el robot real dejaria un sesgo de
   * exactamente `g(q)` —varios N·m en hombro y codo— que el ajuste atribuiria a
   * friccion de Coulomb, con buen R² y coeficientes inventados. Por eso el
   * identificador usa esta columna, no `tau`.
   *
   * Se registra POST-saturacion (parte de `tau_cmd`), que es lo correcto: si el
   * actuador quedo recortado, el par entregado es el recortado, no el que la ley
   * pidio.
   */
  Vector6d tau_phys = Vector6d::Zero();

  /// Variable de deslizamiento del controlador (SMC/ASTSMC); 0 si no aplica.
  Vector6d s = Vector6d::Zero();

  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
  Eigen::Vector3d xyz_des = Eigen::Vector3d::Zero();
  /// Angulo del error de orientacion, ||log(R_des^T R)||. NUNCA angulos de
  /// Euler: evitan el wrap y son la metrica que usa la FASE 10.
  double theta_err = 0.0;

  /// Wrench del TCP medido (ft_data del robot real); 0 en simulacion.
  Vector6d wrench = Vector6d::Zero();

  std::string state;
};

/**
 * Logger CSV con cabecera de TRAZABILIDAD (FASE 3).
 *
 * El fichero empieza con lineas de comentario `#` que fijan de que corrida
 * salieron los datos:
 *
 *   # git_sha=...        commit del workspace con el que se compilo
 *   # params_hash=...    hash del conjunto EFECTIVO de parametros
 *   # timestamp=...      hora local de inicio (ISO 8601)
 *   # controller_id=...  prefijo del controlador (fl, smc, astsmc, ...)
 *
 * Se hashea el conjunto EFECTIVO de parametros, no el fichero YAML: asi el
 * hash tambien recoge los overrides que llegan por linea de comandos
 * (sweep.joint, friction.f_v, ...). Hashear solo el YAML daria el mismo valor
 * para dos corridas que en realidad fueron distintas — justo lo que la
 * trazabilidad debe impedir.
 *
 * OJO: `numpy.genfromtxt(names=True)` NO salta estas lineas por si solo — toma
 * la PRIMERA linea del fichero como cabecera de columnas, comentario incluido.
 * El lector debe contar las lineas `#` iniciales y pasarlas por `skip_header`
 * (asi lo hace ur5_identification/residual.py::load_csv). `csv.DictReader`
 * tiene el mismo problema. Se documenta aqui porque es justo el tipo de detalle
 * que rompe un analisis meses despues.
 */
class CsvLogger
{
public:
  CsvLogger() = default;
  ~CsvLogger();

  /// output_dir vacio -> $HOME/.ros/ur5_dyn_control
  bool open(const std::string & output_dir,
            const std::string & prefix,
            int test_num,
            const std::map<std::string, std::string> & metadata = {});

  void log(const LogSample & sample);

  void close();
  bool isOpen() const { return csv_.is_open(); }
  const std::string & path() const { return path_; }

  /// Hash FNV-1a de una lista de "clave=valor" ya ordenada. Publico para que el
  /// nodo pueda calcularlo sobre sus parametros efectivos.
  static std::string hashParameters(const std::map<std::string, std::string> & params);

private:
  std::ofstream csv_;
  std::string path_;
};

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_CSV_LOGGER_HPP
