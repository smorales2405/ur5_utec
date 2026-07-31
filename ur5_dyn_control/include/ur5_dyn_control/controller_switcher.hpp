#ifndef UR5_DYN_CONTROL_CONTROLLER_SWITCHER_HPP
#define UR5_DYN_CONTROL_CONTROLLER_SWITCHER_HPP

#include <functional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

namespace ur5_dyn_control
{

/**
 * Cliente asincrono de /controller_manager/switch_controller.
 *
 * Uso tipico en los nodos de control dinamico:
 *  - arranque: activar [forward_effort_controller] (y jsb si aplica),
 *    desactivar el controlador de posicion si esta activo;
 *  - apagado: switch inverso opcional.
 *
 * La peticion se envia con timeout largo: con gz_ros2_control el switch se
 * consuma en el update() del proximo paso de simulacion, que con el mundo
 * pausado ocurre recien al despausar.
 */
class ControllerSwitcher
{
public:
  ControllerSwitcher(rclcpp::Node * node,
                     const std::string & controller_manager_ns = "/controller_manager");

  bool waitForService(std::chrono::seconds timeout);

  /**
   * Consulta list_controllers (async) y llama cb(ready, to_activate):
   *  - ready: todos los 'names' estan cargados (inactive o active);
   *  - to_activate: subconjunto de 'names' que NO esta activo aun
   *    (los ya activos se filtran para no romper el switch STRICT).
   */
  void checkControllersLoaded(
    const std::vector<std::string> & names,
    std::function<void(bool, std::vector<std::string>)> cb);

  /**
   * Igual, pero filtrando TAMBIEN la lista de desactivacion:
   *  - to_deactivate: subconjunto de 'deactivate_names' que esta ACTIVO.
   *
   * Pedir que se desactive un controlador ya inactivo hace fallar un switch
   * STRICT completo. En el UR5e real ocurre en cuanto el driver arranca con
   * `activate_joint_controller:=false`: el scaled_joint_trajectory_controller
   * queda cargado pero inactivo, y sin este filtro el nodo abortaria antes de
   * comandar el primer par.
   */
  void checkControllersLoaded(
    const std::vector<std::string> & activate_names,
    const std::vector<std::string> & deactivate_names,
    std::function<void(bool, std::vector<std::string>,
                       std::vector<std::string>)> cb);

  /// Envia la peticion (no bloquea); cb(ok) se invoca al llegar la respuesta.
  void requestSwitch(const std::vector<std::string> & activate,
                     const std::vector<std::string> & deactivate,
                     std::function<void(bool)> cb,
                     double timeout_s = 60.0,
                     bool strict = true,
                     bool activate_asap = true);

private:
  rclcpp::Node * node_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_client_;
};

/**
 * Despausa (o pausa) el mundo de Gazebo Fortress via 'ign service'.
 * Devuelve true si el servicio respondio true.
 */
bool setIgnWorldPaused(const std::string & world_name, bool paused,
                       const rclcpp::Logger & logger);

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_CONTROLLER_SWITCHER_HPP
