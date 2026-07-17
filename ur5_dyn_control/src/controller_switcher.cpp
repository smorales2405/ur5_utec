#include "ur5_dyn_control/controller_switcher.hpp"

#include <cstdio>
#include <memory>

namespace ur5_dyn_control
{

ControllerSwitcher::ControllerSwitcher(rclcpp::Node * node,
                                       const std::string & controller_manager_ns)
: node_(node)
{
  client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>(
    controller_manager_ns + "/switch_controller");
  list_client_ = node_->create_client<controller_manager_msgs::srv::ListControllers>(
    controller_manager_ns + "/list_controllers");
}

void ControllerSwitcher::checkControllersLoaded(
  const std::vector<std::string> & names,
  std::function<void(bool, std::vector<std::string>)> cb)
{
  if (!list_client_->service_is_ready()) {
    cb(false, {});
    return;
  }
  auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  list_client_->async_send_request(
    req,
    [cb, names](
      rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future)
    {
      const auto res = future.get();
      if (!res) {
        cb(false, {});
        return;
      }
      std::vector<std::string> to_activate;
      bool all_loaded = true;
      for (const auto & name : names) {
        bool found = false;
        for (const auto & c : res->controller) {
          if (c.name == name) {
            found = true;
            if (c.state != "active") {
              to_activate.push_back(name);
            }
            break;
          }
        }
        if (!found) {
          all_loaded = false;
          break;
        }
      }
      cb(all_loaded, std::move(to_activate));
    });
}

bool ControllerSwitcher::waitForService(std::chrono::seconds timeout)
{
  return client_->wait_for_service(timeout);
}

void ControllerSwitcher::requestSwitch(const std::vector<std::string> & activate,
                                       const std::vector<std::string> & deactivate,
                                       std::function<void(bool)> cb,
                                       double timeout_s,
                                       bool strict,
                                       bool activate_asap)
{
  auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  req->activate_controllers = activate;
  req->deactivate_controllers = deactivate;
  req->strictness = strict
    ? controller_manager_msgs::srv::SwitchController::Request::STRICT
    : controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  req->activate_asap = activate_asap;
  req->timeout = rclcpp::Duration::from_seconds(timeout_s);

  client_->async_send_request(
    req,
    [cb, logger = node_->get_logger()](
      rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future)
    {
      const bool ok = future.get() && future.get()->ok;
      if (!ok) {
        RCLCPP_ERROR(logger, "switch_controller devolvio ok=false");
      }
      cb(ok);
    });
}

bool setIgnWorldPaused(const std::string & world_name, bool paused,
                       const rclcpp::Logger & logger)
{
  const std::string cmd =
    "ign service -s /world/" + world_name + "/control "
    "--reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean "
    "--timeout 3000 --req 'pause: " + (paused ? "true" : "false") + "' 2>/dev/null";

  FILE * pipe = popen(cmd.c_str(), "r");
  if (!pipe) {
    RCLCPP_ERROR(logger, "No se pudo ejecutar 'ign service' para (des)pausar el mundo");
    return false;
  }
  char buffer[256];
  std::string output;
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    output += buffer;
  }
  const int rc = pclose(pipe);
  const bool ok = (rc == 0) && (output.find("true") != std::string::npos);
  if (ok) {
    RCLCPP_INFO(logger, "Mundo '%s' %s.", world_name.c_str(),
                paused ? "pausado" : "despausado");
  } else {
    RCLCPP_ERROR(logger, "Fallo al %s el mundo '%s' (rc=%d, out='%s')",
                 paused ? "pausar" : "despausar", world_name.c_str(), rc, output.c_str());
  }
  return ok;
}

}  // namespace ur5_dyn_control
