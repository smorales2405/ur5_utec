#!/usr/bin/env python3
"""
Orquestador de arranque para el bringup effort del UR5e en Gazebo Fortress.

Problema que resuelve: con la simulacion pausada, gz_ros2_control solo corre
el update() del controller_manager en cada paso de fisica, asi que los
spawners estandar expiran al intentar ACTIVAR controladores (cargar y
configurar si funciona en pausa). Y con la fisica corriendo desde t=0, el
brazo se vence por gravedad antes de que el JTC se active.

Secuencia (valida con el mundo pausado O corriendo):
  1. Espera el servicio /controller_manager/switch_controller.
  2. Envia la peticion de activacion de [joint_state_broadcaster,
     joint_trajectory_controller] de forma ASINCRONA con timeout largo
     (el controller_manager la retiene hasta el proximo update()).
  3. Despausa el mundo via 'ign service /world/<world>/control'.
  4. La activacion se consuma en el primer paso de fisica: el JTC toma como
     hold la pose actual = initial_value = pose inicial -> el robot nunca cae.
"""

import argparse
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController

STRICT = SwitchController.Request.STRICT


class ActivateAndUnpause(Node):
    def __init__(self, args):
        super().__init__("activate_and_unpause")
        self.args = args
        self.cli = self.create_client(
            SwitchController, args.controller_manager + "/switch_controller"
        )

    def run(self) -> int:
        if not self.cli.wait_for_service(timeout_sec=self.args.service_timeout):
            self.get_logger().error(
                f"Servicio {self.args.controller_manager}/switch_controller no disponible"
            )
            return 1

        req = SwitchController.Request()
        req.activate_controllers = list(self.args.activate)
        req.deactivate_controllers = []
        req.strictness = STRICT
        req.activate_asap = True
        req.timeout = Duration(sec=self.args.switch_timeout)

        self.get_logger().info(f"Solicitando activacion de {req.activate_controllers} ...")
        future = self.cli.call_async(req)

        # Da tiempo a que la peticion llegue al controller_manager antes de
        # despausar (minimiza los pasos de fisica sin controlador activo).
        time.sleep(0.3)

        self.unpause_world()

        t0 = time.time()
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() - t0 > self.args.switch_timeout + 5.0:
                self.get_logger().error("Timeout esperando la respuesta del switch")
                return 1

        if future.result() is not None and future.result().ok:
            self.get_logger().info("Controladores activados y mundo corriendo.")
            return 0
        self.get_logger().error(f"switch_controller fallo: {future.result()}")
        return 1

    def unpause_world(self):
        cmd = [
            "ign", "service",
            "-s", f"/world/{self.args.world}/control",
            "--reqtype", "ignition.msgs.WorldControl",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "3000",
            "--req", "pause: false",
        ]
        for attempt in range(self.args.unpause_retries):
            try:
                out = subprocess.run(
                    cmd, capture_output=True, text=True, timeout=10.0
                )
                if out.returncode == 0 and "true" in out.stdout:
                    self.get_logger().info(f"Mundo '{self.args.world}' despausado.")
                    return
                self.get_logger().warn(
                    f"Intento {attempt + 1}: unpause respondio "
                    f"rc={out.returncode} stdout={out.stdout.strip()!r}"
                )
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f"Intento {attempt + 1}: unpause sin respuesta")
            time.sleep(1.0)
        self.get_logger().error(
            "No se pudo despausar el mundo (¿nombre de mundo correcto?). "
            "La activacion quedara pendiente hasta que se despause manualmente."
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller-manager", default="/controller_manager")
    parser.add_argument("--world", default="default")
    parser.add_argument(
        "--activate", nargs="+",
        default=["joint_state_broadcaster", "joint_trajectory_controller"],
    )
    parser.add_argument("--service-timeout", type=float, default=60.0)
    parser.add_argument("--switch-timeout", type=int, default=60)
    parser.add_argument("--unpause-retries", type=int, default=5)
    args, ros_argv = parser.parse_known_args()

    rclpy.init(args=ros_argv)
    node = ActivateAndUnpause(args)
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
