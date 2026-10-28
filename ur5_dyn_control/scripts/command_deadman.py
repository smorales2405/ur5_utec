#!/usr/bin/env python3
"""
Vigilante de comandos de par en el UR5e REAL (B7.0): si el nodo de control deja
de publicar, par cero y el mando vuelve al controlador de trayectoria.

El hueco que cierra
-------------------
`forward_effort_controller` conserva el ULTIMO comando recibido y el driver lo
reenvía al robot en cada ciclo, así que el `RobotReceiveTimeout` del robot
(20 ms, fijo en ur_robot_driver 2.14) nunca vence mientras el driver viva. Si el
nodo de control termina limpio, su destructor publica par cero; si MUERE (fallo,
`kill -9`, OOM), o se queda en DONE, el último par sigue aplicándose. Es lo que
hizo girar 4.7 rad a shoulder_pan entre dos corridas de la campaña de fricción.

Qué hace
--------
Se ARMA con el primer comando no nulo (en PRE_HOLD y WAIT_STATE el nodo real
publica ceros exactos: G3) y, armado, si pasan `timeout` segundos sin ningún
mensaje:

  1. publica par cero (varias veces: un solo mensaje puede perderse),
  2. pide al controller_manager desactivar `forward_effort_controller` y
     activar `scaled_joint_trajectory_controller` (BEST_EFFORT), que sostiene
     la pose actual en posición,
  3. se desarma hasta el siguiente comando no nulo (sus propios ceros no lo
     rearman).

Corre en su PROPIO proceso (lo lanza ur5e_real.launch.py como un nodo aparte):
tiene que sobrevivir a la muerte del nodo de control. No protege contra la
muerte de los dos a la vez (`kill -9` al launch entero): para eso queda el paro
de emergencia.

Plazo de 100 ms: el mayor hueco entre ciclos del nodo en 21 corridas reales
(fl_500-507, smc_710-790) fue 26.6 ms. Medido en Gazebo con `kill -9` al nodo:
salta a los 100 ms y el controller_manager hace el switch. En Gazebo NO se usa:
alli el nodo publica g(q) ya en PRE_HOLD (lo arma) y se bloquea >100 ms al
despausar el mundo, lo que da un disparo en falso al arrancar.
"""

import time

import rclpy
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class CommandDeadman(Node):

    def __init__(self):
        super().__init__("command_deadman")
        self.topic = self.declare_parameter(
            "command_topic", "/forward_effort_controller/commands").value
        self.timeout = float(self.declare_parameter("timeout", 0.10).value)
        cm = self.declare_parameter("controller_manager", "/controller_manager").value
        self.activate = list(self.declare_parameter(
            "activate_controllers", ["scaled_joint_trajectory_controller"]).value)
        self.deactivate = list(self.declare_parameter(
            "deactivate_controllers", ["forward_effort_controller"]).value)
        self.n_joints = int(self.declare_parameter("n_joints", 6).value)
        self.zero_repeats = int(self.declare_parameter("zero_repeats", 5).value)

        self.armed = False
        self.last = 0.0
        self.trips = 0
        self.pub = self.create_publisher(Float64MultiArray, self.topic, 10)
        self.create_subscription(Float64MultiArray, self.topic, self.on_cmd, 10)
        self.switch = self.create_client(SwitchController, f"{cm}/switch_controller")
        # 200 Hz: la deteccion llega como mucho 5 ms tarde sobre `timeout`.
        self.create_timer(0.005, self.check)
        self.get_logger().info(
            f"vigilando {self.topic}: si calla {1e3 * self.timeout:.0f} ms, par cero y "
            f"-{self.deactivate} +{self.activate}")

    def on_cmd(self, msg):
        now = time.monotonic()
        if self.armed:
            self.last = now
        elif any(v != 0.0 for v in msg.data):
            self.armed = True
            self.last = now
            self.get_logger().info("armado: el nodo de control esta comandando par")

    def check(self):
        if not self.armed:
            return
        quiet = time.monotonic() - self.last
        if quiet <= self.timeout:
            return
        self.armed = False
        self.trips += 1
        zero = Float64MultiArray(data=[0.0] * self.n_joints)
        for _ in range(self.zero_repeats):
            self.pub.publish(zero)
        self.get_logger().error(
            f"DEADMAN: {1e3 * quiet:.0f} ms sin comandos. Par cero publicado y "
            f"control devuelto a {self.activate}.")
        if not self.switch.service_is_ready():
            self.get_logger().error(
                "switch_controller no disponible: el brazo queda con par CERO "
                "(G3: flota compensado por el robot). Paro de emergencia a mano.")
            return
        req = SwitchController.Request()
        req.activate_controllers = self.activate
        req.deactivate_controllers = self.deactivate
        req.strictness = SwitchController.Request.BEST_EFFORT
        self.switch.call_async(req).add_done_callback(self.on_switch)

    def on_switch(self, fut):
        res = fut.result()
        if res is not None and res.ok:
            self.get_logger().warn("DEADMAN: controlador de trayectoria activo; "
                                   "el brazo se sostiene en posicion.")
        else:
            self.get_logger().error("DEADMAN: el switch fallo; el brazo queda con "
                                    "par CERO. Paro de emergencia a mano.")


def main():
    rclpy.init()
    node = CommandDeadman()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
