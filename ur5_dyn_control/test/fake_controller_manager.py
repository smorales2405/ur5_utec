#!/usr/bin/env python3
"""
controller_manager y driver falsos, para test_controller_switch_retry.py y test_torque_gates.py.

En el robot (2026-11-02, 2 de 7 arranques) el driver registró `failed to send
response to /controller_manager/list_controllers (timeout)`: `wait_for_service` solo
mira el grafo, y el servidor puede responder antes de emparejar al cliente. El nodo
esperaba esa respuesta para siempre.

Solo en un ROS_DOMAIN_ID propio: se niega a arrancar sin `--domain` igual al del
entorno, porque un cliente real lo tomaría por el del robot.

  --drop-list N       las N primeras listas se atienden y su respuesta se pierde
  --drop-switch N     los N primeros switch se APLICAN y su respuesta se pierde
  --lose-switch N     los N primeros switch se pierden enteros: no se aplican
  --friction MODO     servicio de G4: none (no existe), ok o reject
  --drop-friction N   las N primeras respuestas de G4 se pierden
  --dq V              velocidad que publica /joint_states en las seis juntas
  --silent 1          no publica /joint_states
  --stop-state-after S  deja de publicar /joint_states S segundos tras activar el par
"""

import argparse
import json
import os
import sys
import time

import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from ur_msgs.srv import SetFrictionModelParameters

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
Q_INIT = [1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 3.1416]
FRICTION_SRV = "/friction_model_controller/set_friction_model_parameters"


class FakeCm(Node):
    def __init__(self, args):
        super().__init__("fake_controller_manager")
        self.args = args
        self.state = {"forward_effort_controller": "inactive",
                      "scaled_joint_trajectory_controller": "active"}
        self.counts = {"list": 0, "switch": 0, "friction": 0, "tau": 0}
        self.t_effort = None
        self.srv_list = self.create_service(
            ListControllers, "/controller_manager/list_controllers", self._on_list)
        self.srv_switch = self.create_service(
            SwitchController, "/controller_manager/switch_controller", self._on_switch)
        self.real_send = {"list": self.srv_list.send_response,
                          "switch": self.srv_switch.send_response}
        # Perder la respuesta: el executor llama a send_response tras el callback.
        self.srv_list.send_response = lambda res, hdr: self._send("list", res, hdr)
        self.srv_switch.send_response = lambda res, hdr: self._send("switch", res, hdr)
        if args.friction != "none":
            self.srv_friction = self.create_service(
                SetFrictionModelParameters, FRICTION_SRV, self._on_friction)
            self.real_send["friction"] = self.srv_friction.send_response
            self.srv_friction.send_response = lambda res, hdr: self._send("friction", res, hdr)
        self.create_subscription(Float64MultiArray, "/forward_effort_controller/commands",
                                 self._on_tau, 10)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(0.004, self._tick)

    def _event(self, **kw):
        with open(self.args.log, "a") as fh:
            fh.write(json.dumps(kw) + "\n")

    def _send(self, kind, res, hdr):
        drop = {"list": self.args.drop_list,
                "switch": self.args.drop_switch + self.args.lose_switch,
                "friction": self.args.drop_friction}[kind]
        if self.counts[kind] <= drop:
            self._event(kind=f"{kind}_lost", n=self.counts[kind], response="perdida")
            return
        self.real_send[kind](res, hdr)

    def _on_list(self, _req, res):
        self.counts["list"] += 1
        for name, state in self.state.items():
            c = ControllerState()
            c.name, c.state = name, state
            res.controller.append(c)
        return res

    def _on_switch(self, req, res):
        self.counts["switch"] += 1
        lost = self.counts["switch"] <= self.args.lose_switch
        if not lost:
            for name in req.deactivate_controllers:
                self.state[name] = "inactive"
            for name in req.activate_controllers:
                self.state[name] = "active"
            if self.state["forward_effort_controller"] == "active" and self.t_effort is None:
                self.t_effort = time.time()
        self._event(kind="switch", n=self.counts["switch"], applied=not lost,
                    state=dict(self.state))
        res.ok = True
        return res

    def _on_friction(self, req, res):
        self.counts["friction"] += 1
        self._event(kind="friction", n=self.counts["friction"],
                    viscous=list(req.parameters.viscous_scale),
                    coulomb=list(req.parameters.coulomb_scale))
        res.success = self.args.friction == "ok"
        return res

    def _on_tau(self, msg):
        self.counts["tau"] += 1
        if any(abs(v) > 0.0 for v in msg.data) and "tau_nonzero" not in self.counts:
            self.counts["tau_nonzero"] = 1
            self._event(kind="tau_nonzero", tau=list(msg.data))
        if self.counts["tau"] % 25 == 0:
            self._event(kind="tau", tau=list(msg.data))

    def _tick(self):
        if self.args.silent:
            return
        if (self.args.stop_state_after > 0.0 and self.t_effort is not None
                and time.time() - self.t_effort > self.args.stop_state_after):
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = Q_INIT
        msg.velocity = [self.args.dq] * 6
        msg.effort = [0.0] * 6
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--domain", type=int, required=True)
    ap.add_argument("--log", required=True)
    ap.add_argument("--drop-list", type=int, default=0)
    ap.add_argument("--drop-switch", type=int, default=0)
    ap.add_argument("--lose-switch", type=int, default=0)
    ap.add_argument("--friction", choices=["none", "ok", "reject"], default="none")
    ap.add_argument("--drop-friction", type=int, default=0)
    ap.add_argument("--dq", type=float, default=0.0)
    ap.add_argument("--silent", type=int, default=0)
    ap.add_argument("--stop-state-after", type=float, default=0.0)
    args = ap.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != str(args.domain):
        sys.exit("ROS_DOMAIN_ID del entorno distinto de --domain: no arranco")
    rclpy.init()
    node = FakeCm(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, rclpy._rclpy_pybind11.RCLError):
        pass


if __name__ == "__main__":
    main()
