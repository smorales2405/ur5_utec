#!/usr/bin/env python3
"""
controller_manager falso que pierde respuestas, para test_controller_switch_retry.py.

En el robot (2026-11-02, 2 de 7 arranques) el driver registró `failed to send
response to /controller_manager/list_controllers (timeout)`: `wait_for_service` solo
mira el grafo, y el servidor puede responder antes de emparejar al cliente. El nodo
esperaba esa respuesta para siempre.

Solo en un ROS_DOMAIN_ID propio: se niega a arrancar sin `--domain` igual al del
entorno, porque un cliente real lo tomaría por el del robot.

  --drop-list N       las N primeras listas se atienden y su respuesta se pierde
  --drop-switch N     los N primeros switch se APLICAN y su respuesta se pierde
  --lose-switch N     los N primeros switch se pierden enteros: no se aplican
"""

import argparse
import json
import os
import sys

import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
Q_INIT = [1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 3.1416]


class FakeCm(Node):
    def __init__(self, args):
        super().__init__("fake_controller_manager")
        self.args = args
        self.state = {"forward_effort_controller": "inactive",
                      "scaled_joint_trajectory_controller": "active"}
        self.counts = {"list": 0, "switch": 0}
        self.srv_list = self.create_service(
            ListControllers, "/controller_manager/list_controllers", self._on_list)
        self.srv_switch = self.create_service(
            SwitchController, "/controller_manager/switch_controller", self._on_switch)
        self.real_send = {"list": self.srv_list.send_response,
                          "switch": self.srv_switch.send_response}
        # Perder la respuesta: el executor llama a send_response tras el callback.
        self.srv_list.send_response = lambda res, hdr: self._send("list", res, hdr)
        self.srv_switch.send_response = lambda res, hdr: self._send("switch", res, hdr)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(0.004, self._tick)

    def _event(self, **kw):
        with open(self.args.log, "a") as fh:
            fh.write(json.dumps(kw) + "\n")

    def _send(self, kind, res, hdr):
        drop = self.args.drop_list if kind == "list" else (
            self.args.drop_switch + self.args.lose_switch)
        if self.counts[kind] <= drop:
            self._event(kind=kind, n=self.counts[kind], response="perdida")
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
        self._event(kind="switch", n=self.counts["switch"], applied=not lost,
                    state=dict(self.state))
        res.ok = True
        return res

    def _tick(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = Q_INIT
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--domain", type=int, required=True)
    ap.add_argument("--log", required=True)
    ap.add_argument("--drop-list", type=int, default=0)
    ap.add_argument("--drop-switch", type=int, default=0)
    ap.add_argument("--lose-switch", type=int, default=0)
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
