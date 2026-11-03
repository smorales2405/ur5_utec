#!/usr/bin/env python3
"""
Driver UR falso para probar los runners por posición de punta a punta sin robot.

Lo que un runner ve del driver real (medido en el robot el 2026-11-02): el orden
de `/joint_states` distinto al de la tabla, la escala de velocidad, el JTC que al
primer punto con `time_from_start = 0` SALTA a él, `zero_ftsensor` y el wrench.

Solo tiene sentido en un ROS_DOMAIN_ID propio: el robot real vive en otro, y un
runner que hablase con él movería el brazo. Por eso se niega a arrancar sin
`--domain` igual al del entorno.

Cada evento (trayectoria recibida, cero, cambio de controlador) va a `--log` como
una línea JSON.
"""

import argparse
import json
import os
import sys
import time

import numpy as np
import rclpy
from controller_manager_msgs.srv import SwitchController
import pinocchio as pin
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from scipy.interpolate import CubicHermiteSpline
from trajectory_msgs.msg import JointTrajectory

from ur5_identification.force_estimator import K_CURRENT

#: Orden de la tabla.
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
#: Orden en que el broadcaster real publica: no es el de la tabla.
PUBLISH_ORDER = [2, 1, 5, 3, 4, 0]
PERIOD = 0.004              # [s]


def _seconds(duration) -> float:
    return duration.sec + duration.nanosec * 1e-9


class FakeUr(Node):
    def __init__(self, q0, scaling, log_path, ft_gain_kg=None, tcp_z=0.0, rotor=None):
        super().__init__("fake_ur_driver")
        # Con `ft_gain_kg` el wrench es de verdad el peso de esa masa en ejes de la
        # herramienta, con ruido como el medido (0.11 N); el signo va en la masa.
        self.ft_gain_kg = ft_gain_kg
        self.tcp_z = tcp_z
        # Con `rotor` el brazo sigue el spline del JTC y `effort` es la corriente que
        # haría falta: M q̈ + g + fricción + el rotor, que el modelo no conoce.
        self.rotor = None if rotor is None else np.asarray(rotor, float)
        self.ddq = np.zeros(6)
        self.dq_true = np.zeros(6)
        from ur5_trajectory_optimization.gain_tuning.closed_loop import (
            TCP_OFFSET, Plant, default_urdf)
        self.tip_offset = TCP_OFFSET
        self.plant = Plant(default_urdf())
        self.q = np.asarray(q0, float)
        self.q_prev = self.q.copy()
        self.scaling = scaling
        self.log_path = log_path
        self.traj = None
        self.pub_js = self.create_publisher(JointState, "/joint_states", 100)
        self.pub_ft = self.create_publisher(
            WrenchStamped, "/force_torque_sensor_broadcaster/wrench",
            qos_profile_sensor_data)
        self.pub_tcp = self.create_publisher(PoseStamped, "/tcp_pose_broadcaster/pose", 10)
        self.pub_scaling = self.create_publisher(
            Float64, "/speed_scaling_state_broadcaster/speed_scaling", 10)
        self.create_subscription(
            JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory",
            self._on_trajectory, 10)
        self.create_service(SwitchController, "/controller_manager/switch_controller",
                            self._on_switch)
        self.create_service(Trigger, "/io_and_status_controller/zero_ftsensor",
                            self._on_zero)
        self.create_timer(PERIOD, self._tick)

    def _event(self, **kw):
        with open(self.log_path, "a") as fh:
            fh.write(json.dumps(kw) + "\n")

    def _on_switch(self, req, res):
        self._event(kind="switch", activate=list(req.activate_controllers))
        res.ok = True
        return res

    def _on_zero(self, _req, res):
        self._event(kind="zero", t=time.time())
        res.success = True
        return res

    def _on_trajectory(self, msg):
        order = [msg.joint_names.index(n) for n in JOINTS]
        times = np.array([_seconds(p.time_from_start) for p in msg.points])
        qs = np.array([[p.positions[i] for i in order] for p in msg.points])
        # Como el JTC: de donde esté al primer punto, en el tiempo de ese punto.
        # Con t = 0 el "movimiento" es un salto de la consigna.
        jump = float(np.abs(self.q - qs[0]).max()) if times[0] == 0.0 else 0.0
        if times[0] > 0.0:
            times = np.r_[0.0, times]
            qs = np.vstack([self.q, qs])
        self._event(kind="trajectory", points=len(msg.points),
                    duration=float(times[-1]), jump=jump, target=qs[-1].tolist())
        if self.scaling > 0.0:
            self.traj = (time.time(), times, qs)
            if self.rotor is not None:
                vel = np.array([[p.velocities[i] for i in order] for p in msg.points])
                if len(vel) < len(qs):
                    vel = np.vstack([np.zeros(6), vel])
                self.spline = CubicHermiteSpline(times, qs, vel)

    def _current(self, dq):
        tau = pin.rnea(self.plant.model, self.plant.data, self.q, dq, self.ddq)
        tau = tau + self.rotor * self.ddq + 2.0 * np.sign(dq) + 5.0 * dq
        return tau / K_CURRENT + np.random.normal(0.0, 0.01, 6)

    def _tick(self):
        now = time.time()
        if self.traj is not None:
            t0, times, qs = self.traj
            t = now - t0
            if self.rotor is not None:
                self.q_prev = self.q
                tc = min(t, times[-1])
                self.q = self.spline(tc)
                self.dq_true = self.spline(tc, 1)
                self.ddq = self.spline(tc, 2) if t < times[-1] else np.zeros(6)
            else:
                self.q_prev, self.q = self.q, np.array(
                    [np.interp(t, times, qs[:, j]) for j in range(6)])
            if t > times[-1]:
                self.traj = None
                self.dq_true = np.zeros(6)
        else:
            self.q_prev = self.q
            self.dq_true = np.zeros(6)
            self.ddq = np.zeros(6)
        stamp = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = stamp
        js.name = [JOINTS[i] for i in PUBLISH_ORDER]
        js.position = [float(self.q[i]) for i in PUBLISH_ORDER]
        dq = self.dq_true if self.rotor is not None else (self.q - self.q_prev) / PERIOD
        js.velocity = [float(dq[i]) for i in PUBLISH_ORDER]
        if self.rotor is not None:
            cur = self._current(dq)
            js.effort = [float(cur[i]) for i in PUBLISH_ORDER]
        else:
            js.effort = [0.1] * 6
        self.pub_js.publish(js)
        w = WrenchStamped()
        w.header.stamp = stamp
        pin.framesForwardKinematics(self.plant.model, self.plant.data, self.q)
        tip = self.plant.data.oMf[self.plant.tcp]
        rz = np.diag([-1.0, -1.0, 1.0])     # base del controlador
        tcp = rz @ (tip.translation + tip.rotation @ np.array(
            [0.0, 0.0, self.tcp_z - self.tip_offset]))
        quat = pin.Quaternion(rz @ tip.rotation)
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "base"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            float(v) for v in tcp)
        pose.pose.orientation.x, pose.pose.orientation.y = float(quat.x), float(quat.y)
        pose.pose.orientation.z, pose.pose.orientation.w = float(quat.z), float(quat.w)
        self.pub_tcp.publish(pose)
        if self.ft_gain_kg is None:
            w.wrench.force.z = 5.0
        else:
            f = tip.rotation.T @ np.array([0.0, 0.0, -self.ft_gain_kg * 9.8])
            f = f + np.array([0.7, -0.4, 1.1]) + np.random.normal(0.0, 0.11, 3)
            w.wrench.force.x, w.wrench.force.y, w.wrench.force.z = (float(v) for v in f)
        self.pub_ft.publish(w)
        self.pub_scaling.publish(Float64(data=float(self.scaling)))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--domain", type=int, required=True,
                    help="tiene que coincidir con ROS_DOMAIN_ID del entorno")
    ap.add_argument("--q0", type=float, nargs=6, required=True)
    ap.add_argument("--scaling", type=float, default=1.0)
    ap.add_argument("--log", required=True)
    ap.add_argument("--ft-gain-kg", type=float, default=None)
    ap.add_argument("--tcp-z", type=float, default=0.0,
                    help="TCP del pendant en z de tool0 [m]")
    ap.add_argument("--rotor", type=float, nargs=6, default=None,
                    help="inercia que el modelo no conoce [kg.m2]: effort = corriente real")
    args = ap.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != str(args.domain):
        sys.exit("ROS_DOMAIN_ID del entorno distinto de --domain: no arranco")
    rclpy.init()
    node = FakeUr(args.q0, args.scaling, args.log, args.ft_gain_kg, args.tcp_z, args.rotor)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass


if __name__ == "__main__":
    main()
