#!/usr/bin/env python3
"""
Calibración corriente → par MULTIPOSTURA (FASE 2).

Resuelve la degeneración que tiene la calibración a partir de un solo barrido.

El problema
-----------
`calibrate_current.py` saca `k` de la suma entre sentidos, que en la meseta vale
`g(q) + C·v`. Pero dentro de un barrido esa suma apenas varía: en `wrist_1`
recorrió de −0.2076 a −0.1843 A, un 11 % de rango. Ajustar con un término
independiente —sesgo de corriente, o error de gravedad del modelo, que son
matemáticamente indistinguibles— es entonces una extrapolación larguísima:

    sin sesgo   ->  k = 8.985,  residuo 1.726 %
    con sesgo   ->  k = 4.733,  residuo 0.426 %   (i0 = 0.173 A)

El segundo ajusta cuatro veces mejor y elimina la deriva de `k` con la velocidad
(correlación 0.86 en el primero), pero **los datos no pueden distinguirlos**: dan
`k` que difieren un 90 %, y con ello toda la fricción en N·m.

La solución
-----------
Recorrer un rango AMPLIO de par gravitatorio, moviendo la junta a varias
posturas y midiendo en cada una. En `wrist_1`, `g` va de −1.82 a 0 N·m entre
−135° y 0°, así que la ordenada en el origen deja de ser extrapolación y `k` e
`i0` se separan.

En cada postura se hace un barrido CORTO —amplitud pequeña y una sola velocidad,
en los dos sentidos— porque solo interesa la SUMA, que es la parte gravitatoria.
La amplitud se mantiene pequeña para que cada postura dé un par medio distinto:
con ±45° los barridos se solaparían y volveríamos al mismo problema.

Qué NO arregla
--------------
`wrist_3` no tiene par gravitatorio en NINGUNA postura —su eje es el de la
herramienta y la brida es simétrica respecto a él—, así que su `k` no es
identificable ni así. Hay que prestársela de otra junta del mismo tamaño.

Uso
---
    ros2 run ur5_identification calibrate_multipose.py \\
        --joint 3 --angles -135 -90 -45 0 --friction-level 0.0
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile
import time

import numpy as np
import pinocchio as pin
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ur5_identification.campaign_levels import JOINT_NAMES, LEVELS

JTC_TOPIC = "/scaled_joint_trajectory_controller/joint_trajectory"


class Mover(Node):
    """Lleva el robot a cada postura con el JTC. Control de POSICIÓN."""

    def __init__(self):
        super().__init__("multipose_mover")
        self._q = None
        self.create_subscription(JointState, "/joint_states", self._cb, 10)
        self._pub = self.create_publisher(JointTrajectory, JTC_TOPIC, 1)

    def _cb(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if all(n in idx for n in JOINT_NAMES):
            self._q = [msg.position[idx[n]] for n in JOINT_NAMES]

    def joint_state(self, timeout=3.0):
        self._q = None
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._q is not None:
                return list(self._q)
        return None

    def move_to(self, q_target, speed=0.25, settle=2.0):
        q = self.joint_state()
        if q is None:
            return False, "sin /joint_states"
        err = max(abs(a - b) for a, b in zip(q, q_target))
        if err < 1e-3:
            return True, "ya en posición"
        dur = max(3.0, err / max(speed, 1e-6))
        msg = JointTrajectory()
        msg.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q_target]
        pt.velocities = [0.0] * 6
        pt.time_from_start = Duration(sec=int(dur), nanosec=int((dur % 1) * 1e9))
        msg.points = [pt]
        self._pub.publish(msg)
        t0 = time.time()
        while time.time() - t0 < dur + settle:
            rclpy.spin_once(self, timeout_sec=0.1)
        q = self.joint_state()
        err = max(abs(a - b) for a, b in zip(q, q_target)) if q else 9e9
        return err < 0.05, f"error final {err:.4f} rad"


def temp_params(base_params, joint, angle_rad, amplitude_rad, velocity):
    """Params del barrido CORTO para una postura concreta."""
    cfg = yaml.safe_load(open(base_params))
    root = list(cfg)[0]
    rp = cfg[root]["ros__parameters"]
    q = [float(v) for v in rp["q_init"]]
    q[joint] = float(angle_rad)
    # COPIAS separadas, no el mismo objeto: `yaml.safe_dump` emite un
    # anchor/alias (`&id001` / `*id001`) cuando la misma lista aparece dos
    # veces, y el parser de ROS 2 los rechaza con "Will not support aliasing".
    # También se fuerza `float`: safe_dump no sabe representar np.float64.
    rp["q_init"] = list(q)
    rp["sweep"]["q_fixed"] = list(q)
    rp["sweep"]["joint"] = joint
    rp["sweep"]["amplitude"] = float(amplitude_rad)
    rp["sweep"]["velocities"] = [float(velocity)]
    fd, path = tempfile.mkstemp(suffix=".yaml", prefix=f"calib_j{joint}_")
    with os.fdopen(fd, "w") as fh:
        yaml.safe_dump(cfg, fh, default_flow_style=False, sort_keys=False)
    return path, q


def sweep_sums(csv_path, joint, model, data):
    """(suma de corriente, par del modelo) de la pareja (+v, −v) del barrido."""
    n_meta = 0
    with open(csv_path) as fh:
        for line in fh:
            if line.startswith("#"):
                n_meta += 1
            else:
                break
    d = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None,
                      encoding="utf-8", skip_header=n_meta, invalid_raise=False)
    st = np.asarray(d["state"], dtype=str)
    col = lambda p: np.column_stack([d[p % i] for i in range(1, 7)])  # noqa: E731
    q, dq, cur = col("q%d"), col("dq%d"), col("cur%d")
    levels = sorted({s.rsplit("_", 1)[0] for s in st
                     if s.startswith("SWEEP_") and s.rsplit("_", 1)[-1] in ("POS", "NEG")})
    for lv in levels:
        m_p, m_n = st == f"{lv}_POS", st == f"{lv}_NEG"
        if m_p.sum() < 20 or m_n.sum() < 20:
            continue
        si = 0.5 * (cur[m_p, joint].mean() + cur[m_n, joint].mean())
        tm = []
        for m in (m_p, m_n):
            tm.append(np.mean([pin.rnea(model, data, q[k], dq[k], np.zeros(6))[joint]
                               for k in np.flatnonzero(m)[::10]]))
        return si, 0.5 * (tm[0] + tm[1]), 0.5 * (cur[m_p, joint].mean()
                                                 - cur[m_n, joint].mean())
    return None, None, None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--joint", type=int, required=True, choices=range(6))
    ap.add_argument("--angles", type=float, nargs="+", required=True,
                    metavar="DEG", help="posturas de la junta, en grados")
    ap.add_argument("--amplitude", type=float, default=10.0,
                    help="amplitud del barrido corto [deg]; pequeña a propósito, "
                         "para que cada postura dé un par medio distinto")
    ap.add_argument("--velocity", type=float, default=0.1)
    ap.add_argument("--friction-level", choices=list(LEVELS), default=None)
    ap.add_argument("--params-file", default=None)
    ap.add_argument("--test-base", type=int, default=800)
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--gravity", type=float, default=9.8)
    ap.add_argument("--out-dir", default=os.path.expanduser("~/.ros/ur5_dyn_control"))
    args = ap.parse_args()

    from ament_index_python.packages import get_package_share_directory
    if args.params_file is None:
        args.params_file = os.path.join(
            get_package_share_directory("ur5_dyn_control"), "config",
            "sweep_params.yaml")
    if args.urdf is None:
        args.urdf = os.path.join(
            get_package_share_directory("ur5_kinematics"), "ur5e.urdf")

    model = pin.buildModelFromUrdf(args.urdf)
    model.gravity.linear = np.array([0.0, 0.0, -args.gravity])
    data = model.createData()
    j = args.joint

    # ¿Varía la gravedad lo suficiente entre las posturas pedidas? Si no, el
    # ajuste vuelve a ser el mismo mal condicionado que se quiere evitar.
    base = yaml.safe_load(open(args.params_file))
    rp = base[list(base)[0]]["ros__parameters"]
    gs = []
    for a in args.angles:
        q = np.array(rp["q_init"], dtype=float)
        q[j] = np.radians(a)
        gs.append(pin.computeGeneralizedGravity(model, data, q)[j])
    print(f"Calibración multipostura — junta {j} ({JOINT_NAMES[j]})\n")
    print(f"{'ángulo [deg]':>14}{'g(q) [N·m]':>14}")
    print("-" * 28)
    for a, g in zip(args.angles, gs):
        print(f"{a:14.1f}{g:14.4f}")
    rango = float(np.ptp(gs))
    print(f"\n  rango de par gravitatorio: {rango:.4f} N·m")
    if rango < 0.2:
        print("  ABORTA: el par apenas varía entre estas posturas, así que `k` "
              "seguiría sin separarse del sesgo. Elija ángulos donde g cambie, "
              "o esta junta no es calibrable por gravedad.")
        return 1

    rclpy.init()
    mover = Mover()
    pts = []
    try:
        for i, ang in enumerate(args.angles):
            print(f"\n── postura {i + 1}/{len(args.angles)}: {ang:.1f}° "
                  f"(g = {gs[i]:+.4f} N·m) ──")
            pf, q_target = temp_params(args.params_file, j, np.radians(ang),
                                       np.radians(args.amplitude), args.velocity)
            ok, msg = mover.move_to(q_target)
            print(f"  ir a la postura: {'OK' if ok else 'FALLO'} — {msg}")
            if not ok:
                print("  Se salta esta postura.")
                continue

            tn = args.test_base + i
            cmd = ["ros2", "run", "ur5_identification", "run_current_sweep.py",
                   "--joint", str(j), "--test-num", str(tn),
                   "--params-file", pf, "--out-dir", args.out_dir, "--yes"]
            if args.friction_level:
                cmd += ["--friction-level", args.friction_level]
            rc = subprocess.run(cmd, timeout=600).returncode
            csv = os.path.join(args.out_dir, f"cur_{tn}.csv")
            if rc != 0 or not os.path.exists(csv):
                print(f"  barrido FALLÓ (rc={rc})")
                continue
            si, tm, fr = sweep_sums(csv, j, model, data)
            if si is None:
                print("  sin pareja (+v,−v) utilizable")
                continue
            print(f"  suma = {si:+.5f} A   modelo = {tm:+.5f} N·m   "
                  f"fricción = {fr:+.5f} A")
            pts.append((si, tm, fr, ang))
    finally:
        mover.destroy_node()
        rclpy.shutdown()

    if len(pts) < 3:
        print(f"\nSolo {len(pts)} posturas útiles; hacen falta al menos 3 para "
              f"separar `k` del sesgo.")
        return 1

    s = np.array([p[0] for p in pts])
    m = np.array([p[1] for p in pts])
    # m = k*(s - i0) = k*s - k*i0
    A = np.column_stack([s, np.ones_like(s)])
    c, *_ = np.linalg.lstsq(A, m, rcond=None)
    k, b = float(c[0]), float(c[1])
    i0 = -b / k if abs(k) > 1e-12 else float("nan")
    resid = A @ c - m
    rel = float(np.sqrt((resid ** 2).mean()) / np.sqrt((m ** 2).mean()))
    # Sin sesgo, para poder comparar con lo que daba un solo barrido.
    k1 = float(np.linalg.lstsq(s.reshape(-1, 1), m, rcond=None)[0][0])
    r1 = float(np.sqrt(((s * k1 - m) ** 2).mean()) / np.sqrt((m ** 2).mean()))

    print(f"\n{'=' * 56}")
    print(f"  posturas útiles: {len(pts)}   rango de la suma: "
          f"{np.ptp(s):.5f} A  ({100 * np.ptp(s) / max(abs(s.mean()), 1e-9):.1f} %)")
    print(f"  con sesgo : k = {k:8.4f} N·m/A   i0 = {i0:+.5f} A   "
          f"residuo {100 * rel:.3f} %")
    print(f"  sin sesgo : k = {k1:8.4f} N·m/A                      "
          f"residuo {100 * r1:.3f} %")
    print(f"{'=' * 56}")
    if np.ptp(s) < 0.05:
        print("  AVISO: la suma sigue variando poco; `k` e `i0` no se separan "
              "bien. Amplíe el rango de posturas.")
    print(f"\n  Para convertir: --k {k:.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
