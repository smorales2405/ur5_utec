#!/usr/bin/env python3
"""
Barrido de excitación por CONTROL DE POSICIÓN, registrando corriente de motor.

Alternativa al barrido por par para la campaña de fricción (FASE 2). Manda la
trayectoria al `scaled_joint_trajectory_controller` del driver y graba
`/joint_states`, incluido el campo `effort`.

Por qué existe
--------------
El barrido por par no puede mover las muñecas. La ley FL entrega `M_jj·kp_j·e`
de par por radián de error, y aun subiendo `kp` hasta el límite de estabilidad
discreta el `wrist_2` saturó en los 8.4 N·m de `tau_scale = 0.30` sin llegar a
moverse: su fricción estática es mayor. El servo interno del robot **sí** tiene
autoridad para moverlas.

Ventajas sobre el barrido por par: la junta se mueve, no hay comandos de par
—control de POSICIÓN— y no aplica la política de gravedad (G3).

El campo `effort` NO es par
---------------------------
Compuerta G5, ya verificada: `/joint_states.effort` son **corrientes de motor**
(`hardware_interface.cpp:800` lee el campo RTDE `actual_current`). Este script
graba la corriente en crudo, en columnas `cur1..cur6`, y NO la llama par.

La conversión a par la hace `calibrate_current.py` a partir de este mismo
registro: en la meseta, a la misma posición y misma rapidez,

    i(+v)·k = g(q) + C(q,v)v + f_v·v + f_c
    i(−v)·k = g(q) + C(q,v)v − f_v·v − f_c

La SUMA deja `g(q) + C·v`, que el modelo conoce → de ahí sale `k` por junta. La
DIFERENCIA deja la fricción. O sea, el mismo barrido da la calibración y la
medida, sin necesitar las constantes de motor del fabricante.

La referencia no se reimplementa
--------------------------------
La tabla `{q,dq,ddq}` la genera el MISMO `JointSweepGenerator` de C++, volcada
con `reference_table_out`. Reescribir el perfil en Python haría que el barrido
por par y el barrido por posición no fueran comparables, que es justo lo que
esta vía pretende permitir.

Uso
---
    # driver arriba, External Control en marcha, robot en q_init
    ros2 run ur5_identification run_current_sweep.py --joint 4 --test-num 954
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from datetime import datetime

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ur5_identification.campaign_levels import (FRICTION_SRV, JOINT_NAMES,
                                                 LEVELS)
from ur_msgs.srv import SetFrictionModelParameters

JTC_TOPIC = "/scaled_joint_trajectory_controller/joint_trajectory"


def build_reference(joint: int, params_file: str, out_csv: str,
                    timeout: float = 120.0) -> bool:
    """
    Vuelca la tabla del barrido usando el generador de C++.

    Se lanza el nodo con `perform_switch:=false` y `perform_unpause:=false`: NO
    toca controladores ni comanda par, solo construye la tabla y la escribe.
    """
    cmd = ["ros2", "run", "ur5_dyn_control", "gz_fl_control_node", "--ros-args",
           "--params-file", params_file,
           "-p", "use_sim_time:=false",
           "-p", "perform_switch:=false",
           "-p", "perform_unpause:=false",
           "-p", f"sweep.joint:={joint}",
           "-p", f"reference_table_out:={out_csv}"]
    if os.path.exists(out_csv):
        os.remove(out_csv)
    try:
        subprocess.run(cmd, timeout=timeout, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
    except subprocess.TimeoutExpired:
        pass          # el nodo se queda en PRE_HOLD a propósito; ya escribió
    return os.path.exists(out_csv) and os.path.getsize(out_csv) > 0


def load_reference(path: str):
    n_meta, dt = 0, 0.002
    with open(path) as fh:
        for line in fh:
            if line.startswith("#"):
                n_meta += 1
                if line.startswith("# dt="):
                    dt = float(line.split("=")[1])
            else:
                break
    d = np.genfromtxt(path, delimiter=",", names=True, dtype=None,
                      encoding="utf-8", skip_header=n_meta)
    col = lambda p: np.column_stack([d[p % i] for i in range(1, 7)])  # noqa: E731
    return dt, col("q%d"), col("dq%d"), np.asarray(d["phase"], dtype=str)


class SweepRunner(Node):
    def __init__(self):
        super().__init__("current_sweep_runner")
        self.samples = []
        self._recording = False
        self.create_subscription(JointState, "/joint_states", self._cb, 100)
        self._pub = self.create_publisher(JointTrajectory, JTC_TOPIC, 1)
        self._switch = self.create_client(
            SwitchController, "/controller_manager/switch_controller")
        self._fric = self.create_client(SetFrictionModelParameters, FRICTION_SRV)

    def _cb(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(n in idx for n in JOINT_NAMES):
            return
        q = [msg.position[idx[n]] for n in JOINT_NAMES]
        dq = [msg.velocity[idx[n]] for n in JOINT_NAMES]
        # `effort` es CORRIENTE de motor (G5), no par. Si el driver no la
        # publica, se registra NaN en vez de un cero que parecería una medida.
        cur = ([msg.effort[idx[n]] for n in JOINT_NAMES]
               if len(msg.effort) >= 6 else [float("nan")] * 6)
        self._last = (q, dq, cur)
        if self._recording:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.samples.append((t, q, dq, cur))

    def wait_state(self, timeout: float = 5.0):
        self._last = None
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if getattr(self, "_last", None) is not None:
                return self._last
        return None

    def ensure_jtc(self, timeout: float = 10.0):
        """El barrido va por POSICIÓN: hay que asegurar que manda el JTC."""
        if not self._switch.wait_for_service(timeout_sec=timeout):
            return False, "servicio switch_controller no disponible"
        req = SwitchController.Request()
        req.activate_controllers = ["scaled_joint_trajectory_controller"]
        req.deactivate_controllers = ["forward_effort_controller"]
        req.strictness = SwitchController.Request.BEST_EFFORT
        fut = self._switch.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        ok = fut.done() and fut.result() is not None and fut.result().ok
        return bool(ok), "ok" if ok else "rechazado"

    def set_friction(self, level: str, timeout: float = 10.0):
        """
        Fija las escalas de compensación interna y VERIFICA la respuesta (G4).

        Sin esta llamada el controlador no impone nada y el valor efectivo no
        queda registrado, así que la corrida no sería reproducible ni
        comparable con las demás.
        """
        visc, coul = LEVELS[level]
        if not self._fric.wait_for_service(timeout_sec=timeout):
            return False, f"servicio {FRICTION_SRV} no disponible"
        req = SetFrictionModelParameters.Request()
        req.parameters.viscous_scale = [float(v) for v in visc]
        req.parameters.coulomb_scale = [float(c) for c in coul]
        fut = self._fric.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            return False, "sin respuesta del servicio"
        ok = bool(getattr(fut.result(), "success", True))
        return ok, "ok" if ok else "rechazado"

    def run(self, dt, q_ref, dq_ref, stride: int):
        """
        Publica la tabla submuestreada y graba mientras se ejecuta.

        Se submuestrea porque el JTC interpola: mandarle 140 000 puntos a 500 Hz
        no mejora el seguimiento y sí hace el mensaje ingobernable. Se dan
        posición Y velocidad en cada punto, que es lo que hace que la meseta
        salga realmente a velocidad constante.
        """
        msg = JointTrajectory()
        msg.joint_names = list(JOINT_NAMES)
        for k in range(0, len(q_ref), stride):
            t = k * dt
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q_ref[k]]
            pt.velocities = [float(v) for v in dq_ref[k]]
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            msg.points.append(pt)
        dur = len(q_ref) * dt

        self.samples.clear()
        self._recording = True
        self._pub.publish(msg)
        t0 = time.time()
        while time.time() - t0 < dur + 3.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        self._recording = False
        return dur


def write_csv(path, samples, dt, phase, meta):
    """
    CSV con el MISMO esquema que el del barrido por par, salvo que la columna de
    par se llama `cur*`: es corriente, no par, y nombrarla `tau` invitaría a
    identificar fricción sobre una magnitud que no lo es.
    """
    if not samples:
        return False
    t0 = samples[0][0]
    with open(path, "w") as fh:
        for k, v in meta.items():
            fh.write(f"# {k}={v}\n")
        fh.write("t_sim")
        for g in ("q", "dq", "cur"):
            for i in range(1, 7):
                fh.write(f",{g}{i}")
        fh.write(",state\n")
        n = len(phase)
        for t, q, dq, cur in samples:
            rel = t - t0
            idx = min(int(round(rel / dt)), n - 1)
            fh.write(f"{rel:.9f}")
            for arr in (q, dq, cur):
                for v in arr:
                    fh.write(f",{v:.9f}")
            fh.write(f",{phase[idx] if idx >= 0 else 'UNKNOWN'}\n")
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--joint", type=int, required=True, choices=range(6))
    ap.add_argument("--test-num", type=int, required=True)
    ap.add_argument("--params-file", default=None)
    ap.add_argument("--stride", type=int, default=50,
                    help="submuestreo de la tabla para el JTC (50 -> 10 Hz)")
    ap.add_argument("--friction-level", choices=list(LEVELS), default=None,
                    help="nivel de compensación interna a fijar antes de barrer "
                         "(G4). Sin él NO se toca y el CSV lo anota como "
                         "'no fijado en esta corrida'")
    ap.add_argument("--yes", action="store_true",
                    help="omite la confirmación (para cuando lo invoca el "
                         "runner de campaña, que ya confirmó)")
    ap.add_argument("--out-dir", default=os.path.expanduser("~/.ros/ur5_dyn_control"))
    args = ap.parse_args()

    if args.params_file is None:
        from ament_index_python.packages import get_package_share_directory
        args.params_file = os.path.join(
            get_package_share_directory("ur5_dyn_control"),
            "config", "sweep_params.yaml")

    ref_csv = f"/tmp/sweep_ref_j{args.joint}.csv"
    print(f"Generando la tabla con el generador de C++ (junta {args.joint})...")
    if not build_reference(args.joint, args.params_file, ref_csv):
        print("  FALLO: no se pudo volcar la tabla de referencias")
        return 1
    dt, q_ref, dq_ref, phase = load_reference(ref_csv)
    print(f"  {len(q_ref)} muestras, dt={dt:.4f} s, {len(q_ref) * dt:.1f} s")

    rclpy.init()
    node = SweepRunner()
    rc = 0
    try:
        st = node.wait_state()
        if st is None:
            print("  FALLO: no llega /joint_states (¿driver arriba?)")
            return 1
        q0 = st[0]
        if any(np.isnan(st[2])):
            print("  AVISO: el driver no publica `effort`; no habrá corriente "
                  "que registrar.")
        err = max(abs(a - b) for a, b in zip(q0, q_ref[0]))
        print(f"  q actual vs inicio de la tabla: {err:.4f} rad")
        if err > 0.15:
            print("  FALLO: el robot no está en el inicio de la tabla. "
                  "Llévelo a q_init antes de lanzar.")
            return 1

        fric_note = "no fijado en esta corrida"
        if args.friction_level:
            okf, msgf = node.set_friction(args.friction_level)
            print(f"  compensación interna «{args.friction_level}»: "
                  f"{'OK' if okf else 'FALLO'} — {msgf}")
            if not okf:
                print("  Sin fijar las escalas la corrida no es reproducible (G4).")
                return 1
            fric_note = args.friction_level

        ok, msg = node.ensure_jtc()
        print(f"  scaled_joint_trajectory_controller activo: "
              f"{'OK' if ok else 'FALLO'} — {msg}")
        if not ok:
            return 1

        print(f"  Va a mover {JOINT_NAMES[args.joint]} por CONTROL DE POSICIÓN "
              f"({len(q_ref) * dt:.0f} s).")
        if not args.yes:
            try:
                if input(f"  Escriba «{JOINT_NAMES[args.joint]}» para lanzar: "
                         ).strip() != JOINT_NAMES[args.joint]:
                    print("  Cancelado.")
                    return 1
            except (EOFError, KeyboardInterrupt):
                print("\n  Cancelado.")
                return 1

        dur = node.run(dt, q_ref, dq_ref, args.stride)
        print(f"  {len(node.samples)} muestras grabadas en {dur:.0f} s")

        out = os.path.join(args.out_dir, f"cur_{args.test_num}.csv")
        os.makedirs(args.out_dir, exist_ok=True)
        meta = {"controller_id": "current_sweep", "test_num": args.test_num,
                "joint": args.joint, "joint_name": JOINT_NAMES[args.joint],
                "timestamp": datetime.now().isoformat(timespec="seconds"),
                "effort_units": "motor_current_A (G5: NO es par)",
                "reference": ref_csv, "stride": args.stride,
                "friction_level": fric_note}
        if write_csv(out, node.samples, dt, phase, meta):
            print(f"  CSV: {out}")
        else:
            print("  FALLO: sin muestras que escribir")
            rc = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
