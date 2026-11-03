#!/usr/bin/env python3
"""
B1: ¿el F/T integrado ve una masa conocida, con qué ganancia y con qué signo?

Por qué existe. El cierre de B1 (pos_801/802/803, 2026-11-02) comparaba el wrench
antes y después de montar la masa: con 0.915 kg (8.97 N) cambió menos de 1 N,
mientras que la vía de corriente vio 9.04 N. Montar exige tocar el robot y entre
medias el offset del sensor puede cambiar por causas ajenas a la masa (deriva de
~1 N entre corridas, medida), así que esa comparación no separa nada.

Aquí la masa no se toca: se INCLINA la herramienta. El peso gira en los ejes del
sensor y lo que se compara es la diferencia entre cada pose inclinada y el reposo
que tiene a cada lado, de modo que ningún cero anterior y ninguna deriva lenta
entran en el resultado. La incisión no puede dar esto: va con la orientación
constante (A4) y el peso no cambia en ejes del sensor.

    gain = medido / esperado, con esperado = Rᵀ(q) · (0, 0, −m·g)

  gain ≈ +1  el wrench es la fuerza EXTERNA sobre la brida (el peso, hacia abajo).
             El convenio de force_estimator (hoja sobre el entorno) es el opuesto.
  gain ≈ −1  el wrench es la fuerza de la brida sobre la carga.
  gain ≈  0  no ve la masa, o el payload del pendant la está compensando.

Requisitos: masa montada y pesada, **payload 0 y TCP (0, 0, 0)** en el pendant,
External Control en play, slider al 100 %. Control de POSICIÓN con el JTC.

Uso:
    run_ft_tilt_check.py --test-num 1 --masa-kg 0.179
    run_ft_tilt_check.py --analizar ~/.ros/ur5_dyn_control/tilt_1.csv
"""

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

#: Postura de partida, con la punta a 0.33 m de la base: las inclinaciones quedan
#: lejos de la mesa (se comprueba: MIN_TIP_Z).
Q_BASE = np.array([1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 3.1416])
#: Inclinación [rad]: 0.84 N laterales con el acople (1.75 N), 4.3 N con el gripper.
TILT = 0.5
MOVE_SPEED = 0.15           # [rad/s]
SETTLE = 1.0                # [s] tras el fin nominal del movimiento
HOLD = 3.0                  # [s] de promedio en cada pose
POSTURE_TOL = 0.01          # [rad]
MIN_TIP_Z = 0.15            # [m] en base_link; el tablero está en -0.046
GRAVITY = 9.8
#: Veredicto sobre |gain|.
GAIN_OK = (0.8, 1.2)
GAIN_BLIND = 0.2
JOINT_TILTED = {"wrist_1": 3, "wrist_2": 4}
AXES = "xyz"


def sequence(q_base=Q_BASE, tilt=TILT):
    """Poses (nombre, q): cada inclinación entre dos reposos, para restar la deriva."""
    q_base = np.asarray(q_base, float)
    out = [("reposo", q_base.copy())]
    for name, j in JOINT_TILTED.items():
        for sign, label in ((1.0, "+"), (-1.0, "-")):
            q = q_base.copy()
            q[j] += sign * tilt
            out += [(name + label, q), ("reposo", q_base.copy())]
    return out


def weight_in_tool(plant, q, mass, g=GRAVITY):
    """Peso de la masa en ejes de la herramienta [N]: la fuerza externa sobre la brida."""
    import pinocchio as pin
    pin.framesForwardKinematics(plant.model, plant.data, np.asarray(q, float))
    return plant.data.oMf[plant.tcp].rotation.T @ np.array([0.0, 0.0, -mass * g])


def fit_gain(plant, rows, mass):
    """Ajusta `gain` con `rows` = [(nombre, q, F media en herramienta), ...].

    Cada pose inclinada se compara con la media de los reposos a sus lados. Devuelve
    gain, rms del residuo [N] y el detalle (nombre, medido, esperado) por pose.
    """
    detail = []
    for i, (name, q, f) in enumerate(rows):
        if name == "reposo":
            continue
        _, q_a, f_a = rows[i - 1]
        _, q_b, f_b = rows[i + 1]
        meas = np.asarray(f, float) - 0.5 * (np.asarray(f_a, float) + np.asarray(f_b, float))
        exp = weight_in_tool(plant, q, mass) - 0.5 * (
            weight_in_tool(plant, q_a, mass) + weight_in_tool(plant, q_b, mass))
        detail.append((name, meas, exp))
    m = np.concatenate([d[1] for d in detail])
    e = np.concatenate([d[2] for d in detail])
    gain = float(e @ m / (e @ e))
    rms = float(np.sqrt(np.mean((m - gain * e) ** 2)))
    return {"gain": gain, "rms": rms, "detail": detail}


def verdict(gain: float) -> str:
    size = abs(gain)
    if size < GAIN_BLIND:
        return ("NO VE LA MASA: si el payload del pendant es 0, el F/T no sirve como "
                "medida de fuerza en esta configuracion.")
    if not GAIN_OK[0] <= size <= GAIN_OK[1]:
        return "GANANCIA FUERA DE RANGO: no usar el F/T como medida sin revisarlo."
    if gain > 0:
        return ("OK, y el wrench es la fuerza EXTERNA sobre la brida: opuesto al "
                "convenio de force_estimator (hoja sobre el entorno).")
    return ("OK, y el wrench es la fuerza de la brida sobre la carga: el mismo "
            "convenio que force_estimator.")


def report(result, mass):
    print(f"\n  {'pose':10s} {'medido [N]':>26s} {'esperado [N]':>26s}")
    for name, meas, exp in result["detail"]:
        print(f"  {name:10s} " + " ".join(f"{v:+8.3f}" for v in meas)
              + " " + " ".join(f"{v:+8.3f}" for v in exp))
    gain = result["gain"]
    print(f"\n  gain = {gain:+.3f}  (masa vista {gain * mass:+.3f} kg de {mass} kg); "
          f"residuo rms {result['rms']:.3f} N")
    print(f"  {verdict(gain)}")


def write_csv(path, meta, poses):
    """`poses`: [(indice, nombre, q, [(t, F[3], T[3]), ...]), ...]."""
    with open(path, "w") as fh:
        for k, v in meta.items():
            fh.write(f"# {k}={v}\n")
        cols = ["pose_idx", "pose", "t_wall"]
        cols += [f"q{i}" for i in range(1, 7)]
        cols += ["fx", "fy", "fz", "tx", "ty", "tz"]
        fh.write(",".join(cols) + "\n")
        for idx, name, q, samples in poses:
            qtxt = ",".join(f"{v:.6f}" for v in q)
            for t, f, tq in samples:
                fh.write(f"{idx},{name},{t:.6f},{qtxt},"
                         + ",".join(f"{v:.6f}" for v in (*f, *tq)) + "\n")


def read_csv(path):
    """(masa, filas para fit_gain) de un CSV de este script."""
    mass, rows_by_idx = None, {}
    with open(path) as fh:
        for line in fh:
            if line.startswith("# masa_kg="):
                mass = float(line.split("=")[1])
            if line.startswith("#") or line.startswith("pose_idx"):
                continue
            p = line.strip().split(",")
            entry = rows_by_idx.setdefault(int(p[0]), (p[1], [float(v) for v in p[3:9]], []))
            entry[2].append([float(v) for v in p[9:12]])
    rows = [(rows_by_idx[i][0], np.array(rows_by_idx[i][1]),
             np.mean(rows_by_idx[i][2], axis=0)) for i in sorted(rows_by_idx)]
    return mass, rows


def plan_problems(plant, q_now):
    """Por qué no ejecutar esta secuencia desde `q_now`; lista vacía si se puede."""
    from run_position_incision import start_move
    from ur5_identification.gravity_sweeps import tip_position
    problems = []
    low = min(tip_position(plant, q)[2] for _, q in sequence())
    if low < MIN_TIP_Z:
        problems.append(f"la punta bajaria a z = {low:.3f} m (minimo {MIN_TIP_Z} m)")
    _, problem = start_move(plant, q_now, Q_BASE)
    if problem:
        problems.append(problem)
    return problems


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--test-num", type=int)
    ap.add_argument("--masa-kg", type=float, help="masa montada, pesada")
    ap.add_argument("--hold", type=float, default=HOLD, help="promedio en cada pose [s]")
    ap.add_argument("--analizar", help="reanalizar un CSV de este script, sin robot")
    ap.add_argument("--yes", action="store_true", help="no pedir confirmacion")
    ap.add_argument("--out-dir", default=os.path.expanduser("~/.ros/ur5_dyn_control"))
    args = ap.parse_args()

    from ur5_trajectory_optimization.gain_tuning.closed_loop import Plant, default_urdf
    plant = Plant(default_urdf())

    if args.analizar:
        mass, rows = read_csv(args.analizar)
        report(fit_gain(plant, rows, mass), mass)
        return
    if args.test_num is None or args.masa_kg is None or args.masa_kg <= 0.0:
        sys.exit("--test-num y --masa-kg (> 0) son obligatorios para ejecutar")
    out = os.path.join(args.out_dir, f"tilt_{args.test_num}.csv")
    if os.path.exists(out):
        sys.exit(f"{out} ya existe: no se sobrescriben corridas")

    import rclpy
    from geometry_msgs.msg import WrenchStamped
    from rclpy.qos import qos_profile_sensor_data
    from run_friction_campaign_real import Checker
    from run_position_incision import FT_TOPIC, check_speed_scaling, wait_listener

    rclpy.init()
    chk = Checker()
    live = {"on": False, "samples": []}

    def on_wrench(m):
        if live["on"]:
            w = m.wrench
            live["samples"].append((time.time(),
                                    (w.force.x, w.force.y, w.force.z),
                                    (w.torque.x, w.torque.y, w.torque.z)))

    chk.create_subscription(WrenchStamped, FT_TOPIC, on_wrench, qos_profile_sensor_data)

    q_now = chk.joint_state()
    if q_now is None:
        sys.exit("Sin /joint_states: ¿driver arriba?")
    problem = check_speed_scaling(chk)
    if problem:
        sys.exit(problem)
    problems = plan_problems(plant, q_now)
    if problems:
        sys.exit("No se ejecuta: " + "; ".join(problems))
    ok, why = chk.release_effort()
    if not ok:
        sys.exit(f"No se pudo activar el JTC: {why}")
    if not wait_listener(chk, chk._jtc):
        sys.exit("El JTC no escucha en su topico: no se ha movido nada.")

    poses = sequence()
    total = len(poses) * (TILT / MOVE_SPEED + SETTLE + 1.5 + args.hold)
    print(f"Inclinacion de la herramienta: {len(poses)} poses, +-{TILT} rad en wrist_1 y "
          f"wrist_2, unos {total:.0f} s. Masa {args.masa_kg} kg.")
    if not args.yes:
        input("  Pendant con payload 0 y TCP (0, 0, 0)? Paro de emergencia a mano. "
              "Enter para empezar... ")

    recorded = []
    for idx, (name, q) in enumerate(poses):
        chk.go_home(q.tolist(), speed=MOVE_SPEED, settle=SETTLE)
        still, _ = chk.wait_until_still()
        reached = chk.joint_state()
        left = (np.abs(np.asarray(reached) - q).max() if reached is not None
                else float("inf"))
        if not still or left > POSTURE_TOL:
            sys.exit(f"No se alcanzo la pose {name} (error {left:.4f} rad, quieto: "
                     f"{still}). Sin CSV.")
        live["samples"] = []
        live["on"] = True
        t0 = time.time()
        while time.time() - t0 < args.hold:
            rclpy.spin_once(chk, timeout_sec=0.01)
        live["on"] = False
        if not live["samples"]:
            sys.exit(f"Sin F/T en {FT_TOPIC} durante la pose {name}. Sin CSV.")
        recorded.append((idx, name, list(reached), list(live["samples"])))
        f = np.mean([s[1] for s in live["samples"]], axis=0)
        print(f"  {idx + 1}/{len(poses)} {name:10s} F = "
              + " ".join(f"{v:+7.3f}" for v in f) + f"  ({len(live['samples'])} muestras)")

    write_csv(out, {"masa_kg": args.masa_kg, "test_num": args.test_num,
                    "tilt_rad": TILT, "ft_topic": FT_TOPIC, "frame": "tool0_controller"},
              recorded)
    print(f"CSV: {out}")
    mass, rows = read_csv(out)
    report(fit_gain(plant, rows, mass), mass)
    chk.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
