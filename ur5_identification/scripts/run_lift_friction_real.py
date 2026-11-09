#!/usr/bin/env python3
"""
D36: fricción de shoulder_lift a baja velocidad, con la carga del corte, en el UR5e REAL.

Barrido por CONTROL DE POSICIÓN (el controlador de trayectoria; ningún par), ±0.06 rad a 0.005–0.1
rad/s, en una postura con el 93 % de la carga gravitatoria del corte. G4 a 0.0, como la campaña de
agosto. La fricción sale de la corriente: ½ (i⁺ − i⁻) · k. Detalle: `ur5_identification/
lift_friction.py` y `docs/09d_air_bringup_protocol.md`.

Como las demás sesiones: prepara y comprueba, y la corrida la confirma el operador escribiendo el
nombre de la junta. Nunca sobrescribe un CSV con datos.

Uso
---
    run_lift_friction_real.py --dry-run
    run_lift_friction_real.py --payload-kg 0.155
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from datetime import datetime

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ur5_identification.force_estimator import K_CURRENT  # noqa: E402
from ur5_identification.friction_table import interp, joint_friction  # noqa: E402
from ur5_identification.gravity_sweeps import (MIN_CLEARANCE, TABLE_Z,  # noqa: E402
                                               incision_posture, sweep_min_tip_z)
from ur5_identification.lift_friction import (AMPLITUDE, LIFT, SPEEDS, TEST_NUM, TIP,  # noqa: E402
                                              duration_s, sweep_overrides)

#: Velocidad del traslado a la postura con el JTC [rad/s] y tolerancia de llegada [rad].
MOVE_SPEED = 0.10
POSTURE_TOL = 0.02
JOINT_NAME = "shoulder_lift_joint"


def share(name: str) -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("ur5_dyn_control"), "config", name)


def write_params(q, path: str) -> str:
    """`sweep_params.yaml` con el barrido de esta medida; lo demás, tal cual."""
    doc = yaml.safe_load(open(share("sweep_params.yaml")))
    node = next(iter(doc))
    doc[node]["ros__parameters"]["sweep"].update(sweep_overrides(q))
    doc[node]["ros__parameters"]["q_init"] = [float(v) for v in q]
    with open(path, "w") as fh:
        yaml.safe_dump(doc, fh, default_flow_style=None, sort_keys=False)
    return path


def plan(plant):
    from ur5_trajectory_optimization.gain_tuning.closed_loop import Q_INIT
    from run_gravity_sweeps_real import path_min_tip_z
    import pinocchio as pin
    q = incision_posture(plant, *TIP)
    clear = sweep_min_tip_z(plant, q, LIFT, AMPLITUDE) - TABLE_Z
    move = path_min_tip_z(plant, np.asarray(Q_INIT, float), q) - TABLE_Z
    g_lift = float(pin.computeGeneralizedGravity(plant.model, plant.data, q)[LIFT])
    problems = [f"{what}: la punta baja a {1e3 * c:.0f} mm del tablero"
                for what, c in (("barriendo", clear), ("en el traslado", move))
                if c < MIN_CLEARANCE]
    return q, clear, move, g_lift, problems


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--test-num", type=int, default=TEST_NUM)
    ap.add_argument("--payload-kg", type=float, default=None,
                    help="payload configurado en el pendant (el acople pesado)")
    ap.add_argument("--dry-run", action="store_true", help="solo la postura y el plan")
    ap.add_argument("--csv-dir", default=os.path.expanduser("~/.ros/ur5_dyn_control"))
    ap.add_argument("--log-dir", default=os.path.expanduser("~/.ros/air_bringup"))
    args = ap.parse_args(argv)

    from ur5_trajectory_optimization.gain_tuning.closed_loop import Plant, default_urdf
    q, clear, move, g_lift, problems = plan(Plant(default_urdf()))
    csv = os.path.join(args.csv_dir, f"cur_{args.test_num}.csv")
    print("=" * 78)
    print("  D36 — FRICCIÓN DE shoulder_lift A BAJA VELOCIDAD, UR5e REAL (control de posición)")
    print("=" * 78)
    print(f"  postura: punta y = {TIP[0]}, z = {TIP[1]}; q = {np.round(q, 4).tolist()}")
    print(f"  carga gravitatoria de shoulder_lift: {g_lift:.1f} N·m (en el corte, unos −45)")
    print(f"  barrido ±{AMPLITUDE} rad a {list(SPEEDS)} rad/s, unos {duration_s():.0f} s; la punta "
          f"no baja de {1e3 * clear:.0f} mm (traslado: {1e3 * move:.0f} mm)")
    print(f"  G4 a 0.0, sin par: el controlador de trayectoria mueve la junta. CSV: {csv}")
    for msg in problems:
        print(f"  !! {msg}")
    if problems:
        print("  NO es seguro: no se lanza nada.")
        return 1
    if os.path.exists(csv):
        print(f"  {csv} ya existe: no se sobrescriben corridas. Use otro --test-num.")
        return 1
    print("\n  La corrida la lanza la sesión. No se lanza a mano.")
    if args.dry_run:
        return 0
    if args.payload_kg is None:
        print("\n  --payload-kg es obligatorio para ejecutar.")
        return 1

    print("\n  Antes de seguir:")
    for item in ("acople montado SIN la hoja",
                 f"payload del pendant = {args.payload_kg:.3f} kg, CoG (-2.2, 2.1, 29.4) mm",
                 "velocidad reducida activa; paro de emergencia a mano",
                 "nadie dentro del espacio de trabajo; mesa despejada",
                 "ningún otro proyecto ROS 2 en el dominio del robot"):
        print(f"    [ ] {item}")
    try:
        if input("\n  Escriba «ACEPTO» para iniciar la sesión: ").strip() != "ACEPTO":
            print("  Cancelado.")
            return 1
        typed = float(input("  Escriba la masa del payload que ve en el pendant [kg]: "))
    except (EOFError, KeyboardInterrupt, ValueError):
        print("\n  Cancelado.")
        return 1
    if abs(typed - args.payload_kg) > 0.001:
        print(f"  {typed} ≠ --payload-kg {args.payload_kg}. Cancelado.")
        return 1

    import rclpy
    from run_friction_campaign_real import Checker, confirm, preflight
    from ur5_identification.campaign_levels import LEVELS
    os.makedirs(args.log_dir, exist_ok=True)
    params = write_params(q, os.path.join(args.log_dir, f"lift_friction_{args.test_num}.yaml"))
    rclpy.init()
    chk = Checker()
    session = {"started": datetime.now().isoformat(timespec="seconds"), "block": "D36",
               "test_num": args.test_num, "params_file": params, "q": [float(v) for v in q],
               "payload_kg": args.payload_kg}
    rc = 1
    try:
        ok, msg = chk.set_friction(*LEVELS["0.0"])
        print(f"\n  G4 a 0.0: {'OK' if ok else 'FALLÓ'} — {msg}")
        if not ok:
            return 1
        ok, msg = chk.go_home(q.tolist(), speed=MOVE_SPEED)
        print(f"  traslado con el JTC: {'OK' if ok else 'FALLO'} — {msg}")
        if not ok:
            return 1
        good, probs, _ = preflight(chk, q.tolist(), POSTURE_TOL)
        for m in probs:
            print(f"    !! {m}")
        if not good:
            return 1
        print(f"\n  Va a mover {JOINT_NAME} ±{AMPLITUDE} rad por CONTROL DE POSICIÓN, sin par.")
        if not confirm(JOINT_NAME, "moverla por posición"):
            print("    Parada por el operador.")
            return 1
        log = os.path.join(args.log_dir, f"run_cur_{args.test_num}.log")
        cmd = ["ros2", "run", "ur5_identification", "run_current_sweep.py", "--joint", str(LIFT),
               "--test-num", str(args.test_num), "--params-file", params,
               "--friction-level", "0.0", "--out-dir", args.csv_dir, "--yes"]
        print(f"  Construye la tabla (unos segundos) y barre (unos {duration_s():.0f} s). Registro: "
              f"{log}")
        # La salida, a pantalla y al registro: en D36 se mandaba solo al registro, y la espera
        # parecia un cuelgue.
        with open(log, "w") as fh:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, env=dict(os.environ, PYTHONUNBUFFERED="1"))
            for line in proc.stdout:
                fh.write(line)
                fh.flush()
                print(f"    | {line.rstrip()}")
            sweep_rc = proc.wait(timeout=60)
        session["sweep_rc"] = sweep_rc
        if sweep_rc != 0 or not os.path.exists(csv):
            print(f"  *** el barrido falló (rc={sweep_rc}); ver {log} ***")
            return 1
        measured = joint_friction(csv, LIFT, float(K_CURRENT[LIFT]))
        session["friction_Nm"] = {float(v): float(f) for v, f in measured.items()}
        fr = next(iter(yaml.safe_load(open(share("incision_common.yaml"))).values()))[
            "ros__parameters"]["friction"]
        row = np.reshape(fr["table"], (6, -1))[LIFT]
        print("\n  velocidad [rad/s]   medida aquí [N·m]   tabla actual (agosto, q_init)")
        for v, f in sorted(measured.items()):
            print(f"     {v:7.3f}          {f:7.3f}              {interp(fr['table_speeds'], row, v):7.3f}")
        print("\n  Barrido completo. Pásele la salida y el registro a quien lo revise.")
        rc = 0
    except KeyboardInterrupt:
        print("\n  Interrumpido por el operador.")
    finally:
        session["finished"] = datetime.now().isoformat(timespec="seconds")
        path = os.path.join(args.log_dir, f"session_lift_friction_{datetime.now():%Y%m%d_%H%M%S}.yaml")
        with open(path, "w") as fh:
            yaml.safe_dump(session, fh, default_flow_style=False, sort_keys=False)
        print(f"\n  Registro de sesión: {path}")
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
