#!/usr/bin/env python3
"""
B4.4 — Barridos de gravedad en el UR5e REAL, postura por postura (semiautomático).

Mide `α_g`, el error de gravedad que ve el SMC en el robot: `g_real − g_UR`
(ver ur5_identification/gravity_sweeps.py). En cada postura del trazo de la
incisión barre, una a una, las juntas con gravedad (hombro, codo, wrist_1)
±0.05 rad a 0.05 y 0.10 rad/s en los dos sentidos, con el CT por par
(config/gravity_sweep_params.yaml). Igual que la campaña de fricción: el script
prepara y comprueba, pero **cada corrida la confirma el operador escribiendo el
nombre de la junta**.

Antes de cada sesión (plan, B4.4 y B7.0)
----------------------------------------
- Acople montado SIN la hoja; payload del pendant = acople (masa PESADA, CoG
  del CAD (−2.2, 2.1, 29.4) mm desde tool0). Se declara con `--payload-kg` y se
  confirma tecleando el mismo valor.
- Driver arriba, External Control en marcha, B7.0 hecho en la sesión.

Qué hace
--------
  1. Calcula las posturas (IK, punta en la línea del corte) y comprueba que
     ningún barrido ni ningún traslado entre posturas baja la punta a menos de
     50 mm del tablero. `--dry-run` se queda aquí: posturas, holguras y comandos.
  2. Fija G4 = 0.0 por servicio (se verifica).
  3. Por postura: la lleva con el JTC (posición, despacio), y por junta: lanza
     `ur5e_real.launch.py controller:=fl trajectory_type:=joint_sweep` con
     `sweep_q_fixed` y `q_init` en la postura y `tool_mounted:=true`; al acabar
     devuelve el mando al JTC y recoloca. Aborta la sesión si el nodo entra en
     SAFE_HOLD o el brazo no se detiene.
  4. Nunca sobrescribe un CSV: con `--resume` salta los que existen.

Uso
---
    run_gravity_sweeps_real.py --dry-run
    run_gravity_sweeps_real.py --test-base 1100 --payload-kg 0.191
    run_gravity_sweeps_real.py --test-base 1100 --payload-kg 0.191 --resume
    analyze_gravity_sweeps.py ~/.ros/ur5_dyn_control/fl_11??.csv
"""

from __future__ import annotations

import argparse
import os
import sys
from datetime import datetime

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ur5_identification.campaign_levels import JOINT_NAMES, LEVELS  # noqa: E402
from ur5_identification.gravity_sweeps import (  # noqa: E402
    DEFAULT_POSTURES, GRAVITY_JOINTS, MIN_CLEARANCE, TABLE_Z, incision_posture,
    sweep_min_tip_z, tip_position)

#: Velocidad articular de los traslados entre posturas con el JTC [rad/s].
MOVE_SPEED = 0.10
#: Tolerancia de llegada a la postura antes de lanzar [rad].
POSTURE_TOL = 0.02


def path_min_tip_z(plant, qa, qb, n: int = 41) -> float:
    """Cota mínima de la punta en el traslado articular lineal qa -> qb."""
    return float(min(tip_position(plant, qa + s * (np.asarray(qb) - qa))[2]
                     for s in np.linspace(0.0, 1.0, n)))


def plan(plant, names, joints, amplitude):
    """Posturas, holguras y problemas de geometría (lista vacía si todo bien)."""
    by_name = {n: (y, z) for n, y, z in DEFAULT_POSTURES}
    out, problems = [], []
    prev = None
    for name in names:
        if name not in by_name:
            raise SystemExit(f"postura desconocida {name!r} "
                             f"(validas: {', '.join(by_name)})")
        y, z = by_name[name]
        q = incision_posture(plant, y, z)
        clear = {j: sweep_min_tip_z(plant, q, j, amplitude) - TABLE_Z for j in joints}
        move = None if prev is None else path_min_tip_z(plant, prev, q) - TABLE_Z
        for j, c in clear.items():
            if c < MIN_CLEARANCE:
                problems.append(f"{name}: barriendo {JOINT_NAMES[j]} la punta queda "
                                f"a {1e3 * c:.0f} mm del tablero")
        if move is not None and move < MIN_CLEARANCE:
            problems.append(f"traslado a {name}: la punta baja a {1e3 * move:.0f} mm")
        out.append({"name": name, "tip": [-0.1333, y, z], "q": q,
                    "clearance": clear, "move_clearance": move})
        prev = q
    return out, problems


def launch_args(q):
    txt = ",".join(f"{v:.6f}" for v in q)
    return [f"sweep_q_fixed:={txt}", f"q_init:={txt}"]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--test-base", type=int, default=None)
    ap.add_argument("--payload-kg", type=float, default=None,
                    help="payload configurado en el pendant (el acople pesado)")
    ap.add_argument("--postures", nargs="+", default=[n for n, _, _ in DEFAULT_POSTURES])
    ap.add_argument("--joints", type=int, nargs="+", default=list(GRAVITY_JOINTS))
    ap.add_argument("--tau-scale", type=float, default=0.30)
    ap.add_argument("--params-file", default=None)
    ap.add_argument("--dry-run", action="store_true",
                    help="solo posturas, holguras y comandos; no conecta con ROS")
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--log-dir", default=os.path.expanduser("~/.ros/gravity_sweeps"))
    args = ap.parse_args(argv)

    if args.params_file is None:
        from ament_index_python.packages import get_package_share_directory
        args.params_file = os.path.join(get_package_share_directory("ur5_dyn_control"),
                                        "config", "gravity_sweep_params.yaml")
    rp = list(yaml.safe_load(open(args.params_file)).values())[0]["ros__parameters"]
    amp = float(rp["sweep"]["amplitude"])
    vels = rp["sweep"]["velocities"]

    from ur5_trajectory_optimization.gain_tuning.closed_loop import Plant, default_urdf
    plant = Plant(default_urdf())            # con el acople: la punta es el TCP
    postures, problems = plan(plant, args.postures, args.joints, amp)

    print("=" * 74)
    print("  B4.4 — BARRIDOS DE GRAVEDAD, UR5e REAL")
    print(f"  {len(postures)} posturas x {len(args.joints)} juntas = "
          f"{len(postures) * len(args.joints)} corridas; ±{amp} rad a {vels} rad/s")
    print(f"  tau_scale = {args.tau_scale}   params = {args.params_file}")
    print("=" * 74)
    print(f"  {'postura':<12}{'punta (y, z) [m]':>18}   holgura del barrido [mm] "
          f"({', '.join(JOINT_NAMES[j].replace('_joint', '') for j in args.joints)})"
          f"   traslado")
    for p in postures:
        cl = "  ".join(f"{1e3 * p['clearance'][j]:4.0f}" for j in args.joints)
        mv = "—" if p["move_clearance"] is None else f"{1e3 * p['move_clearance']:.0f}"
        print(f"  {p['name']:<12}{p['tip'][1]:>9.4f}{p['tip'][2]:>9.4f}   {cl:>24}"
              f"   {mv:>8}")
        print(f"  {'':<12}q = {np.round(p['q'], 4).tolist()}")
    for msg in problems:
        print(f"  !! {msg}")
    if problems:
        print("  Geometria NO segura: no se lanza nada.")
        return 1

    base = args.test_base if args.test_base is not None else 0
    print("\n  Corridas:")
    for ip, p in enumerate(postures):
        for j in args.joints:
            n = base + 10 * ip + j
            print(f"    fl_{n}: {p['name']} · {JOINT_NAMES[j]}\n"
                  f"      ros2 launch ur5_dyn_control ur5e_real.launch.py controller:=fl "
                  f"params_file:={args.params_file} trajectory_type:=joint_sweep "
                  f"sweep_joint:={j} tool_mounted:=true tau_scale:={args.tau_scale} "
                  f"test_num:={n} {' '.join(launch_args(p['q']))}")
    if args.dry_run:
        return 0
    if args.test_base is None or args.payload_kg is None:
        print("\n  --test-base y --payload-kg son obligatorios para ejecutar.")
        return 1

    print("\n  Antes de seguir (plan B4.4 y checklist §7):")
    for item in ("acople montado SIN la hoja",
                 f"payload del pendant = {args.payload_kg:.3f} kg, CoG (-2.2, 2.1, 29.4) mm",
                 "B7.0 hecho en esta sesion (RobotReceiveTimeout y desconexion)",
                 "velocidad reducida activa; paro de emergencia a mano",
                 "nadie dentro del espacio de trabajo; mesa despejada"):
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
        print(f"  {typed} ≠ --payload-kg {args.payload_kg}: corrija el pendant o el "
              f"argumento. Cancelado.")
        return 1

    import rclpy
    from run_friction_campaign_real import Checker, confirm, preflight, run_one
    os.makedirs(args.log_dir, exist_ok=True)
    rclpy.init()
    chk = Checker()
    session = {"started": datetime.now().isoformat(timespec="seconds"),
               "block": "B4.4", "payload_kg": args.payload_kg,
               "tau_scale": args.tau_scale, "params_file": args.params_file,
               "postures": [{"name": p["name"], "tip": p["tip"],
                             "q": [float(v) for v in p["q"]]} for p in postures],
               "runs": []}
    rc = 0
    try:
        visc, coul = LEVELS["0.0"]
        ok, msg = chk.set_friction(visc, coul)
        print(f"\n  G4 a 0.0: {'OK' if ok else 'FALLÓ'} — {msg}")
        session["g4_ok"] = ok
        if not ok:
            print("  Sin G4 = 0 el robot suma su fricción a la nuestra. Se aborta.")
            return 1
        for ip, p in enumerate(postures):
            print(f"\n{'=' * 74}\n  POSTURA {p['name']}  punta {p['tip']}\n{'=' * 74}")
            ok, msg = chk.go_home(p["q"].tolist(), speed=MOVE_SPEED)
            print(f"  traslado con el JTC: {'OK' if ok else 'FALLO'} — {msg}")
            if not ok:
                rc = 1
                raise SystemExit
            for j in args.joints:
                n = args.test_base + 10 * ip + j
                csv = os.path.expanduser(f"~/.ros/ur5_dyn_control/fl_{n}.csv")
                print(f"\n  [{p['name']} · {JOINT_NAMES[j]} · test {n}]")
                if os.path.exists(csv):
                    if args.resume:
                        print("    ya existe el CSV — se salta (--resume)")
                        continue
                    print(f"    {csv} ya existe: no se sobrescriben corridas.")
                    rc = 1
                    raise SystemExit
                good, probs, q = preflight(chk, p["q"].tolist(), POSTURE_TOL)
                for msg in probs:
                    print(f"    !! {msg}")
                if not good:
                    rc = 1
                    raise SystemExit
                print(f"    Va a mover {JOINT_NAMES[j]} ±{amp} rad (±{np.degrees(amp):.1f}°) "
                      f"a {vels} rad/s, con par.")
                if not confirm(JOINT_NAMES[j]):
                    print("    Saltada por el operador.")
                    continue
                res = run_one(j, n, args.params_file, args.tau_scale, args.log_dir,
                              tool_mounted=True, extra_args=launch_args(p["q"]))
                res["posture"] = p["name"]
                ok_rel, msg_rel = chk.release_effort()
                res["effort_released"] = ok_rel
                print(f"    control devuelto al JTC: {'OK' if ok_rel else 'FALLO'} — {msg_rel}")
                still, drift = chk.wait_until_still()
                res["settled"], res["drift_after"] = still, drift
                session["runs"].append(res)
                print(f"    CSV: {'OK' if res['csv_exists'] else 'NO SE CREÓ'} {res['csv']}")
                if res.get("safe_hold") or not still:
                    print("\n  *** SAFE_HOLD o brazo sin detenerse: se aborta la sesión. ***")
                    rc = 1
                    raise SystemExit
                ok, msg = chk.go_home(p["q"].tolist(), speed=MOVE_SPEED)
                if not ok:
                    print(f"    !! no se pudo recolocar: {msg}")
                    rc = 1
                    raise SystemExit
    except SystemExit:
        pass
    except KeyboardInterrupt:
        print("\n  Interrumpido por el operador.")
        rc = 1
    finally:
        session["finished"] = datetime.now().isoformat(timespec="seconds")
        path = os.path.join(args.log_dir, f"session_{datetime.now():%Y%m%d_%H%M%S}.yaml")
        with open(path, "w") as fh:
            yaml.safe_dump(session, fh, default_flow_style=False, sort_keys=False,
                           allow_unicode=True)
        print(f"\n  Registro de sesión: {path}")
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
