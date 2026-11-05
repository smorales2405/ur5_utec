#!/usr/bin/env python3
"""
Rampa de ganancias del SMC en el UR5e REAL, escalón por escalón (plan V16, D25).

Sube desde r = 0.50, por debajo de lo probado limpio en las seis juntas, hasta las ganancias
candidatas (r = 1), barriendo ±0.20 rad una junta cada vez con las demás sostenidas. Si una
corrida no está limpia, para la sesión. Protocolo: docs/09c_gain_ramp_protocol.md.

Igual que B4.4, el script prepara y comprueba, pero **cada corrida la confirma el operador
escribiendo el nombre de la junta**, y **cada escalón nuevo escribiendo SUBIR**.

Nunca pasa `discrete_check.allow_unstable`: si el chequeo de arranque del nodo rechaza un
escalón, ese escalón no se corre.

Uso
---
    run_gain_ramp_real.py --gains-file <candidata> --dry-run
    run_gain_ramp_real.py --gains-file <candidata> --test-base 1400 --payload-kg 0.155
    run_gain_ramp_real.py --gains-file <candidata> --test-base 1400 --payload-kg 0.155 --resume
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from datetime import datetime

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from analyze_vibration import TAU_MAX_NOMINAL, band_rms, load, rolling_mean  # noqa: E402
from ur5_identification.campaign_levels import JOINT_NAMES, LEVELS  # noqa: E402
from ur5_identification.gain_ramp import (PRE_FAILURE_BAND, RAMP_ACCEL,  # noqa: E402
                                          RAMP_AMPLITUDE, RAMP_JOINTS, RAMP_SPEED, RAMP_STEPS,
                                          RAMP_TIP, TESTED_CLEAN_KP, check_run, over_tested,
                                          plan_runs, step_gains)
from ur5_identification.gravity_sweeps import (MIN_CLEARANCE, TABLE_Z,  # noqa: E402
                                               incision_posture, sweep_min_tip_z)

#: Velocidad articular del traslado a la postura con el JTC [rad/s], y tolerancia [rad].
MOVE_SPEED = 0.10
POSTURE_TOL = 0.02
#: Guardas del nodo: las de la rampa de `docs/09_real_bringup.md` §6.
WATCHDOG_SAT_FRAC = 0.10
WATCHDOG_Q_ERR = 0.30
#: Bandas de modo y ventana del rizado [Hz, s], las de `analyze_vibration.py`.
BANDS = (PRE_FAILURE_BAND, (40.0, 55.0))
WINDOW_S = 0.2
#: Lo que se ignora al principio de la corrida [s]: sostén y rampa de entrada.
SKIP_S = 6.0


def load_gains(path: str) -> dict:
    return next(iter(yaml.safe_load(open(path)).values()))["ros__parameters"]


def run_indicators(csv: str, tau_scale: float) -> dict:
    """Los indicadores de `check_run`, sobre el CSV de una corrida."""
    d, i, _ = load(csv)
    t = d[:, i["t_wall"]] - d[0, i["t_wall"]]
    m = t >= SKIP_S
    dt = float(np.median(np.diff(t)))
    n = max(1, int(round(WINDOW_S / dt)))
    top = TAU_MAX_NOMINAL * tau_scale
    ripple, sat, bands = np.zeros(6), np.zeros(6), {b: np.zeros(6) for b in BANDS}
    for j in range(6):
        tau, s = d[m, i[f"tau{j + 1}"]], d[m, i[f"tau{j + 1}_sat"]]
        ripple[j] = rolling_mean(np.diff(tau, prepend=tau[0]) ** 2, n).max() ** 0.5 / top[j]
        sat[j] = rolling_mean(s, n).max()
        for b in BANDS:
            bands[b][j] = band_rms(tau, dt, *b)[0]
    return {"ripple": ripple, "sat": sat, "bands": bands}


def launch_args(q, gains_file: str, lam, phi) -> list:
    """Lo que el launch recibe por encima de lo que pone `run_one`."""
    def fmt(values, digits):
        return ",".join(f"{float(v):.{digits}f}" for v in values)
    return [f"gains_file:={gains_file}", f"lambda_joint:={fmt(lam, 6)}",
            f"phi_joint:={fmt(phi, 6)}", f"sweep_q_fixed:={fmt(q, 6)}", f"q_init:={fmt(q, 6)}",
            "friction_compensation:=viscous_coulomb",
            f"watchdog_sat_frac_max:={WATCHDOG_SAT_FRAC}",
            f"watchdog_q_err_max:={WATCHDOG_Q_ERR}"]


def step_table(plant, q, g: dict, rotor) -> list:
    """Por escalón: K_P en la postura, y G y margen en los extremos de los barridos.

    `margin` lleva el rotor en la planta, como `g7` y el chequeo de arranque en el real;
    `margin_urdf` no: es lo que daría sin él.
    """
    from ur5_trajectory_optimization.gain_tuning.closed_loop import (SmcLaw, mimo_gain_margin,
                                                                     smc_loop_gains)
    rows, z = [], np.zeros(6)
    for r in RAMP_STEPS:
        lam, phi = step_gains(g["lambda"], g["phi_joint"], r)
        law = SmcLaw(lam=lam, eta=g["eta"], phi=phi, alpha=g["alpha"], use_sat=True,
                     alpha_g=g["alpha_g"])
        M, KP, KD = smc_loop_gains(law, plant.model, plant.data, q, z, z)
        kp, gmax = np.diag(KP), np.diag(KD).copy()
        margin = mimo_gain_margin(M + np.diag(rotor), KP, KD, 0.002, 5)
        urdf = mimo_gain_margin(M, KP, KD, 0.002, 5)
        for j in RAMP_JOINTS:
            for offset in (-RAMP_AMPLITUDE, RAMP_AMPLITUDE):
                for accel in (RAMP_ACCEL, 0.0, -RAMP_ACCEL):
                    qs, dq, ddq = q.copy(), z.copy(), z.copy()
                    qs[j] += offset
                    dq[j], ddq[j] = RAMP_SPEED, accel
                    M, KP, KD = smc_loop_gains(law, plant.model, plant.data, qs, dq, ddq)
                    gmax = np.maximum(gmax, np.diag(KD))
                    margin = min(margin, mimo_gain_margin(M + np.diag(rotor), KP, KD, 0.002, 5))
        rows.append({"r": r, "lam": lam, "phi": phi, "kp": kp, "g": gmax, "margin": margin,
                     "margin_urdf": urdf})
    return rows


def geometry_problems(plant, q, q_start) -> tuple:
    """Holguras de los barridos y del traslado, y los problemas que encuentre."""
    from run_gravity_sweeps_real import path_min_tip_z
    clear = {j: sweep_min_tip_z(plant, q, j, RAMP_AMPLITUDE) - TABLE_Z for j in RAMP_JOINTS}
    move = path_min_tip_z(plant, np.asarray(q_start, float), q) - TABLE_Z
    problems = [f"barriendo {JOINT_NAMES[j]} la punta queda a {1e3 * c:.0f} mm del tablero"
                for j, c in clear.items() if c < MIN_CLEARANCE]
    if move < MIN_CLEARANCE:
        problems.append(f"en el traslado la punta baja a {1e3 * move:.0f} mm del tablero")
    return clear, move, problems


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--gains-file", required=True, help="las ganancias candidatas (r = 1)")
    ap.add_argument("--test-base", type=int, default=None)
    ap.add_argument("--payload-kg", type=float, default=None,
                    help="payload configurado en el pendant (el acople pesado)")
    ap.add_argument("--tau-scale", type=float, default=0.30)
    ap.add_argument("--params-file", default=None)
    ap.add_argument("--dry-run", action="store_true",
                    help="solo escalones, holguras y comandos; no conecta con ROS")
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--log-dir", default=os.path.expanduser("~/.ros/gain_ramp"))
    args = ap.parse_args(argv)

    if args.params_file is None:
        from ament_index_python.packages import get_package_share_directory
        args.params_file = os.path.join(get_package_share_directory("ur5_dyn_control"),
                                        "config", "gain_ramp_params.yaml")
    args.gains_file = os.path.abspath(args.gains_file)
    g = load_gains(args.gains_file)

    from ur5_trajectory_optimization.gain_tuning.closed_loop import (Plant, Q_INIT,
                                                                     default_urdf)
    from ur5_trajectory_optimization.gain_tuning.problem import ROTOR_INERTIA_REAL
    plant = Plant(default_urdf())            # con el acople: la punta es el TCP
    q = incision_posture(plant, *RAMP_TIP)
    table = step_table(plant, q, g, ROTOR_INERTIA_REAL)
    clear, move, problems = geometry_problems(plant, q, Q_INIT)

    print("=" * 78)
    print("  RAMPA DE GANANCIAS DEL SMC, UR5e REAL")
    print(f"  ganancias: {args.gains_file}")
    print(f"  params   : {args.params_file}")
    print(f"  tau_scale = {args.tau_scale}   ±{RAMP_AMPLITUDE} rad; postura: punta y = "
          f"{RAMP_TIP[0]}, z = {RAMP_TIP[1]}")
    print(f"  q = {np.round(q, 4).tolist()}")
    print("=" * 78)
    print("  holgura sobre el tablero [mm]: "
          + ", ".join(f"{JOINT_NAMES[j].replace('_joint', '')} {1e3 * c:.0f}"
                      for j, c in clear.items())
          + f"; traslado desde q_init {1e3 * move:.0f}")
    print("\n  K_P [N·m/rad] en la postura; G [N·m/(rad/s)] y margen (5 ciclos) en los barridos.")
    print(f"  probado limpio: K_P {np.round(TESTED_CLEAN_KP, 1).tolist()}")
    for s in table:
        print(f"  r = {s['r']:.2f}  K_P {np.round(s['kp'], 1).tolist()}  G max "
              f"{s['g'].max():.1f}  margen x{s['margin']:.2f}; sin rotor, en la postura, "
              f"x{s['margin_urdf']:.2f}")
        over = over_tested(s["kp"])
        print(f"            por encima de lo probado: {', '.join(over) or 'ninguna'}")
        if s["margin"] < 2.0:
            problems.append(f"r = {s['r']:.2f}: margen x{s['margin']:.2f} por debajo de x2")
        if s["g"].max() > 120.0:
            problems.append(f"r = {s['r']:.2f}: G = {s['g'].max():.1f} por encima de 120")
    for msg in problems:
        print(f"  !! {msg}")
    if problems:
        print("  NO es seguro: no se lanza nada.")
        return 1

    base = args.test_base if args.test_base is not None else 0
    runs = plan_runs(base)
    print(f"\n  {len(runs)} corridas (test = base + 10·escalón + junta):")
    for run in runs:
        print(f"    smc_{run['test_num']}: r = {run['r']:.2f} · {JOINT_NAMES[run['joint']]}")
    # Aquí se imprimía el comando de la primera corrida, y lanzarlo a mano acabó en un
    # paro de emergencia (smc_1405): solo la sesión lleva el robot a la postura.
    print("\n  Las corridas las lanza la sesión, una a una. No se lanzan a mano.")
    if args.dry_run:
        return 0
    if args.test_base is None or args.payload_kg is None:
        print("\n  --test-base y --payload-kg son obligatorios para ejecutar.")
        return 1

    print("\n  Antes de seguir (docs/09c_gain_ramp_protocol.md):")
    for item in ("acople montado SIN la hoja",
                 f"payload del pendant = {args.payload_kg:.3f} kg, CoG (-2.2, 2.1, 29.4) mm",
                 "seguridad de B7.0 hecha en esta sesión",
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
              "argumento. Cancelado.")
        return 1

    import rclpy
    from run_friction_campaign_real import Checker, confirm, preflight, run_one
    from run_gravity_sweeps_real import is_stub, set_aside
    os.makedirs(args.log_dir, exist_ok=True)
    rclpy.init()
    chk = Checker()
    session = {"started": datetime.now().isoformat(timespec="seconds"), "block": "rampa",
               "gains_file": args.gains_file, "payload_kg": args.payload_kg,
               "tau_scale": args.tau_scale, "params_file": args.params_file,
               "q": [float(v) for v in q], "runs": []}
    rc = 0
    csv_of = lambda n: os.path.expanduser(f"~/.ros/ur5_dyn_control/smc_{n}.csv")  # noqa: E731
    try:
        visc, coul = LEVELS["0.0"]
        ok, msg = chk.set_friction(visc, coul)
        print(f"\n  G4 a 0.0: {'OK' if ok else 'FALLÓ'} — {msg}")
        session["g4_ok"] = ok
        if not ok:
            print("  Sin G4 = 0 el robot suma su fricción a la nuestra. Se aborta.")
            return 1
        ok, msg = chk.go_home(q.tolist(), speed=MOVE_SPEED)
        print(f"  traslado con el JTC: {'OK' if ok else 'FALLO'} — {msg}")
        if not ok:
            return 1
        step = None
        for run in runs:
            n, j, s = run["test_num"], run["joint"], table[run["step"]]
            csv = csv_of(n)
            if os.path.exists(csv) and args.resume and is_stub(csv):
                print(f"  corrida anterior sin datos: apartada en {set_aside(csv)}")
            if os.path.exists(csv):
                if args.resume:
                    print(f"  smc_{n} ya existe: se salta (--resume)")
                    continue
                print(f"  {csv} ya existe: no se sobrescriben corridas.")
                rc = 1
                raise SystemExit
            if run["step"] != step:
                step = run["step"]
                print(f"\n{'=' * 78}\n  ESCALÓN r = {s['r']:.2f}: K_P "
                      f"{np.round(s['kp'], 1).tolist()}\n  margen x{s['margin']:.2f}; por "
                      f"encima de lo probado: {', '.join(over_tested(s['kp'])) or 'ninguna'}"
                      f"\n{'=' * 78}")
                if input("  Escriba «SUBIR» para empezar este escalón: ").strip() != "SUBIR":
                    print("  Parada por el operador.")
                    raise SystemExit
            good, probs, _ = preflight(chk, q.tolist(), POSTURE_TOL)
            for msg in probs:
                print(f"    !! {msg}")
            if not good:
                rc = 1
                raise SystemExit
            print(f"\n  [r = {s['r']:.2f} · {JOINT_NAMES[j]} · test {n}]\n    Va a mover "
                  f"{JOINT_NAMES[j]} ±{RAMP_AMPLITUDE} rad, con par.")
            if not confirm(JOINT_NAMES[j]):
                print("    Parada por el operador.")
                raise SystemExit
            res = run_one(j, n, args.params_file, args.tau_scale, args.log_dir,
                          tool_mounted=True, controller="smc",
                          extra_args=launch_args(q, args.gains_file, s["lam"], s["phi"]))
            res.update(r=s["r"], step=run["step"])
            ok_rel, msg_rel = chk.release_effort()
            res["effort_released"] = ok_rel
            print(f"    control devuelto al JTC: {'OK' if ok_rel else 'FALLO'} — {msg_rel}")
            still, drift = chk.wait_until_still()
            res["settled"], res["drift_after"] = still, drift
            session["runs"].append(res)
            margin = re.findall(r"margen de ganancia x([\d.]+)",
                                open(res["node_log"], errors="replace").read())
            res["margin_node"] = float(margin[-1]) if margin else None
            if not res.get("completed") or res.get("safe_hold") or not still:
                print(f"\n  *** PARAR: corrida incompleta (estado {res['state_reached']}), "
                      "SAFE_HOLD o brazo sin detenerse. Revise el log antes de seguir. ***")
                rc = 1
                raise SystemExit
            ind = run_indicators(csv, args.tau_scale)
            prev = (run_indicators(csv_of(run["previous"]), args.tau_scale)
                    if run["previous"] and os.path.exists(csv_of(run["previous"])) else None)
            reasons = check_run(ind, prev, run["ratio"] if prev else None)
            res["stop_reasons"] = reasons
            print(f"    rizado máx {100 * ind['ripple'].max():.2f} % del tope; "
                  f"saturado {100 * ind['sat'].max():.1f} %; banda "
                  f"{int(BANDS[0][0])}-{int(BANDS[0][1])} Hz en lift/codo "
                  f"{ind['bands'][BANDS[0]][1]:.3f}/{ind['bands'][BANDS[0]][2]:.3f} N·m")
            if reasons:
                print("\n  *** PARAR: la corrida no está limpia. ***")
                for reason in reasons:
                    print(f"    - {reason}")
                rc = 1
                raise SystemExit
            ok, msg = chk.go_home(q.tolist(), speed=MOVE_SPEED)
            if not ok:
                print(f"    !! no se pudo recolocar: {msg}")
                rc = 1
                raise SystemExit
        print("\n  Rampa completa y limpia. Esto no prueba la incisión: sigue B7.1, en aire.")
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
