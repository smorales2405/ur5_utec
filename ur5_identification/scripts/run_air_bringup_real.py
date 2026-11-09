#!/usr/bin/env python3
"""
B7.1 — el SMC en aire en el UR5e REAL, con las ganancias finales (plan B7.1, D28, D29, D31).

Dos pasos, una sesión cada uno (docs/09d_air_bringup_protocol.md):

    --paso base       barre shoulder_pan en una postura con la inercia de la incisión, con su r
                      en 0.50 y después en 0.71; las demás juntas, a r = 1
    --paso incision   la incisión en aire, a la altura real, con el acople sin hoja y la mesa
                      despejada, por escalones (--r 0.5, 0.71, 0.85, 1): uno por sesión, y
                      cada uno exige el anterior. shoulder_pan se queda en 0.50; se corrige
                      el exceso de compensación de gravedad del robot (D32) y la fricción se
                      compensa con la tabla medida (D33)

Como la rampa: el script prepara y comprueba, pero cada corrida la confirma el operador
escribiendo una palabra. Nunca pasa `discrete_check.allow_unstable`: si el chequeo de arranque
del nodo rechaza las ganancias, la corrida no se hace.

Uso
---
    run_air_bringup_real.py --gains-file <candidata> --paso base --dry-run
    run_air_bringup_real.py --gains-file <candidata> --paso base --test-base 1600 --payload-kg 0.155
    run_air_bringup_real.py --gains-file <candidata> --paso incision --r 0.5 \\
        --test-base 1600 --payload-kg 0.155
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

from analyze_vibration import load  # noqa: E402
from run_gain_ramp_real import (BANDS, MOVE_SPEED, POSTURE_TOL, WATCHDOG_SAT_FRAC,  # noqa: E402
                                launch_args, load_gains, run_indicators)
from ur5_identification.air_bringup import (BASE_JOINT, BASE_TIP,  # noqa: E402
                                            INCISION_MAX_R, INCISION_MIN_CLEARANCE,
                                            INCISION_Q_ERR,
                                            INCISION_STEPS, MODE_BAND, base_runs,
                                            check_incision, incision_indicators,
                                            incision_runs, label)
from ur5_identification.campaign_levels import JOINT_NAMES, LEVELS  # noqa: E402
from ur5_identification.gain_ramp import (RAMP_ACCEL, RAMP_AMPLITUDE, RAMP_SPEED,  # noqa: E402
                                          check_run, over_tested, step_gains)
from ur5_identification.gravity_sweeps import (MIN_CLEARANCE, TABLE_Z,  # noqa: E402
                                               incision_posture, sweep_min_tip_z)


def csv_path(test_num: int) -> str:
    return os.path.expanduser(f"~/.ros/ur5_dyn_control/smc_{test_num}.csv")


def share(name: str) -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("ur5_dyn_control"), "config", name)


def gains_at(plant, q, g: dict, r, rotor, sweep_joint=None) -> dict:
    """K_P en `q`; G máxima y margen (5 ciclos, con rotor) en `q` y, si se barre una junta,
    en los extremos del barrido."""
    from ur5_trajectory_optimization.gain_tuning.closed_loop import (SmcLaw, mimo_gain_margin,
                                                                     smc_loop_gains)
    lam, phi = step_gains(g["lambda"], g["phi_joint"], r)
    law = SmcLaw(lam=lam, eta=g["eta"], phi=phi, alpha=g["alpha"], use_sat=True,
                 alpha_g=g["alpha_g"])
    z = np.zeros(6)
    M, KP, KD = smc_loop_gains(law, plant.model, plant.data, q, z, z)
    kp, gmax = np.diag(KP), np.diag(KD).copy()
    margin = mimo_gain_margin(M + np.diag(rotor), KP, KD, 0.002, 5)
    if sweep_joint is not None:
        for offset in (-RAMP_AMPLITUDE, RAMP_AMPLITUDE):
            for accel in (RAMP_ACCEL, 0.0, -RAMP_ACCEL):
                qs, dq, ddq = q.copy(), z.copy(), z.copy()
                qs[sweep_joint] += offset
                dq[sweep_joint], ddq[sweep_joint] = RAMP_SPEED, accel
                M, KP, KD = smc_loop_gains(law, plant.model, plant.data, qs, dq, ddq)
                gmax = np.maximum(gmax, np.diag(KD))
                margin = min(margin, mimo_gain_margin(M + np.diag(rotor), KP, KD, 0.002, 5))
    return {"lam": lam, "phi": phi, "kp": kp, "g": gmax, "margin": margin}


def incision_args(gains_file: str, lam, phi) -> list:
    """Lo que el launch recibe en la incisión, además de lo que pone `run_one`. Sin `q_init`:
    el de `incision_common.yaml`."""
    def fmt(values):
        return ",".join(f"{float(v):.6f}" for v in values)
    return [f"gains_file:={gains_file}", f"lambda_joint:={fmt(lam)}", f"phi_joint:={fmt(phi)}",
            "friction_compensation:=table", "gravity_correction:=true",
            f"watchdog_sat_frac_max:={WATCHDOG_SAT_FRAC}",
            f"watchdog_q_err_max:={INCISION_Q_ERR}"]


def incision_geometry(params: dict) -> tuple:
    """(y, z) de la punta al empezar el corte y su cota mínima de diseño, de `incision.*`."""
    inc = params["incision"]
    y = inc["cut_center_y"] - inc["cut_direction"] * (inc["cut_length"] / 2 + inc["cut_lead"])
    z = inc["surface_z"] - inc["cut_depth"]
    return (y, z), z


def plan_base(plant, g, rotor, test_base):
    q = incision_posture(plant, *BASE_TIP)
    rows = []
    for run in base_runs(test_base):
        rows.append({**run, **gains_at(plant, q, g, run["r_joint"], rotor, BASE_JOINT)})
    clear = sweep_min_tip_z(plant, q, BASE_JOINT, RAMP_AMPLITUDE) - TABLE_Z
    problems = [] if clear >= MIN_CLEARANCE else [
        f"barriendo la base, la punta queda a {1e3 * clear:.0f} mm del tablero"]
    return q, rows, clear, problems


def plan_incision(plant, g, rotor, params, test_base):
    """Los escalones de la incisión, con K_P, G y margen al empezar el corte."""
    (y, z), z_min = incision_geometry(params)
    q_cut = incision_posture(plant, y, z)
    rows = [{**run, **gains_at(plant, q_cut, g, run["r_joint"], rotor)}
            for run in incision_runs(test_base)]
    clear = z_min - TABLE_Z
    problems = [] if clear >= INCISION_MIN_CLEARANCE else [
        f"la punta baja a {1e3 * clear:.0f} mm del tablero"]
    return q_cut, rows, clear, problems


def print_row(s, r):
    print(f"  r = {label(r)}  K_P {np.round(s['kp'], 1).tolist()}  G max {s['g'].max():.1f} "
          f"(base {s['g'][0]:.1f})  margen x{s['margin']:.2f}")
    print(f"            por encima de lo probado: {', '.join(over_tested(s['kp'])) or 'ninguna'}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--gains-file", required=True, help="las ganancias candidatas (fila 46)")
    ap.add_argument("--paso", required=True, choices=["base", "incision"])
    ap.add_argument("--r", type=float, default=None, choices=list(INCISION_STEPS),
                    help="escalón de la incisión; uno por sesión, en orden")
    ap.add_argument("--test-base", type=int, default=None)
    ap.add_argument("--payload-kg", type=float, default=None,
                    help="payload configurado en el pendant (el acople pesado)")
    ap.add_argument("--tau-scale", type=float, default=0.30)
    ap.add_argument("--dry-run", action="store_true",
                    help="solo ganancias, holguras y corridas; no conecta con ROS")
    ap.add_argument("--resume", action="store_true", help="paso base: salta los CSV con datos")
    ap.add_argument("--log-dir", default=os.path.expanduser("~/.ros/air_bringup"))
    args = ap.parse_args(argv)
    if args.paso == "incision" and args.r is None:
        print(f"  --r es obligatorio en la incisión: un escalón de {list(INCISION_STEPS)}, en "
              "orden, revisando cada uno antes del siguiente.")
        return 1

    from ur5_trajectory_optimization.gain_tuning.closed_loop import (Plant, Q_INIT,
                                                                     default_urdf)
    from ur5_trajectory_optimization.gain_tuning.problem import ROTOR_INERTIA_REAL
    from run_gravity_sweeps_real import path_min_tip_z
    args.gains_file = os.path.abspath(args.gains_file)
    g = load_gains(args.gains_file)
    plant = Plant(default_urdf())
    base = args.test_base if args.test_base is not None else 0

    print("=" * 78)
    print(f"  B7.1 — SMC EN AIRE, UR5e REAL · paso {args.paso}")
    print(f"  ganancias: {args.gains_file}")
    print(f"  tau_scale = {args.tau_scale}")
    print("=" * 78)
    if args.paso == "base":
        params_file = share("gain_ramp_params.yaml")
        q, rows, clear, problems = plan_base(plant, g, ROTOR_INERTIA_REAL, base)
        target = q
        print(f"  postura: punta y = {BASE_TIP[0]}, z = {BASE_TIP[1]}; q = {np.round(q, 4).tolist()}")
        print(f"  barrido de shoulder_pan ±{RAMP_AMPLITUDE} rad a 0.05 y {RAMP_SPEED} rad/s; la "
              f"punta no baja de {1e3 * clear:.0f} mm sobre el tablero")
        for s in rows:
            print_row(s, s["r_joint"])
        runs = rows
    else:
        params_file = share("smc_params.yaml")
        common = next(iter(yaml.safe_load(open(share("incision_common.yaml"))).values()))[
            "ros__parameters"]
        q_cut, rows, clear, problems = plan_incision(plant, g, ROTOR_INERTIA_REAL, common, base)
        target = np.array(common["q_init"], float)
        print("  incisión de incision_common.yaml, con el acople SIN hoja y la mesa despejada")
        print("  gravedad: se resta el exceso de compensación del robot medido en B4.4 (D32)")
        print("  fricción: la medida por velocidad en la campaña de agosto, en tabla (D33)")
        print("  wrist_1 a r = 1 en todos los escalones (D34); shoulder_pan, a 0.50")
        print(f"  la punta baja a {1e3 * clear:.0f} mm sobre el tablero; guarda de seguimiento "
              f"{INCISION_Q_ERR} rad")
        print(f"  escalones, al empezar el corte (q = {np.round(q_cut, 4).tolist()}):")
        for s in rows:
            print_row(s, s["r_joint"])
        runs = [s for s in rows if s["r"] == args.r]
        prev = runs[0]["previous"]
        if args.r > INCISION_MAX_R:
            problems.append(f"r = {args.r} está bloqueado (D35): por encima de {INCISION_MAX_R}, el "
                            "modo de 31.6 Hz de shoulder_lift y el codo pierde el amortiguamiento "
                            "(docs/09d_air_bringup_protocol.md §5.3)")
        if prev is not None and not os.path.exists(csv_path(prev)):
            problems.append(f"antes de r = {args.r} hay que correr y revisar smc_{prev}, el "
                            "escalón anterior")
    move = path_min_tip_z(plant, np.asarray(Q_INIT, float), target) - TABLE_Z
    print(f"  traslado desde q_init: la punta no baja de {1e3 * move:.0f} mm")
    if move < MIN_CLEARANCE:
        problems.append(f"en el traslado la punta baja a {1e3 * move:.0f} mm del tablero")
    for s in runs:
        if s["margin"] < 2.0:
            problems.append(f"r = {label(s['r_joint'])}: margen x{s['margin']:.2f} bajo x2")
        if s["g"].max() > 120.0:
            problems.append(f"r = {label(s['r_joint'])}: G = {s['g'].max():.1f} sobre 120")
    for msg in problems:
        print(f"  !! {msg}")
    if problems:
        print("  NO es seguro: no se lanza nada.")
        return 1
    print(f"\n  {len(runs)} corrida(s):")
    for s in runs:
        what = JOINT_NAMES[s["joint"]] if s["joint"] is not None else "incisión en aire"
        print(f"    smc_{s['test_num']}: r = {label(s['r_joint'])} · {what}")
    print("\n  Las corridas las lanza la sesión, una a una. No se lanzan a mano.")
    if args.dry_run:
        return 0
    if args.test_base is None or args.payload_kg is None:
        print("\n  --test-base y --payload-kg son obligatorios para ejecutar.")
        return 1

    checklist = ["acople montado SIN la hoja",
                 f"payload del pendant = {args.payload_kg:.3f} kg, CoG (-2.2, 2.1, 29.4) mm",
                 "velocidad reducida activa; paro de emergencia a mano",
                 "nadie dentro del espacio de trabajo",
                 "ningún otro proyecto ROS 2 en el dominio del robot"]
    checklist.append("mesa despejada: sin fijación, bandeja ni muestras; solo el tablero"
                     if args.paso == "incision" else "mesa despejada: sin muestras ni fijación")
    print("\n  Antes de seguir (docs/09d_air_bringup_protocol.md):")
    for item in checklist:
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
    session = {"started": datetime.now().isoformat(timespec="seconds"),
               "block": f"B7.1 {args.paso}", "gains_file": args.gains_file,
               "payload_kg": args.payload_kg, "tau_scale": args.tau_scale,
               "params_file": params_file, "q": [float(v) for v in target], "runs": []}
    rc = 0
    csv_of = csv_path
    try:
        visc, coul = LEVELS["0.0"]
        ok, msg = chk.set_friction(visc, coul)
        print(f"\n  G4 a 0.0: {'OK' if ok else 'FALLÓ'} — {msg}")
        session["g4_ok"] = ok
        if not ok:
            print("  Sin G4 = 0 el robot suma su fricción a la nuestra. Se aborta.")
            return 1
        ok, msg = chk.go_home(target.tolist(), speed=MOVE_SPEED)
        print(f"  traslado con el JTC: {'OK' if ok else 'FALLO'} — {msg}")
        if not ok:
            return 1
        for k, s in enumerate(runs):
            n, csv = s["test_num"], csv_of(s["test_num"])
            if os.path.exists(csv) and args.resume and is_stub(csv):
                print(f"  corrida anterior sin datos: apartada en {set_aside(csv)}")
            if os.path.exists(csv):
                if args.resume and args.paso == "base":
                    print(f"  smc_{n} ya existe: se salta (--resume)")
                    continue
                print(f"  {csv} ya existe: no se sobrescriben corridas.")
                rc = 1
                raise SystemExit
            if k > 0 and input(f"\n  Escriba «SUBIR» para pasar a r = {label(s['r_joint'])}: "
                               ).strip() != "SUBIR":
                print("  Parada por el operador.")
                raise SystemExit
            good, probs, _ = preflight(chk, target.tolist(), POSTURE_TOL)
            for m in probs:
                print(f"    !! {m}")
            if not good:
                rc = 1
                raise SystemExit
            if args.paso == "base":
                word = JOINT_NAMES[BASE_JOINT]
                print(f"\n  [r = {label(s['r_joint'])} · {word} · test {n}]\n    Va a mover "
                      f"{word} ±{RAMP_AMPLITUDE} rad, con par.")
            else:
                word = "INCISION"
                print(f"\n  [r = {label(s['r_joint'])} · incisión en aire · test {n}]\n    Va a "
                      "bajar la punta a 37 mm del tablero y recorrer el corte, con par.")
            if not confirm(word):
                print("    Parada por el operador.")
                raise SystemExit
            if args.paso == "base":
                extra = launch_args(target, args.gains_file, s["lam"], s["phi"])
                res = run_one(BASE_JOINT, n, params_file, args.tau_scale, args.log_dir,
                              tool_mounted=True, controller="smc", extra_args=extra)
            else:
                extra = incision_args(args.gains_file, s["lam"], s["phi"])
                res = run_one(None, n, params_file, args.tau_scale, args.log_dir,
                              tool_mounted=True, controller="smc", extra_args=extra,
                              trajectory_type="incision")
            res.update(pan_r=s.get("pan_r"), r=s.get("r"),
                       r_joint=[float(v) for v in s["r_joint"]])
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
            bands = BANDS if args.paso == "base" else (*BANDS, MODE_BAND)
            ind = run_indicators(csv, args.tau_scale, bands)
            prev = (run_indicators(csv_of(s["previous"]), args.tau_scale, bands)
                    if s["previous"] and os.path.exists(csv_of(s["previous"])) else None)
            reasons = check_run(ind, prev, s["ratio"] if prev else None)
            print(f"    rizado máx {100 * ind['ripple'].max():.2f} % del tope; saturado "
                  f"{100 * ind['sat'].max():.1f} %; banda {int(BANDS[0][0])}-{int(BANDS[0][1])} "
                  f"Hz en base/lift/codo {ind['bands'][BANDS[0]][0]:.3f}/"
                  f"{ind['bands'][BANDS[0]][1]:.3f}/{ind['bands'][BANDS[0]][2]:.3f} N·m")
            if args.paso == "incision":
                mode = ind["bands"][MODE_BAND]
                print(f"    banda {int(MODE_BAND[0])}-{int(MODE_BAND[1])} Hz (la de smc_1610) en "
                      f"lift/codo {mode[1]:.3f}/{mode[2]:.3f} N·m")
                d, i, _ = load(csv)
                inc = incision_indicators(d, i)
                res["incision"] = inc
                reasons += check_incision(inc, TABLE_Z)
                print(f"    TCP: RMS en el corte {1e3 * inc['tcp_rms_cut']:.2f} mm, máximo "
                      f"{1e3 * inc['tcp_max']:.2f} mm; la punta bajó a "
                      f"{1e3 * (inc['z_min'] - TABLE_Z):.0f} mm del tablero")
            res["stop_reasons"] = reasons
            if reasons:
                print("\n  *** PARAR: la corrida no está limpia. ***")
                for reason in reasons:
                    print(f"    - {reason}")
                rc = 1
                raise SystemExit
            if args.paso == "base":
                ok, msg = chk.go_home(target.tolist(), speed=MOVE_SPEED)
                if not ok:
                    print(f"    !! no se pudo recolocar: {msg}")
                    rc = 1
                    raise SystemExit
        print("\n  Paso completo y limpio. Pásele la salida y el registro a quien lo revise "
              "antes del siguiente.")
    except SystemExit:
        pass
    except KeyboardInterrupt:
        print("\n  Interrumpido por el operador.")
        rc = 1
    finally:
        session["finished"] = datetime.now().isoformat(timespec="seconds")
        path = os.path.join(args.log_dir, f"session_{datetime.now():%Y%m%d_%H%M%S}.yaml")
        with open(path, "w") as fh:
            yaml.safe_dump(_plain(session), fh, default_flow_style=False, sort_keys=False,
                           allow_unicode=True)
        print(f"\n  Registro de sesión: {path}")
        rclpy.shutdown()
    return rc


def _plain(obj):
    """Lo que `yaml.safe_dump` acepta: sin arrays de numpy."""
    if isinstance(obj, dict):
        return {k: _plain(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_plain(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return [float(v) for v in obj]
    if isinstance(obj, np.generic):
        return obj.item()
    return obj


if __name__ == "__main__":
    sys.exit(main())
