#!/usr/bin/env python3
"""
B4.6: inercia efectiva por junta en el robot, por control de POSICIÓN.

Una junta cada vez sigue un seno pequeño (de 1 a 4 Hz) y se graba `/joint_states`.
No hay ninguna ley de par en el lazo: manda el JTC, como en B1. El método y sus
límites están en `ur5_identification/inertia_id.py`; el protocolo, en
`docs/09b_inertia_protocol.md`.

Cada segmento se escribe al terminar: si el robot para a medias, lo grabado se
conserva y se analiza. No se sobrescribe ninguna corrida.

Uso (driver arriba, External Control en play, slider al 100 %, TCP (0, 0, 0)):
    run_inertia_id.py --solo-plan
    run_inertia_id.py --test-num 1301 --masa-kg 0.155
    run_inertia_id.py --analizar ~/.ros/ur5_dyn_control/iner_1301.csv [otro.csv ...]
"""

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

#: Qué se mide en cada postura. En "corte_medio" `M_11` del URDF es el doble que en
#: "inicio" y el rotor no cambia: es la comprobación del método en las juntas donde
#: el modelo varía. wrist_2 y wrist_3 apenas cambian: no se repiten.
PLAN = (("inicio", (0, 1, 2, 3, 4, 5)), ("corte_medio", (0, 1, 2, 3)))
STRIDE = 5                  # 100 Hz al JTC: 25 puntos por ciclo a 4 Hz
Q_TOL = 0.005               # [rad] para empezar un segmento
#: Amplitud medida / mandada: fuera de aquí el JTC no está siguiendo el seno.
RATIO_OK = (0.7, 1.3)
#: Cota para planificar, NO un dato: el mayor rotor que cabe esperar [kg·m²].
PLAN_ROTOR = np.array([1.5, 1.5, 1.5, 0.3, 0.3, 0.3])
#: Fracción del par máximo que se admite como pico de inercia previsto.
MAX_TAU_FRACTION = 0.30
DRAIN_TIME = 0.3            # [s]; ver run_position_incision.DRAIN_TIME


def build_plan(plant, postures=None, joints=None, freqs=None):
    """Lista de segmentos y las posturas {nombre: q}."""
    from ur5_identification.gravity_sweeps import DEFAULT_POSTURES, incision_posture
    from ur5_identification.inertia_id import FREQS, excitation
    by_name = {n: (y, z) for n, y, z in DEFAULT_POSTURES}
    q_of, segments = {}, []
    for name, default in PLAN:
        if postures and name not in postures:
            continue
        q_of[name] = incision_posture(plant, *by_name[name])
        for j in default:
            if joints and j not in joints:
                continue
            for f in (freqs or FREQS[j]):
                segments.append(excitation(name, q_of[name], j, f))
    return segments, q_of


def plan_problems(plant, segments, mass=0.0, cog_mm=(0.0, 0.0, 0.0)):
    """Por qué no ejecutar este plan; lista vacía si se puede."""
    from run_position_incision import start_move
    from ur5_identification.campaign_levels import JOINT_NAMES
    from ur5_identification.gravity_sweeps import MIN_CLEARANCE, TABLE_Z, tip_position
    from ur5_identification.inertia_id import model_inertia
    from ur5_trajectory_optimization.gain_tuning.problem import TAU_MAX
    problems = []
    # Los traslados entre posturas, de ida y de vuelta a la primera.
    stops = []
    for s in segments:
        if not stops or s.posture != stops[-1][0]:
            stops.append((s.posture, s.q0))
    stops.append(stops[0])
    for (_, a), (name, b) in zip(stops, stops[1:]):
        _, problem = start_move(plant, a, b, f"postura {name}")
        if problem:
            problems.append(problem)
    for s in segments:
        name = f"{s.posture}/{JOINT_NAMES[s.joint]}/{s.freq:g} Hz"
        low = min(tip_position(plant, q0)[2] for q0 in (
            s.q0 + s.amp * sign * np.eye(6)[s.joint] for sign in (-1.0, 1.0)))
        if low - TABLE_Z < MIN_CLEARANCE:
            problems.append(f"{name}: la punta queda a {1e3 * (low - TABLE_Z):.0f} mm "
                            "del tablero")
        accel = s.amp * (2.0 * np.pi * s.freq) ** 2
        inertia = (model_inertia(plant.model, plant.data, s.q0, s.joint, mass, cog_mm)
                   + PLAN_ROTOR[s.joint])
        if inertia * accel > MAX_TAU_FRACTION * TAU_MAX[s.joint]:
            problems.append(f"{name}: pico previsto de {inertia * accel:.1f} N.m")
    return problems


def segment_problem(seg, samples):
    """Motivo para no seguir tras este segmento, o None."""
    from ur5_identification.campaign_levels import JOINT_NAMES
    from ur5_identification.inertia_id import DT, tracking_ratio
    if len(samples) < 0.4 * seg.duration / DT:
        return f"solo {len(samples)} muestras de /joint_states en {seg.duration:.1f} s"
    t = np.array([s[0] for s in samples])
    q = np.array([s[1] for s in samples])
    cur = np.array([s[3] for s in samples])
    if not np.isfinite(cur).all():
        return "el driver no publica corriente en effort"
    ratio = tracking_ratio({"t_rel": t - t[0], "t": t, "q": q, "joint": seg.joint,
                            "freq": seg.freq, "cycles": seg.cycles, "amp": seg.amp})
    if not RATIO_OK[0] <= ratio <= RATIO_OK[1]:
        return (f"{JOINT_NAMES[seg.joint]} siguio el {100 * ratio:.0f} % de la amplitud "
                "mandada: ¿paro de proteccion? Revisar el pendant.")
    return None


def report(rows, combined, ranges):
    from ur5_identification.campaign_levels import JOINT_NAMES
    print("\n  Fasores: M med, +-, rotor, +-2 ms, s, resid. Celdas de velocidad: rotor cel, s cel.")
    print(f"  {'junta':14s} {'postura':12s} {'n':>2s} {'M med':>8s} {'+-':>7s} "
          f"{'M modelo':>9s} {'rotor':>8s} {'+-2 ms':>7s} {'s':>8s} {'resid':>7s} "
          f"{'rotor cel':>9s} {'s cel':>7s}")
    for r in rows:
        name = JOINT_NAMES[r["joint"]].replace("_joint", "")
        if r["M"] is None:
            print(f"  {name:14s} {r['posture']:12s} {r['n']:2d}  {r['note']}")
            continue
        print(f"  {name:14s} {r['posture']:12s} {r['n']:2d} {r['M']:8.3f} {r['err']:7.3f} "
              f"{r['M_model']:9.3f} {r['dI']:8.3f} {r['sens']:7.3f} {r['s']:8.1f} "
              f"{r['resid']:7.3f} {r['dI_cell']:9.3f} {r['s_cell']:7.1f}  {r['note']}")
    print("\n  Rotor por junta [kg.m2]:")
    for j, c in combined.items():
        name = JOINT_NAMES[j].replace("_joint", "")
        note = "" if c["consistent"] else (
            f"  LAS POSTURAS NO COINCIDEN (difieren {c['spread']:.3f}): no usar sin revisar")
        low, high = ranges[j]
        print(f"    {name:14s} {c['dI']:7.3f} +- {c['err']:.3f}  rango {low:.3f}-{high:.3f}  "
              f"({c['postures']} postura(s)){note}")
    print("  El +- sale del ajuste de fasores y no incluye la diferencia entre estimadores.\n"
          "  Para margenes, el extremo bajo del rango: menos inercia, menos margen.")
    missing = sorted(set(range(6)) - set(combined))
    if missing:
        print("    sin medida valida: " + ", ".join(JOINT_NAMES[j] for j in missing))


def analyze(paths, plant, out_yaml=None):
    """Reúne uno o varios CSV de este script, informa y escribe el YAML."""
    import yaml
    from ur5_identification.inertia_id import (combine_joints, fit_groups, read_csv,
                                               rotor_range)
    segments, mass, cog = [], None, None
    tests = []
    for path in paths:
        meta, segs = read_csv(path)
        m = float(meta.get("masa_kg", 0.0))
        c = tuple(float(v) for v in meta.get("cog_mm", "0 0 0").split())
        if mass is not None and (m != mass or c != cog):
            sys.exit(f"{path}: otra herramienta ({m} kg en {c} mm, antes {mass} kg en "
                     f"{cog} mm): no se pueden reunir")
        mass, cog = m, c
        segments += segs
        tests.append(meta.get("test_num", os.path.basename(path)))
    if not segments:
        sys.exit("Ningun segmento en los CSV.")
    rows = fit_groups(segments, plant.model, plant.data, mass, cog)
    combined = combine_joints(rows)
    ranges = rotor_range(rows)
    print(f"Corridas {tests}: {len(segments)} segmentos; herramienta {mass} kg en {cog} mm.")
    report(rows, combined, ranges)
    if out_yaml:
        if os.path.exists(out_yaml):
            print(f"  {out_yaml} ya existe: no se sobrescribe")
        else:
            with open(out_yaml, "w") as fh:
                yaml.safe_dump({
                    "test_nums": tests, "masa_kg": mass, "cog_mm": list(cog),
                    "rotor_bajo_kg_m2": {int(j): round(r[0], 4) for j, r in ranges.items()},
                    "rotor_alto_kg_m2": {int(j): round(r[1], 4) for j, r in ranges.items()},
                    "rotor_kg_m2": {int(j): round(c["dI"], 4) for j, c in combined.items()},
                    "rotor_err": {int(j): round(c["err"], 4) for j, c in combined.items()},
                    "sensibilidad_2ms": {int(j): round(c["sens"], 4)
                                         for j, c in combined.items()},
                    "posturas_coinciden": {int(j): bool(c["consistent"])
                                           for j, c in combined.items()},
                    "juntas": "0-based (0 = shoulder_pan)"}, fh, sort_keys=False)
            print(f"  YAML: {out_yaml}")
    return rows, combined


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--test-num", type=int)
    ap.add_argument("--masa-kg", type=float, help="herramienta montada, pesada (0 = brida)")
    ap.add_argument("--cog-mm", type=float, nargs=3, default=None,
                    help="CoG en tool0 [mm]; por defecto, el del acople")
    ap.add_argument("--posturas", nargs="+", default=None)
    ap.add_argument("--juntas", type=int, nargs="+", default=None,
                    help="1..6; por defecto, las del plan")
    ap.add_argument("--frecuencias", type=float, nargs="+", default=None)
    ap.add_argument("--solo-plan", action="store_true", help="mostrar el plan, sin robot")
    ap.add_argument("--analizar", nargs="+", help="reanalizar CSV de este script, sin robot")
    ap.add_argument("--yaml", default=None, help="con --analizar: escribir el resultado aqui")
    ap.add_argument("--yes", action="store_true", help="no pedir confirmacion")
    ap.add_argument("--out-dir", default=os.path.expanduser("~/.ros/ur5_dyn_control"))
    args = ap.parse_args()

    from ur5_identification.campaign_levels import JOINT_NAMES
    from ur5_trajectory_optimization.gain_tuning.closed_loop import Plant, default_urdf
    from check_session_config import COG_ACOPLE_MM
    plant = Plant(default_urdf())

    if args.analizar:
        analyze(args.analizar, plant, args.yaml)
        return

    cog = tuple(args.cog_mm) if args.cog_mm else COG_ACOPLE_MM
    joints = [j - 1 for j in args.juntas] if args.juntas else None
    segments, q_of = build_plan(plant, args.posturas, joints, args.frecuencias)
    if not segments:
        sys.exit("El plan esta vacio: revisar --posturas y --juntas.")
    mass = args.masa_kg if args.masa_kg is not None else 0.0
    problems = plan_problems(plant, segments, mass, cog)
    total = sum(s.duration + 3.5 for s in segments)
    print(f"Plan: {len(segments)} segmentos, unos {total / 60.0:.1f} min sin los traslados.")
    for s in segments[:: max(len(segments) // 8, 1)]:
        accel = s.amp * (2.0 * np.pi * s.freq) ** 2
        print(f"  {s.posture:12s} {JOINT_NAMES[s.joint]:20s} {s.freq:4g} Hz  "
              f"+-{np.degrees(s.amp):5.2f} deg  {accel:5.1f} rad/s2")
    if problems:
        sys.exit("No se ejecuta: " + "; ".join(problems))
    if args.solo_plan:
        return
    if args.test_num is None or args.masa_kg is None or args.masa_kg < 0.0:
        sys.exit("--test-num y --masa-kg (>= 0) son obligatorios para ejecutar")
    out = os.path.join(args.out_dir, f"iner_{args.test_num}.csv")
    out_yaml = os.path.join(args.out_dir, f"iner_{args.test_num}.yaml")
    if os.path.exists(out):
        sys.exit(f"{out} ya existe: no se sobrescriben corridas")

    import rclpy
    from run_current_sweep import SweepRunner
    from run_position_incision import (check_pendant_tcp, check_speed_scaling,
                                       move_to_start, start_move, wait_listener)
    from ur5_identification.inertia_id import DT, append_segment, write_header

    rclpy.init()
    node = SweepRunner()
    st = node.wait_state()
    if st is None:
        sys.exit("Sin /joint_states: ¿driver arriba?")
    problem = check_speed_scaling(node)
    if problem:
        sys.exit(problem)
    problem = check_pendant_tcp(node, plant, st[0])
    if problem:
        sys.exit(problem)
    label = "inicio de la prueba"
    err, problem = start_move(plant, st[0], segments[0].q0, label)
    if problem:
        sys.exit(problem)
    ok, why = node.ensure_jtc()
    if not ok:
        sys.exit(f"No se pudo activar el JTC: {why}")
    if not wait_listener(node, node._pub):
        sys.exit("El JTC no escucha en su topico: no se ha movido nada.")
    if not args.yes:
        input("Pendant con TCP (0, 0, 0), nada en la mesa, zona despejada, paro de "
              "emergencia a mano. Enter para empezar... ")

    def go_to(q0, ask=False, name=label):
        now = node.wait_state()
        if now is None:
            sys.exit("Sin /joint_states: se para aqui.")
        gap, why_not = start_move(plant, now[0], q0, name)
        if why_not:
            sys.exit(why_not)
        if gap > Q_TOL:
            move_to_start(now[0], q0, gap, ask=ask, label=name)

    go_to(segments[0].q0)
    completed = 0
    with open(out, "w") as fh:
        write_header(fh, {"controller_id": "position_jtc", "test_num": args.test_num,
                          "masa_kg": args.masa_kg, "cog_mm": " ".join(f"{v:g}" for v in cog),
                          "stride": STRIDE, "dt": DT})
        for index, seg in enumerate(segments):
            go_to(seg.q0, name=f"postura {seg.posture}")
            problem = check_speed_scaling(node)
            if problem:
                print(f"  PARADA antes del segmento {index + 1}: {problem}")
                break
            t_drain = time.time()
            while time.time() - t_drain < DRAIN_TIME:
                rclpy.spin_once(node, timeout_sec=0.01)
            node.run(DT, seg.q, seg.dq, STRIDE)
            samples = list(node.samples)
            append_segment(fh, index, seg, samples)
            problem = segment_problem(seg, samples)
            print(f"  {index + 1}/{len(segments)} {seg.posture:12s} "
                  f"{JOINT_NAMES[seg.joint]:20s} {seg.freq:4g} Hz  {len(samples)} muestras")
            if problem:
                print(f"  PARADA tras el segmento {index + 1}: {problem}")
                break
            completed += 1
    print(f"CSV: {out}  ({completed} de {len(segments)} segmentos completos)")
    if completed < len(segments):
        print("  CORRIDA INCOMPLETA: se analiza lo que hay; no se mueve mas el robot.")
    else:
        go_to(segments[0].q0)
    analyze([out], plant, out_yaml)
    node.destroy_node()
    rclpy.shutdown()
    if completed < len(segments):
        sys.exit(1)


if __name__ == "__main__":
    main()
