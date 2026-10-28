#!/usr/bin/env python3
"""
B4.4: α_g a partir de los barridos de gravedad (run_gravity_sweeps_real.py).

Por corrida (una postura, una junta barrida) imprime, en N·m:

  ĝ             gravedad del modelo del nodo en la postura
  g_real − ĝ    parte par de k⊙i a q emparejada, por velocidad (tienen que
                coincidir: si no, queda fricción par o un término sin modelar)
  g_UR − ĝ      lo que suma el robot menos ĝ (k⊙i − τ_cmd con el retardo D)
  Δ             g_real − g_UR: el error de gravedad que ve el SMC en el real
  α             |Δ| / |ĝ|

y al final α_g por junta (el peor entre posturas). Ver
ur5_identification/gravity_sweeps.py para el porqué de cada término.

Uso:
    analyze_gravity_sweeps.py ~/.ros/ur5_dyn_control/fl_11??.csv
    analyze_gravity_sweeps.py --out alpha_g.yaml ~/.ros/ur5_dyn_control/fl_11??.csv
"""

import argparse
import os
import sys

import numpy as np
import yaml

from ur5_identification.campaign_levels import JOINT_NAMES
from ur5_identification.gravity_sweeps import (DEFAULT_POSTURES, alpha_by_joint,
                                               analyze_run, incision_posture)

#: |g_UR − ĝ| en el hombro por encima del cual se sospecha del payload [N·m]
#: (la firma del acople en estas posturas es ~1 N·m; check_session_config.py).
PAYLOAD_HINT = 0.5


def posture_name(q, plant):
    best, name = np.inf, "?"
    for n, y, z in DEFAULT_POSTURES:
        d = np.abs(incision_posture(plant, y, z) - q).max()
        if d < best:
            best, name = d, n
    return name if best < 1e-3 else "?"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--delay", type=int, default=None,
                    help="retardo de tuberia [ciclos]; por defecto 5 real, 1 Gazebo")
    ap.add_argument("--out", default=None, help="YAML con alpha_g por junta")
    a = ap.parse_args(argv)

    from ur5_trajectory_optimization.gain_tuning.closed_loop import Plant, default_urdf
    plant = Plant(default_urdf())
    results = []
    print(f"{'corrida':<14}{'postura':<13}{'junta':<21}{'ĝ':>8}  {'g_real−ĝ @ v [rad/s]':<28}"
          f"{'g_UR−ĝ':>8}{'Δ':>9}{'α':>8}")
    for p in a.csv:
        name = os.path.basename(p)
        try:
            r = analyze_run(p, delay=a.delay)
        except ValueError as e:
            print(f"{name:<14}se salta: {e}")
            continue
        j = r["joint"]
        lv = "  ".join(f"{x['g_real_minus_ghat']:+.3f}@{v:.2f}" for v, x in r["levels"].items())
        print(f"{name:<14}{posture_name(r['q_fixed'], plant):<13}"
              f"{JOINT_NAMES[j]:<21}{r['g_hat'][j]:8.2f}  {lv:<28}"
              f"{r['g_ur_minus_ghat'][j]:8.3f}{r['delta']:9.3f}{r['alpha']:8.4f}")
        # En Gazebo g_UR = 0 (no hay compensacion interna): el aviso no aplica.
        if r["real"] and abs(r["g_ur_minus_ghat"][1]) > PAYLOAD_HINT:
            print(f"{'':<14}!! g_UR − ĝ en shoulder_lift = {r['g_ur_minus_ghat'][1]:+.2f}: "
                  f"¿payload del pendant distinto del acople?")
        results.append(r)
    if not results:
        return 1
    alpha = alpha_by_joint(results)
    print("\nα_g por junta (peor postura):  " + "   ".join(
        f"{JOINT_NAMES[j].replace('_joint', '')} {v:.4f}" for j, v in sorted(alpha.items())))
    worst = max(alpha.values())
    print(f"α_g para la ley (escalar, el peor): {worst:.4f}   "
          f"(origen: {'real' if results[0]['real'] else 'GAZEBO: prueba del método'})")
    if a.out:
        doc = {"alpha_g": float(worst),
               "alpha_g_por_junta": {JOINT_NAMES[j]: float(v) for j, v in alpha.items()},
               "real": bool(results[0]["real"]),
               "corridas": [os.path.basename(r["csv"]) for r in results]}
        with open(a.out, "w") as fh:
            yaml.safe_dump(doc, fh, sort_keys=False, allow_unicode=True)
        print(f"-> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
