#!/usr/bin/env python3
"""
Retardo de tubería del lazo de par a partir de CSV de corridas (B7.0).

Para cada CSV busca los escalones del par mandado (los cambios de signo del
feedforward de Coulomb en las inversiones de un barrido), promedia la respuesta
de la corriente alineada en ellos y ajusta `k ⊙ i(k) = a · τ_cmd(k − D)`. `D` es
el `delay_steps` del evaluador (ver ur5_identification/pipeline_delay.py).

Uso:
    analyze_pipeline_delay.py ~/.ros/ur5_dyn_control/fl_50{0,2,6,7}.csv
    analyze_pipeline_delay.py --joint 2 --perfil ~/.ros/ur5_dyn_control/fl_500.csv
"""

import argparse
import os
import sys

import numpy as np

from ur5_identification.pipeline_delay import estimate_from_csv

MIN_EVENTS = 5
MAX_RESID = 0.20


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--joint", type=int, default=None, help="1..6 (por defecto, "
                    "la de mas escalones)")
    ap.add_argument("--dt", type=float, default=0.002)
    ap.add_argument("--perfil", action="store_true",
                    help="imprimir la respuesta media alineada")
    a = ap.parse_args(argv)

    by_kind = {True: [], False: []}
    print(f"{'corrida':<16}{'origen':<8}{'junta':>6}{'metodo':>8}{'eventos':>9}"
          f"{'D [ciclos]':>12}{'D [ms]':>8}{'ganancia':>10}{'resid.':>8}")
    for p in a.csv:
        name = os.path.basename(p)
        try:
            r = estimate_from_csv(p, a.joint)
        except ValueError as e:
            print(f"{name:<16}se salta: {e}")
            continue
        kind = "real" if r["real"] else "gazebo"
        extra = (f"{r['gain']:10.3f}{100 * r['rms_rel']:7.1f}%"
                 if r["method"] == "steps" else f"   rho {r['rho']:.2f}")
        print(f"{name:<16}{kind:<8}{r['joint']:>6}{r['method']:>8}"
              f"{r['n_events']:>9}{r['delay']:>12.2f}{1e3 * r['delay'] * a.dt:>8.1f}"
              f"{extra}")
        # Al promedio solo entran ajustes limpios: con un escalon suelto, o con
        # la junta acelerando en la ventana (residuo alto), D no es fiable.
        if (r["method"] == "steps" and r["n_events"] >= MIN_EVENTS
                and r["rms_rel"] <= MAX_RESID):
            by_kind[r["real"]].append((r["delay"], r["n_events"]))
            if a.perfil:
                ks = np.arange(len(r["U"])) - 6
                print("    k      " + " ".join(f"{k:6d}" for k in ks))
                print("    tau    " + " ".join(f"{v:6.2f}" for v in r["U"]))
                print("    k*i    " + " ".join(f"{v:6.2f}" for v in r["Y"]))
                print("    dq*1e3 " + " ".join(f"{1e3 * v:6.2f}" for v in r["dq"]))
    for real, vals in by_kind.items():
        if not vals:
            continue
        D = np.array([v for v, _ in vals])
        w = np.array([n for _, n in vals], float)
        mean = float((D * w).sum() / w.sum())
        print(f"\n{'REAL' if real else 'GAZEBO'}: D = {mean:.2f} ciclos "
              f"({1e3 * mean * a.dt:.1f} ms), ponderado por eventos; "
              f"rango {D.min():.2f}–{D.max():.2f} en {len(D)} corridas, "
              f"{int(w.sum())} escalones")
    return 0


if __name__ == "__main__":
    sys.exit(main())
