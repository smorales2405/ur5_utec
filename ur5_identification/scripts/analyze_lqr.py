#!/usr/bin/env python3
"""
Criterios de aceptación del LQR-SDRE (FASE 4).

Lee los DOS ficheros que escribe `gz_lqr_sdre_control_node`:

  · `lqr_<n>.csv`      — CSV unificado (esquema común a los 4 controladores):
                         seguimiento, esfuerzo, saturación.
  · `lqr_diag_<n>.csv` — diagnóstico por paso, propio de esta ley:
                         max Re(eig(A − B K)), cond(M), margen de
                         controlabilidad, residuo e iteraciones de la CARE y
                         tiempos de cómputo.

Los tres criterios del plan:

  1. `max Re(eig(A − B K)) < 0` en el 100 % de los pasos de una corrida completa.
     Se evalúa con la K REALMENTE vigente en cada ciclo, no solo en los ciclos
     en que se resuelve la CARE: entre actualizaciones A(q,q̇) se mueve y una K
     congelada puede dejar de estabilizar. Ese es justo el riesgo que introduce
     la decimación.
  2. Tiempo de cómputo por ciclo medido y por debajo del presupuesto declarado
     (un período del lazo). Se reportan media, p95, p99 y MÁXIMO: un solo ciclo
     desbordado ya rompe el determinismo, así que el máximo es la cifra que
     manda.
  3. Seguimiento de la incisión sin saturación sostenida.

Uso:
  analyze_lqr.py --test 1
  analyze_lqr.py --csv ~/.ros/ur5_dyn_control/lqr_1.csv \
                 --diag ~/.ros/ur5_dyn_control/lqr_diag_1.csv
  analyze_lqr.py --test 1 --plot /tmp/lqr_eig.png
"""

import argparse
import os

import numpy as np

# Ventana de MESETA del corte, relativa al inicio de TRACK [s]. Misma que
# analyze_smc.py: es donde el robot está en régimen (feed constante) y donde se
# miden las métricas que van al paper. Ver docs/01_trajectory.md.
CUT_PLATEAU = (15.90, 23.90)

DEFAULT_DIR = os.path.expanduser("~/.ros/ur5_dyn_control")


def _read(path):
    """genfromtxt(names=True) NO salta las líneas '#': hay que contarlas."""
    n_meta = 0
    with open(path) as fh:
        for line in fh:
            if line.startswith("#"):
                n_meta += 1
            else:
                break
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None,
                         encoding="utf-8", skip_header=n_meta,
                         invalid_raise=False)


def load_main(path):
    d = _read(path)
    m = np.char.startswith(d["state"].astype(str), "TRACK")
    if not m.any():
        raise SystemExit(f"{path}: no hay filas en estado TRACK")

    def col(pat):
        return np.column_stack([d[pat % i][m] for i in range(1, 7)])

    t = d["t_sim"][m]
    return {"t": t - t[0], "t_abs": t,
            "q": col("q%d"), "q_des": col("q%d_des"),
            "tau": col("tau%d"), "sat": col("tau%d_sat"),
            "xyz": np.column_stack([d[k][m] for k in ("x", "y", "z")]),
            "xyz_des": np.column_stack([d[k][m] for k in ("x_des", "y_des", "z_des")]),
            "theta_err": d["theta_err"][m]}


def load_diag(path, t0=None):
    d = _read(path)
    t = d["t_sim"]
    out = {n: d[n] for n in d.dtype.names}
    out["t_rel"] = t - (t[0] if t0 is None else t0)
    return out


def tracking(d, window=None):
    m = (np.ones(len(d["t"]), bool) if window is None
         else (d["t"] >= window[0]) & (d["t"] <= window[1]))
    e = d["q"][m] - d["q_des"][m]
    ep = np.linalg.norm(d["xyz"][m] - d["xyz_des"][m], axis=1)
    tau = d["tau"][m]
    return {
        "n": int(m.sum()),
        "rmse_q": float(np.sqrt((e ** 2).mean())),
        "max_q": float(np.abs(e).max()),
        "rmse_tcp_mm": float(1e3 * np.sqrt((ep ** 2).mean())),
        "max_tcp_mm": float(1e3 * ep.max()),
        "theta_max_mrad": float(1e3 * d["theta_err"][m].max()),
        "tau_max": float(np.abs(tau).max()),
        "effort": float((tau ** 2).sum()),
        "sat_pct": float(100.0 * d["sat"][m].mean()),
        # Saturación SOSTENIDA: la racha más larga con alguna junta recortada.
        # Un pico aislado es inocuo; una racha significa que el actuador manda.
        "sat_run_max": _longest_run(d["sat"][m].any(axis=1)),
    }


def _longest_run(flags):
    best = run = 0
    for f in flags:
        run = run + 1 if f else 0
        best = max(best, run)
    return int(best)


def stability(diag, window=None):
    t = diag["t_rel"]
    m = (np.ones(len(t), bool) if window is None
         else (t >= window[0]) & (t <= window[1]))
    e = diag["max_re_eig"][m]
    bad = ~(e < 0.0)
    return {
        "n": int(m.sum()),
        "max_re_max": float(e.max()),
        "max_re_min": float(e.min()),
        "n_bad": int(bad.sum()),
        "pct_ok": float(100.0 * (1.0 - bad.mean())),
        "t_first_bad": float(t[m][bad][0]) if bad.any() else float("nan"),
        "cond_M_max": float(diag["cond_M"][m].max()),
        "ctrl_margin_min": float(diag["ctrl_margin"][m].min()),
        "residual_max": float(diag["care_residual"][m].max()),
        "iters_max": float(diag["care_iters"][m].max()),
        "fails": float(diag["care_fails"][m].max()),
        "update_pct": float(100.0 * diag["care_updated"][m].mean()),
    }


def timing(diag, budget_us, window=None):
    t = diag["t_rel"]
    m = (np.ones(len(t), bool) if window is None
         else (t >= window[0]) & (t <= window[1]))
    out = {}
    for key, col in (("ley", "t_law_us"), ("CARE", "t_care_us")):
        v = diag[col][m]
        if key == "CARE":                      # solo los ciclos que resolvieron
            v = v[diag["care_updated"][m] > 0.5]
        if v.size == 0:
            continue
        out[key] = {
            "mean": float(v.mean()),
            "p95": float(np.percentile(v, 95)),
            "p99": float(np.percentile(v, 99)),
            "max": float(v.max()),
            "over_pct": float(100.0 * (v > budget_us).mean()),
        }
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--test", type=int, default=None,
                    help=f"numero de ensayo; busca en {DEFAULT_DIR}")
    ap.add_argument("--csv", default=None)
    ap.add_argument("--diag", default=None)
    ap.add_argument("--control-rate", type=float, default=500.0)
    ap.add_argument("--plot", default=None,
                    help="PNG con max Re(lambda) vs t (artefacto del paper)")
    args, _ = ap.parse_known_args()

    csv = args.csv or (os.path.join(DEFAULT_DIR, f"lqr_{args.test}.csv")
                       if args.test is not None else None)
    diagp = args.diag or (os.path.join(DEFAULT_DIR, f"lqr_diag_{args.test}.csv")
                          if args.test is not None else None)
    if not csv or not os.path.exists(csv):
        raise SystemExit(f"no se encuentra el CSV unificado: {csv}")

    budget_us = 1e6 / args.control_rate
    d = load_main(csv)
    print(f"CSV      : {csv}")
    print(f"TRACK    : {len(d['t'])} muestras, {d['t'][-1]:.2f} s")

    print("\n=== 3) Seguimiento y esfuerzo ===")
    hdr = (f"{'ventana':<34}{'RMSE q':>10}{'TCP mm':>9}{'max TCP':>9}"
           f"{'theta mrad':>12}{'max|tau|':>10}{'sat %':>8}{'racha sat':>10}")
    print(hdr)
    print("-" * len(hdr))
    for tag, win in (("TRACK completo", None), ("meseta del corte (regimen)", CUT_PLATEAU)):
        r = tracking(d, win)
        print(f"{tag:<34}{r['rmse_q']:10.6f}{r['rmse_tcp_mm']:9.4f}"
              f"{r['max_tcp_mm']:9.4f}{r['theta_max_mrad']:12.4f}"
              f"{r['tau_max']:10.2f}{r['sat_pct']:8.2f}{r['sat_run_max']:10d}")

    if not diagp or not os.path.exists(diagp):
        print(f"\n(sin CSV de diagnostico: {diagp}) — no se pueden evaluar los "
              f"criterios 1 y 2")
        return

    diag = load_diag(diagp, t0=d["t_abs"][0])
    # El diagnóstico cubre TODOS los estados; se recorta a la ventana de TRACK.
    win_track = (0.0, float(d["t"][-1]))

    print("\n=== 1) Estabilidad del esquema congelado: max Re(eig(A - B K)) ===")
    for tag, win in (("TRACK completo", win_track),
                     ("meseta del corte (regimen)", CUT_PLATEAU)):
        s = stability(diag, win)
        veredicto = "CUMPLE" if s["n_bad"] == 0 else f"FALLA en t={s['t_first_bad']:.3f} s"
        print(f"  {tag:<28} n={s['n']:6d}  max Re en [{s['max_re_min']:8.3f}, "
              f"{s['max_re_max']:8.3f}] rad/s  ->  {s['pct_ok']:6.2f} % < 0  [{veredicto}]")
    s = stability(diag, win_track)
    print(f"  cond(M) max = {s['cond_M_max']:.1f} | margen de controlabilidad min = "
          f"{s['ctrl_margin_min']:.3e}")
    print(f"  CARE: residuo max = {s['residual_max']:.2e} | iteraciones max = "
          f"{s['iters_max']:.0f} | fallos = {s['fails']:.0f} | "
          f"ciclos que resuelven = {s['update_pct']:.1f} %")

    print(f"\n=== 2) Tiempo de computo (presupuesto {budget_us:.0f} us "
          f"= 1/{args.control_rate:.0f} Hz) ===")
    hdr = f"{'bloque':<10}{'media':>10}{'p95':>10}{'p99':>10}{'max':>10}{'>presup %':>11}"
    print(hdr)
    print("-" * len(hdr))
    for key, v in timing(diag, budget_us, win_track).items():
        print(f"{key:<10}{v['mean']:10.1f}{v['p95']:10.1f}{v['p99']:10.1f}"
              f"{v['max']:10.1f}{v['over_pct']:11.2f}")
    print("  (us; 'CARE' solo sobre los ciclos que efectivamente resolvieron)")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        t = diag["t_rel"]
        m = (t >= win_track[0]) & (t <= win_track[1])
        fig, ax = plt.subplots(figsize=(9, 3.6))
        ax.plot(t[m], diag["max_re_eig"][m], lw=0.8)
        ax.axhline(0.0, color="r", ls="--", lw=1.0)
        ax.axvspan(*CUT_PLATEAU, color="0.85", zorder=0, label="meseta del corte")
        ax.set_xlabel("t desde el inicio de TRACK [s]")
        ax.set_ylabel(r"max Re$(\lambda(A-BK))$  [rad/s]")
        ax.set_title("LQR-SDRE: estabilidad del esquema congelado en la incision")
        ax.legend(loc="lower right", fontsize=8)
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=150)
        print(f"\nfigura: {args.plot}")


if __name__ == "__main__":
    main()
