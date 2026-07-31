#!/usr/bin/env python3
"""
Métricas del SMC para la FASE 5: precisión, chattering y espectro.

Distingue REGIMEN de TRANSITORIO. El criterio del plan —"con `sat`, ‖s‖ acotada
por O(φ) en régimen"— no se puede evaluar sobre toda la fase TRACK: la
trayectoria de incisión encadena cinco fases con reposos entre ellas, y en cada
arranque `s` salta porque la referencia acelera desde cero. Ese pico es un
transitorio de alcance, no el régimen deslizante. Por eso se reporta por
separado la ventana de la MESETA del corte, que es donde el robot está en
régimen (feed constante, aceleración nula) y donde además se miden las métricas
del paper.

Métricas (definiciones únicas, las mismas que usará la FASE 10):
  RMSE articular y cartesiano, max |s|, TV(tau) = sum |Δtau|, RMS(Δtau),
  y el espectro de tau (FFT) para el contenido de chattering.
"""

import argparse
import os

import numpy as np

# Ventana de MESETA del corte, relativa al inicio de TRACK [s]. Sale de la
# trayectoria de incisión: fase `cut` en [13.90, 25.90] con meseta de feed
# constante en [15.90, 23.90] (ver docs/01_trajectory.md).
CUT_PLATEAU = (15.90, 23.90)


def load(path):
    n_meta = 0
    with open(path) as fh:
        for line in fh:
            if line.startswith("#"):
                n_meta += 1
            else:
                break
    d = np.genfromtxt(path, delimiter=",", names=True, dtype=None,
                      encoding="utf-8", skip_header=n_meta, invalid_raise=False)
    m = np.char.startswith(d["state"].astype(str), "TRACK")
    col = lambda p: np.column_stack([d[p % i][m] for i in range(1, 7)])  # noqa: E731
    t = d["t_sim"][m]
    return {"t": t - t[0], "q": col("q%d"), "q_des": col("q%d_des"),
            "tau": col("tau%d"), "s": col("s%d"),
            "xyz": np.column_stack([d[k][m] for k in ("x", "y", "z")]),
            "xyz_des": np.column_stack([d[k][m] for k in ("x_des", "y_des", "z_des")]),
            "sat": col("tau%d_sat")}


def metrics(d, window=None):
    m = np.ones(len(d["t"]), bool) if window is None else \
        (d["t"] >= window[0]) & (d["t"] <= window[1])
    e = d["q"][m] - d["q_des"][m]
    ep = np.linalg.norm(d["xyz"][m] - d["xyz_des"][m], axis=1)
    tau = d["tau"][m]
    dtau = np.diff(tau, axis=0)
    s = d["s"][m]
    return {
        "n": int(m.sum()),
        "rmse_q": float(np.sqrt((e ** 2).mean())),
        "max_q": float(np.abs(e).max()),
        "rmse_tcp_mm": float(1e3 * np.sqrt((ep ** 2).mean())),
        "s_max": float(np.abs(s).max()),
        "s_rms": float(np.sqrt((s ** 2).mean())),
        "TV": float(np.abs(dtau).sum()),
        "rms_dtau": float(np.sqrt((dtau ** 2).mean())),
        "tau_max": float(np.abs(tau).max()),
        "effort": float((tau ** 2).sum()),
        "sat_pct": float(100.0 * d["sat"][m].mean()),
    }


def spectrum(d, joint=1, window=CUT_PLATEAU, fs=500.0):
    """Espectro de tau en la meseta: dónde vive la energía del chattering."""
    m = (d["t"] >= window[0]) & (d["t"] <= window[1])
    x = d["tau"][m, joint]
    x = x - x.mean()
    n = len(x)
    if n < 64:
        return None
    f = np.fft.rfftfreq(n, 1.0 / fs)
    P = np.abs(np.fft.rfft(x * np.hanning(n))) ** 2
    P /= P.sum()
    # Fracción de energía por encima de 20 Hz: el chattering vive ahí, la
    # dinámica de la trayectoria (feed de 10 mm/s) está muy por debajo.
    hi = float(P[f > 20.0].sum())
    centroid = float((f * P).sum())
    return {"f": f, "P": P, "hi_frac": hi, "centroid": centroid}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", nargs="+", required=True)
    ap.add_argument("--labels", nargs="+", default=None)
    ap.add_argument("--phi", type=float, default=0.05)
    ap.add_argument("--joint", type=int, default=1, help="junta para el espectro")
    args, _ = ap.parse_known_args()
    labels = args.labels or [os.path.basename(c) for c in args.csv]

    hdr = f"{'corrida':<14}{'RMSE q':>10}{'TCP mm':>9}{'max|s|':>9}{'|s|/phi':>9}" \
          f"{'TV(tau)':>11}{'RMS dtau':>10}{'>20Hz %':>9}"
    for tag, window in (("TRACK COMPLETO (incluye transitorios de fase)", None),
                        ("MESETA DEL CORTE (regimen)", CUT_PLATEAU)):
        print(f"\n=== {tag} ===")
        print(hdr)
        print("-" * len(hdr))
        for path, lab in zip(args.csv, labels):
            if not os.path.exists(path):
                print(f"{lab:<14}  (sin datos)")
                continue
            d = load(path)
            r = metrics(d, window)
            sp = spectrum(d, args.joint) if window is not None else None
            hi = f"{100 * sp['hi_frac']:8.2f}" if sp else "       —"
            print(f"{lab:<14}{r['rmse_q']:10.6f}{r['rmse_tcp_mm']:9.4f}"
                  f"{r['s_max']:9.5f}{r['s_max'] / args.phi:9.2f}"
                  f"{r['TV']:11.0f}{r['rms_dtau']:10.4f}{hi:>9}")
    print(f"\n(criterio del plan: con sat, |s| = O(phi) en REGIMEN; phi = {args.phi})")


if __name__ == "__main__":
    main()
