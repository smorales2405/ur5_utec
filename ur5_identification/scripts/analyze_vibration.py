#!/usr/bin/env python3
"""
Indicadores de VIBRACION de una corrida, junta a junta, y ajuste de rampas.

Existe porque el error de seguimiento no ve un ciclo limite. En smc_712 el codo
oscilo a 35.6 Hz contra el tope de par —el brazo entero sacudiendose— con un
error de 0.03 a 0.2 GRADOS. Lo que si lo delata es el par.

1) POR CORRIDA (siempre):

    rizado   = RMS de |tau[k] - tau[k-1]| en una ventana, / el tope de la junta
    sat      = fraccion de ciclos con el comando recortado
    A_banda  = amplitud RMS de tau en cada banda de modo, y su pico

   Las bandas por defecto son los DOS modos que aparecen en el UR5e barriendo
   el codo (docs/09_real_bringup.md §6.4):

    20-35 Hz  el que se fue en smc_712 (25.5 Hz en su tramo sano, 35.7 al irse)
    40-55 Hz  uno secundario del codo que pierde amortiguamiento con su G

   Una sola banda ancha los mezcla, y una estrecha puede mirar el que no es:
   la primera version de este analisis ajusto el de 48 Hz creyendo que era el
   de 25.

   Para codo y shoulder_lift se imprime ademas el % respecto al nivel de
   smc_712 en los 14 s previos a irse (medido, ver REF_PRE_FALLO).

2) RAMPA (con --ramp-joint y --g): ajusta el LIMITE sin tener que alcanzarlo.

   - Si la junta rampeada es la que se MUEVE, su G inyecta ruido (∝ G) y ademas
     puede quitar amortiguamiento:  A ≈ c·G/(1 − G/Gc)  ->  G/A lineal en G.
   - Si esta QUIETA, no inyecta nada (sigma(q_punto) = 0 exacto) y solo puede
     quitar amortiguamiento:        A ≈ c/(1 − G/Gc)    ->  1/A lineal en G.

   La recta corta cero en Gc. Si la pendiente sale positiva o r es pobre, NO
   hay perdida de amortiguamiento medible y Gc no se reporta como limite.

Uso:
    analyze_vibration.py smc_718.csv
    analyze_vibration.py smc_719.csv smc_720.csv smc_721.csv \\
        --ramp-joint 1 --sweep-joint 2 --g 86.2,119.8,150.9
"""

import argparse
import os
import sys

import numpy as np

#: Par maximo nominal del UR5e [N.m]; el tope real es este por `tau_scale`.
TAU_MAX_NOMINAL = np.array([150.0, 150.0, 150.0, 28.0, 28.0, 28.0])

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]

#: diag M(q_init) [kg m^2] y cuanto del encoder (2^20 cuentas/vuelta). Sirven
#: para traducir la amplitud de PAR del modo a amplitud de POSICION en cada
#: junta, x = A / (M_ii * w^2), y compararla con una cuenta: una junta quieta no
#: puede cerrar ningun lazo sobre un modo que su encoder no resuelve
#: (docs/09_real_bringup.md 6.6). Es la razon de que su G no cuente hasta que
#: la junta que se mueve haya hecho crecer el modo.
INERTIA_Q_INIT = np.array([1.0582, 2.5914, 0.8815, 0.02324, 0.00535, 0.00026])
ENCODER_LSB = 2.0 * np.pi / 2 ** 20      # 5.99e-6 rad nominal; 5.72e-6 medido

#: Amplitud RMS de tau [N.m] en smc_712, t = 60-74 s: los 14 s ANTERIORES a que
#: el brazo entrase en ciclo limite. Por junta (indice 0..5) y banda.
REF_PRE_FALLO = {
    (20.0, 35.0): {1: 0.089, 2: 0.081},
    (40.0, 55.0): {1: 0.061, 2: 0.071},
}


def load(path):
    hdr, rows, meta = None, [], {}
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                if "=" in line:
                    k, v = line[1:].strip().split("=", 1)
                    meta[k.strip()] = v.strip()
                continue
            if hdr is None:
                hdr = line.strip().split(",")
                continue
            rows.append(line)
    # La ultima columna (`state`) es texto: fuera.
    d = np.genfromtxt(rows, delimiter=",", usecols=range(len(hdr) - 1))
    return d, {n: i for i, n in enumerate(hdr)}, meta


def rolling_mean(x, n):
    c = np.cumsum(np.insert(np.asarray(x, float), 0, 0.0))
    return (c[n:] - c[:-n]) / n


def band_rms(x, dt, f1, f2):
    """Amplitud RMS de x en [f1, f2] Hz (Parseval, corregido por la Hanning)."""
    x = x - np.mean(x)
    n = len(x)
    w = np.hanning(n)
    P = np.abs(np.fft.rfft(x * w)) ** 2
    f = np.fft.rfftfreq(n, dt)
    b = (f >= f1) & (f <= f2)
    if not b.any():
        return 0.0, 0.0
    return (float(np.sqrt(2.0 * P[b].sum() / (n ** 2 * np.mean(w ** 2)))),
            float(f[b][np.argmax(P[b])]))


def analyze(path, args):
    d, i, meta = load(path)
    t = d[:, i["t_wall"]] - d[0, i["t_wall"]]
    m = (t >= args.skip) & (t < (args.until if args.until else np.inf))
    dt = float(np.median(np.diff(t)))
    n = max(1, int(round(args.window / dt)))

    print(f"\n=== {os.path.basename(path)} "
          f"(test {meta.get('test_num', '?')}, git {meta.get('git_sha', '?')[:7]}) ===")
    print(f"  {m.sum()} ciclos analizados, t = {args.skip:.0f}.."
          f"{(args.until or t[-1]):.0f} s, ventana {args.window:.2f} s")

    tope = TAU_MAX_NOMINAL * args.tau_scale
    cab = "".join(f"   A {int(f1)}-{int(f2)} Hz        " for f1, f2 in args.bands)
    print(f"\n  junta          rizado/tope  max sat {cab}  x/LSB({int(args.bands[0][0])}-{int(args.bands[0][1])})")
    amps = {}
    for j in range(6):
        if args.joint is not None and j != args.joint:
            continue
        tau = d[m, i[f"tau{j + 1}"]]
        sat = d[m, i[f"tau{j + 1}_sat"]]
        rip = rolling_mean(np.diff(tau, prepend=tau[0]) ** 2, n) ** 0.5
        sf = rolling_mean(sat, n)
        r = float(rip.max()) / tope[j]

        cols = ""
        for f1, f2 in args.bands:
            a, fp = band_rms(tau, dt, f1, f2)
            amps[(j, f1, f2)] = a
            ref = REF_PRE_FALLO.get((f1, f2), {}).get(j)
            pct = f"{100 * a / ref:4.0f}%" if ref else "     "
            cols += f"  {a:6.3f}@{fp:4.1f} {pct}  "
        # Visibilidad del modo principal en ESTA junta, en cuentas de encoder.
        f1, f2 = args.bands[0]
        a0, fp0 = band_rms(tau, dt, f1, f2)
        xlsb = a0 / (INERTIA_Q_INIT[j] * (2 * np.pi * max(fp0, 1.0)) ** 2) / ENCODER_LSB
        marca = "  <-- VIBRACION" if r > 0.05 or sf.max() > 0.10 else ""
        print(f"  {JOINTS[j]:<14} {100 * r:7.2f} % {sf.max():7.3f} {cols} {xlsb:6.2f}{marca}")
    print("  (% = respecto a smc_712 en los 14 s previos a irse; x/LSB = amplitud "
          "del modo en cuentas de encoder, RMS)")
    return amps


def ramp_fit(all_amps, args):
    g = np.asarray(args.g, float)
    if len(g) != len(all_amps):
        sys.exit(f"--g tiene {len(g)} valores y hay {len(all_amps)} CSV")
    if len(g) < 3:
        print("\n  (rampa: hacen falta >= 3 corridas para ajustar)")
        return
    moving = args.ramp_joint == args.sweep_joint
    forma = "G/A" if moving else "1/A"
    print(f"\n=== RAMPA sobre {JOINTS[args.ramp_joint]} "
          f"({'la que se MUEVE' if moving else 'QUIETA'}): recta {forma} frente a G ===")
    gcs = []
    for f1, f2 in args.bands:
        for j in (1, 2):
            a = np.array([amp[(j, f1, f2)] for amp in all_amps])
            y = g / a if moving else 1.0 / a
            p = np.polyfit(g, y, 1)
            r = float(np.corrcoef(g, y)[0, 1])
            if p[0] < 0 and r < -0.8:
                gc = -p[1] / p[0]
                gcs.append(gc)
                txt = f"Gc = {gc:6.1f}   (r {r:+.3f})"
            else:
                txt = (f"sin perdida de amortiguamiento medible "
                       f"(pendiente {p[0]:+.2e}, r {r:+.3f})")
            serie = " ".join(f"{v:.3f}" for v in a)
            print(f"  {int(f1)}-{int(f2)} Hz  {JOINTS[j]:<14} A = [{serie}]  ->  {txt}")
    if gcs:
        gmin = min(gcs)
        print(f"\n  Gc mas bajo: {gmin:.1f}.  Siguiente escalon permitido si "
              f"G < 0.8·Gc = {0.8 * gmin:.1f}")
    else:
        print("\n  Ninguna banda pierde amortiguamiento con esta G: la junta "
              "rampeada no es la que desestabiliza estos modos.")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--window", type=float, default=0.2,
                    help="ventana del rizado y de la fraccion saturada [s]")
    ap.add_argument("--skip", type=float, default=6.0,
                    help="segundos iniciales que se ignoran (HOLD + rampa)")
    ap.add_argument("--until", type=float, default=None,
                    help="segundo final analizado (p. ej. antes de un paro)")
    ap.add_argument("--tau-scale", type=float, default=0.30,
                    help="el que llevaba la corrida, para normalizar el rizado")
    ap.add_argument("--joint", type=int, default=None, help="solo esta junta (0..5)")
    ap.add_argument("--bands", type=str, default="20-35,40-55",
                    help="bandas de modo en Hz, p. ej. '20-35,40-55'")
    ap.add_argument("--ramp-joint", type=int, default=None,
                    help="junta cuya lambda se rampeo (0..5)")
    ap.add_argument("--sweep-joint", type=int, default=2,
                    help="junta que se movia en el barrido (0..5)")
    ap.add_argument("--g", type=str, default=None,
                    help="G de la junta rampeada en cada CSV, en orden")
    a = ap.parse_args(argv)
    a.bands = [tuple(float(x) for x in b.split("-")) for b in a.bands.split(",")]
    if (a.ramp_joint is None) != (a.g is None):
        ap.error("--ramp-joint y --g van juntos")
    if a.g:
        a.g = [float(x) for x in a.g.replace(",", " ").split()]

    all_amps = [analyze(p, a) for p in a.csv]
    if a.ramp_joint is not None:
        ramp_fit(all_amps, a)
    return 0


if __name__ == "__main__":
    sys.exit(main())
