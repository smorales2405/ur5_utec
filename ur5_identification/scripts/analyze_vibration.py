#!/usr/bin/env python3
"""
Indicadores de VIBRACION de una corrida, junta a junta.

Existe porque el error de seguimiento no ve un ciclo limite. En smc_712 el codo
oscilo a 35.6 Hz contra el tope de par —el brazo entero sacudiendose— con un
error de 0.03 a 0.2 GRADOS. Lo que si lo delata es el par:

    rizado   = RMS de |tau[k] - tau[k-1]| en una ventana, / el tope de la junta
    sat      = fraccion de ciclos con el comando recortado
    f_dom    = frecuencia dominante del espectro de tau
    E>20Hz   = fraccion de la energia de tau por encima de 20 Hz

Medido sobre smc_712 (ventana de 0.2 s):

    tramo          rizado/tope   sat        f_dom     E>20Hz
    sano (69 s)    <= 0.97 %     0.000        --      --
    el fallo       46 - 69 %     0.66-0.91   35.6 Hz  99.7 %

Uso:
    analyze_vibration.py ~/.ros/ur5_dyn_control/smc_712.csv
    analyze_vibration.py smc_74*.csv --joint 2 --lambda-joint 20,20,45,20,20,20

Con `--lambda-joint` ademas imprime la ganancia derivativa D = M_ii*lambda_i que
esa corrida llevaba, que es la variable que gobierna el fenomeno.
"""

import argparse
import os
import sys

import numpy as np

#: Par maximo nominal del UR5e [N.m]; el tope real es este por `tau_scale`.
TAU_MAX_NOMINAL = np.array([150.0, 150.0, 150.0, 28.0, 28.0, 28.0])

#: diag M(q_init) en la pose del barrido [kg m^2] (Pinocchio, URDF sin
#: herramienta). M_jj no depende de la propia junta j, solo de las distales,
#: asi que es CONSTANTE a lo largo del barrido de j.
INERTIA_SWEEP = np.array([1.0582, 2.5914, 0.8815, 0.02324, 0.00535, 0.00026])

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]


def load(path):
    hdr, rows = None, []
    meta = {}
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
    d = np.genfromtxt(rows, delimiter=",")
    return d, {n: i for i, n in enumerate(hdr)}, meta


def rolling_mean(x, n):
    c = np.cumsum(np.insert(np.asarray(x, float), 0, 0.0))
    return (c[n:] - c[:-n]) / n


def analyze(path, args):
    d, i, meta = load(path)
    t = d[:, i["t_wall"]] - d[0, i["t_wall"]]
    # Se descarta el arranque: HOLD + rampa saturan de forma legitima.
    t0 = args.skip
    n = max(1, int(round(args.window / np.median(np.diff(t)))))

    print(f"\n=== {os.path.basename(path)} "
          f"(test {meta.get('test_num', '?')}, git {meta.get('git_sha', '?')[:7]}) ===")
    print(f"  {len(t)} ciclos, {t[-1]:.1f} s, ventana {args.window:.2f} s "
          f"({n} ciclos), se ignoran los primeros {t0:.0f} s")
    if args.lambda_joint is not None:
        D = INERTIA_SWEEP * args.lambda_joint
        print("  M_ii*lambda_i [N.m por rad/s]: "
              + " ".join(f"{v:.1f}" for v in D))
        print("  OJO: es solo la mitad del control EQUIVALENTE. La ganancia "
              "derivativa completa es")
        print("  G = M_ii*lambda_i + K_i/phi_i, y con phi pequena el segundo "
              "termino domina (docs/09_real_bringup.md §2).")

    tope = TAU_MAX_NOMINAL * args.tau_scale
    print("\n  junta          rizado/tope   max sat    f_dom    E>20Hz   |e|max")
    peor = 0.0
    for j in range(6):
        if args.joint is not None and j != args.joint:
            continue
        tau = d[:, i[f"tau{j + 1}"]]
        sat = d[:, i[f"tau{j + 1}_sat"]]
        e = d[:, i[f"q{j + 1}"]] - d[:, i[f"q{j + 1}_des"]]
        m = t >= t0
        rip = rolling_mean(np.diff(tau, prepend=tau[0])[m] ** 2, n) ** 0.5
        sf = rolling_mean(sat[m], n)
        r = float(rip.max()) / tope[j]
        peor = max(peor, r)

        # El espectro se toma en el tramo de rizado MAXIMO, no en toda la
        # corrida: si el ensayo acaba en paro de emergencia, la rampa lenta de
        # despues es continua (DC) y entierra la oscilacion. Sobre smc_712, con
        # toda la corrida el codo daba 4 % de energia >20 Hz; sobre el tramo que
        # de verdad vibro, 99.7 %.
        dt = float(np.median(np.diff(t)))
        tau_m = tau[m]
        k = int(np.argmax(rip))
        lo = max(0, k - 2 * n)
        seg = tau_m[lo:lo + 4 * n]
        x = seg - np.mean(seg)
        P = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
        f = np.fft.rfftfreq(len(x), dt)
        fd = f[np.argmax(P[1:]) + 1] if len(P) > 1 else 0.0
        hf = P[f > 20].sum() / max(P.sum(), 1e-300)

        marca = "  <-- VIBRACION" if r > 0.05 or sf.max() > 0.10 else ""
        print(f"  {JOINTS[j]:<14} {100 * r:8.2f} %  {sf.max():7.3f} "
              f"{fd:8.1f} Hz {100 * hf:7.1f} % {np.degrees(np.abs(e[m]).max()):7.2f} deg"
              f"{marca}")
    return peor


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--window", type=float, default=0.2,
                    help="ventana del rizado y de la fraccion saturada [s]")
    ap.add_argument("--skip", type=float, default=6.0,
                    help="segundos iniciales que se ignoran (HOLD + rampa)")
    ap.add_argument("--tau-scale", type=float, default=0.30,
                    help="el que llevaba la corrida, para normalizar el rizado")
    ap.add_argument("--joint", type=int, default=None, help="solo esta junta (0..5)")
    ap.add_argument("--lambda-joint", type=str, default=None,
                    help="6 lambdas coma-separadas, para imprimir D = M*lambda")
    a = ap.parse_args(argv)
    if a.lambda_joint:
        v = [float(x) for x in a.lambda_joint.replace(",", " ").split()]
        if len(v) != 6:
            ap.error("--lambda-joint necesita 6 valores")
        a.lambda_joint = np.array(v)
    for p in a.csv:
        analyze(p, a)
    return 0


if __name__ == "__main__":
    sys.exit(main())
