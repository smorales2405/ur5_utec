#!/usr/bin/env python3
"""
Constante corriente->par `k` a partir de una corrida de CONTROL DE PAR.

    k_i = media(tau_phys_i) / media(cur_i)      [N·m/A]

Sin URDF, sin gravedad y sin modelo: `tau_phys` es el par que la junta entrega
de verdad (comandado mas la gravedad que pone el robot, G3) y `cur` es el campo
`effort` de /joint_states, que en el UR5e es CORRIENTE de motor (G5). Los dos
estan en la misma fila del CSV unificado.

Por que hace falta otra via
---------------------------
`calibrate_current.py` saca `k` de la SUMA entre sentidos, que vale g(q)+C·v y
la conoce el modelo. Eso exige que la gravedad cargue la junta, y hay tres donde
no lo hace en ninguna postura (docs/02_friction_real.md 1.2). `wrist_3` es el
caso duro: su eje es el de la herramienta, asi que NUNCA tiene par gravitatorio
y no hay multipostura que lo arregle.

Esta via no depende de la gravedad, asi que sirve para las seis. Y como no usa
el URDF, tambien zanja la anomalia de `wrist_1` (6): con el metodo de gravedad
no se puede distinguir una `k` distinta de un error de masas del modelo, porque
un error de masas se va entero a `k`.

Autovalidacion en Gazebo
------------------------
Alli `cur` es el esfuerzo articular en N·m, la MISMA magnitud que `tau_phys`, asi
que `k` tiene que salir 1.0. Si no sale, el problema esta en esta cadena y no en
el robot. Uselo antes de gastar tiempo de robot.

Uso
---
    ros2 run ur5_identification calibrate_k_from_torque.py \\
        --csv ~/.ros/ur5_dyn_control/fl_500.csv --joint 5
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np


def load(path):
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


def plateaus(d):
    """Mesetas de velocidad constante que etiqueta el generador de barrido."""
    st = np.asarray(d["state"], dtype=str)
    niveles = sorted({s.rsplit("_", 1)[0] for s in st
                      if s.startswith("SWEEP_") and
                      s.rsplit("_", 1)[-1] in ("POS", "NEG")})
    out = []
    for lv in niveles:
        for signo in ("POS", "NEG"):
            m = st == f"{lv}_{signo}"
            if m.sum() >= 20:
                out.append((f"{lv}_{signo}", m))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--csv", required=True)
    ap.add_argument("--joint", type=int, required=True, choices=range(6))
    ap.add_argument("--min-cur", type=float, default=0.02,
                    help="corriente minima [A] para que una meseta cuente: por "
                         "debajo, k = tau/cur amplifica el ruido sin limite")
    args = ap.parse_args()

    d = load(args.csv)
    j = args.joint
    if f"cur{j+1}" not in d.dtype.names:
        print("El CSV no tiene columnas `cur`: es anterior a que el nodo de par "
              "registrara /joint_states.effort. Hay que repetir la corrida.")
        return 1
    cur = d[f"cur{j+1}"]
    if np.all(np.isnan(cur)):
        print("La columna de corriente es NaN: el driver no publica `effort`.")
        return 1

    ventanas = plateaus(d)
    if not ventanas:
        # Sin barrido etiquetado se cae a la fase de seguimiento entera.
        st = np.asarray(d["state"], dtype=str)
        ventanas = [("TRACK", st == "TRACK")]

    print(f"Junta {j} — k = media(tau_phys)/media(cur) sobre {len(ventanas)} ventanas\n")
    print(f"{'ventana':>18}{'n':>7}{'tau_phys':>11}{'cur':>10}{'k':>10}")
    print("-" * 56)
    ks, pesos = [], []
    for nombre, m in ventanas:
        t = d[f"tau{j+1}_phys"][m]
        c = cur[m]
        ok = np.isfinite(t) & np.isfinite(c)
        if ok.sum() < 20:
            continue
        tm, cm = t[ok].mean(), c[ok].mean()
        if abs(cm) < args.min_cur:
            print(f"{nombre:>18}{ok.sum():7d}{tm:11.4f}{cm:10.4f}"
                  f"{'—':>10}   (|cur| < {args.min_cur})")
            continue
        k = tm / cm
        ks.append(k)
        pesos.append(abs(cm))
        print(f"{nombre:>18}{ok.sum():7d}{tm:11.4f}{cm:10.4f}{k:10.4f}")

    if not ks:
        print("\nNinguna ventana con corriente suficiente. Suba la velocidad del "
              "barrido o baje --min-cur asumiendo mas ruido.")
        return 2

    ks = np.asarray(ks)
    pesos = np.asarray(pesos)
    # Ponderada por |cur|: las mesetas lentas tienen mucho peor relacion
    # senal/ruido y no deben pesar igual que las rapidas.
    k_med = float(np.average(ks, weights=pesos))
    disp = float(ks.std(ddof=1)) if len(ks) > 1 else float("nan")
    print(f"\n  k = {k_med:.4f} N·m/A   (ponderada por |cur|)")
    print(f"  dispersion entre ventanas = {disp:.4f}  ({100 * disp / abs(k_med):.2f} %)")
    if abs(k_med - 1.0) < 0.05 and disp < 0.05:
        print("\n  k ~ 1.0: esto es una corrida de GAZEBO, donde `cur` ya esta en")
        print("  N·m. Confirma que la cadena tau_phys/cur funciona; la `k` de")
        print("  verdad solo sale del robot real.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
