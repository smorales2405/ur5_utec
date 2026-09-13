#!/usr/bin/env python3
"""
Detecta una carga FANTASMA en el pendant del UR5e a partir de un CSV de par.

El UR5e compensa la gravedad por dentro con el payload configurado en el
pendant (G3: `gravity_in_command = false`). Si ese payload no coincide con lo
que hay en la brida, el robot empuja con un par que no corresponde a nada
fisico, y lo absorben nuestro `tau_cmd` y la friccion estatica. Paso de verdad
(docs/02_friction_real.md 8.7): el payload se puso a 0 para la campana, volvio
solo al valor por defecto de la instalacion (1.068 kg) y nadie lo vio en un mes.

Lo que se mira: en las juntas QUIETAS en q_init, lo que el robot ANADE al
comando,

    g_robot = cur * k - tau_cmd        [N.m]

comparado con lo medido el 2026-09-10 con la brida desnuda y payload 0
(`smc_712`..`smc_718`, identico en las siete corridas). Una masa de 1.068 kg en
la brida mueve shoulder_lift ~5-6 N.m y wrist_1 ~1 N.m.

Salvedad: con la junta quieta la friccion estatica puede absorber hasta +-F_c
(7 N.m en las juntas grandes), asi que la separacion no es limpia en teoria.
En la practica ha sido nitida en las 14 corridas en modo par revisadas
(-23.5..-26.2 con fantasma frente a -19.0 sin el, en shoulder_lift).

Uso:
    check_pendant_payload.py ~/.ros/ur5_dyn_control/smc_719.csv
"""

import argparse
import os
import sys

import numpy as np

#: k medida por diferencia entre sentidos (docs/02_friction_real.md 8.4) [N.m/A]
K = np.array([11.139, 11.124, 11.001, 8.666, 8.542, 8.644])
Q_INIT = np.array([1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 3.1416])
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]

#: g_robot sin carga en q_init, MEDIDO (2026-09-10, brida desnuda, payload 0).
#: Solo las juntas que la gravedad carga en esa pose y que estaban quietas.
REF_SIN_CARGA = {1: -19.02, 3: -1.70}
#: Umbral de aviso: la mitad de la firma de 1.068 kg.
UMBRAL = {1: 2.5, 3: 0.5}


def load(path):
    hdr, rows = None, []
    with open(path, errors="replace") as f:
        for line in f:
            if line.startswith("#"):
                continue
            if hdr is None:
                hdr = line.strip().split(",")
                continue
            rows.append(line)
    d = np.genfromtxt(rows, delimiter=",", usecols=range(len(hdr) - 1),
                      invalid_raise=False)
    return d, {n: i for i, n in enumerate(hdr)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+")
    a = ap.parse_args(argv)
    rc = 0
    for p in a.csv:
        d, i = load(p)
        print(f"\n=== {os.path.basename(p)} ===")
        if "cur1" not in i:
            print("  sin columnas `cur`: CSV anterior a G5, no se puede comprobar")
            continue
        q = d[:, [i[f"q{j}"] for j in range(1, 7)]]
        dq = d[:, [i[f"dq{j}"] for j in range(1, 7)]]
        print("  junta          g_robot   sin carga   diferencia")
        sospecha = False
        for j in (1, 3):
            quieta = (np.abs(q[:, j] - Q_INIT[j]) < 0.05) & (np.abs(dq[:, j]) < 1e-3)
            if quieta.sum() < 100:
                print(f"  {JOINTS[j]:<14} (se movia: no se evalua)")
                continue
            g = d[quieta, i[f"cur{j + 1}"]] * K[j] - d[quieta, i[f"tau{j + 1}"]]
            g = float(np.nanmedian(g))
            delta = g - REF_SIN_CARGA[j]
            aviso = "  <-- CARGA FANTASMA?" if abs(delta) > UMBRAL[j] else ""
            print(f"  {JOINTS[j]:<14} {g:+8.2f}   {REF_SIN_CARGA[j]:+8.2f}   "
                  f"{delta:+7.2f}{aviso}")
            if aviso:
                sospecha = True
        if sospecha:
            rc = 1
            print("  -> revisa el payload del pendant: Instalacion > Payload. "
                  "Vuelve solo al valor por defecto al recargar el programa.")
    return rc


if __name__ == "__main__":
    sys.exit(main())
