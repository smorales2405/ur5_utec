#!/usr/bin/env python3
"""
Comprueba, a partir de un CSV de par del UR5e real, dos ajustes del robot que no
se ven en ningun log y que ya han pasado desapercibidos:

1. PAYLOAD DEL PENDANT (docs/02_friction_real.md 8.7). El robot compensa por
   dentro la gravedad del payload configurado (G3). Volvio solo a 1.068 kg al
   recargar el programa y asi estuvo un mes. Se mira en las juntas QUIETAS en
   q_init lo que el robot anade al comando,

       g_robot = cur*k - tau_cmd

   contra lo medido con la brida desnuda y payload 0 (smc_712..718). 1.068 kg en
   la brida mueven shoulder_lift ~5-6 N.m y wrist_1 ~1 N.m.

2. G4, COMPENSACION INTERNA DE FRICCION (docs/00_prereqs.md G4: se opera a 0.0,
   fijado por servicio en cada sesion). Si no se llama al servicio, el robot
   usa sus escalas por defecto y SUMA su propia compensacion a nuestro
   feedforward. Paso el 2026-09-12 (smc_790, 719-723). Se mira en la junta
   BARRIDA la parte IMPAR en la velocidad de g_robot, por diferencia entre
   sentidos a q emparejada: la gravedad (y un payload fantasma) es par en v y
   se cancela; lo que queda es friccion que anade el robot. Medido en el codo:

       G4 = 0        |impar| <= 0.03 N.m   (fl_502, smc_713..718)
       G4 default    6.2 / 7.1 / 8.8 N.m a 0.05 / 0.2 / 0.5 rad/s  (smc_719..723)

Uso:
    check_session_config.py ~/.ros/ur5_dyn_control/smc_<n>.csv
Sale con codigo 1 si algun ajuste no es el esperado.
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
REF_SIN_CARGA = {1: -19.02, 3: -1.70}
#: Aviso de payload: la mitad de la firma de 1.068 kg.
UMBRAL_PAYLOAD = {1: 2.5, 3: 0.5}
#: F_c fisica medida (02 8.4). El aviso de G4 salta con una parte impar mayor
#: que el 15 % de ella: con G4 = 0 se mide <= 0.03 N.m en el codo, con las
#: escalas por defecto 6.2 N.m ya a 0.05 rad/s.
F_C = np.array([6.85, 6.76, 7.46, 1.78, 1.96, 2.35])
FRAC_G4 = 0.15


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


def odd_part(q, v, y, level, tol=0.005, bin_rad=np.radians(1.0)):
    """Media de [y(+v) - y(-v)]/2 a q emparejada en bins de 1 grado."""
    ok = np.isfinite(q) & np.isfinite(v) & np.isfinite(y)
    mp = ok & (np.abs(v - level) < tol)
    mn = ok & (np.abs(v + level) < tol)
    if mp.sum() < 200 or mn.sum() < 200:
        return None
    lo, hi = max(q[mp].min(), q[mn].min()), min(q[mp].max(), q[mn].max())
    a, b = [], []
    for b0 in np.arange(lo, hi - bin_rad, bin_rad):
        s = (q >= b0) & (q < b0 + bin_rad)
        if (s & mp).sum() > 5 and (s & mn).sum() > 5:
            a.append(np.median(y[s & mp]))
            b.append(np.median(y[s & mn]))
    return 0.5 * (np.mean(a) - np.mean(b)) if len(a) > 10 else None


def check_payload(d, i):
    q = d[:, [i[f"q{j}"] for j in range(1, 7)]]
    dq = d[:, [i[f"dq{j}"] for j in range(1, 7)]]
    ok = True
    print("  PAYLOAD  junta          g_robot   sin carga   diferencia")
    for j in (1, 3):
        quieta = (np.abs(q[:, j] - Q_INIT[j]) < 0.05) & (np.abs(dq[:, j]) < 1e-3)
        if quieta.sum() < 100:
            print(f"           {JOINTS[j]:<14} (se movia: no se evalua)")
            continue
        g = float(np.nanmedian(d[quieta, i[f"cur{j + 1}"]] * K[j]
                               - d[quieta, i[f"tau{j + 1}"]]))
        delta = g - REF_SIN_CARGA[j]
        mal = abs(delta) > UMBRAL_PAYLOAD[j]
        ok &= not mal
        print(f"           {JOINTS[j]:<14} {g:+8.2f}   {REF_SIN_CARGA[j]:+8.2f}   "
              f"{delta:+7.2f}{'  <-- CARGA FANTASMA?' if mal else ''}")
    if not ok:
        print("           -> pendant: Instalacion > Payload. Vuelve solo al "
              "valor por defecto al recargar el programa.")
    return ok


def check_g4(d, i):
    q = d[:, [i[f"q{j}"] for j in range(1, 7)]]
    sw = int(np.argmax(np.nanmax(q, 0) - np.nanmin(q, 0)))
    if f"dq{sw + 1}_des" not in i:
        print("  G4       sin dq_des: no se puede evaluar")
        return True
    v = d[:, i[f"dq{sw + 1}_des"]]
    r = d[:, i[f"cur{sw + 1}"]] * K[sw] - d[:, i[f"tau{sw + 1}"]]
    levels = sorted({round(abs(x), 2) for x in v[np.isfinite(v)] if abs(x) > 0.02})
    umbral = FRAC_G4 * F_C[sw]
    vals = [(lv, odd_part(q[:, sw], v, r, lv)) for lv in levels]
    vals = [(lv, x) for lv, x in vals if x is not None]
    if not vals:
        print(f"  G4       {JOINTS[sw]}: sin mesetas en los dos sentidos, no se evalua")
        return True
    peor = max(abs(x) for _, x in vals)
    mal = peor > umbral
    txt = "  ".join(f"{x:+.2f}@{lv:.2f}" for lv, x in vals)
    print(f"  G4       {JOINTS[sw]} (barrida): friccion que anade el robot "
          f"[N.m @ rad/s]  {txt}")
    print(f"           umbral {umbral:.2f}{'  <-- COMPENSACION INTERNA ACTIVA' if mal else '  -> G4 = 0'}")
    if mal:
        print("           -> no se llamo a set_friction_model_parameters con "
              "escalas 0: nuestro feedforward y el del robot se SUMAN.")
    return not mal


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
        ok = check_payload(d, i)
        ok &= check_g4(d, i)
        rc |= 0 if ok else 1
    return rc


if __name__ == "__main__":
    sys.exit(main())
