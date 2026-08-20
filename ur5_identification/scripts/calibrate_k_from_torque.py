#!/usr/bin/env python3
"""
Constante corriente->par `k` a partir de una corrida de CONTROL DE PAR.

    k_i = [tau_phys(+v) - tau_phys(-v)] / [cur(+v) - cur(-v)]      [N·m/A]

Se RESTA entre sentidos, no se divide sin mas, y el motivo no es cosmetico:

    tau_phys = tau_cmd + g_NUESTRO(q)     <- lo que registra la columna
    par real = tau_cmd + g_UR(q)          <- lo que el robot entrega de verdad

El nodo calcula `tau_phys` con SU modelo (G3) mientras el robot compensa la
gravedad con el suyo, que no publican. El cociente crudo sale sesgado por el
cociente de las dos gravedades, asi que NO seria una medida independiente del
URDF. La gravedad es PAR en la velocidad, asi que restando los dos sentidos se
cancela en el numerador y en el denominador a la vez, y con ella toda dependencia
del modelo. Lo mismo hace `calibrate_current.py` con la SUMA para la gravedad.

`cur` es el campo `effort` de /joint_states, que en el UR5e es CORRIENTE de
motor (G5). Los dos estan en la misma fila del CSV unificado.

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


def plateau_pairs(d):
    """
    Pares (+v, -v) del MISMO nivel de velocidad.

    Se exige el par COMPLETO: una meseta suelta no sirve, porque todo el metodo
    se apoya en cancelar la gravedad comparando los dos sentidos.
    """
    st = np.asarray(d["state"], dtype=str)
    niveles = sorted({s.rsplit("_", 1)[0] for s in st
                      if s.startswith("SWEEP_") and
                      s.rsplit("_", 1)[-1] in ("POS", "NEG")})
    out = []
    for lv in niveles:
        m_p = st == f"{lv}_POS"
        m_n = st == f"{lv}_NEG"
        if m_p.sum() >= 20 and m_n.sum() >= 20:
            out.append((lv, m_p, m_n))
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

    pares = plateau_pairs(d)
    if not pares:
        print("No hay pares de mesetas (+v, -v). Este metodo los NECESITA: sin\n"
              "los dos sentidos no se puede cancelar la gravedad, y el cociente\n"
              "crudo quedaria sesgado por el desacuerdo entre nuestro URDF y el\n"
              "modelo interno de UR. Use trajectory_type:=joint_sweep.")
        return 2

    print(f"Junta {j} — k = dif(tau_phys) / dif(cur) sobre {len(pares)} niveles\n")
    print(f"{'nivel':>16}{'dif tau':>11}{'dif cur':>10}{'k':>10}{'|v|':>9}")
    print("-" * 58)
    ks, pesos = [], []
    for lv, m_p, m_n in pares:
        t_p = d[f"tau{j+1}_phys"][m_p]; t_n = d[f"tau{j+1}_phys"][m_n]
        c_p = cur[m_p]; c_n = cur[m_n]
        if not (np.isfinite(t_p).any() and np.isfinite(c_p).any() and
                np.isfinite(t_n).any() and np.isfinite(c_n).any()):
            continue
        dt_ = 0.5 * (np.nanmean(t_p) - np.nanmean(t_n))
        dc_ = 0.5 * (np.nanmean(c_p) - np.nanmean(c_n))
        v = 0.5 * (abs(np.nanmean(d[f"dq{j+1}"][m_p])) +
                   abs(np.nanmean(d[f"dq{j+1}"][m_n])))
        if abs(dc_) < args.min_cur:
            print(f"{lv:>16}{dt_:11.4f}{dc_:10.4f}{'—':>10}{v:9.4f}"
                  f"   (|dif cur| < {args.min_cur})")
            continue
        k = dt_ / dc_
        ks.append(k); pesos.append(abs(dc_))
        print(f"{lv:>16}{dt_:11.4f}{dc_:10.4f}{k:10.4f}{v:9.4f}")

    if not ks:
        print("\nNingun nivel con diferencia de corriente suficiente. Suba la\n"
              "velocidad del barrido o baje --min-cur asumiendo mas ruido.")
        return 2

    ks = np.asarray(ks); pesos = np.asarray(pesos)
    # Ponderada por |dif cur|: las mesetas lentas tienen mucho peor relacion
    # senal/ruido y no deben pesar igual que las rapidas.
    k_med = float(np.average(ks, weights=pesos))
    disp = float(ks.std(ddof=1)) if len(ks) > 1 else float("nan")
    rel = 100 * disp / abs(k_med) if len(ks) > 1 else float("nan")
    print(f"\n  k = {k_med:.4f} N·m/A   ({len(ks)} niveles, ponderada por |dif cur|)")
    print(f"  dispersion entre niveles = {disp:.4f}  ({rel:.2f} %)")
    # La DISPERSION es el criterio, no el valor: si `k` deriva con la velocidad,
    # la suposicion de fondo —que direct_torque entrega el par comandado— no se
    # sostiene, y el numero no vale aunque parezca razonable.
    if rel > 5.0:
        print("\n  AVISO: mas del 5 % de dispersion. Mire la columna |v|: si `k`\n"
              "  deriva con la velocidad, el par comandado no se esta entregando\n"
              "  tal cual y esta medida no es utilizable.")
    if abs(k_med - 1.0) < 0.05 and rel < 5.0:
        print("\n  k ~ 1.0: esto es una corrida de GAZEBO, donde `cur` ya esta en\n"
              "  N·m. Confirma que la cadena funciona; la `k` de verdad solo sale\n"
              "  del robot real.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
