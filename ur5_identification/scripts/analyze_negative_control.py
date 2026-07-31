#!/usr/bin/env python3
"""
Análisis del CONTROL NEGATIVO: campaña de barridos sobre una planta SIN
fricción (FASE 2).

No identifica nada. Verifica que el residuo `tau_cmd - RNEA(q,q̇,q̈)` es cero
junta por junta, y cuando no lo es, cuantifica por qué.

Lo que este análisis destapó: en las juntas cargadas por gravedad aparece un
`F_v` aparente que NO es fricción, sino un desalineo temporal de ~1.5 ms entre
el estado con el que se evalúa RNEA y el par que actúa sobre la planta. La firma
es `(dg/dq)·lag·q̇`, proporcional a la velocidad y por tanto indistinguible de
fricción viscosa. Este script mide `dg/dq` por junta y comprueba que el `F_v`
aparente escala con él.

Uso:
  ros2 run ur5_identification analyze_negative_control.py --base 300
  ros2 run ur5_identification analyze_negative_control.py --base 300 --shift -1
"""

import argparse
import os

import numpy as np
import pinocchio as pin

from ur5_identification.estimator import fit
from ur5_identification.residual import (JOINT_NAMES, compute_residual,
                                         default_urdf, extract_windows)

Q_INIT = np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0])


def gravity_gradient(urdf: str, joint: int, amplitude: float, gravity: float) -> float:
    """
    Pendiente de CUERDA de g(q) sobre el rango barrido [N·m/rad], CON SIGNO.

    El artefacto es `(dg/dq)·lag·q̇`, y el residuo se promedia sobre la meseta,
    así que el predictor correcto es la pendiente media CON SIGNO —es decir la
    cuerda `[g(q_max) − g(q_min)] / (q_max − q_min)`— no `|dg/dq|` medio. Con el
    valor absoluto se pierde justo la información que distingue una junta de
    otra: el hombro y el codo tienen gradientes de signo opuesto y por eso sus
    artefactos salen de signo opuesto.
    """
    m = pin.buildModelFromUrdf(urdf)
    m.gravity.linear = np.array([0.0, 0.0, -gravity])
    d = m.createData()
    qs = np.linspace(Q_INIT[joint] - amplitude, Q_INIT[joint] + amplitude, 200)
    g = []
    for qi in qs:
        q = Q_INIT.copy()
        q[joint] = qi
        g.append(pin.computeGeneralizedGravity(m, d, q)[joint])
    g = np.array(g)
    return float((g[-1] - g[0]) / (qs[-1] - qs[0]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", type=int, default=300,
                    help="test_num de la junta 0; las demás son base+j")
    ap.add_argument("--dir", default=os.path.expanduser("~/.ros/ur5_dyn_control"))
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--gravity", type=float, default=9.8)
    ap.add_argument("--shift", type=int, default=0,
                    help="tau_shift en muestras (corrección del retardo)")
    ap.add_argument("--amplitude", type=float, default=np.pi / 4)
    args, _ = ap.parse_known_args()
    urdf = args.urdf or default_urdf()

    print(f"\nCONTROL NEGATIVO — planta SIN fricción, tau_shift = {args.shift}")
    print("Todo F_v distinto de cero aquí es ARTEFACTO, no fricción.\n")
    print(f"{'junta':<20} {'dg/dq':>9} {'F_v':>10} {'F_c':>10} "
          f"{'lag equiv.':>11}")
    print(f"{'':<20} {'[N.m/rad]':>9} {'[N.m.s/rad]':>10} {'[N.m]':>10} {'[ms]':>11}")
    print("-" * 66)

    rows = []
    for j in range(6):
        path = os.path.join(args.dir, f"fl_{args.base + j}.csv")
        if not os.path.exists(path):
            print(f"{JOINT_NAMES[j]:<20} {'—':>9} {'(sin datos)':>10}")
            continue
        res = compute_residual(path, urdf, gravity=args.gravity,
                               tau_shift=args.shift)
        w = extract_windows(res, j)
        if len(w) < 3:
            print(f"{JOINT_NAMES[j]:<20} {'—':>9} {'(pocas mesetas)':>10}")
            continue
        f = fit(w, "viscous_coulomb")
        dgdq = gravity_gradient(urdf, j, args.amplitude, args.gravity)
        # lag que explicaría ese F_v:  F_v = (dg/dq)·lag
        lag_ms = (-1e3 * f.params.f_v / dgdq) if abs(dgdq) > 1e-9 else float("nan")
        print(f"{JOINT_NAMES[j]:<20} {dgdq:9.2f} {f.params.f_v:10.4f} "
              f"{f.params.f_c:10.4f} {lag_ms:11.2f}")
        rows.append((j, dgdq, f.params.f_v, f.params.f_c, lag_ms))

    if len(rows) >= 3:
        d = np.array([[r[1], r[2]] for r in rows])
        loaded = d[np.abs(d[:, 0]) > 1.0]   # juntas realmente cargadas
        print("-" * 66)
        if len(loaded) >= 2:
            # Si el artefacto es un retardo comun, F_v = -lag * (dg/dq): recta
            # por el origen cuya pendiente ES el retardo.
            slope = float(np.linalg.lstsq(loaded[:, :1], loaded[:, 1:2],
                                          rcond=None)[0][0, 0])
            pred = loaded[:, :1] * slope
            ss_res = float(((loaded[:, 1:2] - pred) ** 2).sum())
            ss_tot = float(((loaded[:, 1:2] - loaded[:, 1:2].mean()) ** 2).sum())
            r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
            print(f"Diagnostico: ajuste F_v = -lag·(dg/dq) sobre las "
                  f"{len(loaded)} juntas cargadas")
            print(f"   lag = {-slope * 1e3:.3f} ms      R² = {r2:.4f}")
            if r2 > 0.9:
                print("   R² alto -> compatible con UN retardo comun a todas.")
            else:
                print("   R² BAJO -> un solo retardo NO explica todas las "
                      "juntas: hay al menos")
                print("   un segundo efecto. La calibracion valida es la "
                      "PER-JUNTA de la tabla,")
                print("   que es empirica y no depende de acertar el "
                      "mecanismo.")
        floor = float(np.abs(d[:, 1]).max())
        print(f"\nCALIBRACION (restar por junta) y suelo global:")
        print(f"   |F_v| < {floor:.4f} N.m.s/rad es indistinguible del "
              f"artefacto en la peor junta.")
        print("   En la campana real, restar el F_v de ESTA tabla junta por "
              "junta antes de")
        print("   reportar friccion. Es empirico: vale aunque el mecanismo no "
              "este cerrado.")


if __name__ == "__main__":
    main()
