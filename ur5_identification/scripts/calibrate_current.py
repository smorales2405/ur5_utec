#!/usr/bin/env python3
"""
Calibración corriente → par, y conversión del barrido por posición a un CSV que
el identificador de fricción pueda consumir sin cambios (FASE 2).

El problema
-----------
El UR5e NO mide par. El campo `effort` de `/joint_states` son corrientes de
motor (compuerta G5, verificada: `hardware_interface.cpp:800` lee el campo RTDE
`actual_current`). Y UR no publica las constantes `k_t·N` que las convertirían.

La solución: sale del propio barrido
------------------------------------
En la meseta, a la misma posición y misma rapidez, en los dos sentidos:

    i(+v)·k = g(q) + C(q,+v)(+v) + f_v·v + f_c
    i(−v)·k = g(q) + C(q,−v)(−v) − f_v·v − f_c

Los términos de gravedad e inercia son PARES en la velocidad (Coriolis es
cuadrático), así que:

    SUMA  ->  k·[i(+v) + i(−v)]/2 = g(q) + C·v      <- lo conoce el modelo
    DIF   ->  k·[i(+v) − i(−v)]/2 = f_v·v + f_c     <- es la fricción

De la primera sale `k` por junta, por mínimos cuadrados sobre todos los niveles
de velocidad. No hacen falta las constantes del fabricante.

Qué produce
-----------
Un CSV con el esquema del barrido por par (`tau*_phys` = k·corriente), de modo
que `run_identification` lo lee sin tocar nada y los dos métodos —control de par
y control de posición— quedan directamente comparables sobre las mismas juntas.

Uso
---
    ros2 run ur5_identification calibrate_current.py \\
        --csv ~/.ros/ur5_dyn_control/cur_954.csv --joint 4 \\
        --out ~/.ros/ur5_dyn_control/fl_954.csv
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import pinocchio as pin


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
    col = lambda p: np.column_stack([d[p % i] for i in range(1, 7)])  # noqa: E731
    return {"t": np.asarray(d["t_sim"], dtype=float),
            "q": col("q%d"), "dq": col("dq%d"), "cur": col("cur%d"),
            "state": np.asarray(d["state"], dtype=str)}


def plateau_pairs(d, joint):
    """
    Agrupa las mesetas en pares (+v, −v) del MISMO nivel de velocidad.

    La etiqueta la pone el generador: `SWEEP_<v>_POS` / `SWEEP_<v>_NEG`. Se
    exige el par completo — una meseta suelta no sirve, porque el método entero
    se apoya en cancelar los términos pares comparando los dos sentidos.
    """
    out = []
    levels = sorted({s.rsplit("_", 1)[0] for s in d["state"]
                     if s.startswith("SWEEP_") and s.rsplit("_", 1)[-1] in ("POS", "NEG")})
    for lv in levels:
        m_p = d["state"] == f"{lv}_POS"
        m_n = d["state"] == f"{lv}_NEG"
        if m_p.sum() < 20 or m_n.sum() < 20:
            continue
        out.append((lv, m_p, m_n))
    return out


def model_torque(model, data, q, dq, joint):
    """Par del MODELO (sin fricción) en la meseta: gravedad + Coriolis."""
    ddq = np.zeros(6)
    return np.array([pin.rnea(model, data, q[k], dq[k], ddq)[joint]
                     for k in range(len(q))])


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--csv", required=True)
    ap.add_argument("--joint", type=int, required=True, choices=range(6))
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--gravity", type=float, default=9.8)
    ap.add_argument("--k", type=float, default=None,
                    help="constante corriente->par [N·m/A] conocida, en vez de "
                         "ajustarla. Necesaria en juntas SIN carga de gravedad "
                         "(wrist_3), donde no es identificable de sus datos")
    ap.add_argument("--out", default=None, help="CSV convertido a par")
    args = ap.parse_args()

    if args.urdf is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            args.urdf = os.path.join(
                get_package_share_directory("ur5_kinematics"), "ur5e.urdf")
        except Exception:
            print("no se pudo resolver el URDF; use --urdf")
            return 1

    d = load(args.csv)
    j = args.joint
    if np.all(np.isnan(d["cur"][:, j])):
        print("La columna de corriente es NaN: el driver no publicó `effort`.")
        return 1

    model = pin.buildModelFromUrdf(args.urdf)
    model.gravity.linear = np.array([0.0, 0.0, -args.gravity])
    data = model.createData()

    pairs = plateau_pairs(d, j)
    if not pairs:
        print("No hay pares de mesetas (+v, -v) utilizables.")
        return 1

    print(f"Calibración de la junta {j} sobre {len(pairs)} niveles de velocidad\n")
    print(f"{'nivel':>16}{'i(+v)':>10}{'i(-v)':>10}{'suma/2':>10}"
          f"{'modelo':>10}{'k':>10}")
    print("-" * 66)
    sums_i, sums_m, rows = [], [], []
    for lv, m_p, m_n in pairs:
        ip, in_ = d["cur"][m_p, j].mean(), d["cur"][m_n, j].mean()
        tp = model_torque(model, data, d["q"][m_p], d["dq"][m_p], j).mean()
        tn = model_torque(model, data, d["q"][m_n], d["dq"][m_n], j).mean()
        si, sm = 0.5 * (ip + in_), 0.5 * (tp + tn)
        k = sm / si if abs(si) > 1e-9 else float("nan")
        sums_i.append(si)
        sums_m.append(sm)
        rows.append((lv, ip, in_, si, sm))
        print(f"{lv:>16}{ip:10.4f}{in_:10.4f}{si:10.4f}{sm:10.4f}{k:10.3f}")

    # k global por minimos cuadrados sobre todos los niveles: mas robusto que
    # promediar cocientes, que explota cuando la suma se acerca a cero.
    A = np.array(sums_i).reshape(-1, 1)
    b = np.array(sums_m)

    # ¿Tiene esta junta carga de gravedad? Si no, `k` NO es identificable de sus
    # propios datos: la suma entre sentidos vale cero por construcción y el
    # ajuste degenera en 0/0.
    #
    # Le pasa a wrist_3 SIEMPRE: gira sobre el eje de la herramienta y la masa
    # que arrastra está centrada en ese eje, así que su par gravitatorio es nulo
    # en cualquier postura. No es un problema de ruido ni de linealidad, y decir
    # "la relación corriente-par no es lineal" ahí sería un diagnóstico falso.
    rms_model = float(np.sqrt((b ** 2).mean()))
    degenerate = rms_model < 0.05          # [N·m]

    if args.k is not None:
        k = float(args.k)
        print(f"\n  k = {k:.4f} N·m/A  (SUMINISTRADA con --k, no ajustada)")
        resid = A.flatten() * k - b
    elif degenerate:
        print(f"\n  El par del modelo es nulo en esta junta "
              f"(RMS = {rms_model:.5f} N·m): NO tiene carga de gravedad, así que")
        print("  `k` no es identificable de sus propios datos.")
        print("\n  La fricción SÍ está medida, en amperios:")
        print(f"\n{'nivel':>16}{'v [rad/s]':>12}{'friccion [A]':>16}")
        print("-" * 44)
        vs, fs = [], []
        for (lv, m_p, m_n), (_, ip, in_, _, _) in zip(pairs, rows):
            v = abs(d["dq"][m_p, j].mean())
            fa = 0.5 * (ip - in_)
            vs.append(v); fs.append(fa)
            print(f"{lv:>16}{v:12.4f}{fa:16.5f}")
        M = np.column_stack([vs, np.ones(len(vs))])
        coef, *_ = np.linalg.lstsq(M, np.array(fs), rcond=None)
        r2 = 1.0 - np.var(M @ coef - np.array(fs)) / max(np.var(fs), 1e-30)
        print(f"\n  f_v = {coef[0]:.5f} A/(rad/s)   f_c = {coef[1]:.5f} A"
              f"   (R² = {r2:.4f})")
        print("\n  Para pasarlo a N·m, tome la `k` de una junta del MISMO tamaño")
        print("  que sí tenga carga —en el UR5e las juntas 4, 5 y 6 comparten")
        print("  motor y reductora— y vuelva a lanzar con --k:")
        print(f"      f_v[N·m·s/rad] = {coef[0]:.5f} * k")
        print(f"      f_c[N·m]       = {coef[1]:.5f} * k")
        return 2
    else:
        k, *_ = np.linalg.lstsq(A, b, rcond=None)
        k = float(k[0])
        resid = A.flatten() * k - b

    # NO se usa R². El par del modelo es casi el MISMO en todos los niveles —el
    # barrido recorre el mismo rango de posturas y domina la gravedad—, así que
    # `var(b) ~ 0` y `1 - var(resid)/var(b)` da números sin sentido (medido:
    # -3.8e7 sobre datos sintéticos donde k se recuperaba EXACTO). Lo que sí
    # significa algo es el residuo RELATIVO a la magnitud del par.
    rel = (float(np.sqrt((resid ** 2).mean()) / rms_model)
           if rms_model > 1e-12 else float("inf"))
    spread = float(np.ptp(b / A.flatten())) if len(b) > 1 else 0.0
    if args.k is None:
        print(f"\n  k = {k:.4f} N·m/A   ({len(pairs)} niveles)")
    print(f"  residuo relativo = {100 * rel:.3f} %   "
          f"dispersión de k entre niveles = {spread:.4f} N·m/A")
    if rel > 0.05:
        print("  AVISO: el residuo relativo supera el 5 %. La relación "
              "corriente-par no es lineal en este rango, o el modelo no "
              "describe bien la postura. La fricción que salga de aquí hay que "
              "tomarla con reservas.")

    # Fricción por diferenciación, con el k recién calculado.
    print(f"\n{'nivel':>16}{'v [rad/s]':>12}{'friccion [N·m]':>16}")
    print("-" * 44)
    for (lv, m_p, m_n), (_, ip, in_, _, _) in zip(pairs, rows):
        v = abs(d["dq"][m_p, j].mean())
        f = k * 0.5 * (ip - in_)
        print(f"{lv:>16}{v:12.4f}{f:16.4f}")

    if args.out:
        # Se reescribe con el esquema del barrido por par para que
        # `run_identification` lo consuma sin cambios.
        cur = d["cur"]
        tau = k * cur
        with open(args.out, "w") as fh:
            fh.write(f"# controller_id=current_sweep_calibrated\n")
            fh.write(f"# source={args.csv}\n")
            fh.write(f"# k_current_to_torque={k:.6f}\n")
            fh.write(f"# k_residuo_relativo={rel:.6f}\n")
            fh.write(f"# joint={j}\n")
            fh.write("t_sim")
            for g in ("q", "dq"):
                for i in range(1, 7):
                    fh.write(f",{g}{i}")
            for i in range(1, 7):
                fh.write(f",tau{i}")
            for i in range(1, 7):
                fh.write(f",tau{i}_phys")
            fh.write(",state\n")
            for n in range(len(d["t"])):
                fh.write(f"{d['t'][n]:.9f}")
                for arr in (d["q"], d["dq"], tau, tau):
                    for v in arr[n]:
                        fh.write(f",{v:.9f}")
                fh.write(f",{d['state'][n]}\n")
        print(f"\n  CSV convertido: {args.out}")
        print(f"  ros2 run ur5_identification run_identification --csv {args.out} \\")
        print(f"      --models viscous_coulomb stribeck")
    return 0


if __name__ == "__main__":
    sys.exit(main())
