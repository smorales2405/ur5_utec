#!/usr/bin/env python3
"""
Identificación de fricción articular a partir de los CSV de barrido (FASE 2).

Uso:
  ros2 run ur5_identification run_identification \
      --csv ~/.ros/ur5_dyn_control/fl_206.csv \
      --out ~/.ros/ur5_dyn_control/friction_j0.yaml

  # Validación contra la verdad inyectada en Gazebo:
  ros2 run ur5_identification run_identification --csv ... --truth 1.5 2.5

  # Varias juntas a la vez (una corrida por junta):
  ros2 run ur5_identification run_identification --csv fl_206.csv fl_207.csv ...
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import yaml

from .estimator import cross_validate, fit, fit_stribeck
from .residual import (JOINT_NAMES, compute_residual, default_urdf,
                       extract_windows, infer_joint_from_csv)


def identify_one(csv_path: str, urdf: str, gravity: float, cutoff: float,
                 trim: float, models: list[str], truth=None) -> dict:
    joint = infer_joint_from_csv(csv_path)
    if joint is None:
        raise RuntimeError(f"{csv_path}: no hay ventanas de meseta "
                           "(state SWEEP_<v>_POS/NEG). ¿Es un CSV de barrido?")

    res = compute_residual(csv_path, urdf, gravity=gravity, cutoff_hz=cutoff)
    windows = extract_windows(res, joint, trim_fraction=trim)
    if not windows:
        raise RuntimeError(f"{csv_path}: ninguna meseta con muestras suficientes")

    print(f"\n{'=' * 72}")
    print(f"{os.path.basename(csv_path)}  ->  junta {joint} ({JOINT_NAMES[joint]})")
    print(f"  fs = {res['fs']:.1f} Hz | {len(windows)} mesetas | "
          f"{sum(len(w.dq) for w in windows)} muestras útiles")
    print(f"{'-' * 72}")
    print("  v_nominal   v_medida   residuo medio   sigma_intra   n")
    for w in sorted(windows, key=lambda w: w.velocity):
        print(f"   {w.velocity:+7.3f}   {w.dq_mean:+8.4f}   {w.residual_mean:+11.4f}"
              f"   {w.residual_std:9.4f}   {len(w.dq):5d}")

    out = {"joint": int(joint), "joint_name": JOINT_NAMES[joint], "csv": csv_path,
           "n_windows": int(len(windows)), "models": {}}

    for model in models:
        print(f"\n  --- modelo '{model}' ---")
        if model == "stribeck":
            r, v_s, delta = fit_stribeck(windows)
            cv = cross_validate(windows, model, v_s=v_s, delta=delta)
            print(f"    v_s = {v_s:.4f} rad/s, delta = {delta:.1f} (rejilla)")
        else:
            r = fit(windows, model)
            cv = cross_validate(windows, model)
        print(r.summary())
        print(cv.summary())

        # Todo a float nativo: yaml.safe_dump no sabe representar np.float64.
        entry = r.params.as_dict()
        entry.update({"r2": float(r.r2), "rmse": float(r.rmse),
                      "r2_cv": float(cv.r2), "rmse_cv": float(cv.rmse),
                      "stderr": [float(x) for x in r.stderr],
                      "ci95": [[float(a), float(b)] for a, b in r.ci95]})
        out["models"][model] = entry

        if truth is not None and model == "viscous_coulomb":
            fv_t, fc_t = truth
            e_v = 100 * abs(r.params.f_v - fv_t) / fv_t if fv_t else float("nan")
            e_c = 100 * abs(r.params.f_c - fc_t) / fc_t if fc_t else float("nan")
            ok = (e_v <= 10.0) and (e_c <= 10.0)
            print(f"\n    VALIDACIÓN contra la verdad inyectada "
                  f"(F_v={fv_t}, F_c={fc_t}):")
            print(f"      F_v: {r.params.f_v:.4f}  error {e_v:5.2f} %")
            print(f"      F_c: {r.params.f_c:.4f}  error {e_c:5.2f} %")
            print(f"      criterio del plan (<= 10 %): "
                  f"{'CUMPLE' if ok else 'NO CUMPLE'}")
            out["truth"] = {"f_v": float(fv_t), "f_c": float(fc_t),
                            "error_pct": [float(e_v), float(e_c)],
                            "passes_10pct": bool(ok)}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", nargs="+", required=True,
                    help="CSV(s) de barrido, uno por junta")
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--gravity", type=float, default=9.8)
    ap.add_argument("--cutoff", type=float, default=10.0,
                    help="corte del filtro de fase cero [Hz]")
    ap.add_argument("--trim", type=float, default=0.1,
                    help="fracción recortada en cada extremo de la meseta")
    ap.add_argument("--models", nargs="+",
                    default=["viscous", "viscous_coulomb"],
                    choices=["viscous", "viscous_coulomb", "stribeck"])
    ap.add_argument("--truth", nargs=2, type=float, default=None,
                    metavar=("F_V", "F_C"),
                    help="verdad inyectada en Gazebo, para validar")
    ap.add_argument("--out", default=None, help="YAML de salida")
    args, _ = ap.parse_known_args()

    urdf = args.urdf or default_urdf()
    results = [identify_one(c, urdf, args.gravity, args.cutoff, args.trim,
                            args.models, args.truth) for c in args.csv]

    if args.out:
        # YAML consumible por ur5_dyn_control: 6 valores por parámetro, en el
        # orden canónico de juntas. Las juntas sin identificar quedan a 0, que
        # equivale a no compensar.
        f_v = [0.0] * 6
        f_c = [0.0] * 6
        for r in results:
            m = r["models"].get("viscous_coulomb") or r["models"].get("viscous")
            f_v[r["joint"]] = float(m.get("f_v", 0.0))
            f_c[r["joint"]] = float(m.get("f_c", 0.0))
        doc = {"friction": {"f_v": f_v, "f_c": f_c},
               "identification": results}
        os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
        with open(args.out, "w") as fh:
            yaml.safe_dump(doc, fh, sort_keys=False, default_flow_style=False)
        print(f"\nParámetros escritos en {args.out}")
        print(f"  f_v = {[round(x, 4) for x in f_v]}")
        print(f"  f_c = {[round(x, 4) for x in f_c]}")

    if any(r.get("truth", {}).get("passes_10pct") is False for r in results):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
