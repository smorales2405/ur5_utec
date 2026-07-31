#!/usr/bin/env python3
"""
Ensayo de TIEMPO DE ALCANCE del SMC (criterio pendiente de la FASE 5).

El criterio del plan —"con `sign`, `s` alcanza el entorno de cero en tiempo
finito, coherente con la cota"— no es medible en una corrida normal: la fase
RAMP deja el robot exactamente sobre la tabla de referencias, así que al entrar
en TRACK ya está SOBRE la superficie deslizante y no hay fase de alcance que
cronometrar (docs/05_smc.md §6).

Con `initial_offset` el destino de la rampa se desplaza, de modo que TRACK
arranca con un error conocido y `dq_e ≈ 0`, es decir

    s_i(0) = λ_i · offset_i

La cota que hay que contrastar sale de la dinámica del modo deslizante. Con
modelo perfecto y `ρ = sgn(s)`:

    M ṡ = −K ⊙ sgn(s)      ⟹      |ṡ_i| = K_i / M_ii ≥ η_i / M_ii

y con el escalado por inercia de la FASE 5, `η_i = M_ii · a_reach`, queda

    |ṡ_i| ≥ a_reach        ⟹      t_alcance,i ≤ |s_i(0)| / a_reach

Es una COTA SUPERIOR: `K = η + |cota de incertidumbre|` es mayor que `η`, así
que el alcance real puede ser más rápido. Lo que invalidaría el modelo sería
medir un tiempo MAYOR que la cota.
"""

import argparse

import numpy as np


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
    return {"t": t - t[0], "s": col("s%d"), "q": col("q%d"),
            "q_des": col("q%d_des"), "tau": col("tau%d")}


def reaching_times(d, phi, a_reach, lam, settle_window=2.0):
    """
    Tiempo de alcance = PRIMERA entrada de |s_i| en la capa límite.

    No sirve exigir que se quede dentro para siempre: la incisión encadena cinco
    fases con reposos, y en cada arranque `s` vuelve a saltar por el transitorio
    de la referencia (docs/05_smc.md §3). Eso son transitorios NUEVOS, no un
    fallo del alcance inicial, así que se miden y se reportan aparte en vez de
    contaminar la medida.

    Se registra además el PICO de |s| durante el alcance: la cota por junta
    `t ≤ |s_i(0)|/a_reach` sale de suponer `M` diagonal, y con la `M` real
    acoplada una junta puede alejarse de la superficie antes de volver.
    """
    t, s = d["t"], d["s"]
    rows = []
    for i in range(6):
        si = np.abs(s[:, i])
        s0 = si[0]
        idx = np.flatnonzero(si < phi)
        t_reach = t[idx[0]] if len(idx) else np.nan
        during = si[t <= (t_reach if np.isfinite(t_reach) else t[-1])]
        later = si[t > settle_window]
        rows.append({
            "joint": i + 1,
            "s0": s0,
            "t_reach": t_reach,
            "bound": s0 / a_reach if a_reach > 0 else np.inf,
            "s_peak": float(during.max()) if len(during) else s0,
            "s_later_max": float(later.max()) if len(later) else np.nan,
        })
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--phi", type=float, default=0.05)
    ap.add_argument("--a-reach", type=float, default=1.0,
                    help="eta_i = M_ii * a_reach [rad/s^2]")
    ap.add_argument("--lam", type=float, default=20.0)
    ap.add_argument("--offset", type=float, default=0.05,
                    help="offset articular aplicado [rad]")
    args, _ = ap.parse_known_args()

    d = load(args.csv)
    lam = np.full(6, args.lam)
    rows = reaching_times(d, args.phi, args.a_reach, lam)

    print(f"\nEnsayo de tiempo de alcance — {args.csv}")
    print(f"  phi = {args.phi}   a_reach = {args.a_reach} rad/s^2   "
          f"lambda = {args.lam}   offset = {args.offset} rad")
    print(f"  s(0) predicho = lambda*offset = {args.lam * args.offset:.4f} rad/s\n")

    hdr = (f"{'junta':>6}{'s(0) medido':>13}{'t_alcance':>12}"
           f"{'cota |s0|/a':>13}{'pico |s|':>11}{'max|s| >2s':>12}  veredicto")
    print(hdr)
    print("-" * len(hdr))
    ok_all = True
    for r in rows:
        if np.isnan(r["t_reach"]):
            verdict = "NO alcanza"
            ok_all = False
        elif r["t_reach"] <= r["bound"]:
            verdict = "cumple"
        else:
            verdict = "EXCEDE la cota"
            ok_all = False
        print(f"{r['joint']:>6}{r['s0']:>13.5f}{r['t_reach']:>12.4f}"
              f"{r['bound']:>13.4f}{r['s_peak']:>11.4f}"
              f"{r['s_later_max']:>12.4f}  {verdict}")

    print(f"\nVeredicto global: {'CUMPLE' if ok_all else 'REVISAR'} "
          f"(la cota es SUPERIOR: alcanzar antes es correcto)")
    overs = [r for r in rows if r["s_peak"] > 1.05 * r["s0"]]
    if overs:
        print("\nSobrepico durante el alcance (|s| crece antes de bajar): juntas "
              + ", ".join(f"{r['joint']} (x{r['s_peak'] / r['s0']:.2f})" for r in overs))
        print("  La cota por junta supone M DIAGONAL. Con la M real acoplada,")
        print("  ṡ = −M⁻¹K·sgn(s) mezcla las juntas y una puede alejarse de la")
        print("  superficie antes de volver: lo que garantiza la convergencia es")
        print("  V = ½sᵀMs, no el decrecimiento de cada |s_i| por separado.")
    print("\nLas excursiones con t > 2 s son transitorios de las 5 fases de la")
    print("incision (docs/05_smc.md §3), no fallos del alcance inicial.")


if __name__ == "__main__":
    main()
