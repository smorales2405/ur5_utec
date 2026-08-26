#!/usr/bin/env python3
"""
FASE 7 — sintonía multiobjetivo de las ganancias del SMC.

    ros2 run ur5_trajectory_optimization run_gain_tuning -- --mode scalar -j 8

Encadena lo que pide el plan: NSGA-II, ε-restricción con SLSQP, métricas de
frente (HV / IGD / cobertura), selección por punto de rodilla, certificación
KKT y comparación contra el método previo de suma ponderada. Deja en
`results/gain_tuning/<controlador>/`:

    pareto.csv           frentes de los dos métodos, con ganancias físicas
    convergence.csv      HV por generación de NSGA-II
    metrics.yaml         HV/IGD/cobertura/tiempos/KKT/línea base
    selected_gains.yaml  cargable TAL CUAL por gz_smc_control_node

La referencia NO se regenera aquí: se lee la tabla `{q,dq,ddq}` que vuelca el
propio nodo con `reference_table_out`, de modo que optimizador y robot comparten
trayectoria exactamente (ver el encabezado de `closed_loop.py`).
"""

from __future__ import annotations

import argparse
import os
import time

import numpy as np
import yaml

from ..metrics import compute_coverage, compute_hv, compute_igd, filter_nondominated
from ..multiobjective_optimizer import select_solution
from .closed_loop import CuttingForce, Plant, load_reference
from .optimize import (alpha_sensitivity, certify_kkt, hv_reference_point,
                       run_epsilon_constraint, run_nsga2,
                       run_weighted_sum_baseline, seed_points)
from .problem import (CHI_SAFETY_DEFAULT, CHI_THRESHOLD,
                      FRICTION_REAL_G4_0, PENALTY,
                      TCP_TOL_MM_DEFAULT, disturbance_bound,
                      friction_residual_bound, make_evaluator)
from .problem import SmcParameterization  # noqa: E402

DEFAULT_REF = os.path.expanduser("~/.ros/ur5_dyn_control/incision_ref.csv")
OBJ_NAMES = ["f1_iae_m_s", "f2_effort_N2m2s", "f3_tv_Nm"]
CON_NAMES = ["g1_tau", "g2_dq", "g3_chi", "g4_tcp", "g5_alcance"]


def _default_urdf() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("ur5_kinematics"), "ur5e.urdf")
    except Exception:
        return "/home/utec/ur5_ws/install/ur5_kinematics/share/ur5_kinematics/ur5e.urdf"


def _results_dir(controller: str, test: int | None,
                 out: str | None = None) -> str:
    """
    Directorio de salida.

    Por defecto va junto al modulo, y eso tiene una trampa: lanzado como
    `python3 -m ...` tras hacer source del workspace, Python coge la copia
    INSTALADA y los resultados acaban en `install/`, donde el siguiente
    `colcon build` los borra. `--out` lo hace explicito.
    """
    if out:
        path = os.path.abspath(os.path.expanduser(out))
    else:
        here = os.path.dirname(os.path.abspath(__file__))
        base = os.path.abspath(os.path.join(here, "..", "..", "results", "gain_tuning"))
        path = os.path.join(base,
                            controller if test is None else f"{controller}/test{test}")
    os.makedirs(path, exist_ok=True)
    return path


def _save_pareto(path, param, method, X, F, G):
    """Guarda el frente con las variables de decisión Y las ganancias físicas.

    Las dos cosas: las variables de decisión hacen la corrida reproducible, las
    ganancias físicas hacen la tabla legible sin tener que saber que la búsqueda
    va en log10.
    """
    rows, header = [], ["method"] + param.names + ["lambda%d" % i for i in range(1, 7)]
    n_phi = 6 if param.mode == "full_phi" else 1
    header += (["eta%d" % i for i in range(1, 7)]
               + (["phi%d" % i for i in range(1, 7)] if n_phi == 6 else ["phi"])
               + OBJ_NAMES + CON_NAMES)
    for k in range(len(X)):
        lam, eta, phi = param.gains(X[k])
        g = G[k] if G is not None else [np.nan] * 3
        rows.append([method] + list(np.atleast_1d(X[k])) + list(lam) + list(eta)
                    + list(np.atleast_1d(phi)) + list(F[k]) + list(g))
    new = not os.path.exists(path)
    with open(path, "a" if not new else "w") as fh:
        if new:
            fh.write(",".join(header) + "\n")
        for r in rows:
            fh.write(",".join(str(v) if isinstance(v, str) else f"{float(v):.10g}"
                              for v in r) + "\n")
    print(f"  guardado → {path}  ({len(rows)} filas, método={method})")


def _load_pareto(path: str, param) -> dict:
    """
    Recupera un frente ya guardado (`pareto.csv`) para rehacer la SELECCIÓN sin
    repetir la búsqueda.

    La búsqueda cuesta más de una hora; la selección, segundos. Cuando lo que
    hay que corregir es el criterio de selección —no el frente— repetir NSGA-II
    sería tirar el tiempo.
    """
    d = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")
    method = np.asarray(d["method"], dtype=str)
    X = np.column_stack([d[n] for n in param.names])
    F = np.column_stack([d[n] for n in OBJ_NAMES])
    out = {}
    for tag in ("nsga2", "epsilon"):
        m = method == tag
        out[tag] = {"X": np.atleast_2d(X[m]), "F": np.atleast_2d(F[m])}
    return out


def _save_selected_yaml(path, lam, eta, phi, alpha, meta: dict):
    doc = {"gz_smc_control_node": {"ros__parameters": {
        "lambda": [round(float(v), 8) for v in lam],
        "eta": [round(float(v), 8) for v in eta],
        # El nodo lee `phi` escalar y `phi_joint` opcional de 6, y phi_joint
        # GANA si esta presente. Escribir solo `phi` con una solucion por junta
        # dejaria las seis al mismo valor sin que nadie lo notara.
        **({"phi_joint": [round(float(v), 8) for v in np.atleast_1d(phi)],
            "phi": round(float(np.atleast_1d(phi)[0]), 8)}
           if np.atleast_1d(phi).size == 6
           else {"phi": round(float(phi), 8)}),
        "alpha": round(float(alpha), 8),
        "switching_function": "sat",
    }}}
    with open(path, "w") as fh:
        fh.write("# FASE 7 — ganancias seleccionadas por sintonía multiobjetivo.\n")
        fh.write("# Generado por run_gain_tuning.py; NO editar a mano.\n")
        for k, v in meta.items():
            fh.write(f"#   {k}: {v}\n")
        fh.write("#\n# Se fusiona sobre config/smc_params.yaml (solo ganancias).\n")
        yaml.safe_dump(doc, fh, default_flow_style=False, sort_keys=False)
    print(f"  guardado → {path}")


def _py(obj):
    """numpy → tipos nativos; yaml.safe_dump no representa np.float64."""
    if isinstance(obj, dict):
        return {k: _py(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_py(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return [_py(v) for v in obj.tolist()]
    if isinstance(obj, (np.floating, float)):
        return float(obj)
    if isinstance(obj, (np.integer, int)):
        return int(obj)
    if isinstance(obj, (np.bool_, bool)):
        return bool(obj)
    return obj


def main(argv=None):
    ap = argparse.ArgumentParser(description="FASE 7 — sintonía multiobjetivo del SMC")
    ap.add_argument("--controller", default="smc")
    # `choices` sale de la propia parametrizacion: escribir la lista a mano
    # aqui la dejo desincronizada al anadir `full_phi`.
    ap.add_argument("--mode", default="scalar",
                    choices=sorted(SmcParameterization._N_VAR),
                    help="scalar (3 vars) | full (13) | full_phi (18, phi "
                         "por junta: el umbral de chattering NO es comun)")
    ap.add_argument("--ref", default=DEFAULT_REF, help="tabla de referencias del nodo")
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--pop", type=int, default=40)
    ap.add_argument("--gen", type=int, default=60)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--eps-steps", type=int, default=12)
    ap.add_argument("--slsqp-maxiter", type=int, default=None,
                    help="por defecto se escala con la dimensión (200/n_var)")
    ap.add_argument("--alpha", type=float, default=0.3)
    ap.add_argument("--f-cut", type=float, default=5.0,
                    help="meseta de la fuerza de corte [N]; 0 = planta sin fuerza")
    ap.add_argument("--chi-safety", type=float, default=CHI_SAFETY_DEFAULT,
                    help="fraccion del umbral de chattering MEDIDO por junta "
                         "en la que se permite operar. Los umbrales son dato "
                         "(docs/05_smc.md 7.6); esto es la unica decision")
    ap.add_argument("--tcp-tol-mm", type=float, default=TCP_TOL_MM_DEFAULT,
                    help="g4: RMSE de TCP admisible en la meseta del corte "
                         "(cota DECLARADA, no medida)")
    ap.add_argument("-j", "--jobs", type=int, default=1)
    ap.add_argument("--test", type=int, default=None)
    ap.add_argument("--no-baseline", action="store_true")
    ap.add_argument("--out", default=None,
                    help="directorio de salida. Sin esto va junto al "
                         "modulo, que lanzado como `python3 -m` tras "
                         "source acaba siendo install/ y lo borra el "
                         "siguiente build")
    ap.add_argument("--friction", action="store_true",
                    help="planta CON la friccion real medida (G4 = 0.0). AVISO: "
                         "el modelo discreto NO esta validado contra Gazebo "
                         "todavia — ver closed_loop.JointFriction. Por defecto, "
                         "planta ideal")
    ap.add_argument("--reselect", action="store_true",
                    help="rehace selección/KKT/α desde el pareto.csv existente, "
                         "sin repetir NSGA-II ni la ε-restricción")
    ap.add_argument("--weights", type=float, nargs=3, default=[1.0, 1.0, 1.0])
    args, sobrantes = ap.parse_known_args(argv)
    # `parse_known_args` es necesario porque ros2 inyecta sus propios argumentos,
    # pero tragarse en SILENCIO un argumento mal escrito es un desastre en una
    # corrida de una hora: se descubre al final, con los resultados en otro
    # sitio o con la configuracion equivocada. Se avisa de lo que no es de ROS.
    ajenos = [a for a in sobrantes
              if a.startswith("--") and not a.startswith("--ros-args")]
    if ajenos:
        print(f"\n  AVISO: argumentos no reconocidos, se IGNORAN: {' '.join(ajenos)}")
        print("         (si era un typo, cancele ahora: la corrida es larga)\n")

    urdf = args.urdf or _default_urdf()
    outdir = _results_dir(args.controller, args.test, args.out)
    t_start = time.time()

    print("=" * 72)
    print(f"FASE 7 — sintonía multiobjetivo · controlador={args.controller} "
          f"· parametrización={args.mode}")
    print("=" * 72)

    ref = load_reference(args.ref)
    plant = Plant(urdf)
    # La friccion es OPT-IN mientras su modelo no este validado: contra la
    # corrida 403 de Gazebo, sobre la misma configuracion, el evaluador da
    # errores articulares 100-1200x mayores. No se puede optimizar contra eso.
    # Ver el docstring de closed_loop.JointFriction.
    friction = FRICTION_REAL_G4_0 if args.friction else None
    # La cota se calcula ANTES del evaluador: g5 la necesita. Antes se calculaba
    # despues y solo servia para sembrar, que es lo que invalido las dos
    # primeras corridas.
    _force = CuttingForce(f_cut=args.f_cut) if args.f_cut > 0 else None
    d_force = (disturbance_bound(plant, ref, _force)
               if _force is not None else np.zeros(6))
    d_fric = friction_residual_bound(ref, FRICTION_REAL_G4_0)
    d_bound = d_force + d_fric
    ev = make_evaluator(ref, plant, mode=args.mode, alpha=args.alpha,
                        f_cut=args.f_cut, chi_safety=args.chi_safety,
                        tcp_tol_mm=args.tcp_tol_mm, friction=friction,
                        d_bound=d_bound)
    param = ev.param

    print(f"referencia : {ref.n} muestras · dt={ref.dt:.4f} s · {ref.n * ref.dt:.1f} s")
    print(f"inercia    : diag M(q_init) = {np.array2string(param.inertia, precision=5)}")
    print(f"variables  : {param.n_var}  {param.names}")
    print(f"restricción: χ_i ≤ {args.chi_safety} · umbral_i · "
          f"RMSE TCP ≤ {args.tcp_tol_mm} mm · fuerza de corte = {args.f_cut} N")
    print(f"             umbral_i = {np.array2string(CHI_THRESHOLD, precision=2)}"
          f"  ->  limite = {np.array2string(args.chi_safety * CHI_THRESHOLD, precision=3)}")
    if friction is None:
        print("fricción   : NINGUNA — planta ideal, NO predice el robot real")
    elif True:
        print("fricción   : *** MODELO SIN VALIDAR (100-1200x de error frente a "
              "Gazebo) ***")
    else:
        print(f"fricción   : real, G4 = 0.0 (docs/02_friction_real.md §3.1)\n"
              f"             F_v = {np.array2string(friction.f_v, precision=2)}\n"
              f"             F_c = {np.array2string(friction.f_c, precision=2)}")

    # ── Punto de partida: las ganancias de la FASE 5 ─────────────────────────
    x_f5 = param.encode(np.full(6, 20.0), param.inertia * 1.0, 0.05)
    F_f5 = ev.objectives(x_f5)
    G_f5 = ev.constraints(x_f5)
    r_f5 = ev.result(x_f5)
    print(f"\nFASE 5 (λ=20, η=I·1.0, φ=0.05): f={np.array2string(F_f5, precision=4)}")
    print(f"  g={np.array2string(G_f5, precision=4)}  "
          f"{'FACTIBLE' if np.all(G_f5 <= 0) else 'INFACTIBLE'}"
          f"  ·  TCP={r_f5.rmse_tcp_mm:.3f} mm  χ={r_f5.chi_max:.3f}")

    t0 = time.time()
    single = ev.objectives(x_f5 + 1e-7)   # punto nuevo → mide coste real
    del single
    print(f"coste por evaluación: {time.time() - t0:.2f} s")

    # ── Siembra derivada del análisis ────────────────────────────────────────
    # La perturbacion que `eta` tiene que cubrir son DOS cosas: la fuerza de
    # corte y la friccion que el feedforward no cancela. Sin el segundo termino
    # se siembran ganancias que dejan juntas CLAVADAS —medido en la FASE 5, el
    # feedforward tiende a f_c por debajo y nunca lo supera, asi que el margen
    # de arranque solo puede darlo el termino conmutado (docs/05_smc.md §7.4)—.
    #
    # Se usa la cota ANALITICA y no el stick-slip simulado: el modelo de
    # friccion del evaluador no valida contra Gazebo (ver JointFriction), y una
    # perturbacion acotada es lo que pide la teoria de SMC de todos modos.
    seeds = seed_points(ev, d_bound)
    print(f"\ncota de perturbación:")
    print(f"  fuerza de corte |Jᵀw| = {np.array2string(d_force, precision=4)}")
    print(f"  friccion residual     = {np.array2string(d_fric, precision=4)}")
    print(f"  TOTAL                 = {np.array2string(d_bound, precision=4)}")
    print(f"población inicial: {len(seeds)} semillas derivadas + LHS")

    pareto_csv = os.path.join(outdir, "pareto.csv")

    if args.reselect:
        print(f"\n── RESELECCIÓN desde {pareto_csv} (sin repetir la búsqueda) ──")
        saved = _load_pareto(pareto_csv, param)
        X_nsga, F_nsga = saved["nsga2"]["X"], saved["nsga2"]["F"]
        X_eps, F_eps = saved["epsilon"]["X"], saved["epsilon"]["F"]
        nsga = {"X": X_nsga, "F": F_nsga, "G": None, "convergence": [],
                "n_eval": 0, "elapsed_s": 0.0, "sec_per_eval": float("nan"),
                "convergence_hv_ref": None}
        hv_ref = hv_reference_point(F_nsga)
        lo_f = F_nsga.min(axis=0)
        hi_f = F_nsga.max(axis=0)
        eps = {"F": F_eps, "X": X_eps,
               "eps_values": F_eps[:, 0] if len(F_eps) else np.array([]),
               "success": np.zeros(len(F_eps), dtype=bool),
               "nit": np.zeros(len(F_eps), dtype=int),
               "lo": lo_f, "scale": np.where(hi_f - lo_f > 0, hi_f - lo_f, 1.0),
               "elapsed_s": 0.0}
        print(f"  cargadas {len(F_nsga)} de NSGA-II y {len(F_eps)} de ε-restricción")
    else:
        print(f"\n── NSGA-II (pop={args.pop}, gen={args.gen}, seed={args.seed}, "
              f"jobs={args.jobs}) ──")
        nsga = run_nsga2(ev, urdf, args.ref, pop_size=args.pop, n_gen=args.gen,
                         seed=args.seed, n_jobs=args.jobs, verbose=True, x0=seeds)
        F_nsga, X_nsga = nsga["F"], nsga["X"]
        print(f"  {len(F_nsga)} soluciones no dominadas · {nsga['n_eval']} "
              f"evaluaciones · {nsga['elapsed_s'] / 60:.1f} min "
              f"({nsga['sec_per_eval']:.2f} s/eval)")

        hv_ref = hv_reference_point(F_nsga)
        print(f"  punto de referencia HV = {np.array2string(hv_ref, precision=4)}")

        print(f"\n── ε-restricción + SLSQP ({args.eps_steps} niveles sobre f1) ──")
        eps = run_epsilon_constraint(ev, F_nsga, X_nsga, eps_obj_idx=0,
                                     n_steps=args.eps_steps,
                                     maxiter=args.slsqp_maxiter, urdf=urdf,
                                     ref_path=args.ref, n_jobs=args.jobs,
                                     verbose=True)
        F_eps, X_eps = eps["F"], eps["X"]
        ok = int(eps["success"].sum())
        print(f"  {ok}/{len(F_eps)} convergieron · {eps['elapsed_s'] / 60:.1f} min")

    # ── Métricas de frente ───────────────────────────────────────────────────
    def _clean(F, X):
        m = np.all(np.isfinite(F), axis=1) & np.all(F < PENALTY / 10.0, axis=1)
        return F[m], X[m], m

    F_n, X_n, m_n = _clean(F_nsga, X_nsga)
    F_e, X_e, m_e = _clean(F_eps, X_eps)
    G_n = nsga["G"][m_n] if nsga["G"] is not None else None
    F_all = np.vstack([F_n, F_e]) if len(F_e) else F_n
    X_all = np.vstack([X_n, X_e]) if len(F_e) else X_n

    # SOLO se selecciona entre soluciones FACTIBLES. Sin este filtro, los puntos
    # de ε-restricción que no convergieron entran en el frente no dominado y
    # ganan justamente porque violan restricciones: dominan en el espacio de
    # objetivos a costa de χ y del error de TCP. La primera corrida seleccionó
    # así una rodilla con χ = 0.816 (límite 0.8) y TCP = 1.0022 mm (límite 1.0).
    G_all = np.array([ev.constraints(x) for x in X_all])
    feas = np.all(G_all <= 0.0, axis=1)
    n_feas_n, n_feas_e = int(feas[:len(F_n)].sum()), int(feas[len(F_n):].sum())
    print(f"\nfactibles: NSGA-II {n_feas_n}/{len(F_n)} · "
          f"ε-restricción {n_feas_e}/{len(F_e)}")
    if not feas.any():
        print("  ¡NINGUNA solución factible! Se selecciona sobre el frente "
              "completo y se marca como tal.")
        feas = np.ones(len(F_all), dtype=bool)

    F_sel, X_sel = F_all[feas], X_all[feas]
    nd = filter_nondominated(F_sel)
    F_ref_front = F_sel[nd]

    metrics = {
        "hv_nsga2": compute_hv(F_n, hv_ref),
        "hv_epsilon": compute_hv(F_e, hv_ref) if len(F_e) else 0.0,
        "hv_combined": compute_hv(F_ref_front, hv_ref),
        "hv_ref_point": hv_ref,
        "igd_nsga2": compute_igd(F_n, F_ref_front),
        "igd_epsilon": compute_igd(F_e, F_ref_front) if len(F_e) else None,
        "coverage_nsga2_over_epsilon": compute_coverage(F_n, F_e) if len(F_e) else None,
        "coverage_epsilon_over_nsga2": compute_coverage(F_e, F_n) if len(F_e) else None,
        "n_nondominated_combined": int(nd.sum()),
        "n_feasible_nsga2": n_feas_n,
        "n_feasible_epsilon": n_feas_e,
        "n_total_nsga2": int(len(F_n)),
        "n_total_epsilon": int(len(F_e)),
    }
    print("\n── Métricas de frente ──")
    for k in ("hv_nsga2", "hv_epsilon", "hv_combined", "igd_nsga2", "igd_epsilon",
              "coverage_nsga2_over_epsilon", "coverage_epsilon_over_nsga2"):
        print(f"  {k:32s} {metrics[k]}")

    # ── Selección por punto de rodilla ───────────────────────────────────────
    idx, dist = select_solution(F_ref_front, method="knee", F_ref=F_ref_front)
    x_knee = X_sel[nd][idx]
    F_knee = F_ref_front[idx]
    lam_k, eta_k, phi_k = param.gains(x_knee)
    r_knee = ev.result(x_knee)
    print("\n── Punto de rodilla ──")
    print(f"  λ   = {np.array2string(lam_k, precision=4)}")
    print(f"  η   = {np.array2string(eta_k, precision=6)}")
    _phi_txt = (np.array2string(np.atleast_1d(phi_k), precision=4)
                if np.atleast_1d(phi_k).size > 1 else f"{float(phi_k):.5f}")
    print(f"  φ   = {_phi_txt}   ·  dist. utopía (norm.) = {dist:.4f}")
    print(f"  f   = {np.array2string(F_knee, precision=5)}")
    G_knee = ev.constraints(x_knee)
    print(f"  TCP = {r_knee.rmse_tcp_mm:.4f} mm · RMSE q = {r_knee.rmse_q:.3e} "
          f"· max|s| = {r_knee.s_max:.5f} · χ = {r_knee.chi_max:.3f}")
    print(f"  g   = {np.array2string(G_knee, precision=4)}  "
          f"{'FACTIBLE' if np.all(G_knee <= 0) else '*** INFACTIBLE ***'}")

    # El extremo de máxima precisión del frente, para acotar cuánto cuesta el
    # compromiso: la rodilla equilibra los tres objetivos, pero conviene saber
    # qué seguimiento se deja sobre la mesa al hacerlo.
    i_best = int(np.argmin(F_ref_front[:, 0]))
    r_best = ev.result(X_sel[nd][i_best])
    print(f"  [extremo min-f1 del frente: f1 = {F_ref_front[i_best, 0]:.5g}, "
          f"TCP = {r_best.rmse_tcp_mm:.4f} mm, χ = {r_best.chi_max:.3f}]")

    # ── Certificación KKT ────────────────────────────────────────────────────
    lo, scale = eps["lo"], eps["scale"]
    w = np.array(args.weights, dtype=float)
    print("\n── Certificación KKT (problema de ε-restricción resuelto por SLSQP) ──")
    if args.reselect:
        # En reselección no se conoce el nivel ε con que se resolvió cada fila
        # guardada, así que se resuelve UN nivel nuevo sobre el frente factible.
        # Con `n_steps=1` ese nivel es el ε más exigente (mínimo de f1 en el
        # frente), y el resultado es un punto SQP genuino —que es lo que el plan
        # pide certificar— por el coste de un nivel en vez de la campaña entera.
        one = run_epsilon_constraint(ev, F_ref_front, X_sel[nd], eps_obj_idx=0,
                                     n_steps=1, maxiter=args.slsqp_maxiter,
                                     urdf=urdf, ref_path=args.ref,
                                     n_jobs=args.jobs, verbose=True)
        eps_lv = float(one["eps_values"][0])
        kkt = certify_kkt(ev, one["X"][0], 0, eps_lv, scale, lo)
        kkt["eps_level"] = eps_lv
        kkt["slsqp_success"] = bool(one["success"][0])
    elif len(F_e):
        # Se certifica el nivel ε cuya solución queda más cerca de la rodilla,
        # que es la que se propone como ganancia final.
        j = int(np.argmin(np.linalg.norm((F_e - F_knee) / scale, axis=1)))
        j_full = int(np.flatnonzero(m_e)[j])
        kkt = certify_kkt(ev, X_e[j], 0, float(eps["eps_values"][j_full]), scale, lo)
        kkt["eps_level"] = float(eps["eps_values"][j_full])
        kkt["slsqp_success"] = bool(eps["success"][j_full])
    else:
        kkt = {"note": "sin soluciones de ε-restricción que certificar"}
    for k in ("eps_level", "slsqp_success", "stationarity_residual",
              "stationarity_relative", "max_violation", "complementarity",
              "active", "multipliers", "note"):
        if k in kkt:
            print(f"  {k:24s} {kkt[k]}")

    # ── Sensibilidad a alpha ─────────────────────────────────────────────────
    print("\n── Sensibilidad a α (ganancias fijas del knee) ──")
    alpha_rows = alpha_sensitivity(ev, x_knee)
    print(f"  {'α':>5}{'TCP mm':>11}{'max|s|':>10}{'χ':>8}{'TV(τ)':>12}  factible")
    for r in alpha_rows:
        print(f"  {r['alpha']:5.1f}{r['tcp_rmse_mm']:11.4f}{r['s_max']:10.5f}"
              f"{r['chi']:8.3f}{r['f3_chatter']:12.1f}  "
              f"{'sí' if r['feasible'] else 'NO'}")

    # ── Línea base: suma ponderada sobre sustituto cúbico ────────────────────
    baseline = None
    if not args.no_baseline and args.mode == "scalar":
        print("\n── Línea base: suma ponderada + sustituto polinómico cúbico ──")
        baseline = run_weighted_sum_baseline(ev, w, scale, lo, seed=args.seed)
        s_knee = float((w * (F_knee - lo) / scale).sum())
        s_base = baseline["scalar_true"]
        r_base = ev.result(baseline["x"])
        baseline["tcp_rmse_mm"] = r_base.rmse_tcp_mm
        baseline["chi"] = r_base.chi_max
        print(f"  R² del sustituto            {baseline['fit_r2']:.4f}")
        print(f"  línea base   f={np.array2string(baseline['F'], precision=5)}  "
              f"TCP={r_base.rmse_tcp_mm:.4f} mm  χ={r_base.chi_max:.3f}")
        print(f"  rodilla      f={np.array2string(F_knee, precision=5)}  "
              f"TCP={r_knee.rmse_tcp_mm:.4f} mm  χ={r_knee.chi_max:.3f}")
        if not baseline["feasible"]:
            # Un porcentaje de mejora contra un punto que viola restricciones no
            # significa nada: el escalarizado de la suma ponderada no distingue
            # factible de infactible, que es precisamente su defecto. Se reporta
            # el hecho, no un número que aparentaría rigor.
            viol = [n for n, g in zip(CON_NAMES, baseline["G"]) if g > 0]
            baseline["improvement_pct_knee_vs_baseline"] = None
            print(f"  → línea base INFACTIBLE (viola {', '.join(viol)}): la mejora "
                  f"porcentual no es comparable.")
            print("    Ese es el resultado: la suma ponderada sobre sustituto no "
                  "respeta las restricciones duras.")
        else:
            denom = abs(s_base)
            impr = 100.0 * (s_base - s_knee) / denom if denom > 1e-12 else float("nan")
            baseline["improvement_pct_knee_vs_baseline"] = impr
            print(f"  escalarizado línea base     {s_base:.6g}")
            print(f"  escalarizado rodilla        {s_knee:.6g}")
            print(f"  mejora                      {impr:.1f} %")
    elif args.mode == "full":
        print("\n(línea base omitida: el sustituto cúbico en 13 variables tendría "
              "560 términos)")

    # ── Salidas ──────────────────────────────────────────────────────────────
    print("\n── Salidas ──")
    if args.reselect:
        # El frente no ha cambiado: reescribirlo solo arriesgaría corromper el
        # fichero del que se acaba de leer.
        print(f"  pareto.csv y convergence.csv intactos (reselección)")
    else:
        if os.path.exists(pareto_csv):
            os.remove(pareto_csv)
        _save_pareto(pareto_csv, param, "nsga2", X_n, F_n, G_n)
        if len(F_e):
            G_e = np.array([ev.constraints(x) for x in X_e])
            _save_pareto(pareto_csv, param, "epsilon", X_e, F_e, G_e)

        conv = os.path.join(outdir, "convergence.csv")
        with open(conv, "w") as fh:
            fh.write("gen,hypervolume,n_nondominated\n")
            for g, hv, nn in nsga["convergence"]:
                fh.write(f"{g},{hv},{nn}\n")
        print(f"  guardado → {conv}")

    _save_selected_yaml(
        os.path.join(outdir, "selected_gains.yaml"), lam_k, eta_k, phi_k, args.alpha,
        {"metodo": "knee point sobre frente combinado NSGA-II + ε-restricción",
         "objetivos": f"f1={F_knee[0]:.6g} f2={F_knee[1]:.6g} f3={F_knee[2]:.6g}",
         "TCP_RMSE_mm": f"{r_knee.rmse_tcp_mm:.4f}",
         "chi": f"{np.max(np.asarray(r_knee.chi_joint) / CHI_THRESHOLD):.4f}"
                f" del umbral (limite {args.chi_safety})",
         "semilla": args.seed, "parametrizacion": args.mode,
         "fuerza_corte_N": args.f_cut,
         "AVISO": "evaluador offline sin retardo de tuberia ni ruido de q̇: "
                  "re-verificar en Gazebo antes de la campana de la FASE 8"})

    report = {
        "run": {"controller": args.controller, "mode": args.mode, "seed": args.seed,
                "pop_size": args.pop, "n_gen": args.gen, "alpha": args.alpha,
                "f_cut_N": args.f_cut, "chi_safety": args.chi_safety,
                "chi_threshold": CHI_THRESHOLD.tolist(),
                "reference": args.ref, "urdf": urdf, "jobs": args.jobs,
                "weights": list(w)},
        "cost": {"sec_per_eval": nsga["sec_per_eval"], "n_eval_nsga2": nsga["n_eval"],
                 "nsga2_min": nsga["elapsed_s"] / 60.0,
                 "epsilon_min": eps["elapsed_s"] / 60.0,
                 "total_min": (time.time() - t_start) / 60.0,
                 "evaluator": ev.stats()},
        "phase5_start": {"F": F_f5, "G": G_f5, "tcp_rmse_mm": r_f5.rmse_tcp_mm,
                         "chi": r_f5.chi_max, "feasible": bool(np.all(G_f5 <= 0))},
        "metrics": dict(metrics, convergence_hv_ref=nsga["convergence_hv_ref"]),
        "selected": {"x": x_knee, "lambda": lam_k, "eta": eta_k, "phi": np.atleast_1d(phi_k).tolist(),
                     "F": F_knee, "norm_dist_utopia": dist,
                     "tcp_rmse_mm": r_knee.rmse_tcp_mm, "rmse_q": r_knee.rmse_q,
                     "s_max": r_knee.s_max, "chi": r_knee.chi_max},
        "min_f1_extreme": {"F": F_ref_front[i_best],
                           "tcp_rmse_mm": r_best.rmse_tcp_mm,
                           "chi": r_best.chi_max},
        "disturbance_bound_Nm": d_bound,
        "disturbance_force_Nm": d_force,
        "disturbance_friction_Nm": d_fric,
        "tcp_tol_mm": args.tcp_tol_mm,
        "kkt": kkt,
        "alpha_sensitivity": alpha_rows,
        "baseline_weighted_sum": baseline,
        "epsilon": {"n_success": int(eps["success"].sum()),
                    "n_steps": len(eps["success"]),
                    "normalisation_lo": lo, "normalisation_scale": scale},
    }
    mpath = os.path.join(outdir, "metrics.yaml")
    with open(mpath, "w") as fh:
        yaml.safe_dump(_py(report), fh, default_flow_style=False, sort_keys=False)
    print(f"  guardado → {mpath}")
    print(f"\nTotal: {(time.time() - t_start) / 60.0:.1f} min")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
