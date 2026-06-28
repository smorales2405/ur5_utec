#!/usr/bin/env python3
"""
Pilar 1 — Estudio de convergencia de cuadraturas (Unidad 5).

Aplica trapecio, Simpson, Romberg y Gauss-Legendre (implementadas a mano en
``numerical_integration``) a los dos integrandos del problema (longitud de arco
y esfuerzo articular) para un punto de via ``O★`` fijo, y mide el error frente a
una referencia de malla muy fina.

Salidas:
  results/final/integration_comparison.csv
      columnas: objetivo, metodo, n, valor, error_abs, error_rel, n_evals
  results/final/plots/integration_convergence.png
      error absoluto vs n (log-log), una curva por método y objetivo.

Uso (tras 'colcon build' y 'source install/setup.bash'):
  python3 scripts/compare_integration.py
  python3 scripts/compare_integration.py --via 0.50 0.0 0.43
"""

from __future__ import annotations
import argparse, math, os, re, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from ament_index_python.packages import get_package_share_directory

from ur5_trajectory_optimization.run_optimization import (
    _load_yaml, _build_config, _pkg_base,
)
from ur5_trajectory_optimization.multiobjective_optimizer import TrajectoryEvaluator
from ur5_trajectory_optimization.integrands import (
    make_arclength_integrand, make_effort_integrand,
    keypoints_from_fixed, time_domain,
)
from ur5_trajectory_optimization.trajectory_model import spline_arc_length
from ur5_trajectory_optimization import numerical_integration as ni


# ─────────────────────────────────────────────────────────────────────────────

def _final_dir() -> tuple[str, str]:
    results_d = os.path.join(_pkg_base(), 'results', 'final')
    plots_d   = os.path.join(results_d, 'plots')
    os.makedirs(results_d, exist_ok=True)
    os.makedirs(plots_d,   exist_ok=True)
    return results_d, plots_d


def load_via_star(results_d: str, default=(0.504081, -0.000313, 0.430253)) -> np.ndarray:
    """
    Lee el O★ canónico del Pilar 2 (``selected_solution_final.yaml``) si existe,
    o del ``selected_solution.yaml`` del CU3; si no, usa el knee-point por defecto.
    """
    import yaml
    for name in ('selected_solution_final.yaml', 'selected_solution.yaml'):
        path = os.path.join(results_d, name)
        if not os.path.exists(path):
            # selected_solution.yaml del CU3 vive en results/ (raíz)
            path = os.path.join(os.path.dirname(results_d), name)
        if os.path.exists(path):
            with open(path) as fh:
                doc = yaml.safe_load(fh)
            try:
                via = doc['pick_place_node']['ros__parameters']['point_O']
                print(f"  O★ leído de {path}")
                return np.array(via, dtype=float)
            except (KeyError, TypeError):
                pass
    print("  O★ no encontrado en YAML; usando knee-point por defecto.")
    return np.array(default, dtype=float)


# ─────────────────────────────────────────────────────────────────────────────

def _apply_method(method: str, f, a: float, b: float, n: int):
    """Aplica una regla y devuelve (valor, n_evals) usando un contador."""
    cf = ni.CountingFunction(f)
    if method == 'trapezoid':
        val = ni.trapezoid(cf, a, b, n)
    elif method == 'simpson':
        val = ni.simpson(cf, a, b, n)
    elif method == 'romberg':
        max_k = max(1, round(math.log2(n)))      # 2**max_k = n subintervalos
        val, _ = ni.romberg(cf, a, b, max_k=max_k, tol=0.0)
    elif method == 'gauss':
        val = ni.gauss_legendre(cf, a, b, n)
    else:
        raise ValueError(method)
    return val, cf.n_calls


def run_study(objetivo: str, f, a: float, b: float, ref: float,
              n_values, methods) -> list:
    rows = []
    print(f"\n── Integrando: {objetivo}  (referencia = {ref:.6f}) ──")
    for method in methods:
        for n in n_values:
            val, nev = _apply_method(method, f, a, b, n)
            err_abs = abs(val - ref)
            err_rel = err_abs / abs(ref) if ref != 0.0 else float('nan')
            rows.append((objetivo, method, n, val, err_abs, err_rel, nev))
            print(f"  {method:>10s}  n={n:>4d}  val={val:14.6f}  "
                  f"err_abs={err_abs:.3e}  evals={nev}")
    return rows


# ─────────────────────────────────────────────────────────────────────────────

def _save_csv(path: str, rows: list) -> None:
    with open(path, 'w') as fh:
        fh.write('# Pilar 1 — convergencia de cuadraturas sobre los integrandos del CU3\n')
        fh.write('objetivo,metodo,n,valor,error_abs,error_rel,n_evals\n')
        for obj, m, n, val, ea, er, nev in rows:
            fh.write(f'{obj},{m},{n},{val:.8f},{ea:.8e},{er:.8e},{nev}\n')
    print(f"  saved → {path}")


def _save_plot(path: str, rows: list, n_values, methods) -> None:
    objetivos = ['arclength', 'effort']
    titles = {
        'arclength': r'$f_2$ — Longitud de arco  $\int\,\|\dot p(t)\|\,dt$',
        'effort':    r'$f_1$ — Esfuerzo  $\int\,\sum_i\tau_i(t)^2\,dt$',
    }
    colors = {'trapezoid': 'tab:blue', 'simpson': 'tab:green',
              'romberg': 'tab:red', 'gauss': 'tab:purple'}
    markers = {'trapezoid': 'o', 'simpson': 's', 'romberg': '^', 'gauss': 'D'}

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    for ax, obj in zip(axes, objetivos):
        for m in methods:
            ns  = [r[2] for r in rows if r[0] == obj and r[1] == m]
            ers = [r[4] for r in rows if r[0] == obj and r[1] == m]
            if not ns:
                continue
            ax.loglog(ns, np.maximum(ers, 1e-16), marker=markers[m],
                      color=colors[m], label=m, lw=1.5, ms=6)
        ax.set_xlabel('n  (subintervalos / nodos)')
        ax.set_ylabel('Error absoluto vs. referencia')
        ax.set_title(titles[obj], fontsize=11)
        ax.grid(True, which='both', linestyle='--', alpha=0.4)
        ax.legend(fontsize=9)

    fig.suptitle('Pilar 1 — Convergencia de cuadraturas (Newton-Cotes, Romberg, Gauss-Legendre)',
                 fontsize=12, fontweight='bold')
    plt.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches='tight')
    print(f"  saved → {path}")
    plt.close(fig)


# ─────────────────────────────────────────────────────────────────────────────

def main(argv=None):
    parser = argparse.ArgumentParser(description='Pilar 1 — estudio de convergencia.')
    parser.add_argument('--via', nargs=3, type=float, default=None, metavar=('X', 'Y', 'Z'),
                        help='Punto de via O★ (por defecto: leído del YAML canónico).')
    args = parser.parse_args(argv)

    print("=" * 64)
    print("Pilar 1 — Integración numérica comparada (Unidad 5)")
    print("=" * 64)

    pp_share  = get_package_share_directory('ur5_pick_place')
    opt_share = get_package_share_directory('ur5_trajectory_optimization')
    kin_share = get_package_share_directory('ur5_kinematics')

    pp_params  = _load_yaml(os.path.join(pp_share,  'config', 'pick_place_params.yaml'))
    opt_params = _load_yaml(os.path.join(opt_share, 'config', 'optimization_params.yaml'))
    urdf_path  = os.path.join(kin_share, 'ur5e.urdf')
    config     = _build_config(pp_params, opt_params)

    fp = opt_params.get('final_project', {})
    n_values = fp.get('integration_n_values', [4, 8, 16, 32, 64, 128])
    ref_n    = int(fp.get('integration_reference_n', 2048))

    results_d, plots_d = _final_dir()
    via = np.array(args.via, dtype=float) if args.via else load_via_star(results_d)
    print(f"\nO★ = {via.tolist()}")

    print("\nConstruyendo TrajectoryEvaluator (Pinocchio + IK)…")
    ev = TrajectoryEvaluator(config, urdf_path)
    fixed = ev._fixed
    keypoints = keypoints_from_fixed(via, fixed)

    # ── Dominio del estudio: segmento suave principal B-O-C [t_B, t_C] ─────────
    # El integrando se evalúa sobre el spline que contiene el punto de via O★.
    # Integrar a través de las uniones (dominio completo [t_A, t_D]) introduce
    # ESQUINAS: la velocidad cartesiana se anula en cada extremo clamped del
    # spline, de modo que ‖ṗ(t)‖ tiene picos en V en t_B y t_C.  Un esquema
    # global de alto orden (Gauss-Legendre) pierde allí su convergencia
    # espectral; la forma correcta de integrar a través de uniones es por
    # segmentos (lo que hace f2).  Aquí se aísla un único spline suave para que
    # el estudio compare el ORDEN real de cada regla.
    a, b = float(fixed['t_B']), float(fixed['t_C'])
    print(f"Dominio del estudio (spline suave B-O-C): [t_B, t_C] = [{a:.3f}, {b:.3f}] s")

    # ── Integrando de longitud: referencia analítica = arco exacto del segmento ─
    f_arc = make_arclength_integrand(keypoints, ev._pts_per_seg)
    ref_arc = spline_arc_length(
        [keypoints[1], keypoints[2], keypoints[3]], ev._pts_per_seg, n_quad=64)

    # ── Integrando de esfuerzo: referencia = malla muy fina (trapecio n grande) ─
    f_eff = make_effort_integrand(via, fixed, ev)
    print(f"\nCalculando referencia de esfuerzo con n={ref_n} (malla fina)…")
    ref_eff = ni.trapezoid(f_eff, a, b, ref_n)
    print(f"  ref_eff = {ref_eff:.6f}")

    methods = ['trapezoid', 'simpson', 'romberg', 'gauss']
    rows  = run_study('arclength', f_arc, a, b, ref_arc, n_values, methods)
    rows += run_study('effort',    f_eff, a, b, ref_eff, n_values, methods)

    print("\nGuardando resultados…")
    _save_csv(os.path.join(results_d, 'integration_comparison.csv'), rows)
    _save_plot(os.path.join(plots_d, 'integration_convergence.png'),
               rows, n_values, methods)
    print("\nDone.")


if __name__ == '__main__':
    main()
