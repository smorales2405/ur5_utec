"""
Pilar 2 — Runner mono-objetivo con restricciones (Unidad 7 / 1b).

Corre los tres solvers (máxima inclinación, búsqueda directa y SLSQP) desde la
misma semilla y produce el O★ canónico del Trabajo Integrador, resolviendo el
conflicto previo YAML vs CSV: un único punto alimenta todas las cifras del paper.

Salidas (en ``results/final/``):
  singleobjective_comparison.csv      — métodos, O óptimo, f1, d_min, evals, iters
  selected_solution_final.yaml        — O★ canónico (SLSQP, validado)
  baseline_vs_optimized.csv           — CU2 baseline vs O★ (% de mejora)
  plots/steepest_descent_path.png     — contorno de J̃ + iteraciones

Uso (tras 'colcon build' y 'source install/setup.bash'):
  ros2 run ur5_trajectory_optimization run_singleobjective
  python3 -m ur5_trajectory_optimization.run_singleobjective
"""

from __future__ import annotations
import argparse, os, re
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from ament_index_python.packages import get_package_share_directory

from .run_optimization import _load_yaml, _build_config, _pkg_base
from .multiobjective_optimizer import TrajectoryEvaluator
from .singleobjective_optimizer import (
    steepest_descent, direct_search, slsqp_reference,
    scalar_objective, penalized_objective,
)


# ─────────────────────────────────────────────────────────────────────────────

def _final_dir() -> tuple[str, str]:
    results_d = os.path.join(_pkg_base(), 'results', 'final')
    plots_d   = os.path.join(results_d, 'plots')
    os.makedirs(results_d, exist_ok=True)
    os.makedirs(plots_d,   exist_ok=True)
    return results_d, plots_d


def _load_cu3_knee(default=(0.504081, -0.000313, 0.430253)) -> np.ndarray:
    """Lee el knee-point del CU3 (results/selected_solution.yaml)."""
    path = os.path.join(_pkg_base(), 'results', 'selected_solution.yaml')
    if os.path.exists(path):
        with open(path) as fh:
            doc = yaml.safe_load(fh)
        try:
            return np.array(doc['pick_place_node']['ros__parameters']['point_O'], float)
        except (KeyError, TypeError):
            pass
    return np.array(default, float)


# ─────────────────────────────────────────────────────────────────────────────

def _save_comparison_csv(path: str, results: dict) -> None:
    with open(path, 'w') as fh:
        fh.write('# Pilar 2 — comparación de solvers mono-objetivo con restricciones\n')
        fh.write('# f1: esfuerzo [N2*m2*s]  |  d_min: holgura TCP-obstáculo [m]\n')
        fh.write('metodo,xopt_x,xopt_y,xopt_z,f1,d_min,n_eval,n_iter,exito\n')
        for name, r in results.items():
            x = r['x']
            fh.write(f"{name},{x[0]:.6f},{x[1]:.6f},{x[2]:.6f},"
                     f"{r['f1']:.4f},{r['d_min']:.4f},{r['n_eval']},{r['n_iter']},"
                     f"{int(bool(r['success']))}\n")
    print(f"  saved → {path}")


def _save_final_yaml(path: str, via: np.ndarray, f1: float, f2: float,
                     d_min: float, source: str) -> None:
    doc = {'pick_place_node': {'ros__parameters':
           {'point_O': [round(float(v), 6) for v in via]}}}
    with open(path, 'w') as fh:
        fh.write(f"# Trabajo Integrador — O★ canónico  (source={source})\n")
        fh.write(f"# f1_effort={f1:.4f} N2*m2*s  f2_arclen={f2:.4f} m  "
                 f"clearance={d_min:.4f} m\n")
        yaml.dump(doc, fh, default_flow_style=False, sort_keys=False)
    print(f"  saved → {path}")


def _save_baseline_csv(path: str, via_b, f1_b, f2_b, cl_b,
                       via_o, f1_o, f2_o, cl_o) -> None:
    def pct(base, opt, lower_better=True):
        if base == 0 or base != base:
            return float('nan')
        return (base - opt) / base * 100.0 if lower_better else (opt - base) / base * 100.0
    pf1 = pct(f1_b, f1_o, True)
    pf2 = pct(f2_b, f2_o, True)
    pcl = pct(cl_b, cl_o, False)
    with open(path, 'w') as fh:
        fh.write('# CU2 baseline vs O★ canónico del Trabajo Integrador\n')
        fh.write('# pct_improve: positivo = mejor\n')
        fh.write('solution,via_x,via_y,via_z,f1_effort_N2m2s,f2_arclen_m,clearance_m,'
                 'pct_improve_f1,pct_improve_f2,pct_improve_clearance\n')
        fh.write(f'CU2-baseline,{via_b[0]:.6f},{via_b[1]:.6f},{via_b[2]:.6f},'
                 f'{f1_b:.4f},{f2_b:.4f},{cl_b:.4f},-,-,-\n')
        fh.write(f'O-star-final,{via_o[0]:.6f},{via_o[1]:.6f},{via_o[2]:.6f},'
                 f'{f1_o:.4f},{f2_o:.4f},{cl_o:.4f},'
                 f'{pf1:.2f},{pf2:.2f},{pcl:.2f}\n')
    print(f"  saved → {path}")
    return pf1, pf2, pcl


def _plot_descent_path(path: str, evaluator, bounds, mu, d_safe, history,
                       x_star, fixed_y) -> None:
    """Contorno de J̃ en el plano (via_x, via_z) a y=fixed_y + iteraciones."""
    nx, nz = 22, 22
    xs = np.linspace(bounds[0, 0], bounds[0, 1], nx)
    zs = np.linspace(bounds[2, 0], bounds[2, 1], nz)
    Z = np.zeros((nz, nx))
    for iz, zz in enumerate(zs):
        for ix, xx in enumerate(xs):
            Z[iz, ix] = penalized_objective(
                evaluator, np.array([xx, fixed_y, zz]), mu, d_safe, bounds)

    # La región IK-infactible dispara la penalización (~1e16) y aplasta la
    # escala; se recorta al rango FACTIBLE para revelar el tazón de J̃ y la
    # zona infactible se muestra en gris (color 'over').
    feas = Z < 1.0e5
    if feas.any():
        vmin, vmax = float(Z[feas].min()), float(Z[feas].max())
    else:
        vmin, vmax = float(Z.min()), float(Z.max())
    levels = np.linspace(vmin, vmax, 25)

    fig, ax = plt.subplots(figsize=(8, 6))
    cmap = plt.get_cmap('viridis').copy()
    cmap.set_over('lightgray')
    cf = ax.contourf(xs, zs, np.clip(Z, vmin, vmax), levels=levels,
                     cmap=cmap, extend='max')
    cbar = fig.colorbar(cf, ax=ax, label=r'$\tilde J(O)$  (factible; gris = IK infactible)')
    ax.contour(xs, zs, np.clip(Z, vmin, vmax), levels=levels[::3],
               colors='white', linewidths=0.4, alpha=0.5)

    pts = np.array([h[0] for h in history])   # (k, 3)
    ax.plot(pts[:, 0], pts[:, 2], 'o-', color='red', ms=6, lw=2.0,
            zorder=8, label='máxima inclinación')
    ax.plot(pts[0, 0], pts[0, 2], 's', color='white', ms=10, label='inicio $x_0$')
    ax.plot(x_star[0], x_star[2], '*', color='gold', ms=20,
            markeredgecolor='k', label='O★ (SLSQP)')
    ax.axvline(bounds[0, 0], color='cyan', ls='--', lw=1.2,
               label=r'cota activa $x=0.50$')

    # Margen para que el tramo vertical sobre x=0.50 y la esquina z=0.25 no
    # queden ocultos contra los bordes del eje.
    ax.set_xlim(bounds[0, 0] - 0.012, bounds[0, 1])
    ax.set_ylim(bounds[2, 0] - 0.010, bounds[2, 1])
    ax.set_xlabel(r'$via_x$ [m]')
    ax.set_ylabel(r'$via_z$ [m]')
    ax.set_title(f'Pilar 2 — Máxima inclinación sobre $\\tilde J$  '
                 f'(corte $y={fixed_y:.3f}$)', fontsize=11)
    ax.legend(loc='upper right', fontsize=8)
    plt.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches='tight')
    print(f"  saved → {path}")
    plt.close(fig)


# ─────────────────────────────────────────────────────────────────────────────

def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Pilar 2 — optimización mono-objetivo con restricciones.')
    parser.add_argument('--x0', nargs=3, type=float, default=None,
                        metavar=('X', 'Y', 'Z'), help='Semilla (por defecto: config sd_x0).')
    args = parser.parse_args(argv)

    print("=" * 64)
    print("Pilar 2 — Optimización mono-objetivo con restricciones (1b)")
    print("=" * 64)

    pp_share  = get_package_share_directory('ur5_pick_place')
    opt_share = get_package_share_directory('ur5_trajectory_optimization')
    kin_share = get_package_share_directory('ur5_kinematics')

    pp_params  = _load_yaml(os.path.join(pp_share,  'config', 'pick_place_params.yaml'))
    opt_params = _load_yaml(os.path.join(opt_share, 'config', 'optimization_params.yaml'))
    urdf_path  = os.path.join(kin_share, 'ur5e.urdf')
    config     = _build_config(pp_params, opt_params)

    fp = opt_params.get('final_project', {})
    mu       = float(fp.get('penalty_mu', 1.0e5))
    x0       = np.array(args.x0 if args.x0 else fp.get('sd_x0', [0.70, 0.0, 0.40]), float)
    sd_tol   = float(fp.get('sd_tol', 1.0e-4))
    sd_maxit = int(fp.get('sd_max_iter', 50))

    bounds = np.array([
        [opt_params['via_x_min'], opt_params['via_x_max']],
        [opt_params['via_y_min'], opt_params['via_y_max']],
        [opt_params['via_z_min'], opt_params['via_z_max']],
    ])

    print("\nConstruyendo TrajectoryEvaluator (Pinocchio + IK)…")
    ev = TrajectoryEvaluator(config, urdf_path)
    r_grip = ev._obstacle['r_grip']
    d_safe = r_grip + float(fp.get('d_safe_extra', config.get('obstacle_delta_safe', 0.05)))
    print(f"  d_safe = r_grip + extra = {r_grip:.3f} + "
          f"{d_safe - r_grip:.3f} = {d_safe:.3f} m")
    print(f"  semilla x0 = {x0.tolist()}  |  mu = {mu:.1e}")

    # ── Solvers ───────────────────────────────────────────────────────────────
    print("\n── Máxima inclinación (penalización + búsqueda lineal) ──")
    r_sd = steepest_descent(ev, x0, bounds, mu, d_safe, tol=sd_tol, max_iter=sd_maxit)
    print(f"  x={r_sd['x']}  f1={r_sd['f1']:.2f}  d_min={r_sd['d_min']:.4f}  "
          f"iters={r_sd['n_iter']}  evals={r_sd['n_eval']}")

    print("\n── Búsqueda directa (coordenadas, sin gradiente) ──")
    r_ds = direct_search(ev, x0, bounds, mu, d_safe, tol=sd_tol)
    print(f"  x={r_ds['x']}  f1={r_ds['f1']:.2f}  d_min={r_ds['d_min']:.4f}  "
          f"iters={r_ds['n_iter']}  evals={r_ds['n_eval']}")

    print("\n── SLSQP (referencia, restricciones explícitas) ──")
    r_sl = slsqp_reference(ev, x0, bounds, d_safe)
    print(f"  x={r_sl['x']}  f1={r_sl['f1']:.2f}  d_min={r_sl['d_min']:.4f}  "
          f"iters={r_sl['n_iter']}  evals={r_sl['n_eval']}  ok={r_sl['success']}")

    results = {'steepest_descent': r_sd, 'direct_search': r_ds, 'slsqp': r_sl}

    # ── O★ canónico = SLSQP, validado contra máxima inclinación ───────────────
    x_star = r_sl['x']
    f1_o, d_min_o = scalar_objective(ev, x_star)
    _f1, f2_o, _f3, _g1 = ev.evaluate(x_star)

    print("\n── Consistencia entre solvers ──")
    for name, r in results.items():
        print(f"  {name:18s}  ‖x − O★‖ = {np.linalg.norm(r['x'] - x_star):.4f} m  "
              f"(x_via={r['x'][0]:.4f})")

    knee = _load_cu3_knee()
    print(f"\n  knee-point CU3       = {knee.tolist()}")
    print(f"  O★ (single-objective) = {x_star.tolist()}")
    print(f"  ‖O★ − knee‖           = {np.linalg.norm(x_star - knee):.4f} m")

    # ── Salidas ───────────────────────────────────────────────────────────────
    results_d, plots_d = _final_dir()
    print("\nGuardando resultados…")
    _save_comparison_csv(os.path.join(results_d, 'singleobjective_comparison.csv'),
                         results)
    _save_final_yaml(os.path.join(results_d, 'selected_solution_final.yaml'),
                     x_star, f1_o, f2_o, d_min_o, source='slsqp')

    # Baseline CU2 vs O★
    via_cu2 = np.array(pp_params['pick_place_node']['ros__parameters']['point_O'], float)
    f1_b, f2_b, f3_b, _g1b = ev.evaluate(via_cu2)
    cl_b = -f3_b
    _save_baseline_csv(os.path.join(results_d, 'baseline_vs_optimized.csv'),
                       via_cu2, f1_b, f2_b, cl_b, x_star, f1_o, f2_o, d_min_o)

    # Trayectoria de máxima inclinación
    _plot_descent_path(os.path.join(plots_d, 'steepest_descent_path.png'),
                       ev, bounds, mu, d_safe, r_sd['history'], x_star,
                       fixed_y=float(x_star[1]))

    print("\nDone.  O★ canónico en results/final/selected_solution_final.yaml")


if __name__ == '__main__':
    main()
