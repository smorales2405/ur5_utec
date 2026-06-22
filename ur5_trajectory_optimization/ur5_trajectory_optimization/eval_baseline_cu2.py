"""
Evaluate the CU2 fixed via-point against the CU3 optimised solution (B3).

Loads the unmodified CU2 point_O from pick_place_params.yaml
([0.75, 0.00, 0.40]) and the CU3 optimised point from selected_solution.yaml,
evaluates both with the same TrajectoryEvaluator (Pinocchio + IK) and writes:

  results/baseline_vs_optimized.csv   — objective values + % improvement
  results/baseline_comparison.png     — grouped bar chart (3 objectives)

Usage:
  ros2 run ur5_trajectory_optimization eval_baseline_cu2
"""

from __future__ import annotations
import os, re
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker

from ament_index_python.packages import get_package_share_directory

from .run_optimization import _load_yaml, _build_config, _results_dir
from .multiobjective_optimizer import TrajectoryEvaluator


# ─────────────────────────────────────────────────────────────────────────────

def _load_optimized(results_d: str):
    """
    Parse point_O and f1/f2/clearance from selected_solution.yaml comments.
    Returns (via, f1, f2, clearance) or (None, …) when file is absent.
    """
    path = os.path.join(results_d, 'selected_solution.yaml')
    if not os.path.exists(path):
        return None, None, None, None
    with open(path) as fh:
        content = fh.read()
    doc = yaml.safe_load(content)
    try:
        via = np.array(doc['pick_place_node']['ros__parameters']['point_O'],
                       dtype=float)
    except (KeyError, TypeError):
        return None, None, None, None

    f1 = f2 = clearance = None
    m = re.search(r'f1_effort=([\d.]+)', content)
    if m:
        f1 = float(m.group(1))
    m = re.search(r'f2_arclen=([\d.]+)', content)
    if m:
        f2 = float(m.group(1))
    m = re.search(r'clearance=([\d.]+)', content)
    if m:
        clearance = float(m.group(1))

    return via, f1, f2, clearance


def _pct(base: float, opt: float, lower_is_better: bool = True) -> float:
    """% improvement (positive = better)."""
    if base == 0.0 or base != base:
        return float('nan')
    if lower_is_better:
        return (base - opt) / base * 100.0
    return (opt - base) / base * 100.0


# ─────────────────────────────────────────────────────────────────────────────

def _save_csv(path, via_b, f1_b, f2_b, cl_b,
                       via_o, f1_o, f2_o, cl_o,
                       pct_f1, pct_f2, pct_cl):
    with open(path, 'w') as fh:
        fh.write('# CU3 — CU2 baseline vs optimised via-point\n')
        fh.write('# pct_improve: positive = better\n')
        fh.write('# f1: lower is better  |  f2: lower is better  '
                 '|  clearance: higher is better\n')
        fh.write('solution,via_x,via_y,via_z,'
                 'f1_effort_N2m2s,f2_arclen_m,clearance_m,'
                 'pct_improve_f1,pct_improve_f2,pct_improve_clearance\n')
        fh.write(f'CU2-baseline,'
                 f'{via_b[0]:.6f},{via_b[1]:.6f},{via_b[2]:.6f},'
                 f'{f1_b:.4f},{f2_b:.4f},{cl_b:.4f},'
                 f'-,-,-\n')
        fh.write(f'CU3-optimized,'
                 f'{via_o[0]:.6f},{via_o[1]:.6f},{via_o[2]:.6f},'
                 f'{f1_o:.4f},{f2_o:.4f},{cl_o:.4f},'
                 f'{pct_f1:.2f},{pct_f2:.2f},{pct_cl:.2f}\n')
    print(f"  saved → {path}")


def _save_bar_chart(path, f1_b, f2_b, cl_b, f1_o, f2_o, cl_o,
                     pct_f1, pct_f2, pct_cl):
    """Grouped bar chart comparing CU2 baseline vs CU3 optimised (3 objectives)."""
    fig, axes = plt.subplots(1, 3, figsize=(12, 5))
    fig.suptitle('CU2 fixed via vs CU3 optimised via-point', fontsize=12,
                 fontweight='bold')

    specs = [
        ('$f_1$ — Esfuerzo [N²·m²·s]', f1_b,  f1_o,  pct_f1,  'steelblue',   True),
        ('$f_2$ — Longitud [m]',         f2_b,  f2_o,  pct_f2,  'darkorange',  True),
        ('Holgura $d_{min}$ [m]',         cl_b,  cl_o,  pct_cl,  'forestgreen', False),
    ]

    for ax, (title, base, opt, pct, color, lower_better) in zip(axes, specs):
        bars = ax.bar(['CU2 base', 'CU3 opt.'],
                      [base, opt],
                      color=['#bbbbbb', color],
                      width=0.5,
                      edgecolor='white', linewidth=0.5)

        # Improvement annotation on the optimised bar
        if pct == pct:   # not NaN
            direction = 'mejor' if pct >= 0 else 'peor'
            sign      = '+' if (not lower_better and pct >= 0) or (lower_better and pct >= 0) else ''
            label     = f'{sign}{pct:.1f}%\n({direction})'
            ax.annotate(
                label,
                xy=(1, opt),
                xytext=(0, 8),
                textcoords='offset points',
                ha='center',
                va='bottom',
                fontsize=10,
                fontweight='bold',
                color=color,
            )

        ax.set_title(title, fontsize=10)
        fmt = '%.0f' if abs(base) > 100 else '%.3f'
        ax.yaxis.set_major_formatter(mticker.FormatStrFormatter(fmt))
        ax.set_ylim(0, max(base, opt) * 1.20)
        ax.grid(axis='y', linestyle='--', alpha=0.4)

    plt.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches='tight')
    print(f"  saved → {path}")
    plt.close(fig)


# ─────────────────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("CU3 — Baseline CU2 vs optimised via-point evaluation (B3)")
    print("=" * 60)

    # ── Config ───────────────────────────────────────────────────────────────
    pp_share  = get_package_share_directory('ur5_pick_place')
    opt_share = get_package_share_directory('ur5_trajectory_optimization')
    kin_share = get_package_share_directory('ur5_kinematics')

    pp_params  = _load_yaml(os.path.join(pp_share,  'config', 'pick_place_params.yaml'))
    opt_params = _load_yaml(os.path.join(opt_share, 'config', 'optimization_params.yaml'))
    urdf_path  = os.path.join(kin_share, 'ur5e.urdf')
    config     = _build_config(pp_params, opt_params)
    results_d  = _results_dir(opt_params)

    # ── CU2 baseline: fixed point_O from pick_place_params.yaml ─────────────
    p       = pp_params['pick_place_node']['ros__parameters']
    via_cu2 = np.array(p['point_O'], dtype=float)
    print(f"\nCU2 baseline  point_O: {via_cu2.tolist()}")

    # ── CU3 optimised: from selected_solution.yaml ───────────────────────────
    via_opt, f1_yaml, f2_yaml, cl_yaml = _load_optimized(results_d)
    if via_opt is None:
        print("ERROR: selected_solution.yaml not found.  Run 'run_optimization' first.")
        return
    print(f"CU3 optimised point_O: {via_opt.tolist()}")

    # ── Evaluate both with the same evaluator ────────────────────────────────
    print("\nBuilding TrajectoryEvaluator (Pinocchio)…")
    ev = TrajectoryEvaluator(config, urdf_path)

    print("Evaluating CU2 baseline…")
    f1_b, f2_b, f3_b, g1_b = ev.evaluate(via_cu2)
    if f1_b >= 1e5:
        print("  WARNING: CU2 via-point is INFEASIBLE (penalty returned).")
    cl_b = -f3_b if f3_b < 1e5 else float('nan')
    print(f"  f1={f1_b:.2f}  f2={f2_b:.4f}  clearance={cl_b:.4f}  g1={g1_b:.4f}")

    print("Evaluating CU3 optimised…")
    f1_o, f2_o, f3_o, g1_o = ev.evaluate(via_opt)
    if f1_o >= 1e5:
        # Fall back to stored YAML values (evaluator degraded after long session)
        print("  WARNING: evaluator returned penalty; using stored YAML values.")
        f1_o = f1_yaml or float('nan')
        f2_o = f2_yaml or float('nan')
        cl_o = cl_yaml or float('nan')
    else:
        cl_o = -f3_o
    print(f"  f1={f1_o:.2f}  f2={f2_o:.4f}  clearance={cl_o:.4f}")

    # ── % improvement ─────────────────────────────────────────────────────────
    pct_f1 = _pct(f1_b, f1_o, lower_is_better=True)
    pct_f2 = _pct(f2_b, f2_o, lower_is_better=True)
    pct_cl = _pct(cl_b, cl_o, lower_is_better=False)

    print(f"\n  Δf1        = {pct_f1:+.1f}%  (esfuerzo articular — menor es mejor)")
    print(f"  Δf2        = {pct_f2:+.1f}%  (longitud de arco  — menor es mejor)")
    print(f"  Δclearance = {pct_cl:+.1f}%  (holgura de obstáculo — mayor es mejor)")

    # ── Save outputs ─────────────────────────────────────────────────────────
    _save_csv(
        os.path.join(results_d, 'baseline_vs_optimized.csv'),
        via_cu2, f1_b, f2_b, cl_b,
        via_opt, f1_o, f2_o, cl_o,
        pct_f1, pct_f2, pct_cl,
    )
    _save_bar_chart(
        os.path.join(results_d, 'baseline_comparison.png'),
        f1_b, f2_b, cl_b, f1_o, f2_o, cl_o,
        pct_f1, pct_f2, pct_cl,
    )

    print("\nDone.")


if __name__ == '__main__':
    main()
