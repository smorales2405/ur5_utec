#!/usr/bin/env python3
"""
Pareto-front visualisation for the CU3 multi-objective optimisation.

Generates three figures:
  1. 3D scatter of the Pareto front (f1, f2, f3 = clearance).
  2. Pairwise 2D projections (f1-f2, f1-f3, f2-f3).
  3. Parallel-coordinate plot.

Usage:
  python3 scripts/plot_pareto.py                        # auto-finds results/
  python3 scripts/plot_pareto.py --results /path/to/dir  # explicit path
"""

from __future__ import annotations
import argparse, os, sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


# ─────────────────────────────────────────────────────────────────────────────
def _default_results_dir() -> str:
    home = os.environ.get('HOME', '/tmp')
    return os.path.join(home, 'ur5_ws', 'src', 'ur5_utec',
                        'ur5_trajectory_optimization', 'results')


def _load_csv(path: str) -> tuple[np.ndarray, np.ndarray]:
    """Returns (X, F) arrays from a pareto CSV."""
    data = np.loadtxt(path, delimiter=',', skiprows=1)
    if data.ndim == 1:
        data = data[np.newaxis, :]
    return data[:, :3], data[:, 3:6]


def _load_selected(results_dir: str):
    import yaml, re
    path = os.path.join(results_dir, 'selected_solution.yaml')
    if not os.path.exists(path):
        return None
    with open(path) as f:
        content = f.read()

    doc = yaml.safe_load(content)
    try:
        via = doc['pick_place_node']['ros__parameters']['point_O']
    except (KeyError, TypeError):
        return None

    # Metadata lives in comment lines: "# f1_effort=X  f2_arclen=Y  clearance=Z"
    f1 = f2 = f3 = None
    m = re.search(r'f1_effort=([\d.]+)', content)
    if m:
        f1 = float(m.group(1))
    m = re.search(r'f2_arclen=([\d.]+)', content)
    if m:
        f2 = float(m.group(1))
    m = re.search(r'clearance=([\d.]+)', content)
    if m:
        f3 = -float(m.group(1))   # f3 = −clearance (objective to minimise)

    return {'via': via, 'f1': f1, 'f2': f2, 'f3': f3}


# ─────────────────────────────────────────────────────────────────────────────
def plot_3d(F_n: np.ndarray, F_e: np.ndarray | None, selected, title_suffix=''):
    fig = plt.figure(figsize=(9, 7))
    ax  = fig.add_subplot(111, projection='3d')

    # f3 is stored as -clearance; flip for axis label
    def _c(F): return -F[:, 2]  # clearance = -f3

    ax.scatter(F_n[:, 0], F_n[:, 1], _c(F_n),
               c='steelblue', s=30, alpha=0.7, label='NSGA-II Pareto')

    if F_e is not None and len(F_e):
        ax.scatter(F_e[:, 0], F_e[:, 1], _c(F_e),
                   c='darkorange', s=40, marker='^', alpha=0.8,
                   label='ε-constraint')

    if selected:
        ax.scatter(selected['f1'], selected['f2'], -selected['f3'],
                   c='red', s=160, marker='*', zorder=10, label='Selected (knee)')

    ax.set_xlabel('f₁ — Joint effort [N²·m²·s]', labelpad=10)
    ax.set_ylabel('f₂ — Arc length [m]',          labelpad=10)
    ax.set_zlabel('Clearance d_min [m]',           labelpad=10)
    ax.set_title(f'Pareto front — 3 objectives{title_suffix}')
    ax.legend(loc='upper left', fontsize=8)
    plt.tight_layout()
    return fig


def plot_2d_projections(F_n: np.ndarray, F_e: np.ndarray | None, selected):
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    pairs = [
        (0, 1, 'f₁ — Joint effort [N²·m²·s]', 'f₂ — Arc length [m]'),
        (0, 2, 'f₁ — Joint effort [N²·m²·s]', 'f₃ = −clearance [m]'),
        (1, 2, 'f₂ — Arc length [m]',          'f₃ = −clearance [m]'),
    ]

    for ax, (i, j, xl, yl) in zip(axes, pairs):
        ax.scatter(F_n[:, i], F_n[:, j], c='steelblue', s=20,
                   alpha=0.6, label='NSGA-II')
        if F_e is not None and len(F_e):
            ax.scatter(F_e[:, i], F_e[:, j], c='darkorange', s=30,
                       marker='^', alpha=0.8, label='ε-constraint')
        if selected:
            ax.scatter(selected[f'f{i+1}'], selected[f'f{j+1}'],
                       c='red', s=150, marker='*', zorder=10, label='Selected')
        ax.set_xlabel(xl, fontsize=9)
        ax.set_ylabel(yl, fontsize=9)
        ax.legend(fontsize=7)
        ax.grid(True, linestyle='--', alpha=0.4)

    fig.suptitle('Pareto front — pairwise projections', fontsize=11)
    plt.tight_layout()
    return fig


def plot_parallel_coords(F_n: np.ndarray, F_e: np.ndarray | None):
    labels = ['f₁\nEffort\n[N²·m²·s]', 'f₂\nArc-len\n[m]', 'f₃\n−Clear.\n[m]']
    n_axes = 3

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.set_xlim(0, n_axes - 1)
    ax.set_xticks(range(n_axes))
    ax.set_xticklabels(labels, fontsize=10)
    ax.set_yticks([])

    def _normalise(F):
        lo, hi = F.min(axis=0), F.max(axis=0)
        denom = hi - lo
        denom[denom == 0] = 1.0
        return (F - lo) / denom

    F_all = np.vstack([F_n, F_e]) if (F_e is not None and len(F_e)) else F_n
    F_norm = _normalise(F_all)
    n_nsga = len(F_n)

    for k, row in enumerate(F_norm):
        color = 'steelblue' if k < n_nsga else 'darkorange'
        alpha = 0.25 if k < n_nsga else 0.5
        lw    = 0.7  if k < n_nsga else 1.2
        ax.plot(range(n_axes), row, color=color, alpha=alpha, lw=lw)

    from matplotlib.lines import Line2D
    handles = [Line2D([0], [0], color='steelblue', lw=1.5, label='NSGA-II')]
    if F_e is not None and len(F_e):
        handles.append(Line2D([0], [0], color='darkorange', lw=1.5,
                               label='ε-constraint'))
    ax.legend(handles=handles, loc='upper right', fontsize=9)
    ax.set_title('Parallel coordinates — normalised objectives', fontsize=11)
    ax.grid(axis='x', linestyle='--', alpha=0.3)
    plt.tight_layout()
    return fig


# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--results', default='', help='Path to results directory')
    parser.add_argument('--save', action='store_true',
                        help='Save figures as PNG instead of showing')
    args = parser.parse_args()

    results_d = args.results or _default_results_dir()
    print(f"Results directory: {results_d}")

    nsga2_csv = os.path.join(results_d, 'pareto_nsga2.csv')
    eps_csv   = os.path.join(results_d, 'pareto_epsilon.csv')

    if not os.path.exists(nsga2_csv):
        print(f"ERROR: {nsga2_csv} not found. Run 'run_optimization' first.")
        sys.exit(1)

    _, F_n = _load_csv(nsga2_csv)

    F_e = None
    if os.path.exists(eps_csv):
        _, F_e = _load_csv(eps_csv)

    selected = _load_selected(results_d)
    if selected and selected.get('f1') is not None:
        print(f"Selected solution: f1={selected['f1']:.4f}  "
              f"f2={selected['f2']:.4f}  clearance={-selected['f3']:.4f} m")
    elif selected:
        print(f"Selected solution: via={selected['via']}  (metadata not in file)")

    fig1 = plot_3d(F_n, F_e, selected)
    fig2 = plot_2d_projections(F_n, F_e, selected)
    fig3 = plot_parallel_coords(F_n, F_e)

    if args.save:
        for fig, name in [(fig1, '3d'), (fig2, '2d'), (fig3, 'parallel')]:
            path = os.path.join(results_d, f'pareto_{name}.png')
            fig.savefig(path, dpi=150, bbox_inches='tight')
            print(f"Saved: {path}")
    else:
        plt.show()


if __name__ == '__main__':
    main()
