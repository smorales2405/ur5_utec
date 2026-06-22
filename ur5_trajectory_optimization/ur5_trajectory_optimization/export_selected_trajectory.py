"""
Export a selected Pareto-front solution as a ROS 2 param override YAML.

Este script usa argparse (NO es un nodo ROS2): los argumentos van directamente
después del comando, sin --ros-args ni -p.

Uso:
  # Knee point automático del frente ε-constraint (recomendado):
  ros2 run ur5_trajectory_optimization export_trajectory

  # Knee point automático del frente NSGA-II:
  ros2 run ur5_trajectory_optimization export_trajectory --source nsga2

  # Índice específico del frente ε-constraint:
  ros2 run ur5_trajectory_optimization export_trajectory --index 3

  # Índice específico del frente NSGA-II:
  ros2 run ur5_trajectory_optimization export_trajectory --source nsga2 --index 7

El archivo generado <results_dir>/selected_solution.yaml se pasa al launch:
  ros2 launch ur5_pick_place pick_place.launch.py \\
    extra_params_file:=$HOME/ur5_ws/src/ur5_utec/ur5_trajectory_optimization/results/selected_solution.yaml
"""

from __future__ import annotations
import os, sys, argparse
import numpy as np
import yaml

from ament_index_python.packages import get_package_share_directory

from .multiobjective_optimizer import select_solution


def _results_root(opt_params: dict) -> str:
    """Always returns results/ root — used for writing selected_solution.yaml."""
    home = os.environ.get('HOME', '/tmp')
    return opt_params.get('results_dir', '') or os.path.join(
        home, 'ur5_ws', 'src', 'ur5_utec', 'ur5_trajectory_optimization', 'results')


def _results_dir(opt_params: dict, test_id: int | None = None) -> str:
    """Returns the directory where Pareto CSVs are read from (testN/ when given)."""
    root = _results_root(opt_params)
    if test_id is not None:
        return os.path.join(root, f'test{test_id}')
    return root


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Export a Pareto-front solution as ROS 2 param override YAML.')
    parser.add_argument('--index', '-i', type=int, default=-1,
                        help='Row index in the chosen CSV (-1 = auto-select)')
    parser.add_argument('--source', choices=['nsga2', 'epsilon'], default='epsilon',
                        help='Which CSV to use (default: epsilon)')
    parser.add_argument('--method', choices=['knee', 'min_effort', 'weighted'],
                        default='',
                        help='Selection method when --index=-1 (overrides config)')
    parser.add_argument('--test', '-t', type=int, default=None, metavar='N',
                        help='Test number N: read/write from results/testN/')
    args = parser.parse_args(argv)

    # Load optimization params for results_dir
    opt_share  = get_package_share_directory('ur5_trajectory_optimization')
    opt_yaml   = os.path.join(opt_share, 'config', 'optimization_params.yaml')
    with open(opt_yaml) as f:
        opt_params = yaml.safe_load(f)

    csv_dir   = _results_dir(opt_params, test_id=args.test)   # where CSVs live
    export_d  = _results_root(opt_params)                      # where YAML is written
    csv_name  = f'pareto_{args.source}.csv'
    csv_path  = os.path.join(csv_dir, csv_name)

    if not os.path.exists(csv_path):
        print(f"ERROR: {csv_path} not found. Run 'run_optimization' first.", file=sys.stderr)
        sys.exit(1)

    data   = np.loadtxt(csv_path, delimiter=',', skiprows=1)
    if data.ndim == 1:
        data = data[np.newaxis, :]

    X = data[:, :3]    # via_x, via_y, via_z
    F = data[:, 3:6]   # f1, f2, f3

    # Build combined reference front (NSGA-II + ε) for normalisation, so that
    # the knee is not artificially biased by the narrow range of a single front.
    F_ref = F
    for other_csv in ['pareto_nsga2.csv', 'pareto_epsilon.csv']:
        other_path = os.path.join(csv_dir, other_csv)
        if other_path == csv_path or not os.path.exists(other_path):
            continue
        try:
            other = np.loadtxt(other_path, delimiter=',', skiprows=1)
            if other.ndim == 1:
                other = other[np.newaxis, :]
            F_ref = np.vstack([F_ref, other[:, 3:6]])
        except Exception:
            pass

    sel_method  = args.method or opt_params.get('selection_method', 'knee')
    sel_weights = opt_params.get('weights', [1.0, 1.0, 1.0])
    norm_dist   = None

    if args.index < 0:
        idx, norm_dist = select_solution(
            F, method=sel_method, weights=sel_weights, F_ref=F_ref,
        )
        print(f"Auto-selected ({sel_method}): index {idx}  "
              f"norm_dist={norm_dist:.4f}")
    else:
        idx = args.index
        if idx >= len(X):
            print(f"ERROR: index {idx} out of range (Pareto front has {len(X)} solutions).")
            sys.exit(1)
        sel_method = 'manual'

    via = X[idx]
    f   = F[idx]

    print(f"Selected solution:")
    print(f"  point_O = {via.tolist()}")
    print(f"  f1 (effort)    = {f[0]:.4f} N²·m²·s")
    print(f"  f2 (arc-len)   = {f[1]:.4f} m")
    print(f"  f3 (clearance) = {-f[2]:.4f} m")

    # Write override YAML — only pick_place_node/ros__parameters is valid for rcl.
    # Extra metadata goes as comments so the file parses cleanly.
    out_path = os.path.join(export_d, 'selected_solution.yaml')
    doc = {
        'pick_place_node': {
            'ros__parameters': {
                'point_O': [round(float(v), 6) for v in via],
            }
        },
    }
    with open(out_path, 'w') as fh:
        fh.write(f"# CU3 selected solution — source={csv_name}"
                 f"  method={sel_method}  idx={idx}\n")
        fh.write(f"# f1_effort={f[0]:.4f} N2*m2*s  "
                 f"f2_arclen={f[1]:.4f} m  "
                 f"clearance={-f[2]:.4f} m\n")
        if norm_dist is not None:
            fh.write(f"# norm_dist_utopia={norm_dist:.4f}"
                     f"  (normalised over combined NSGA-II+ε front)\n")
        yaml.dump(doc, fh, default_flow_style=False, sort_keys=False)

    print(f"\nWritten: {out_path}")
    print(
        "\nTo run with optimised via-point:\n"
        "  ros2 run ur5_pick_place pick_place_node \\\n"
        f"    --ros-args --params-file {get_package_share_directory('ur5_pick_place')}"
        f"/config/pick_place_params.yaml \\\n"
        f"               --params-file {out_path}"
    )


if __name__ == '__main__':
    main()
