"""
Export a selected Pareto-front solution as a ROS 2 param override YAML.

Usage:
  # Knee point (auto-selected):
  ros2 run ur5_trajectory_optimization export_trajectory

  # Specific index from pareto_nsga2.csv:
  ros2 run ur5_trajectory_optimization export_trajectory --ros-args -p index:=7

The generated file <results_dir>/selected_solution.yaml can be passed
to pick_place_node at runtime:
  ros2 run ur5_pick_place pick_place_node \\
    --ros-args --params-file <pick_place_params.yaml> \\
               --params-file <results_dir>/selected_solution.yaml
"""

from __future__ import annotations
import os, sys, argparse
import numpy as np
import yaml

from ament_index_python.packages import get_package_share_directory

from .multiobjective_optimizer import select_knee_point


def _results_dir(opt_params: dict) -> str:
    d = opt_params.get('results_dir', '')
    if not d:
        home = os.environ.get('HOME', '/tmp')
        d = os.path.join(home, 'ur5_ws', 'src', 'ur5_utec',
                         'ur5_trajectory_optimization', 'results')
    return d


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Export a Pareto-front solution as ROS 2 param override YAML.')
    parser.add_argument('--index', '-i', type=int, default=-1,
                        help='Row index in pareto_nsga2.csv (-1 = knee point)')
    parser.add_argument('--source', choices=['nsga2', 'epsilon'], default='nsga2',
                        help='Which CSV to use (default: nsga2)')
    args = parser.parse_args(argv)

    # Load optimization params for results_dir
    opt_share  = get_package_share_directory('ur5_trajectory_optimization')
    opt_yaml   = os.path.join(opt_share, 'config', 'optimization_params.yaml')
    with open(opt_yaml) as f:
        opt_params = yaml.safe_load(f)

    results_d = _results_dir(opt_params)
    csv_name  = f'pareto_{args.source}.csv'
    csv_path  = os.path.join(results_d, csv_name)

    if not os.path.exists(csv_path):
        print(f"ERROR: {csv_path} not found. Run 'run_optimization' first.", file=sys.stderr)
        sys.exit(1)

    data   = np.loadtxt(csv_path, delimiter=',', skiprows=1)
    if data.ndim == 1:
        data = data[np.newaxis, :]

    X = data[:, :3]    # via_x, via_y, via_z
    F = data[:, 3:6]   # f1, f2, f3

    if args.index < 0:
        idx = select_knee_point(F)
        print(f"Auto-selected knee point: index {idx}")
    else:
        idx = args.index
        if idx >= len(X):
            print(f"ERROR: index {idx} out of range (Pareto front has {len(X)} solutions).")
            sys.exit(1)

    via = X[idx]
    f   = F[idx]

    print(f"Selected solution:")
    print(f"  point_via = {via.tolist()}")
    print(f"  f1 (effort)    = {f[0]:.4f} N²·m²·s")
    print(f"  f2 (arc-len)   = {f[1]:.4f} m")
    print(f"  f3 (clearance) = {-f[2]:.4f} m")

    # Write override YAML
    out_path = os.path.join(results_d, 'selected_solution.yaml')
    doc = {
        'pick_place_node': {
            'ros__parameters': {
                'point_via': [round(float(v), 6) for v in via],
            }
        },
        '_cu3_info': {
            'source':          csv_name,
            'selected_idx':    int(idx),
            'f1_effort_Nm2s':  float(f[0]),
            'f2_arclen_m':     float(f[1]),
            'f3_clearance_m':  float(-f[2]),
        },
    }
    with open(out_path, 'w') as fh:
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
