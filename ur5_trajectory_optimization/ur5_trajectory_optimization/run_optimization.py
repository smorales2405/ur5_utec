"""
Entry point: runs the two-stage CU3 optimisation pipeline.

Stage 1 — NSGA-II: approximates the Pareto front.
Stage 2 — ε-constraint: refines compromise solutions.

Usage (after colcon build and source install/setup.bash):
  ros2 run ur5_trajectory_optimization run_optimization

Or standalone:
  python3 -m ur5_trajectory_optimization.run_optimization

Results written to:
  <results_dir>/pareto_nsga2.csv
  <results_dir>/pareto_epsilon.csv
  <results_dir>/selected_solution.yaml
"""

from __future__ import annotations
import os, sys, time
import numpy as np
import yaml

from ament_index_python.packages import get_package_share_directory

from .ik_interface import rpy_to_matrix
from .multiobjective_optimizer import (
    TrajectoryEvaluator,
    run_nsga2,
    run_epsilon_constraint,
    select_knee_point,
)


# ─────────────────────────────────────────────────────────────────────────────
# Config loading helpers
# ─────────────────────────────────────────────────────────────────────────────

def _load_yaml(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def _build_config(pp_params: dict, opt_params: dict) -> dict:
    """Merge pick_place_params and optimization_params into a single config dict."""
    p  = pp_params['pick_place_node']['ros__parameters']
    rpy = p['tcp_orientation_rpy']
    cfg = {
        # Trajectory fixed points
        'point_A_pre':    p['point_A_pre'],
        'point_A':        p['point_A'],
        'point_B':        p['point_B'],
        'point_B_post':   p['point_B_post'],
        'R_tcp':          rpy_to_matrix(*rpy),
        'total_duration': p['total_duration'],
        'pre_post_duration': p['pre_post_duration'],
        'home_joint_angles': p['home_joint_angles'],
        # Obstacle geometry
        'obstacle_center':       p['obstacle_center'],
        'obstacle_half_extents': p['obstacle_half_extents'],
        'obstacle_r_grip':       p['obstacle_r_grip'],
        'obstacle_delta_safe':   p.get('obstacle_delta_safe', 0.05),
        # Joint limits
        'joint_q_min': p['joint_q_min'],
        'joint_q_max': p['joint_q_max'],
        # Optimiser-specific
        'pts_per_seg':    opt_params.get('pts_per_seg', 8),
        'ik_max_iter':    opt_params.get('ik_max_iter', 80),
        'ik_tol':         opt_params.get('ik_tol',      1e-4),
        'ik_lambda':      opt_params.get('ik_lambda',   0.05),
        'ik_alpha':       opt_params.get('ik_alpha',    0.8),
    }
    return cfg


def _results_dir(opt_params: dict) -> str:
    d = opt_params.get('results_dir', '')
    if not d:
        home = os.environ.get('HOME', '/tmp')
        d = os.path.join(home, 'ur5_ws', 'src', 'ur5_utec',
                         'ur5_trajectory_optimization', 'results')
    os.makedirs(d, exist_ok=True)
    return d


# ─────────────────────────────────────────────────────────────────────────────
# CSV helpers
# ─────────────────────────────────────────────────────────────────────────────

def _save_pareto_csv(path: str, X: np.ndarray, F: np.ndarray, G: np.ndarray | None = None):
    header = 'via_x,via_y,via_z,f1_effort,f2_arclen,f3_clearance'
    if G is not None:
        header += ',g1_constr'
        data = np.hstack([X, F, G])
    else:
        data = np.hstack([X, F])
    np.savetxt(path, data, delimiter=',', header=header, comments='', fmt='%.8f')
    print(f"  saved → {path}")


# ─────────────────────────────────────────────────────────────────────────────
# Selected solution YAML
# ─────────────────────────────────────────────────────────────────────────────

def _save_selected_yaml(path: str, via: np.ndarray, F: np.ndarray, idx: int):
    doc = {
        'pick_place_node': {
            'ros__parameters': {
                'point_via': [round(float(v), 6) for v in via],
            }
        },
    }
    with open(path, 'w') as f:
        f.write(f"# CU3 selected solution — knee idx={idx}\n")
        f.write(f"# f1_effort={F[0]:.4f} N2*m2*s  "
                f"f2_arclen={F[1]:.4f} m  "
                f"clearance={-F[2]:.4f} m\n")
        yaml.dump(doc, f, default_flow_style=False, sort_keys=False)
    print(f"  saved → {path}")


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("CU3 — Multi-objective trajectory optimisation")
    print("=" * 60)

    # ── Load config ──────────────────────────────────────────────────────────
    pp_share  = get_package_share_directory('ur5_pick_place')
    opt_share = get_package_share_directory('ur5_trajectory_optimization')

    pp_yaml  = os.path.join(pp_share,  'config', 'pick_place_params.yaml')
    opt_yaml = os.path.join(opt_share, 'config', 'optimization_params.yaml')

    print(f"\nLoading params:")
    print(f"  {pp_yaml}")
    print(f"  {opt_yaml}")

    pp_params  = _load_yaml(pp_yaml)
    opt_params = _load_yaml(opt_yaml)
    config     = _build_config(pp_params, opt_params)
    results_d  = _results_dir(opt_params)

    # ── URDF path ────────────────────────────────────────────────────────────
    kin_share = get_package_share_directory('ur5_kinematics')
    urdf_path = os.path.join(kin_share, 'ur5e.urdf')
    print(f"  URDF: {urdf_path}")

    # ── Bounds ───────────────────────────────────────────────────────────────
    bounds = np.array([
        [opt_params['via_x_min'], opt_params['via_x_max']],
        [opt_params['via_y_min'], opt_params['via_y_max']],
        [opt_params['via_z_min'], opt_params['via_z_max']],
    ])
    print(f"\nDecision variable bounds (Pinocchio frame):")
    print(f"  x_via ∈ [{bounds[0,0]:.2f}, {bounds[0,1]:.2f}]  m")
    print(f"  y_via ∈ [{bounds[1,0]:.2f}, {bounds[1,1]:.2f}]  m")
    print(f"  z_via ∈ [{bounds[2,0]:.2f}, {bounds[2,1]:.2f}]  m")

    # ── Stage 1: NSGA-II ─────────────────────────────────────────────────────
    pop_size = opt_params.get('pop_size', 60)
    n_gen    = opt_params.get('n_gen',   120)
    seed     = opt_params.get('seed',    42)

    print(f"\n── Stage 1: NSGA-II  (pop={pop_size}, gen={n_gen}) ──")
    t0 = time.time()
    evaluator = TrajectoryEvaluator(config, urdf_path)
    nsga2_res = run_nsga2(evaluator, bounds, pop_size, n_gen, seed, verbose=True)
    print(f"  Elapsed: {time.time() - t0:.1f} s")

    X_p, F_p, G_p = nsga2_res['X'], nsga2_res['F'], nsga2_res['G']
    print(f"  Pareto front: {len(X_p)} solutions")

    # Remove infeasible (penalised) members before saving
    feasible_mask = np.all(F_p < 1e5, axis=1)
    if feasible_mask.sum() == 0:
        print("  WARNING: no feasible solutions found in Pareto front.")
        feasible_mask = np.ones(len(X_p), dtype=bool)

    X_f = X_p[feasible_mask]
    F_f = F_p[feasible_mask]
    G_f = G_p[feasible_mask] if G_p is not None else None

    _save_pareto_csv(
        os.path.join(results_d, 'pareto_nsga2.csv'), X_f, F_f, G_f)

    # ── Stage 2: ε-constraint ────────────────────────────────────────────────
    n_eps      = opt_params.get('n_epsilon_steps', 25)
    eps_obj    = opt_params.get('epsilon_obj_idx', 0)

    print(f"\n── Stage 2: ε-constraint  (n_ε={n_eps}, obj={eps_obj}) ──")
    t1 = time.time()
    eps_res = run_epsilon_constraint(
        evaluator, F_f, X_f, bounds,
        eps_obj_idx=eps_obj, n_steps=n_eps, verbose=True,
    )
    print(f"  Elapsed: {time.time() - t1:.1f} s")

    X_e = eps_res['X']
    F_e = eps_res['F']

    eps_feasible = np.all(F_e < 1e5, axis=1)
    X_e = X_e[eps_feasible]
    F_e = F_e[eps_feasible]

    _save_pareto_csv(
        os.path.join(results_d, 'pareto_epsilon.csv'), X_e, F_e)

    # ── Select knee point ────────────────────────────────────────────────────
    combined_F = np.vstack([F_f, F_e]) if len(F_e) else F_f
    combined_X = np.vstack([X_f, X_e]) if len(X_e) else X_f
    knee_idx   = select_knee_point(combined_F)

    print(f"\n── Knee-point selection ──")
    print(f"  idx = {knee_idx}")
    print(f"  via = {combined_X[knee_idx]}")
    print(f"  F   = {combined_F[knee_idx]}")

    _save_selected_yaml(
        os.path.join(results_d, 'selected_solution.yaml'),
        combined_X[knee_idx], combined_F[knee_idx], knee_idx,
    )

    print(f"\nDone. Results in {results_d}/")
    print(
        "\nTo validate in Gazebo:\n"
        "  ros2 launch ur5_pick_place ur5_robotiq_gz.launch.py\n"
        "  ros2 launch ur5_pick_place pick_place.launch.py "
        f"  # then override: --ros-args --params-file {results_d}/selected_solution.yaml"
    )


if __name__ == '__main__':
    main()
