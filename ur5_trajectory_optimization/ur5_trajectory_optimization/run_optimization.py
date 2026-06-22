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
import argparse, os, sys, time
import numpy as np
import yaml

from ament_index_python.packages import get_package_share_directory

from .ik_interface import rpy_to_matrix
from .multiobjective_optimizer import (
    TrajectoryEvaluator,
    run_nsga2,
    run_epsilon_constraint,
    run_epsilon_constraint_2d,
    select_solution,
)
from .metrics import (
    HV_REF_POINT,
    compute_hv,
    compute_spacing,
    compute_igd,
    compute_coverage,
    filter_nondominated,
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
        'point_A':    p['point_A'],
        'point_B':        p['point_B'],
        'point_C':        p['point_C'],
        'point_D':   p['point_D'],
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


def _pkg_base() -> str:
    home = os.environ.get('HOME', '/tmp')
    return os.path.join(home, 'ur5_ws', 'src', 'ur5_utec',
                        'ur5_trajectory_optimization')


def _results_dir(opt_params: dict) -> str:
    d = opt_params.get('results_dir', '') or os.path.join(_pkg_base(), 'results')
    os.makedirs(d, exist_ok=True)
    return d


def _test_dirs(opt_params: dict, test_id: int | None):
    """
    Return (results_d, pareto_plots_d) resolving any --test N override.

    test_id=None  → results/,  results/                           (backward-compat)
    test_id=N     → results/testN/,  results/testN/plots/pareto/
    """
    base = _results_dir(opt_params)
    if test_id is None:
        return base, base
    results_d = os.path.join(_pkg_base(), 'results', f'test{test_id}')
    plots_d   = os.path.join(results_d, 'plots', 'pareto')
    os.makedirs(results_d, exist_ok=True)
    os.makedirs(plots_d,   exist_ok=True)
    return results_d, plots_d


# ─────────────────────────────────────────────────────────────────────────────
# CSV helpers
# ─────────────────────────────────────────────────────────────────────────────

def _save_convergence_csv(path: str, history: list, ref_point: np.ndarray) -> None:
    """Save per-generation (gen, hypervolume, n_nondominated) to convergence_nsga2.csv."""
    with open(path, 'w') as fh:
        fh.write(f'# NSGA-II convergence — HV reference point: {list(ref_point)}\n')
        fh.write('gen,hypervolume,n_nondominated\n')
        for gen, hv, n_nd in history:
            hv_str = f'{hv:.6f}' if hv == hv else 'nan'   # NaN-safe
            fh.write(f'{gen},{hv_str},{n_nd}\n')
    print(f"  saved → {path}")


def _save_method_comparison_csv(
    path:       str,
    F_N:        np.ndarray,
    F_E:        np.ndarray,
    t_nsga2:    float,
    t_eps:      float,
    n_eval_N:   int,
    n_eval_E:   int,
) -> None:
    """
    Compute and write method_comparison.csv (one row per method).

    Columns:
      method, n_solutions, n_nondominated, hypervolume, spacing,
      igd_vs_combined, c_vs_other, c_by_other, time_s, n_evals

    Metrics:
      hypervolume     — HV of non-dominated subset w.r.t. HV_REF_POINT.
      spacing         — Schott spacing of non-dominated subset.
      igd_vs_combined — IGD from this method's ND front to the combined ND front.
      c_vs_other      — C(A,B): fraction of other method's ND front dominated by A.
      c_by_other      — C(B,A): fraction of A's ND front dominated by other method.
    """
    has_eps = len(F_E) > 0

    mask_N = filter_nondominated(F_N)
    FN_nd  = F_N[mask_N]

    if has_eps:
        mask_E = filter_nondominated(F_E)
        FE_nd  = F_E[mask_E]
        combined = np.vstack([FN_nd, FE_nd])
    else:
        FE_nd    = np.zeros((0, 3))
        combined = FN_nd

    comb_mask = filter_nondominated(combined)
    F_comb_nd = combined[comb_mask]

    def _metrics(F_all, F_nd):
        if len(F_nd) < 2:
            return 0.0, 0.0, 0.0
        hv  = compute_hv(F_nd)
        sp  = compute_spacing(F_nd)
        igd = compute_igd(F_nd, F_comb_nd) if len(F_comb_nd) >= 1 else 0.0
        return hv, sp, igd

    hv_N,  sp_N,  igd_N  = _metrics(F_N, FN_nd)
    hv_E,  sp_E,  igd_E  = _metrics(F_E, FE_nd)

    if has_eps and len(FN_nd) and len(FE_nd):
        c_N_vs_E = compute_coverage(FN_nd, FE_nd)
        c_E_vs_N = compute_coverage(FE_nd, FN_nd)
    else:
        c_N_vs_E = c_E_vs_N = 0.0

    cols = ('method,n_solutions,n_nondominated,hypervolume,spacing,'
            'igd_vs_combined,c_vs_other,c_by_other,time_s,n_evals')
    rows = [
        ('NSGA-II',       len(F_N), len(FN_nd), hv_N, sp_N, igd_N,
         c_N_vs_E, c_E_vs_N, t_nsga2, n_eval_N),
        ('e-constraint',  len(F_E), len(FE_nd), hv_E, sp_E, igd_E,
         c_E_vs_N, c_N_vs_E, t_eps,   n_eval_E),
    ]
    with open(path, 'w') as fh:
        fh.write('# CU3 — method comparison metrics\n')
        fh.write(f'# HV ref_point: {list(HV_REF_POINT)}\n')
        fh.write(cols + '\n')
        for r in rows:
            fh.write(f'{r[0]},{r[1]},{r[2]},{r[3]:.6f},{r[4]:.6f},'
                     f'{r[5]:.6f},{r[6]:.4f},{r[7]:.4f},{r[8]:.1f},{r[9]}\n')
    print(f"  saved → {path}")


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

def _save_selected_yaml(
    path:      str,
    via:       np.ndarray,
    F:         np.ndarray,
    idx:       int,
    method:    str   = 'knee',
    norm_dist: float | None = None,
):
    doc = {
        'pick_place_node': {
            'ros__parameters': {
                'point_O': [round(float(v), 6) for v in via],
            }
        },
    }
    with open(path, 'w') as f:
        f.write(f"# CU3 selected solution — method={method}  idx={idx}\n")
        f.write(f"# f1_effort={F[0]:.4f} N2*m2*s  "
                f"f2_arclen={F[1]:.4f} m  "
                f"clearance={-F[2]:.4f} m\n")
        if norm_dist is not None:
            f.write(f"# norm_dist_utopia={norm_dist:.4f}"
                    f"  (normalised over combined NSGA-II+ε front)\n")
        yaml.dump(doc, f, default_flow_style=False, sort_keys=False)
    print(f"  saved → {path}")


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main(argv=None):
    parser = argparse.ArgumentParser(
        description='CU3 multi-objective trajectory optimisation.')
    parser.add_argument('--test', '-t', type=int, default=None, metavar='N',
                        help='Test number N: results → results/testN/, '
                             'Pareto plots → plots/pareto/testN/')
    args, _ = parser.parse_known_args(argv)

    print("=" * 60)
    print("CU3 — Multi-objective trajectory optimisation")
    if args.test is not None:
        print(f"     Test #{args.test}")
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
    results_d, _ = _test_dirs(opt_params, args.test)

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
    print(f"\nDecision variable bounds (base_link frame):")
    print(f"  x_via ∈ [{bounds[0,0]:.2f}, {bounds[0,1]:.2f}]  m")
    print(f"  y_via ∈ [{bounds[1,0]:.2f}, {bounds[1,1]:.2f}]  m")
    print(f"  z_via ∈ [{bounds[2,0]:.2f}, {bounds[2,1]:.2f}]  m")

    # ── Stage 1: NSGA-II ─────────────────────────────────────────────────────
    pop_size = opt_params.get('pop_size', 60)
    n_gen    = opt_params.get('n_gen',   120)
    seed     = opt_params.get('seed',    42)

    print(f"\n── Stage 1: NSGA-II  (pop={pop_size}, gen={n_gen}) ──")
    t0_nsga2  = time.time()
    evaluator = TrajectoryEvaluator(config, urdf_path)
    nsga2_res = run_nsga2(evaluator, bounds, pop_size, n_gen, seed, verbose=True,
                           hv_ref_point=HV_REF_POINT)
    t_nsga2   = time.time() - t0_nsga2
    n_eval_N  = nsga2_res.get('n_eval', 0)
    print(f"  Elapsed: {t_nsga2:.1f} s  |  evaluations: {n_eval_N}")

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

    # B1 — save per-generation HV convergence
    conv_data = nsga2_res.get('convergence', [])
    if conv_data:
        _save_convergence_csv(
            os.path.join(results_d, 'convergence_nsga2.csv'),
            conv_data, HV_REF_POINT,
        )

    # ── Stage 2: ε-constraint ────────────────────────────────────────────────
    n_eps_f1 = opt_params.get('n_epsilon_f1',
                               opt_params.get('n_epsilon_steps', 25))
    n_eps_f3 = opt_params.get('n_epsilon_f3', 0)

    evaluator.reset_eval_counter()   # count only ε-constraint evaluations
    t0_eps = time.time()
    if n_eps_f3 > 0:
        print(f"\n── Stage 2: ε-constraint 2D  (f1={n_eps_f1}, f3={n_eps_f3}) ──")
        eps_res = run_epsilon_constraint_2d(
            evaluator, F_f, X_f, bounds,
            n_epsilon_f1=n_eps_f1, n_epsilon_f3=n_eps_f3, verbose=True,
        )
    else:
        eps_obj = opt_params.get('epsilon_obj_idx', 0)
        print(f"\n── Stage 2: ε-constraint 1D  (n_ε={n_eps_f1}, obj={eps_obj}) ──")
        eps_res = run_epsilon_constraint(
            evaluator, F_f, X_f, bounds,
            eps_obj_idx=eps_obj, n_steps=n_eps_f1, verbose=True,
        )
    t_eps    = time.time() - t0_eps
    n_eval_E = evaluator.n_eval
    print(f"  Elapsed: {t_eps:.1f} s  |  evaluations: {n_eval_E}")

    X_e = eps_res['X']
    F_e = eps_res['F']

    eps_feasible = np.all(F_e < 1e5, axis=1)
    X_e = X_e[eps_feasible]
    F_e = F_e[eps_feasible]

    _save_pareto_csv(
        os.path.join(results_d, 'pareto_epsilon.csv'), X_e, F_e)

    # B2 — method comparison table
    _save_method_comparison_csv(
        os.path.join(results_d, 'method_comparison.csv'),
        F_f, F_e, t_nsga2, t_eps, n_eval_N, n_eval_E,
    )

    # ── Select compromise solution ───────────────────────────────────────────
    combined_F = np.vstack([F_f, F_e]) if len(F_e) else F_f
    combined_X = np.vstack([X_f, X_e]) if len(X_e) else X_f

    sel_method  = opt_params.get('selection_method', 'knee')
    sel_weights = opt_params.get('weights', [1.0, 1.0, 1.0])
    sel_idx, norm_dist = select_solution(
        combined_F, method=sel_method, weights=sel_weights, F_ref=combined_F,
    )

    print(f"\n── Solution selection  (method={sel_method}) ──")
    print(f"  idx       = {sel_idx}")
    print(f"  via       = {combined_X[sel_idx]}")
    print(f"  F         = {combined_F[sel_idx]}")
    print(f"  norm_dist = {norm_dist:.4f}  (to utopia, normalised over combined front)")

    _save_selected_yaml(
        os.path.join(results_d, 'selected_solution.yaml'),
        combined_X[sel_idx], combined_F[sel_idx], sel_idx,
        method=sel_method, norm_dist=norm_dist,
    )

    if args.test is not None:
        print(f"\nTest #{args.test} results in {results_d}/")
        print(f"  → Pareto plots: run 'plot_pareto.py --test {args.test} --save'")
        print(f"  → Baseline:     run 'eval_baseline_cu2 --test {args.test}'")
    else:
        print(f"\nDone. Results in {results_d}/")
    print(
        "\nTo validate in Gazebo:\n"
        "  ros2 launch ur5_pick_place ur5_robotiq_gz.launch.py\n"
        "  ros2 launch ur5_pick_place pick_place.launch.py "
        f"  # then override: --ros-args --params-file {results_d}/selected_solution.yaml"
    )


if __name__ == '__main__':
    main()
