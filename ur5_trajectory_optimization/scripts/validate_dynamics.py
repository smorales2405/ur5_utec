#!/usr/bin/env python3
"""
Pilar 3 — Validación dinámica por EDO: Euler vs. RK4 (Unidad 6.1–6.2).

Toma el perfil de torques τ(t) del óptimo ``O★`` (RNEA sobre la trayectoria de
referencia IK), integra hacia adelante la dinámica directa (``pin.aba``) con
Euler y con RK4 para varios pasos ``h`` y mide el error.

Diseño del estudio
------------------
La dinámica del manipulador en LAZO ABIERTO es inestable: la trayectoria
nominal es una solución inestable de la EDO y cualquier error del integrador
crece exponencialmente, de modo que integrar los 6 s completos diverge para
AMBOS métodos (se reporta como hallazgo: motiva el control en lazo cerrado de
Gazebo).  Para comparar el ORDEN de los integradores se usa:

  * una ventana acotada centrada en el movimiento del via ``[t_B, t_B+horizon]``
    (el tramo B→O que el via-point optimiza), donde la dinámica permanece
    acotada, y
  * una referencia AUTO-CONSISTENTE (RK4 con paso muy fino) como "verdad" de la
    EDO bajo τ(t) — evita que el ruido de la referencia IK enmascare la
    convergencia de RK4.

Salidas:
  results/final/dynamics_validation.csv   — integrador, h, error_max, error_rms, estable
  results/final/plots/euler_vs_rk4.png    — seguimiento + error vs. h (log-log)

Uso:
  python3 scripts/validate_dynamics.py
"""

from __future__ import annotations
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from ament_index_python.packages import get_package_share_directory

from ur5_trajectory_optimization.run_optimization import _load_yaml, _build_config, _pkg_base
from ur5_trajectory_optimization.multiobjective_optimizer import TrajectoryEvaluator
from ur5_trajectory_optimization.trajectory_model import build_trajectory
from ur5_trajectory_optimization.objective_evaluators import compute_rnea_torques, _finite_diff
from ur5_trajectory_optimization.forward_dynamics_validation import (
    integrate_euler, integrate_rk4, make_tau_interp, tracking_error,
)

STABLE_TOL = 0.5   # rad — umbral de "estable" (error acotado)


# ─────────────────────────────────────────────────────────────────────────────

def _load_via_star(default=(0.5, -0.003817, 0.25)) -> np.ndarray:
    import yaml
    path = os.path.join(_pkg_base(), 'results', 'final', 'selected_solution_final.yaml')
    if os.path.exists(path):
        with open(path) as fh:
            doc = yaml.safe_load(fh)
        try:
            return np.array(doc['pick_place_node']['ros__parameters']['point_O'], float)
        except (KeyError, TypeError):
            pass
    return np.array(default, float)


def _build_reference(ev: TrajectoryEvaluator, via: np.ndarray, pts_per_seg=50):
    """Trayectoria de referencia (IK estricta) y su perfil de torques (RNEA)."""
    traj = build_trajectory(via, ev._fixed, pts_per_seg)
    ik_kw = dict(ev._ik_kwargs); ik_kw['tol'] = 1e-6; ik_kw['max_iter'] = 400
    qs, ok = ev._ik.solve_trajectory(traj, ev._q_home, **ik_kw)
    times  = np.array([wp.timestamp for wp in traj])
    qs     = np.array(qs)
    taus   = np.array(compute_rnea_torques(ev._pin_mdl, ev._pin_dat, list(qs), list(times)))
    dq_ref = np.array(_finite_diff(list(qs), list(times)))
    return times, qs, dq_ref, taus, sum(ok) / len(ok)


def _err_vs(qs_int, t_grid, t_truth, q_truth):
    ref = np.column_stack([np.interp(t_grid, t_truth, q_truth[:, j])
                           for j in range(q_truth.shape[1])])
    err = qs_int - ref
    if not np.all(np.isfinite(err)):
        return float('inf'), float('inf')
    return float(np.max(np.abs(err))), float(np.sqrt(np.mean(err ** 2)))


# ─────────────────────────────────────────────────────────────────────────────

def _save_csv(path: str, rows: list, note: str) -> None:
    with open(path, 'w') as fh:
        fh.write('# Pilar 3 — validación dinámica Euler vs RK4 sobre τ(t) del óptimo\n')
        fh.write(f'# {note}\n')
        fh.write('# error en rad vs. referencia auto-consistente (RK4 fino) en la ventana\n')
        fh.write('integrador,h,error_max,error_rms,estable\n')
        for integ, h, emax, erms, est in rows:
            fh.write(f'{integ},{h:.4f},{emax:.6e},{erms:.6e},{int(est)}\n')
    print(f"  saved → {path}")


def _save_plot(path, t_truth, q_truth, times, qs_ref, demo, rows, window) -> None:
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
    j = int(np.argmax(np.ptp(q_truth, axis=0)))   # articulación con mayor recorrido
    t0, t1 = window

    # (a) Seguimiento en la ventana.
    h_demo, tg, qE, qR = demo
    mref = (times >= t0) & (times <= t1)
    ax1.plot(t_truth, q_truth[:, j], 'k-', lw=2.5, alpha=0.6, label='verdad (RK4 fino)')
    ax1.plot(times[mref], qs_ref[mref, j], ':', color='green', lw=1.5, label='referencia IK')
    ax1.plot(tg, qE[:, j], 'o-', color='tab:orange', ms=3, lw=1.0, label=f'Euler (h={h_demo})')
    ax1.plot(tg, qR[:, j], '^-', color='tab:blue', ms=3, lw=1.0, label=f'RK4 (h={h_demo})')
    ax1.set_xlabel('t [s]'); ax1.set_ylabel(f'q[{j}] [rad]')
    ax1.set_title(f'Seguimiento en ventana del via [{t0:.1f},{t1:.1f}] s (art. {j})', fontsize=11)
    ax1.legend(fontsize=8); ax1.grid(True, ls='--', alpha=0.4)

    # (b) Error vs h (log-log) con guías de pendiente O(h) y O(h⁴).
    for integ, color, mk in [('euler', 'tab:orange', 'o'), ('rk4', 'tab:blue', '^')]:
        hs = np.array([r[1] for r in rows if r[0] == integ])
        es = np.array([r[2] for r in rows if r[0] == integ])
        ax2.loglog(hs, es, marker=mk, color=color, lw=1.5, ms=7, label=integ)
    hs = np.array(sorted({r[1] for r in rows}))
    eE = np.array([r[2] for r in rows if r[0] == 'euler'])
    eR = np.array([r[2] for r in rows if r[0] == 'rk4'])
    ax2.loglog(hs, eE.max() * (hs / hs.max()) ** 1, '--', color='gray', alpha=0.6, label='$O(h)$')
    ax2.loglog(hs, eR.max() * (hs / hs.max()) ** 4, ':',  color='gray', alpha=0.6, label='$O(h^4)$')
    ax2.set_xlabel('paso h [s]'); ax2.set_ylabel('error máximo [rad]')
    ax2.set_title('Orden del integrador (error vs. h)', fontsize=11)
    ax2.legend(fontsize=8); ax2.grid(True, which='both', ls='--', alpha=0.4)

    fig.suptitle('Pilar 3 — Validación dinámica directa: Euler vs. RK4',
                 fontsize=12, fontweight='bold')
    plt.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches='tight')
    print(f"  saved → {path}")
    plt.close(fig)


# ─────────────────────────────────────────────────────────────────────────────

def main(argv=None):
    print("=" * 64)
    print("Pilar 3 — Validación dinámica por EDO: Euler vs. RK4")
    print("=" * 64)

    pp_share  = get_package_share_directory('ur5_pick_place')
    opt_share = get_package_share_directory('ur5_trajectory_optimization')
    kin_share = get_package_share_directory('ur5_kinematics')

    pp_params  = _load_yaml(os.path.join(pp_share,  'config', 'pick_place_params.yaml'))
    opt_params = _load_yaml(os.path.join(opt_share, 'config', 'optimization_params.yaml'))
    urdf_path  = os.path.join(kin_share, 'ur5e.urdf')
    config     = _build_config(pp_params, opt_params)
    fp = opt_params.get('final_project', {})
    h_values = sorted(fp.get('ode_h_values', [0.02, 0.01, 0.005, 0.002]), reverse=True)
    horizon  = float(fp.get('ode_horizon', 1.0))
    ref_h    = float(fp.get('ode_ref_h', 1.0e-4))

    via = _load_via_star()
    print(f"\nO★ = {via.tolist()}")

    ev = TrajectoryEvaluator(config, urdf_path)
    print("\nConstruyendo trayectoria de referencia (IK estricta) + τ(t) (RNEA)…")
    times, qs_ref, dq_ref, taus, rate = _build_reference(ev, via)
    print(f"  N={len(times)} puntos, IK rate={rate:.2%}, |τ|_max={np.max(np.abs(taus)):.1f} N·m")

    tau_of_t = make_tau_interp(times, taus)
    mdl, dat = ev._pin_mdl, ev._pin_dat

    def state_at(t):
        q = np.array([np.interp(t, times, qs_ref[:, j])  for j in range(6)])
        v = np.array([np.interp(t, times, dq_ref[:, j]) for j in range(6)])
        return q, v

    # ── Ventana de validación: movimiento del via [t_B, t_B+horizon] ──────────
    t0 = float(ev._fixed['t_B'])
    t1 = min(t0 + horizon, float(times[-1]))
    q0, v0 = state_at(t0)
    print(f"\nVentana de validación (movimiento del via): [{t0:.2f}, {t1:.2f}] s")

    # Verdad auto-consistente: RK4 con paso muy fino.
    n_ref = int(round((t1 - t0) / ref_h))
    t_truth = t0 + ref_h * np.arange(n_ref + 1)
    q_truth, _ = integrate_rk4(mdl, dat, q0, v0, tau_of_t, t_truth)

    rows, demo = [], None
    print("\nIntegrando hacia adelante (dinámica directa pin.aba)…")
    for h in h_values:
        n = int(round((t1 - t0) / h))
        tg = t0 + h * np.arange(n + 1)
        qE, _ = integrate_euler(mdl, dat, q0, v0, tau_of_t, tg)
        qR, _ = integrate_rk4(mdl, dat, q0, v0, tau_of_t, tg)
        emaxE, ermsE = _err_vs(qE, tg, t_truth, q_truth)
        emaxR, ermsR = _err_vs(qR, tg, t_truth, q_truth)
        rows.append(('euler', h, emaxE, ermsE, np.isfinite(emaxE) and emaxE < STABLE_TOL))
        rows.append(('rk4',   h, emaxR, ermsR, np.isfinite(emaxR) and emaxR < STABLE_TOL))
        print(f"  h={h:.3f}  Euler: emax={emaxE:.3e}  | RK4: emax={emaxR:.3e}  "
              f"(RK4 {emaxE/max(emaxR,1e-18):.0f}× más preciso)")
        if demo is None or abs(h - 0.01) < 1e-9:
            demo = (h, tg, qE, qR)

    # ── Hallazgo: horizonte completo en lazo abierto diverge (ambos) ──────────
    tg_full = float(times[0]) + h_values[-1] * np.arange(
        int(round((times[-1] - times[0]) / h_values[-1])) + 1)
    qEf, _ = integrate_euler(mdl, dat, qs_ref[0], dq_ref[0], tau_of_t, tg_full)
    qRf, _ = integrate_rk4(mdl, dat, qs_ref[0], dq_ref[0], tau_of_t, tg_full)
    efE, _ = tracking_error(qEf, tg_full, times, qs_ref)
    efR, _ = tracking_error(qRf, tg_full, times, qs_ref)
    note = (f"horizonte completo (6s) en lazo abierto DIVERGE: "
            f"Euler emax={efE:.1e}, RK4 emax={efR:.1e} rad  → requiere control en lazo cerrado")
    print(f"\n  Horizonte completo [0,{times[-1]:.0f}] s (lazo abierto):")
    print(f"    Euler emax={efE:.2e}  RK4 emax={efR:.2e}  (ambos divergen → plant inestable)")

    results_d = os.path.join(_pkg_base(), 'results', 'final')
    plots_d   = os.path.join(results_d, 'plots')
    os.makedirs(plots_d, exist_ok=True)
    print("\nGuardando resultados…")
    _save_csv(os.path.join(results_d, 'dynamics_validation.csv'), rows, note)
    _save_plot(os.path.join(plots_d, 'euler_vs_rk4.png'),
               t_truth, q_truth, times, qs_ref, demo, rows, (t0, t1))
    print("\nDone.")


if __name__ == '__main__':
    main()
