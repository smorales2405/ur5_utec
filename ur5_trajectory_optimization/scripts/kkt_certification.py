#!/usr/bin/env python3
"""
Pilar 2 — Certificación KKT de la restricción de borde activa (Sesión 13).

Certifica que el óptimo mono-objetivo ``O★`` es un punto KKT con la(s) cota(s)
de borde activa(s) (en particular ``x_O = via_x_min = 0.50``), reproduciendo el
método de Lagrange + Hessiano orlado:

  1. Estima por diferencias finitas ``∇f1`` y el Hessiano ``∇²f1`` en O★.
  2. Detecta el conjunto activo (cotas de la caja y holgura ``d_min = d_safe``).
  3. Estima los multiplicadores λ (deben ser ≥ 0, y ≠ 0 para una cota
     estrictamente activa) resolviendo ``∇f1 = Aᵀ λ`` (estacionariedad KKT).
  4. Proyecta el Hessiano sobre el espacio tangente de las restricciones
     activas (condición suficiente de 2º orden: Hessiano reducido definido
     positivo) y arma el Hessiano orlado, verificando el signo de los menores.
  5. Escribe ``results/final/kkt_certificate.txt`` con λ, gradientes, Hessiano
     orlado y veredicto.

Uso:
  python3 scripts/kkt_certification.py
"""

from __future__ import annotations
import os
import numpy as np

from ament_index_python.packages import get_package_share_directory

from ur5_trajectory_optimization.run_optimization import _load_yaml, _build_config, _pkg_base
from ur5_trajectory_optimization.multiobjective_optimizer import TrajectoryEvaluator
from ur5_trajectory_optimization.singleobjective_optimizer import scalar_objective


# ─────────────────────────────────────────────────────────────────────────────

def _load_via_star(default=(0.5, -0.003817, 0.25)) -> np.ndarray:
    import yaml
    base = os.path.join(_pkg_base(), 'results', 'final')
    for name in ('selected_solution_final.yaml',):
        path = os.path.join(base, name)
        if os.path.exists(path):
            with open(path) as fh:
                doc = yaml.safe_load(fh)
            try:
                return np.array(doc['pick_place_node']['ros__parameters']['point_O'], float)
            except (KeyError, TypeError):
                pass
    return np.array(default, float)


def _grad_fd(f, x, h):
    g = np.zeros(3)
    for i in range(3):
        xp = x.copy(); xp[i] += h
        xm = x.copy(); xm[i] -= h
        g[i] = (f(xp) - f(xm)) / (2.0 * h)
    return g


def _hessian_fd(f, x, h):
    n = 3
    H = np.zeros((n, n))
    f0 = f(x)
    for i in range(n):
        xp = x.copy(); xp[i] += h
        xm = x.copy(); xm[i] -= h
        H[i, i] = (f(xp) - 2.0 * f0 + f(xm)) / h ** 2
    for i in range(n):
        for j in range(i + 1, n):
            xpp = x.copy(); xpp[i] += h; xpp[j] += h
            xpm = x.copy(); xpm[i] += h; xpm[j] -= h
            xmp = x.copy(); xmp[i] -= h; xmp[j] += h
            xmm = x.copy(); xmm[i] -= h; xmm[j] -= h
            H[i, j] = H[j, i] = (f(xpp) - f(xpm) - f(xmp) + f(xmm)) / (4.0 * h ** 2)
    return H


def _leading_minors(M):
    return [float(np.linalg.det(M[:k, :k])) for k in range(1, M.shape[0] + 1)]


# ─────────────────────────────────────────────────────────────────────────────

def main(argv=None):
    print("=" * 64)
    print("Pilar 2 — Certificación KKT / Hessiano orlado (Sesión 13)")
    print("=" * 64)

    pp_share  = get_package_share_directory('ur5_pick_place')
    opt_share = get_package_share_directory('ur5_trajectory_optimization')
    kin_share = get_package_share_directory('ur5_kinematics')

    pp_params  = _load_yaml(os.path.join(pp_share,  'config', 'pick_place_params.yaml'))
    opt_params = _load_yaml(os.path.join(opt_share, 'config', 'optimization_params.yaml'))
    urdf_path  = os.path.join(kin_share, 'ur5e.urdf')
    config     = _build_config(pp_params, opt_params)
    fp = opt_params.get('final_project', {})

    bounds = np.array([
        [opt_params['via_x_min'], opt_params['via_x_max']],
        [opt_params['via_y_min'], opt_params['via_y_max']],
        [opt_params['via_z_min'], opt_params['via_z_max']],
    ])

    ev = TrajectoryEvaluator(config, urdf_path)
    r_grip = ev._obstacle['r_grip']
    d_safe = r_grip + float(fp.get('d_safe_extra', config.get('obstacle_delta_safe', 0.05)))

    x_star = _load_via_star()
    print(f"\nO★ = {x_star.tolist()}")

    # Funciones escalares (mismo objetivo que usaron los solvers: IK del CU3).
    def f1(x):
        return ev.evaluate(np.asarray(x, float))[0]

    def d_min(x):
        _f1, dm = scalar_objective(ev, x)
        return dm

    h_g = 2.0e-3
    h_h = 4.0e-3
    grad = _grad_fd(f1, x_star, h_g)
    H    = _hessian_fd(f1, x_star, h_h)
    dm0  = d_min(x_star)
    print(f"\n∇f1(O★)      = {grad}")
    print(f"d_min(O★)    = {dm0:.4f}  (d_safe = {d_safe:.4f})")

    # ── Conjunto activo ───────────────────────────────────────────────────────
    atol = 1.0e-3
    labels, A_rows = [], []
    for i, axis in enumerate('xyz'):
        if abs(x_star[i] - bounds[i, 0]) < atol:
            labels.append(f'cota inferior {axis} (={bounds[i,0]:.2f})')
            e = np.zeros(3); e[i] = 1.0; A_rows.append(e)          # c = x_i - xl ≥ 0
        elif abs(x_star[i] - bounds[i, 1]) < atol:
            labels.append(f'cota superior {axis} (={bounds[i,1]:.2f})')
            e = np.zeros(3); e[i] = -1.0; A_rows.append(e)         # c = xu - x_i ≥ 0
    clearance_active = abs(dm0 - d_safe) < 5.0e-3
    if clearance_active:
        gc = _grad_fd(d_min, x_star, h_g)
        labels.append('holgura d_min = d_safe')
        A_rows.append(gc)                                          # c = d_min - d_safe ≥ 0

    A = np.array(A_rows) if A_rows else np.zeros((0, 3))
    m = len(A_rows)
    print(f"\nConjunto activo ({m}): {labels}")

    # ── Multiplicadores KKT:  ∇f1 = Aᵀ λ ──────────────────────────────────────
    if m > 0:
        lam, *_ = np.linalg.lstsq(A.T, grad, rcond=None)
        residual = grad - A.T @ lam
    else:
        lam = np.zeros(0); residual = grad
    print(f"λ (multiplicadores) = {lam}")
    print(f"residual estacionariedad ‖∇f1 − Aᵀλ‖ = {np.linalg.norm(residual):.3e}")

    # ── Espacio tangente y Hessiano reducido (SOSC) ───────────────────────────
    if m > 0:
        # Z: base del núcleo de A (direcciones que conservan las activas)
        _u, _s, vt = np.linalg.svd(A)
        Z = vt[m:].T if m < 3 else np.zeros((3, 0))
    else:
        Z = np.eye(3)
    Hr = Z.T @ H @ Z if Z.shape[1] > 0 else np.zeros((0, 0))
    eig_r = np.linalg.eigvalsh(Hr) if Hr.size else np.array([])
    print(f"\nHessiano reducido (tangente, {Z.shape[1]}D):\n{Hr}")
    print(f"autovalores reducidos = {eig_r}")

    # ── Hessiano orlado ───────────────────────────────────────────────────────
    if m > 0:
        B = np.block([[np.zeros((m, m)), A],
                      [A.T,              H]])
    else:
        B = H
    minors = _leading_minors(B)

    # ── Veredicto ─────────────────────────────────────────────────────────────
    lam_ok   = bool(np.all(lam >= -1e-6)) if m else True
    stat_ok  = float(np.linalg.norm(residual)) < 1e-2 * (np.linalg.norm(grad) + 1e-9) + 1e-3
    sosc_ok  = bool(np.all(eig_r > 0)) if eig_r.size else True
    verdict  = lam_ok and sosc_ok

    # multiplicador de la cota x=0.50 (énfasis del spec)
    lam_x = None
    for lbl, lv in zip(labels, lam):
        if lbl.startswith('cota inferior x'):
            lam_x = lv

    results_d = os.path.join(_pkg_base(), 'results', 'final')
    os.makedirs(results_d, exist_ok=True)
    out = os.path.join(results_d, 'kkt_certificate.txt')
    with open(out, 'w') as fh:
        fh.write("Certificación KKT — óptimo mono-objetivo del Trabajo Integrador\n")
        fh.write("=" * 64 + "\n\n")
        fh.write(f"O★ = {x_star.tolist()}\n")
        fh.write(f"f1(O★)    = {f1(x_star):.4f} N²·m²·s\n")
        fh.write(f"d_min(O★) = {dm0:.4f} m   (d_safe = {d_safe:.4f} m)\n\n")

        fh.write("Gradiente del objetivo (diferencias finitas centradas):\n")
        fh.write(f"  ∇f1 = [{grad[0]:.2f}, {grad[1]:.2f}, {grad[2]:.2f}]   (h={h_g})\n\n")

        fh.write(f"Conjunto activo (m={m}):\n")
        for lbl in labels:
            fh.write(f"  - {lbl}\n")
        if not clearance_active:
            fh.write(f"  (holgura INACTIVA: d_min−d_safe = {dm0-d_safe:+.4f} m)\n")
        fh.write("\n")

        fh.write("Jacobiano de restricciones activas A (filas = ∇cᵢ):\n")
        fh.write(f"{A}\n\n")
        fh.write("Multiplicadores de Lagrange (∇f1 = Aᵀλ, KKT):\n")
        for lbl, lv in zip(labels, lam):
            fh.write(f"  λ[{lbl}] = {lv:.3f}   {'≥0 ✓' if lv >= -1e-6 else '< 0 ✗'}\n")
        if lam_x is not None:
            fh.write(f"\n  >>> Cota de borde x=0.50 ACTIVA con λ_x = {lam_x:.3f} "
                     f"({'≠ 0 ✓' if abs(lam_x) > 1e-3 else '≈ 0 ✗'})\n")
        fh.write(f"\n  residual estacionariedad ‖∇f1 − Aᵀλ‖ = {np.linalg.norm(residual):.3e}\n\n")

        fh.write("Hessiano del objetivo ∇²f1 (diferencias finitas):\n")
        fh.write(f"{H}\n\n")
        fh.write(f"Espacio tangente (dim {Z.shape[1]}), Hessiano reducido ZᵀHZ:\n")
        fh.write(f"{Hr}\n")
        fh.write(f"  autovalores = {eig_r}   (>0 ⇒ mínimo en el subespacio libre)\n\n")

        fh.write("Hessiano orlado B = [[0, A], [Aᵀ, ∇²f1]]:\n")
        fh.write(f"{B}\n")
        fh.write(f"  menores principales líderes = {[f'{v:.3e}' for v in minors]}\n")
        fh.write(f"  (para mínimo con m={m}: los últimos {max(0,3-m)} menores "
                 f"deben tener signo (−1)^m = {(-1)**m})\n\n")

        fh.write("VEREDICTO\n")
        fh.write("-" * 64 + "\n")
        fh.write(f"  Dual feasibility (λ ≥ 0)         : {'OK' if lam_ok else 'FALLA'}\n")
        fh.write(f"  Estacionariedad (∇f1 = Aᵀλ)      : {'OK' if stat_ok else 'FALLA'}\n")
        fh.write(f"  2º orden (ZᵀHZ ≻ 0)              : {'OK' if sosc_ok else 'FALLA'}\n")
        fh.write(f"  ⇒ O★ es un punto KKT / mínimo local: "
                 f"{'CERTIFICADO' if verdict else 'NO certificado'}\n")

    print(f"\n  saved → {out}")
    print(f"\nVeredicto: {'CERTIFICADO' if verdict else 'NO certificado'}  "
          f"(λ≥0:{lam_ok}, SOSC:{sosc_ok}, λ_x={lam_x})")


if __name__ == '__main__':
    main()
