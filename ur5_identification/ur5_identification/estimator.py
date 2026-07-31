"""
Estimación por mínimos cuadrados de los parámetros de fricción, con validación
cruzada y bandas de confianza (FASE 2).

Decisión metodológica importante — la unidad de observación es la MESETA,
no la muestra
------------------------------------------------------------------------
Dentro de una meseta de velocidad constante hay miles de muestras a 500 Hz, pero
NO son observaciones independientes: son la misma condición experimental
repetida, con residuos fuertemente autocorrelacionados. Ajustar sobre las
muestras crudas daría bandas de confianza absurdamente estrechas (n ~ 1e4
cuando la información real es n = número de niveles × 2 sentidos).

Por eso cada meseta se agrega a UNA observación (media del residuo y media de la
velocidad medida), y el ajuste, la covarianza y las bandas se calculan sobre
esas observaciones. La dispersión interna de cada meseta se reporta aparte, como
medida de ruido, no como grados de libertad.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .friction_models import (FrictionParams, n_params, params_from_beta,
                              regressor)


@dataclass
class FitResult:
    params: FrictionParams
    beta: np.ndarray
    stderr: np.ndarray            # error estándar de cada parámetro
    ci95: np.ndarray              # (p, 2) intervalos del 95 %
    r2: float                     # R² del ajuste (en las observaciones usadas)
    rmse: float                   # [N·m]
    n_obs: int
    residual_std_within: float    # dispersión típica DENTRO de las mesetas
    model: str

    def summary(self) -> str:
        lines = [f"  modelo: {self.model}  (n_obs = {self.n_obs} mesetas)"]
        names = {"viscous": ["F_v"], "viscous_coulomb": ["F_v", "F_c"],
                 "stribeck": ["F_v", "F_c", "F_s-F_c"]}[self.model]
        for name, b, se, ci in zip(names, self.beta, self.stderr, self.ci95):
            lines.append(f"    {name:>8} = {b:9.4f}  ± {se:7.4f}"
                         f"   IC95 [{ci[0]:8.4f}, {ci[1]:8.4f}]")
        lines.append(f"    R² = {self.r2:.4f}   RMSE = {self.rmse:.4f} N·m"
                     f"   (ruido intra-meseta σ = {self.residual_std_within:.4f} N·m)")
        return "\n".join(lines)


@dataclass
class CrossValResult:
    r2: float                      # R² sobre las observaciones RETENIDAS
    rmse: float
    per_fold: list = field(default_factory=list)

    def summary(self) -> str:
        return (f"    validación cruzada (leave-one-level-out): "
                f"R² = {self.r2:.4f}   RMSE = {self.rmse:.4f} N·m")


def _aggregate(windows) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Meseta -> (dq medio, residuo medio, velocidad nominal, ruido interno)."""
    dq = np.array([w.dq_mean for w in windows])
    res = np.array([w.residual_mean for w in windows])
    nominal = np.array([w.velocity for w in windows])
    within = float(np.mean([w.residual_std for w in windows])) if windows else 0.0
    return dq, res, nominal, within


def _lstsq_with_cov(phi: np.ndarray, y: np.ndarray):
    beta, *_ = np.linalg.lstsq(phi, y, rcond=None)
    resid = y - phi @ beta
    n, p = phi.shape
    dof = max(n - p, 1)
    sigma2 = float(resid @ resid) / dof
    try:
        cov = sigma2 * np.linalg.inv(phi.T @ phi)
        stderr = np.sqrt(np.clip(np.diag(cov), 0.0, None))
    except np.linalg.LinAlgError:
        stderr = np.full(p, np.nan)
    return beta, stderr, resid


def fit(windows, model: str = "viscous_coulomb", v_s: float = 0.0,
        delta: float = 2.0) -> FitResult:
    """Ajuste LS sobre las mesetas (una observación por meseta)."""
    if len(windows) < n_params(model) + 1:
        raise ValueError(
            f"hacen falta al menos {n_params(model) + 1} mesetas para el modelo "
            f"{model!r}, hay {len(windows)}")

    dq, y, _, within = _aggregate(windows)
    phi = regressor(dq, model, v_s=v_s, delta=delta)
    beta, stderr, resid = _lstsq_with_cov(phi, y)

    ss_res = float(resid @ resid)
    ss_tot = float(((y - y.mean()) ** 2).sum())
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    rmse = float(np.sqrt(ss_res / len(y)))
    ci = np.column_stack([beta - 1.96 * stderr, beta + 1.96 * stderr])

    return FitResult(params=params_from_beta(beta, model, v_s, delta),
                     beta=beta, stderr=stderr, ci95=ci, r2=r2, rmse=rmse,
                     n_obs=len(y), residual_std_within=within, model=model)


def cross_validate(windows, model: str = "viscous_coulomb", v_s: float = 0.0,
                   delta: float = 2.0) -> CrossValResult:
    """
    Validación cruzada dejando fuera un NIVEL DE VELOCIDAD completo cada vez
    (ambos sentidos a la vez).

    Dejar fuera mesetas sueltas sería optimista: el sentido opuesto del mismo
    nivel lleva casi la misma información. Retirar el nivel entero mide de
    verdad si el modelo extrapola a velocidades no vistas, que es lo que el plan
    pide ("ajuste sobre un subconjunto de velocidades, evaluación sobre el
    resto").
    """
    dq, y, nominal, _ = _aggregate(windows)
    levels = sorted(set(np.abs(nominal)))
    p = n_params(model)

    y_true, y_pred, per_fold = [], [], []
    for lv in levels:
        test = np.isclose(np.abs(nominal), lv)
        train = ~test
        if train.sum() < p + 1:
            continue
        phi_tr = regressor(dq[train], model, v_s=v_s, delta=delta)
        beta, *_ = np.linalg.lstsq(phi_tr, y[train], rcond=None)
        phi_te = regressor(dq[test], model, v_s=v_s, delta=delta)
        pred = phi_te @ beta
        y_true.extend(y[test])
        y_pred.extend(pred)
        per_fold.append({"level": float(lv),
                         "rmse": float(np.sqrt(np.mean((y[test] - pred) ** 2)))})

    if not y_true:
        return CrossValResult(r2=float("nan"), rmse=float("nan"))

    y_true = np.array(y_true)
    y_pred = np.array(y_pred)
    ss_res = float(((y_true - y_pred) ** 2).sum())
    ss_tot = float(((y_true - y_true.mean()) ** 2).sum())
    return CrossValResult(
        r2=1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan"),
        rmse=float(np.sqrt(ss_res / len(y_true))),
        per_fold=per_fold)


def fit_stribeck(windows, v_s_grid=None, delta_grid=None) -> tuple[FitResult, float, float]:
    """
    Stribeck: lineal en (F_v, F_c, F_s-F_c) pero NO en (v_s, delta). Se barre una
    rejilla pequeña de esos dos y se queda el mejor ajuste, en vez de lanzar una
    optimización no lineal con semilla arbitraria.
    """
    if v_s_grid is None:
        v_s_grid = np.geomspace(0.005, 0.5, 15)
    if delta_grid is None:
        delta_grid = [1.0, 2.0]

    best, best_vs, best_delta = None, 0.0, 2.0
    for v_s in v_s_grid:
        for delta in delta_grid:
            try:
                r = fit(windows, "stribeck", v_s=v_s, delta=delta)
            except (ValueError, np.linalg.LinAlgError):
                continue
            if best is None or r.rmse < best.rmse:
                best, best_vs, best_delta = r, float(v_s), float(delta)
    if best is None:
        raise ValueError("no se pudo ajustar el modelo de Stribeck")
    return best, best_vs, best_delta
