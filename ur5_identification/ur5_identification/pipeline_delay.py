"""
B7.0: retardo de tubería del lazo de par, medido con los CSV que ya existen.

Qué se mide
-----------
En cada fila `k` del CSV unificado el nodo escribe el par que acaba de mandar
(`tau`, después de G3 y de la saturación) y la corriente del último
`/joint_states` que vio (`cur`). El par que llega al motor es

    k ⊙ i(k) = τ_cmd(k − D) + g_UR(q)            (G4 = 0; en Gazebo g_UR = 0, k = 1)

con `D` el número de ciclos desde que el nodo VE un estado hasta que el par que
calculó con él actúa. Es el `delay_steps` de `closed_loop.simulate`: la corriente
y el estado viajan en el mismo paquete del robot, así que el par mandado en la
fila `k` aparece en la corriente de la fila `k + D`. Y no es un artefacto de la
medida de corriente: la velocidad medida arranca a la vez (ver el análisis del
plan, B7.0).

Cómo
----
Con ESCALONES del par mandado, no con correlación. Los barridos de la FASE 2
cambian el signo del feedforward de Coulomb en cada inversión (un salto de
~2 F_c en dos o tres muestras): promediando la respuesta alineada en esos
instantes se ve el retardo directamente, y el ajuste `y(k) = u(k − D)` da `D`
con resolución fraccionaria. La correlación cruzada (`xcorr_delay`) queda para
las corridas sin escalones (Gazebo); con excitación de banda estrecha —el ciclo
límite a 35 Hz de `smc_712`— mezcla retardo y dinámica y no se usa como medida.
"""

from __future__ import annotations

import numpy as np

from .force_estimator import K_CURRENT, load_columns

#: Escalón mínimo del par mandado en UNA muestra para contar como evento [N·m].
STEP_MIN = 2.0
#: Ventana alrededor del escalón [muestras].
PRE, POST = 6, 16


def is_real(cols: dict) -> bool:
    """El real escribe `t_sim` en reloj de pared; Gazebo, desde 0."""
    return float(cols["t_sim"][0]) > 1e8


def step_events(u: np.ndarray, threshold: float = STEP_MIN,
                min_gap: int = PRE + POST) -> np.ndarray:
    """Índices de la primera muestra con el par ya cambiado, separados entre sí."""
    du = np.diff(np.asarray(u, float))
    cand = np.where(np.abs(du) > threshold)[0] + 1
    out, last = [], -min_gap
    for k in cand:
        if k - last >= min_gap:
            out.append(k)
            last = k
    return np.asarray(out, dtype=int)


def event_average(u, y, events, pre: int = PRE, post: int = POST):
    """Respuesta media alineada en los escalones, con el signo del escalón.

    Devuelve `(U, Y, n)`: media de `u` y de `y` desde `k − pre` hasta
    `k + post − 1`, cada evento referido a su media previa (las tres primeras
    muestras: el feedforward empieza a subir dos antes del umbral). `n` eventos.
    """
    u, y = np.asarray(u, float), np.asarray(y, float)
    U, Y = [], []
    for k in events:
        if k - pre < 0 or k + post > len(u):
            continue
        s = np.sign(u[k] - u[k - 1])
        base = slice(k - pre, k - pre + 3)
        U.append(s * (u[k - pre:k + post] - u[base].mean()))
        Y.append(s * (y[k - pre:k + post] - y[base].mean()))
    if not U:
        return None, None, 0
    return np.mean(U, axis=0), np.mean(Y, axis=0), len(U)


def fit_shift(U: np.ndarray, Y: np.ndarray, max_shift: float = 10.0,
              step: float = 0.01):
    """`D` y ganancia de `Y(k) ≈ a · U(k − D)` por mínimos cuadrados.

    `U` se desplaza con interpolación lineal (fuera de la ventana, su valor del
    borde). Devuelve `(D, a, rms_rel)`, con el residuo relativo al escalón.
    """
    k = np.arange(len(U), dtype=float)
    best = (np.nan, np.nan, np.inf)
    for D in np.arange(0.0, max_shift + step / 2, step):
        Us = np.interp(k - D, k, U)
        den = Us @ Us
        if den <= 0.0:
            continue
        a = (Us @ Y) / den
        r = np.sqrt(np.mean((Y - a * Us) ** 2))
        if r < best[2]:
            best = (float(D), float(a), float(r))
    span = max(abs(U[-1] - U[0]), 1e-12)
    return best[0], best[1], best[2] / span


def xcorr_delay(u, y, max_lag: int = 12, window: int = 25):
    """Pico de la correlación cruzada tras quitar la tendencia lenta.

    Sirve donde no hay escalones y la excitación es de banda ancha (Gazebo).
    Devuelve `(lag, rho)` con el pico refinado por una parábola.
    """
    def hp(x):
        return x - np.convolve(x, np.ones(window) / window, mode="same")
    u, y = hp(np.asarray(u, float)), hp(np.asarray(y, float))
    lags = np.arange(-max_lag, max_lag + 1)
    c = []
    for L in lags:
        a, b = (u[:len(u) - L], y[L:]) if L >= 0 else (u[-L:], y[:len(y) + L])
        a, b = a - a.mean(), b - b.mean()
        den = np.sqrt((a @ a) * (b @ b))
        c.append((a @ b) / den if den > 0 else 0.0)
    c = np.asarray(c)
    i = int(np.argmax(c))
    frac = 0.0
    if 0 < i < len(c) - 1:
        den = c[i - 1] - 2 * c[i] + c[i + 1]
        frac = 0.5 * (c[i - 1] - c[i + 1]) / den if den != 0 else 0.0
    return float(lags[i] + frac), float(c[i])


def estimate_from_csv(path: str, joint: int | None = None,
                      threshold: float = STEP_MIN) -> dict:
    """Retardo de una corrida. `joint` (1..6) o la de más escalones.

    `method` = "steps" si hubo escalones; si no, "xcorr" en la junta con más
    variación de par (solo orientativo: ver el docstring del módulo).
    """
    d = load_columns(path)
    missing = [c for c in ("t_sim", "tau1", "cur1") if c not in d]
    if missing:
        raise ValueError(f"faltan columnas {missing} (CSV anterior a G5?)")
    real = is_real(d)
    k = K_CURRENT if real else np.ones(6)
    joints = [joint] if joint else range(1, 7)
    ev = {j: step_events(d[f"tau{j}"], threshold) for j in joints}
    j = max(ev, key=lambda i: len(ev[i]))
    out = {"csv": path, "real": real, "joint": j}
    u, y = d[f"tau{j}"], d[f"cur{j}"] * k[j - 1]
    U, Y, n = event_average(u, y, ev[j])
    if n > 0:
        D, a, r = fit_shift(U, Y)
        out.update(method="steps", n_events=n, delay=D, gain=a, rms_rel=r,
                   U=U, Y=Y, dq=event_average(u, d[f"dq{j}"], ev[j])[1])
        return out
    j = max(joints, key=lambda i: np.std(np.diff(d[f"tau{i}"])))
    lag, rho = xcorr_delay(d[f"tau{j}"], d[f"cur{j}"] * k[j - 1])
    out.update(method="xcorr", joint=j, n_events=0, delay=lag, rho=rho)
    return out
