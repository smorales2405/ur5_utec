"""
B4.6: inercia efectiva por junta, medida; el URDF no trae la del rotor.

Por qué existe
--------------
La re-sintonía `smc_v6_d5` salió infactible con la planta del URDF: `wrist_1` vale
0.028 kg·m² y con 5 ciclos de retardo eso limita su rigidez a unos 27 N·m/rad. Los
barridos de fricción (`fl_50x`, `fl_90x`) sugieren 0.19 kg·m² añadidos en las tres
muñecas, pero no se hicieron para esto: aceleran poco y en las juntas grandes el
ajuste es poco fiable. Aquí se mide a propósito.

Cómo
----
Por control de POSICIÓN, sin ninguna ley de par en el lazo, una junta cada vez sigue
un seno de amplitud pequeña mientras las demás se quedan quietas. En esa junta:

    k·i = M_jj q̈ + g(q) + fricción(q̇)

`M_jj` no depende de la propia `q_j` y `C_jj = 0`, así que no hace falta modelo. En
fasores, con la corriente y la posición medidas en las mismas muestras:

    Z(ω) = T/Q      Re Z = s − M_jj ω²      Im Z = lo que va con la velocidad

- La fricción, viscosa o de Coulomb, va con la velocidad: cae en `Im Z`.
- Lo que va con la posición sin depender de la frecuencia (la rigidez de la gravedad
  y la de la fricción antes de deslizar) es `s`. El error de gravedad de B4.4, por
  tanto, no entra.
- `M_jj` es la pendiente de `Re Z` frente a ω². Por eso se usan varias frecuencias.

Lo que puede sesgarla es un desfase entre corriente y posición: rota `Z` y pasa
parte de la fricción a `Re Z`. Con la misma velocidad de pico en todas las
frecuencias, un desfase δ imita una inercia de `(4 F_c/(π v) + b)·δ`, unos 0.06 kg·m²
por ms en las juntas grandes y 0.007 en las muñecas. RTDE da las dos señales del mismo
ciclo; aun así se informa de cuánto cambia `M` con ±2 ms.

El rotor de cada junta sale de restar el modelo rígido con la herramienta. No depende
de la postura; si dos posturas no coinciden, algo falla y se dice.

Dos estimadores, porque en el robot no coinciden
------------------------------------------------
Con el mismo dato, `fit_slope` (fasores) y `velocity_cell_fit` (celdas de velocidad)
difieren más que la repetibilidad, que es de menos del 1.2 %: es un sesgo, no ruido.

- Los fasores usan todo el ciclo, y su parte en fase con la posición pesa más justo en
  las inversiones. Si la fricción tarda en cambiar de signo, ahí se cuela: basta un 3-4 %
  del par para mover `M`.
- Las celdas comparan aceleración y frenado a igual velocidad y solo lejos de las
  inversiones, pero suponen que fuera de ellas la fricción depende solo de la velocidad.

La rigidez de gravedad, que el modelo sí da, no los desempata (B4.6). Por eso se informa
del rango (`rotor_range`) y no de un número, y para los márgenes se usa su extremo bajo:
menos inercia, menos margen.

A esta frecuencia el brazo se comporta como rígido: el modo flexible está en 25-35 Hz.
Esta medida no dice nada de ese modo, que sigue limitado por `G ≤ 120` y hay que
comprobarlo en el robot.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .force_estimator import K_CURRENT

DT = 0.002                  # [s]
#: Velocidad de pico [rad/s], la misma a todas las frecuencias: mismo régimen de
#: fricción, y la aceleración crece con la frecuencia. Con 0.20 las juntas grandes
#: llegan a ~12 N·m de inercia; con 0.40 las muñecas, a ~2 N·m (`F_c` ≈ 1.8 N·m).
V_PEAK = np.array([0.20, 0.20, 0.20, 0.40, 0.40, 0.40])
#: Frecuencias [Hz], muy por debajo del modo flexible (25-35 Hz).
FREQS = {0: (1.0, 1.5, 2.0, 3.0), 1: (1.0, 1.5, 2.0, 3.0), 2: (1.0, 1.5, 2.0, 3.0),
         3: (1.0, 2.0, 3.0, 4.0), 4: (1.0, 2.0, 3.0, 4.0), 5: (1.0, 2.0, 3.0, 4.0)}
RAMP = 1.0                  # [s] entrada y salida suaves
SETTLE = 0.5                # [s] que se descartan tras la rampa
MIN_STEADY = 5.0            # [s] de amplitud constante
MIN_CYCLES = 6
#: Un ajuste por junta y postura necesita sobrar al menos un punto (s, M).
MIN_POINTS = 3
#: Por encima de esta incertidumbre relativa, el ajuste no se da por bueno.
MAX_REL_ERR = 0.15
SKEW_TEST = 0.002           # [s]
#: Celdas de velocidad: solo muestras con |v| por encima de esta fracción del pico. Con
#: 0.3, 0.5 y 0.7 el rotor de `wrist_1` da 0.148, 0.147 y 0.141.
CELL_MIN_SPEED = 0.5
CELL_BINS = 5


@dataclass
class Segment:
    posture: str
    joint: int              # 0-based
    freq: float             # [Hz]
    amp: float              # [rad]
    cycles: int
    q0: np.ndarray
    q: np.ndarray           # [n, 6] consigna
    dq: np.ndarray

    @property
    def duration(self) -> float:
        return 2.0 * RAMP + self.cycles / self.freq


def excitation(posture, q0, joint, freq, v_peak=None, dt=DT) -> Segment:
    """Seno en `joint` con entrada y salida suaves: empieza y acaba quieto en `q0`."""
    q0 = np.asarray(q0, float)
    v = float(V_PEAK[joint] if v_peak is None else v_peak)
    w = 2.0 * math.pi * freq
    amp = v / w
    cycles = max(MIN_CYCLES, int(math.ceil(MIN_STEADY * freq)))
    # Múltiplo de dt: la última muestra cae donde la envolvente vale 0.
    n = int(round((2.0 * RAMP + cycles / freq) / dt))
    total = n * dt
    t = np.arange(n + 1) * dt
    env = np.ones_like(t)
    denv = np.zeros_like(t)
    up, down = t < RAMP, t > total - RAMP
    env[up] = np.sin(0.5 * math.pi * t[up] / RAMP) ** 2
    denv[up] = (math.pi / (2.0 * RAMP)) * np.sin(math.pi * t[up] / RAMP)
    env[down] = np.sin(0.5 * math.pi * (total - t[down]) / RAMP) ** 2
    denv[down] = -(math.pi / (2.0 * RAMP)) * np.sin(math.pi * (total - t[down]) / RAMP)
    q = np.tile(q0, (len(t), 1))
    dq = np.zeros_like(q)
    q[:, joint] += amp * env * np.sin(w * t)
    dq[:, joint] = amp * (denv * np.sin(w * t) + env * w * np.cos(w * t))
    return Segment(posture, joint, float(freq), amp, cycles, q0, q, dq)


def steady_window(t_rel, freq, cycles):
    """Máscara de las muestras en amplitud constante, un número entero de ciclos."""
    start = RAMP + SETTLE
    usable = cycles / freq - SETTLE
    n = int(math.floor(usable * freq + 1e-9))
    t_rel = np.asarray(t_rel, float)
    return (t_rel >= start) & (t_rel < start + n / freq)


def harmonic(t, x, freq) -> complex:
    """Fasor X de `x` a `freq`, con x(t) = Re{X e^{jωt}}.

    Con armónicos (Coulomb es casi una onda cuadrada), media y deriva en la base, para
    que no se cuelen en la fundamental.
    """
    t = np.asarray(t, float)
    tc = t - t.mean()
    w = 2.0 * math.pi * freq
    cols = []
    for h in (1, 2, 3, 5):
        cols += [np.sin(h * w * t), np.cos(h * w * t)]
    A = np.column_stack(cols + [np.ones_like(t), tc])
    c = np.linalg.lstsq(A, np.asarray(x, float), rcond=None)[0]
    return complex(c[1], -c[0])


def dynamic_stiffness(t, q, cur, freq, k, skew=0.0):
    """(Z, Q) a `freq`. `skew` [s]: la corriente se lee `skew` segundos después."""
    tau = k * (np.interp(t + skew, t, cur) if skew else np.asarray(cur, float))
    Q = harmonic(t, q, freq)
    return harmonic(t, tau, freq) / Q, Q


def fit_slope(points):
    """Ajusta Re Z = s − M ω² en par [N·m]: `points` = [(freq, Z, Q), ...].

    Devuelve (M, s, error típico de M, residuo rms [N·m]).
    """
    w2 = np.array([(2.0 * math.pi * f) ** 2 for f, _, _ in points])
    amp = np.array([abs(q) for _, _, q in points])
    y = np.array([z.real for _, z, _ in points]) * amp
    A = np.column_stack([amp, -w2 * amp])
    sol = np.linalg.lstsq(A, y, rcond=None)[0]
    res = y - A @ sol
    dof = max(len(points) - 2, 1)
    cov = np.linalg.inv(A.T @ A) * float(res @ res) / dof
    return (float(sol[1]), float(sol[0]), float(math.sqrt(cov[1, 1])),
            float(np.sqrt(np.mean(res ** 2))))


def model_inertia(model, data, q, joint, mass=0.0, cog_mm=(0.0, 0.0, 0.0)) -> float:
    """M_jj del modelo rígido con una masa puntual en `cog_mm` (ejes de tool0)."""
    import pinocchio as pin
    q = np.asarray(q, float)
    bare = float(pin.crba(model, data, q)[joint, joint])
    if not mass:
        return bare
    fid = model.getFrameId("tool0")
    pin.computeJointJacobians(model, data, q)
    pin.framesForwardKinematics(model, data, q)
    J = pin.getFrameJacobian(model, data, fid, pin.LOCAL_WORLD_ALIGNED)
    r = data.oMf[fid].rotation @ (np.asarray(cog_mm, float) * 1e-3)
    # Velocidad del punto: v_tool0 + w x r.
    v = J[:3, joint] + np.cross(J[3:, joint], r)
    return bare + float(mass) * float(v @ v)


# --- CSV ---------------------------------------------------------------------------

COLUMNS = (["seg", "posture", "joint", "freq", "amp", "cycles", "t_rel", "t_wall"]
           + [f"q{i}" for i in range(1, 7)] + [f"dq{i}" for i in range(1, 7)]
           + [f"cur{i}" for i in range(1, 7)])


def write_header(fh, meta) -> None:
    for key, value in meta.items():
        fh.write(f"# {key}={value}\n")
    fh.write(",".join(COLUMNS) + "\n")
    fh.flush()


def append_segment(fh, index, seg: Segment, samples) -> None:
    """`samples`: (t, q, dq, cur) de /joint_states. `joint` va de 1 a 6, como `q1..q6`."""
    if not samples:
        return
    t0 = samples[0][0]
    for t, q, dq, cur in samples:
        fh.write(f"{index},{seg.posture},{seg.joint + 1},{seg.freq:g},{seg.amp:.9f},"
                 f"{seg.cycles},{t - t0:.6f},{t:.6f},"
                 + ",".join(f"{v:.9f}" for v in (*q, *dq, *cur)) + "\n")
    fh.flush()


def read_csv(path):
    """(meta, segmentos). Una corrida abortada se lee hasta donde llegó."""
    meta, rows, cols = {}, {}, None
    with open(path) as fh:
        for line in fh:
            if line.startswith("#"):
                if "=" in line:
                    key, value = line[1:].strip().split("=", 1)
                    meta[key.strip()] = value.strip()
                continue
            if cols is None:
                cols = line.strip().split(",")
                continue
            parts = line.strip().split(",")
            if len(parts) == len(cols):
                rows.setdefault(int(parts[0]), []).append(parts)
    segments = []
    for index in sorted(rows):
        r = rows[index]
        values = np.array([[float(v) for v in p[6:]] for p in r])
        segments.append({
            "seg": index, "posture": r[0][1], "joint": int(r[0][2]) - 1,
            "freq": float(r[0][3]), "amp": float(r[0][4]), "cycles": int(r[0][5]),
            "t_rel": values[:, 0], "t": values[:, 1], "q": values[:, 2:8],
            "dq": values[:, 8:14], "cur": values[:, 14:20]})
    return meta, segments


# --- Análisis ----------------------------------------------------------------------

def segment_point(seg, k=K_CURRENT, skew=0.0):
    """(freq, Z, Q) de un segmento, o None si no llegó a la amplitud constante."""
    mask = steady_window(seg["t_rel"], seg["freq"], seg["cycles"])
    expected = (seg["cycles"] / seg["freq"] - SETTLE) * 0.4 / DT
    if mask.sum() < expected:
        return None
    j = seg["joint"]
    t = seg["t"][mask]
    z, q = dynamic_stiffness(t, seg["q"][mask, j], seg["cur"][mask, j], seg["freq"],
                             k[j], skew)
    return seg["freq"], z, q


def velocity_cell_fit(segments, k=K_CURRENT, min_speed=CELL_MIN_SPEED, bins=CELL_BINS):
    """(M, s) de `τ = M q̈ + s q + F(v)`, lejos de las inversiones; None si no se puede.

    `F` es libre: una constante por celda de velocidad, sentido y segmento. Así la deriva
    térmica de la fricción entre segmentos no entra, y `M` y `s` salen solo de lo que
    cambia entre acelerar y frenar a igual velocidad. En un segmento `q̈ = −ω² q`, de modo
    que hacen falta varias frecuencias para separarlas, igual que en `fit_slope`.
    """
    usable = [s for s in segments if segment_point(s, k) is not None]
    if len({s["freq"] for s in usable}) < MIN_POINTS:
        return None
    acc, pos, tau, cell = [], [], [], []
    for n, seg in enumerate(usable):
        j, w = seg["joint"], 2.0 * math.pi * seg["freq"]
        mask = steady_window(seg["t_rel"], seg["freq"], seg["cycles"])
        t = seg["t"][mask]
        # El movimiento ajustado: derivar la posición medida solo añadiría ruido.
        rot = harmonic(t, seg["q"][mask, j], seg["freq"]) * np.exp(1j * w * t)
        q, v = rot.real, (1j * w * rot).real
        v_max = np.abs(v).max()
        keep = np.abs(v) >= min_speed * v_max
        edges = np.linspace(min_speed * v_max, v_max * (1.0 + 1e-6), bins + 1)
        bin_ = np.clip(np.digitize(np.abs(v[keep]), edges) - 1, 0, bins - 1)
        cell.append(bin_ + bins * (v[keep] > 0) + 2 * bins * n)
        acc.append(-w * w * q[keep])
        pos.append(q[keep])
        tau.append(k[j] * seg["cur"][mask, j][keep])
    cell = np.unique(np.concatenate(cell), return_inverse=True)[1]
    count = np.bincount(cell)

    def within(x):
        # Quitar la media de cada celda equivale a una constante libre por celda.
        x = np.concatenate(x)
        return x - (np.bincount(cell, x) / count)[cell]

    sol = np.linalg.lstsq(np.column_stack([within(acc), within(pos)]), within(tau),
                          rcond=None)[0]
    return float(sol[0]), float(sol[1])


def tracking_ratio(seg) -> float:
    """Amplitud medida / mandada en la junta excitada; 0 si no hay amplitud constante."""
    mask = steady_window(seg["t_rel"], seg["freq"], seg["cycles"])
    if mask.sum() < 10:
        return 0.0
    q = seg["q"][mask, seg["joint"]]
    return abs(harmonic(seg["t"][mask], q, seg["freq"])) / seg["amp"]


def fit_groups(segments, model=None, data=None, mass=0.0, cog_mm=(0.0, 0.0, 0.0),
               k=K_CURRENT):
    """Un ajuste por (junta, postura), con el modelo rígido si se le pasa."""
    groups = {}
    for seg in segments:
        groups.setdefault((seg["joint"], seg["posture"]), []).append(seg)
    out = []
    for (joint, posture), segs in sorted(groups.items()):
        pts = [p for p in (segment_point(s, k) for s in segs) if p is not None]
        row = {"joint": joint, "posture": posture, "n": len(pts), "points": pts,
               "M": None, "s": None, "err": None, "resid": None, "sens": None,
               "M_model": None, "dI": None, "note": "",
               "M_cell": None, "s_cell": None, "dI_cell": None}
        if len({p[0] for p in pts}) < MIN_POINTS:
            row["note"] = f"menos de {MIN_POINTS} frecuencias: sin ajuste"
            out.append(row)
            continue
        row["M"], row["s"], row["err"], row["resid"] = fit_slope(pts)
        sens = []
        for skew in (-SKEW_TEST, SKEW_TEST):
            shifted = [p for p in (segment_point(s, k, skew) for s in segs)
                       if p is not None]
            sens.append(abs(fit_slope(shifted)[0] - row["M"]))
        row["sens"] = max(sens)
        if model is not None:
            q_mean = np.mean(np.vstack([s["q"] for s in segs]), axis=0)
            row["M_model"] = model_inertia(model, data, q_mean, joint, mass, cog_mm)
            row["dI"] = row["M"] - row["M_model"]
        cells = velocity_cell_fit(segs, k)
        if cells is not None:
            row["M_cell"], row["s_cell"] = cells
            if row["M_model"] is not None:
                row["dI_cell"] = row["M_cell"] - row["M_model"]
        if row["M"] <= 0.0:
            row["note"] = "M negativa o nula: NO USABLE (¿signo de la corriente?)"
        elif row["err"] > MAX_REL_ERR * row["M"]:
            row["note"] = "POCA SENAL"
        out.append(row)
    return out


def combine_joints(rows):
    """Rotor por junta, de todas las posturas válidas. {joint: dict}.

    `consistent` es False si dos posturas discrepan más que su incertidumbre: el
    rotor no depende de la postura. `err` no baja de esa discrepancia.
    """
    out = {}
    good = [r for r in rows if r["dI"] is not None and not r["note"]]
    for joint in sorted({r["joint"] for r in good}):
        vals = [r for r in good if r["joint"] == joint]
        d = np.array([r["dI"] for r in vals])
        # El error de un ajuste de cuatro puntos puede salir casualmente mínimo.
        e = np.array([max(r["err"], 1e-3) for r in vals])
        w = 1.0 / e ** 2
        mean = float((w * d).sum() / w.sum())
        err = float(1.0 / math.sqrt(w.sum()))
        spread = float(d.max() - d.min())
        limit = 3.0 * float(np.sqrt((e ** 2).sum())) + max(r["sens"] for r in vals)
        out[joint] = {"dI": mean, "err": max(err, 0.5 * spread), "spread": spread,
                      "sens": max(r["sens"] for r in vals), "postures": len(vals),
                      "consistent": len(vals) < 2 or spread <= limit}
    return out


def rotor_range(rows):
    """{junta: (bajo, alto)} de los dos estimadores y todas las posturas válidas."""
    out = {}
    for r in rows:
        if r["dI"] is None or r["note"]:
            continue
        values = [r["dI"]] + ([] if r["dI_cell"] is None else [r["dI_cell"]])
        low, high = out.get(r["joint"], (math.inf, -math.inf))
        out[r["joint"]] = (min(low, *values), max(high, *values))
    return dict(sorted(out.items()))
