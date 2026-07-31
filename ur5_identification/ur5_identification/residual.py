"""
Residuo de par a partir de un CSV de barrido (FASE 2).

    tau_residual = tau_cmd - RNEA(q, q̇, q̈)

Con el modelo de cuerpo rígido correcto, lo que queda es la fricción articular
(más lo no modelado: masa de la herramienta —supuesto A1—, errores de
calibración, etc.).

Tres cuidados que el plan exige explícitamente:

1. **q̈ filtrado.** El CSV no registra aceleración medida, así que se deriva de
   ``q̇``. Derivar amplifica el ruido, por eso se usa un filtro de FASE CERO
   (``filtfilt``): un Butterworth normal metería un retardo que se traduce en un
   sesgo dependiente de la velocidad, justo la variable independiente del ajuste.

2. **Descarte de tramos de aceleración.** Solo se usan las ventanas marcadas
   como meseta en la columna ``state`` (``SWEEP_<v>_POS`` / ``SWEEP_<v>_NEG``),
   donde q̈ ≈ 0 por construcción. Las rampas (``SWEEP_RAMP``) y las transiciones
   (``SWEEP_MOVE``) se descartan.

3. **Torque = COMANDO, nunca el campo ``effort``.** En el UR5e real ese campo son
   corrientes de motor, no pares físicos (compuerta G5). El CSV ya registra el
   par comandado.
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass

import numpy as np
import pinocchio as pin
from scipy.signal import butter, filtfilt

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

_PLATEAU_RE = re.compile(r"^SWEEP_([0-9.]+)_(POS|NEG)$")


@dataclass
class SweepWindow:
    """Una meseta de velocidad constante: una condición experimental."""

    velocity: float          # velocidad NOMINAL con signo [rad/s]
    joint: int               # junta barrida
    t: np.ndarray
    dq: np.ndarray           # velocidad MEDIDA de la junta barrida
    residual: np.ndarray     # par residual de la junta barrida [N·m]

    @property
    def dq_mean(self) -> float:
        return float(np.mean(self.dq))

    @property
    def residual_mean(self) -> float:
        return float(np.mean(self.residual))

    @property
    def residual_std(self) -> float:
        return float(np.std(self.residual, ddof=1)) if len(self.residual) > 1 else 0.0


def load_csv(path: str) -> dict:
    """Lee el CSV de un nodo de control. Devuelve columnas como arrays."""
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")
    out = {"t": np.asarray(data["t"], dtype=float),
           "state": np.asarray(data["state"], dtype=str)}
    for key, col in (("q", "q%d"), ("dq", "dq%d"), ("tau", "tau%d")):
        out[key] = np.column_stack([np.asarray(data[col % (i + 1)], dtype=float)
                                    for i in range(6)])
    return out


def build_model(urdf_path: str, gravity: float = 9.8):
    model = pin.buildModelFromUrdf(urdf_path)
    if model.nv != 6:
        raise RuntimeError(f"se esperaba un modelo de 6 DOF, nv={model.nv}")
    model.gravity.linear = np.array([0.0, 0.0, -gravity])
    return model, model.createData()


def zero_phase_lowpass(x: np.ndarray, fs: float, cutoff_hz: float,
                       order: int = 4) -> np.ndarray:
    """Butterworth pasa-bajos de fase cero por columnas."""
    nyq = 0.5 * fs
    wn = min(cutoff_hz / nyq, 0.99)
    b, a = butter(order, wn, btype="low")
    pad = 3 * max(len(a), len(b))
    if x.shape[0] <= pad:
        return x.copy()
    return filtfilt(b, a, x, axis=0)


def compute_residual(csv_path: str, urdf_path: str, gravity: float = 9.8,
                     cutoff_hz: float = 10.0) -> dict:
    """
    Calcula el residuo de par en TODA la corrida.

    Devuelve dict con t, q, dq (filtradas), ddq (derivada filtrada), tau_cmd,
    tau_model (RNEA) y residual, todos de forma (N, 6).
    """
    d = load_csv(csv_path)

    # ── Depuración del eje temporal ──────────────────────────────────────────
    # El lazo de control es un timer de PARED contra un reloj de simulación
    # discreto de 1 ms, así que unas pocas filas comparten instante (el reloj no
    # avanzó entre dos ticks). Esas filas no aportan información nueva y hacen
    # que np.gradient divida por cero: el NaN se propaga por filtfilt y
    # contamina TODA la serie. Se descartan antes de derivar.
    t_all = d["t"]
    keep = np.ones(len(t_all), dtype=bool)
    keep[1:] = np.diff(t_all) > 0
    n_drop = int((~keep).sum())
    if n_drop:
        print(f"  [residual] descartadas {n_drop} de {len(t_all)} filas con "
              f"dt <= 0 ({100 * n_drop / len(t_all):.3f} %): el reloj de "
              f"simulación no avanzó entre ticks")
    for k in ("t", "state", "q", "dq", "tau"):
        d[k] = d[k][keep]

    t, q, dq_raw, tau = d["t"], d["q"], d["dq"], d["tau"]

    # Frecuencia de muestreo efectiva para diseñar el filtro: la MEDIANA de dt
    # (robusta a las filas donde el reloj saltó). Las derivadas sí usan los
    # instantes reales.
    dt = np.diff(t)
    fs = 1.0 / float(np.median(dt)) if len(dt) else 500.0

    q_f = zero_phase_lowpass(q, fs, cutoff_hz)
    dq_f = zero_phase_lowpass(dq_raw, fs, cutoff_hz)
    ddq = np.gradient(dq_f, t, axis=0)
    ddq = zero_phase_lowpass(ddq, fs, cutoff_hz)

    model, data = build_model(urdf_path, gravity)
    tau_model = np.empty_like(tau)
    for k in range(len(t)):
        tau_model[k] = pin.rnea(model, data, q_f[k], dq_f[k], ddq[k])

    return {"t": t, "state": d["state"], "q": q_f, "dq": dq_f, "ddq": ddq,
            "tau_cmd": tau, "tau_model": tau_model,
            "residual": tau - tau_model, "fs": fs}


def extract_windows(res: dict, joint: int, trim_fraction: float = 0.1,
                    min_samples: int = 50) -> list[SweepWindow]:
    """
    Parte la corrida en las ventanas útiles marcadas por el generador.

    `trim_fraction` recorta ese porcentaje de muestras en cada extremo de la
    meseta: aunque el perfil garantice q̈ = 0 allí, el lazo cerrado tarda unos
    ciclos en asentar tras la rampa.
    """
    state = res["state"]
    windows: list[SweepWindow] = []

    k = 0
    while k < len(state):
        m = _PLATEAU_RE.match(state[k])
        if m is None:
            k += 1
            continue
        j = k
        while j < len(state) and state[j] == state[k]:
            j += 1
        n = j - k
        cut = int(trim_fraction * n)
        lo, hi = k + cut, j - cut
        if hi - lo >= min_samples:
            v = float(m.group(1)) * (1.0 if m.group(2) == "POS" else -1.0)
            windows.append(SweepWindow(
                velocity=v, joint=joint,
                t=res["t"][lo:hi],
                dq=res["dq"][lo:hi, joint],
                residual=res["residual"][lo:hi, joint]))
        k = j
    return windows


def infer_joint_from_csv(csv_path: str) -> int | None:
    """Junta barrida: la que se mueve durante las mesetas."""
    d = load_csv(csv_path)
    mask = np.array([bool(_PLATEAU_RE.match(s)) for s in d["state"]])
    if not mask.any():
        return None
    amplitude = d["q"][mask].max(axis=0) - d["q"][mask].min(axis=0)
    return int(np.argmax(amplitude))


def default_urdf() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("ur5_kinematics"), "ur5e.urdf")
