"""Fricción de `shoulder_lift` a baja velocidad, con la carga del corte (D36).

En `smc_1650`, con la gravedad y la fricción corregidas, a `shoulder_lift` le faltaban 0.36 N·m en el
corte, donde va a 0.015 rad/s: por debajo de la menor velocidad de la campaña de agosto (0.02) y con el
doble de carga gravitatoria (−45 N·m frente a −21 en `q_init`, donde se midió). Aquí se mide con un
barrido por CONTROL DE POSICIÓN, sin par, como la campaña (`run_current_sweep.py`): la fricción es la
semidiferencia de corrientes entre sentidos por `k` (`friction_table.joint_friction`).

Aquí solo está lo que no necesita ni ROS ni el robot; la sesión es `scripts/run_lift_friction_real.py`.
"""

from __future__ import annotations

LIFT = 1

#: Punta (y, z) de la postura [m]: `shoulder_lift` soporta −42.4 N·m de gravedad, el 93 % de la del
#: corte (−45.4), y barriendo ±`AMPLITUDE` la punta no baja de 105 mm sobre el tablero.
TIP = (0.69, 0.10)

#: Amplitud del barrido [rad]: ±42 mm en la punta.
AMPLITUDE = 0.06

#: Velocidades [rad/s]: la del corte (0.015), por debajo hasta 0.005, y 0.02 y 0.05 para compararlas
#: con la campaña de agosto en `q_init` (efecto de la carga). Hasta 0.1: más rápido, la meseta de
#: ±0.06 rad dura menos de un segundo.
SPEEDS = (0.005, 0.01, 0.015, 0.02, 0.05, 0.10)

#: Lo que se toma de `sweep_params.yaml` para que el generador no reduzca la amplitud del nivel lento
#: (24 s de recorrido a 0.005 rad/s).
MAX_SWEEP_DURATION = 40.0

#: `test_num` por defecto: `cur_1660.csv`.
TEST_NUM = 1660


def sweep_overrides(q_fixed) -> dict:
    """Lo que cambia de `sweep_params.yaml` para este barrido."""
    return {"q_fixed": [float(v) for v in q_fixed], "joint": LIFT, "amplitude": AMPLITUDE,
            "velocities": [float(v) for v in SPEEDS], "max_sweep_duration": MAX_SWEEP_DURATION}


def duration_s(ramp_fraction: float = 0.15, dwell: float = 1.0, approach: float = 3.0) -> float:
    """Duración aproximada del barrido: ida y vuelta por velocidad, más reposos."""
    passes = sum(2 * (2 * AMPLITUDE / v) / (1 - ramp_fraction) for v in SPEEDS)
    return approach + passes + 2 * len(SPEEDS) * dwell
