"""Rampa por escalones de las ganancias del SMC, antes de la primera incisión (plan V16, D25).

La candidata de `smc_v7_rotor` supera lo probado limpio en el robot en `shoulder_pan`,
`wrist_1`, `wrist_2` y `wrist_3`, y el evaluador no tiene modo flexible ni fricción. Así
que se llega a ella por escalones, moviendo la junta para que la fricción estática no tape
una inestabilidad (`docs/09c_gain_ramp_protocol.md`).

Un escalón multiplica `λ` por `r` y divide `φ` por `r`, con `η` fija:

    K/φ -> r K/φ     G = M λ + K/φ -> r G     K_P = (K/φ) λ -> r² K_P

Aquí solo está lo que no necesita ni ROS ni el robot: los escalones, el orden de las
corridas y los criterios de parada. La sesión está en `scripts/run_gain_ramp_real.py`.
"""

from __future__ import annotations

import numpy as np

#: Escalones de `r`. Con 0.50 todas las juntas quedan por debajo de lo probado limpio; en
#: `K_P`, los cuatro son ×0.25, ×0.50, ×0.72 y ×1.
RAMP_STEPS = (0.50, 0.71, 0.85, 1.00)

#: Orden dentro de un escalón: de la junta que menos par mueve a la que más.
#: `wrist_3`, `wrist_2` y `shoulder_pan` no se mueven en la incisión, y `wrist_1` sí, pero
#: las cuatro superan lo probado. El codo sí lo tiene probado: se barre porque fue quien
#: arrancó el ciclo de `smc_712`, y es el que domina el modo que limita el margen de la
#: candidata en el modelo rígido (unos 19 Hz). `shoulder_lift` queda por debajo de lo
#: probado, y barrerla es lo que más acerca la punta al tablero.
RAMP_JOINTS = (5, 4, 3, 2, 0)

#: Punta (y, z) de la postura [m] en base_link, con x la del corte. Está en el alcance del
#: trazo, donde `G` de la base es alta (72 frente al máximo de 76 en la incisión), pero 150 mm
#: sobre la cota del corte: en las posturas del trazo, barrer el codo baja la punta a menos
#: de 50 mm del tablero. Con y = 0.65, ese barrido sube `G` de `shoulder_lift` a 124.
RAMP_TIP = (0.62, 0.15)

#: Amplitud del barrido [rad], y velocidad [rad/s] y aceleración [rad/s²] máximas que da la
#: tabla con `config/gain_ramp_params.yaml`. La aceleración entra en K por `α_M`.
RAMP_AMPLITUDE = 0.20
RAMP_SPEED = 0.20
RAMP_ACCEL = 0.625

#: `K_P` máximo probado limpio [N·m/rad], en `Q_INIT`, sobre `smc_713`, `717`, `718` y `722`.
#: Un test lo recalcula desde las ganancias de esas corridas.
TESTED_CLEAN_KP = np.array([163.4, 1986.6, 1704.2, 49.9, 19.2, 0.9])

#: Criterios de parada de `analyze_vibration.py` y `docs/09_real_bringup.md` §6: rizado de
#: τ sobre el tope y fracción de ciclos saturados. En `smc_718`, el más alto de los limpios,
#: 2.34 % y 0; en el fallo de `smc_712`, 69 % y 0.91.
RIPPLE_MAX = 0.05
SAT_MAX = 0.10

#: Banda del modo de `smc_712`, y su nivel en los 14 s antes de irse [N·m], por junta
#: (`analyze_vibration.REF_PRE_FALLO`). Se para al llegar al 85 % de ese nivel (§6.8); los
#: limpios más altos se quedaron en el 73 %. Solo hay referencia en `shoulder_lift` y el codo.
PRE_FAILURE_BAND = (20.0, 35.0)
PRE_FAILURE_LEVEL = {1: 0.089, 2: 0.081}
PRE_FAILURE_FRACTION = 0.85

#: Crecimiento entre escalones: el ruido de cuantización crece como `G`, o sea como `r`. Se
#: para si una amplitud de banda crece más del doble de eso. No es un valor medido: es una
#: cota declarada, que `smc_713`–`718` no disparan (como mucho, 1.7 veces lo esperado).
GROWTH_MAX = 2.0
#: Por debajo de esto [N·m] no se mira el crecimiento: es el suelo de ruido. `G·Δq̇` vale
#: 0.01 N·m en `wrist_1` (G = 3.5, Δq̇ = 2.86e-3 rad/s); en el fallo de `smc_712`, 0.76.
AMPLITUDE_FLOOR = 0.02

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")


def step_gains(lam, phi, r: float) -> tuple:
    """(λ, φ) del escalón `r`; `η` no cambia."""
    if not 0.0 < r <= 1.0:
        raise ValueError(f"r debe estar en (0, 1], se dio {r}")
    return np.asarray(lam, float) * r, np.asarray(phi, float) / r


def plan_runs(test_base: int, steps=RAMP_STEPS, joints=RAMP_JOINTS) -> list:
    """Corridas en orden. `test_num` = base + 10·escalón + junta, como en B4.4.

    `previous` es el `test_num` de la misma junta en el escalón anterior, con el que se
    compara el crecimiento, y `ratio` el cociente de `r` entre los dos.
    """
    runs = []
    for i, r in enumerate(steps):
        for j in joints:
            runs.append({"step": i, "r": float(r), "joint": int(j),
                         "test_num": test_base + 10 * i + int(j),
                         "previous": test_base + 10 * (i - 1) + int(j) if i else None,
                         "ratio": float(r / steps[i - 1]) if i else None})
    return runs


def over_tested(kp) -> list:
    """Juntas cuyo `K_P` supera lo probado limpio."""
    return [JOINTS[j] for j in range(6) if float(np.asarray(kp)[j]) > TESTED_CLEAN_KP[j]]


def check_run(indicators: dict, previous: dict | None = None,
              ratio: float | None = None) -> list:
    """Motivos para parar la rampa, vacío si la corrida está limpia.

    `indicators` lleva `ripple` y `sat` (fracciones, 6 valores) y `bands`:
    {(f1, f2): 6 amplitudes en N·m}. `previous` son los de la misma junta en el escalón
    anterior, y `ratio` el cociente de `r`.
    """
    reasons = []
    ripple, sat = np.asarray(indicators["ripple"]), np.asarray(indicators["sat"])
    bands = indicators["bands"]
    for j in range(6):
        if ripple[j] > RIPPLE_MAX:
            reasons.append(f"{JOINTS[j]}: rizado {100 * ripple[j]:.1f} % del tope "
                           f"(máximo {100 * RIPPLE_MAX:.0f} %)")
        if sat[j] > SAT_MAX:
            reasons.append(f"{JOINTS[j]}: {100 * sat[j]:.0f} % de ciclos saturados "
                           f"(máximo {100 * SAT_MAX:.0f} %)")
    for j, level in PRE_FAILURE_LEVEL.items():
        a = float(np.asarray(bands[PRE_FAILURE_BAND])[j])
        if a >= PRE_FAILURE_FRACTION * level:
            reasons.append(f"{JOINTS[j]}: banda de {PRE_FAILURE_BAND[0]:.0f}-"
                           f"{PRE_FAILURE_BAND[1]:.0f} Hz en {a:.3f} N·m, el "
                           f"{100 * a / level:.0f} % del nivel previo al fallo de smc_712 "
                           f"(máximo {100 * PRE_FAILURE_FRACTION:.0f} %)")
    if previous is None:
        return reasons
    if ratio is None:
        raise ValueError("con `previous` hace falta `ratio`")
    for band, amps in bands.items():
        before = np.asarray(previous["bands"][band])
        for j in range(6):
            a = float(np.asarray(amps)[j])
            if a > AMPLITUDE_FLOOR and a > GROWTH_MAX * ratio * float(before[j]):
                reasons.append(f"{JOINTS[j]}: banda de {band[0]:.0f}-{band[1]:.0f} Hz de "
                               f"{float(before[j]):.3f} a {a:.3f} N·m con r ×{ratio:.2f}: "
                               f"más del doble de lo esperado por ruido")
    return reasons
