"""B7.1 — el SMC en aire con las ganancias finales, antes del primer corte (plan B7.1, D29).

Ganancias: las de la candidata de la fila 46, con `shoulder_pan` más baja (D28). Dos pasos, una
sesión cada uno (`scripts/run_air_bringup_real.py`, `docs/09d_air_bringup_protocol.md`):

1. **La base, con la inercia de la incisión.** En la pose donde empieza el corte, `M₁₁` vale 2.61,
   frente a 1.86 en la postura de la rampa, y la `G` de `shoulder_pan` con r = 0.71 sube de 44.6 a
   54.0: la de r = 0.85 en la rampa, donde su modo de 33 Hz cayó a ζ ≈ 0.014
   (`09c_gain_ramp_protocol.md` §8.2). Se barre `shoulder_pan` en una postura con esa inercia y la
   punta lejos del tablero, primero con su r en 0.50 y después en 0.71, y las demás juntas a r = 1.
2. **La incisión en aire**, a la altura real y con el acople sin hoja: la trayectoria del corte, con
   la mesa despejada. La punta baja a 37 mm del tablero. **Por escalones** (D31): `smc_1610`, a
   r = 1, vibró a unos 35 Hz en `shoulder_lift` y el codo, con `shoulder_lift` deslizando, algo que la
   rampa nunca probó (`09d_air_bringup_protocol.md` §5.2). Una corrida por sesión, con revisión entre
   escalones. Desde D32, con el exceso de compensación de gravedad del robot corregido (B4.4): en
   `smc_1620` era lo que dejaba la punta siempre por encima. Desde D33, con la fricción medida por
   velocidad: el modelo lineal sobrecompensaba el codo a 0.02–0.05 rad/s (`smc_1630`). Desde D34,
   con `wrist_1` a r = 1: con 0.50 se pegaba y saltaba en el corte (`smc_1640`).

Aquí solo está lo que no necesita ni ROS ni el robot.
"""

from __future__ import annotations

import numpy as np

from .gain_ramp import JOINTS

#: Punta (y, z) de la postura del paso 1 [m], en base_link y con x la del corte. Tiene casi la
#: inercia de la base de la incisión (`M₁₁` = 2.47, frente a 2.61 al empezar el corte), y barriendo
#: la base la punta no baja de 96 mm sobre el tablero. Con r = 0.71 en `shoulder_pan`, su `G` en
#: reposo es 52.2, frente a 54.0 en la incisión. En y = 0.70, la `G` de `shoulder_lift` llega a
#: 120.5 en el barrido y la sesión no lo lanza (g6); aquí, a 118.7.
BASE_TIP = (0.69, 0.05)

#: `r` de `shoulder_pan` en las dos corridas del paso 1, en orden. Las demás juntas, a r = 1.
PAN_STEPS = (0.50, 0.71)

#: La junta barrida en el paso 1.
BASE_JOINT = 0

#: Escalones de `r` de la incisión, los de la rampa (`gain_ramp.RAMP_STEPS`), en todas las juntas
#: menos `shoulder_pan`, que se queda en `PAN_INCISION_R`.
INCISION_STEPS = (0.50, 0.71, 0.85, 1.00)

#: `r` de `shoulder_pan` en la incisión: en el paso 1, con 0.71 su modo de 33 Hz quedó como a
#: r = 0.85 en la rampa (ζ ≈ 0.016), y con 0.50, entre 0.015 y 0.030 (`09d` §5.1).
PAN_INCISION_R = 0.50

#: `r` de `wrist_1` en la incisión (D34): en `smc_1640`, con 0.50, se pegaba el 15 % del corte y
#: aportaba 3.25 mm en el peor instante. Con 1, `K_P` ×4 y ese error a la cuarta parte. Apenas pesa
#: en el modo de 31.6 Hz: su ley responde ahí 0.009 N·m, frente a 0.18 de `shoulder_lift`. A r = 1
#: ya corrió limpia en el robot (`smc_1533`, la rampa).
WRIST1_INCISION_R = 1.00
WRIST1 = 3

#: `test_num` de la incisión: base + 50 + escalón, con la gravedad corregida (D32), la fricción
#: medida (D33) y `wrist_1` a r = 1 (D34). Las del paso 1 son base + 0 y base + 1; base + 10 fue
#: `smc_1610`, la que vibró (§5.2); base + 20, `smc_1620`, sin corregir (§5.3); base + 30, `smc_1630`,
#: con la gravedad (§5.4), y base + 40, `smc_1640`, con la gravedad y la fricción (§5.5). No se
#: reutilizan.
INCISION_OFFSET = 50

#: Banda del modo que vibró en `smc_1610` (34.9 y 36.5 Hz) [Hz]: cae entre las dos de la rampa
#: (20–35 y 40–55 Hz), así que en la incisión se vigila además esta.
MODE_BAND = (30.0, 40.0)

#: Guarda de seguimiento de la incisión [rad]: 21 mm a 0.7 m de alcance, por debajo de los 37 mm que
#: quedan entre la punta y el tablero. Es una cota declarada. El mayor error de la rampa fue 0.022 rad,
#: en `shoulder_pan` a 0.2 rad/s; en la incisión esa junta no se mueve.
INCISION_Q_ERR = 0.03

#: Por debajo de esto [m] sobre el tablero, la punta medida en la corrida es un motivo para parar. El
#: diseño deja 37 mm.
INCISION_MIN_CLEARANCE = 0.030

#: Error de TCP máximo en la incisión [m], para darla por buena: la profundidad del corte. Cota
#: declarada; el evaluador da 1.8–2.1 mm de RMS en la meseta.
INCISION_TCP_MAX = 0.005

#: Ancho de la meseta del corte en `z_des` [m]: las filas a menos de esto del mínimo.
CUT_Z_TOL = 1e-4


def r_vector(pan_r: float) -> np.ndarray:
    """`r` por junta: `pan_r` en `shoulder_pan` y 1 en las demás."""
    if not 0.0 < pan_r <= 1.0:
        raise ValueError(f"r de shoulder_pan debe estar en (0, 1], se dio {pan_r}")
    r = np.ones(6)
    r[BASE_JOINT] = pan_r
    return r


def base_runs(test_base: int, steps=PAN_STEPS) -> list:
    """Corridas del paso 1, en orden. `ratio` es el cociente de `r` con la anterior, por junta."""
    runs = []
    for i, pan_r in enumerate(steps):
        r = r_vector(pan_r)
        runs.append({"test_num": test_base + i, "joint": BASE_JOINT, "pan_r": float(pan_r),
                     "r_joint": r, "previous": test_base + i - 1 if i else None,
                     "ratio": r / r_vector(steps[i - 1]) if i else None})
    return runs


def incision_r_vector(r: float) -> np.ndarray:
    """`r` por junta en el escalón `r` de la incisión: `shoulder_pan` no pasa de `PAN_INCISION_R`,
    y `wrist_1` va a `WRIST1_INCISION_R` (D34)."""
    if r not in INCISION_STEPS:
        raise ValueError(f"r = {r} no es un escalón de la incisión {INCISION_STEPS}")
    out = np.full(6, float(r))
    out[BASE_JOINT] = min(PAN_INCISION_R, r)
    out[WRIST1] = max(WRIST1_INCISION_R, r)
    return out


def incision_runs(test_base: int, steps=INCISION_STEPS) -> list:
    """Escalones de la incisión, en orden. `ratio` es el cociente de `r` con el anterior, por
    junta."""
    runs = []
    for i, r in enumerate(steps):
        rv = incision_r_vector(r)
        runs.append({"test_num": test_base + INCISION_OFFSET + i, "joint": None, "r": float(r),
                     "r_joint": rv,
                     "previous": test_base + INCISION_OFFSET + i - 1 if i else None,
                     "ratio": rv / incision_r_vector(steps[i - 1]) if i else None})
    return runs


def incision_run(test_base: int, r: float) -> dict:
    """El escalón `r` de la incisión."""
    return next(run for run in incision_runs(test_base) if run["r"] == r)


def incision_indicators(d: np.ndarray, i: dict) -> dict:
    """Error de TCP y altura de la punta, sobre las columnas del CSV del nodo.

    `d` e `i` son los de `analyze_vibration.load`. La meseta del corte son las filas con `z_des` a
    menos de `CUT_Z_TOL` de su mínimo: el fondo de la penetración y el corte.
    """
    p = d[:, [i["x"], i["y"], i["z"]]]
    p_des = d[:, [i["x_des"], i["y_des"], i["z_des"]]]
    err = np.linalg.norm(p - p_des, axis=1)
    cut = p_des[:, 2] <= p_des[:, 2].min() + CUT_Z_TOL
    return {"tcp_max": float(err.max()),
            "tcp_rms_cut": float(np.sqrt(np.mean(err[cut] ** 2))),
            "cut_rows": int(cut.sum()),
            "z_min": float(p[:, 2].min()), "z_des_min": float(p_des[:, 2].min())}


def check_incision(ind: dict, table_z: float) -> list:
    """Motivos para no dar por buena la incisión en aire; vacío si está limpia."""
    reasons = []
    clearance = ind["z_min"] - table_z
    if clearance < INCISION_MIN_CLEARANCE:
        reasons.append(f"la punta bajó a {1e3 * clearance:.0f} mm del tablero (mínimo "
                       f"{1e3 * INCISION_MIN_CLEARANCE:.0f} mm)")
    if ind["tcp_max"] > INCISION_TCP_MAX:
        reasons.append(f"error de TCP de {1e3 * ind['tcp_max']:.1f} mm (máximo "
                       f"{1e3 * INCISION_TCP_MAX:.0f} mm, la profundidad del corte)")
    if ind["cut_rows"] == 0:
        reasons.append("el CSV no tiene meseta del corte")
    return reasons


def label(r) -> str:
    """`r` por junta para imprimir: la de la mayoría, y las que son distintas."""
    r = np.round(np.asarray(r, float), 6)
    values, counts = np.unique(r, return_counts=True)
    base = float(values[counts == counts.max()].max())
    other = [f"{JOINTS[j]} {r[j]:.2f}" for j in range(6) if r[j] != base]
    return f"{base:.2f}" + (f" ({', '.join(other)})" if other else "")
