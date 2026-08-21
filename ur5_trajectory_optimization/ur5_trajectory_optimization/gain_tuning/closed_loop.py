"""
Evaluador de LAZO CERRADO offline para la sintonía de ganancias (FASE 7).

Integra la dinámica del UR5e con Pinocchio (ABA) sobre la trayectoria de
incisión, reproduciendo el nodo ROS: mismo modelo, mismo `dt`, misma saturación,
misma política de gravedad y misma ley de control. Cada evaluación devuelve los
tres objetivos y las restricciones duras del plan, sin levantar Gazebo — que es
lo que hace viable un NSGA-II de miles de evaluaciones.

Decisión de diseño: la TABLA DE REFERENCIAS NO SE REIMPLEMENTA AQUÍ
-------------------------------------------------------------------
Se carga la que exporta el propio nodo (`reference_table_out`). La tabla sale de
`IncisionTrajectory` + IK QP + refinamiento de Newton en C++; una segunda
implementación en Python divergiría en silencio de la que realmente se ejecuta,
y entonces se estarían optimizando ganancias para una trayectoria distinta de la
que va al robot. Con el volcado, el evaluador y el nodo comparten referencia
byte a byte.

Lo que este evaluador SÍ aproxima, y hay que declararlo:
  - **Sin retardo de tubería.** El lazo real tiene ~1 ms de desfase entre medir
    el estado y aplicar el par (medido en la FASE 2). Aquí el par actúa en el
    mismo paso. Eso hace el evaluador OPTIMISTA en el chattering, justo la
    métrica que la FASE 5 demostró que depende del umbral discreto.
  - **Estado exacto.** Sin ruido de medida en `q̇`. En la FASE 5 se vio que `s`
    está dominado por ese ruido, así que `max|s|` offline saldrá mejor que el
    medido.
  - **Fricción** según lo que se pase en `friction`; con `None`, planta sin
    fricción. Cuando se pasa, se aplica a nivel de VELOCIDAD y no de par (ver
    `JointFriction`): sumarla al par e integrarla explícitamente diverge en
    `wrist_3` a los 7 pasos. El modelo discreto que resulta —viscoso implícito,
    Coulomb por proyección— es de primer orden en `dt`, igual que el integrador,
    pero NO es el mismo esquema que usa Gazebo, así que la comparación
    evaluador↔Gazebo hay que hacerla sobre las juntas donde ambos son válidos.
Por eso el evaluador se VALIDA contra una corrida de Gazebo antes de usarlo
(criterio del plan), y las ganancias que salgan se re-verifican en simulación.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field

import numpy as np
import pinocchio as pin

TAU_MAX = np.array([150.0, 150.0, 150.0, 28.0, 28.0, 28.0])
DQ_MAX = np.full(6, np.pi)
Q_INIT = np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0])


# ─────────────────────────────────────────────────────────────────────────────
# Referencia
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class Reference:
    dt: float
    q: np.ndarray        # (N, 6)
    dq: np.ndarray
    ddq: np.ndarray
    phase: np.ndarray    # (N,) etiquetas

    @property
    def n(self) -> int:
        return len(self.q)

    def cut_plateau_mask(self) -> np.ndarray:
        """Muestras del régimen del corte (donde se miden las métricas)."""
        # La etiqueta la pone el generador; para la incisión el nodo emite
        # "TRACK", así que se cae a la ventana temporal documentada en
        # docs/01_trajectory.md.
        t = np.arange(self.n) * self.dt
        return (t >= 15.90) & (t <= 23.90)


def load_reference(path: str) -> Reference:
    # OJO: numpy.genfromtxt(names=True) NO salta las lineas `#` — toma la
    # PRIMERA linea del fichero como cabecera de columnas, comentario incluido.
    # Hay que contarlas y pasarlas por skip_header. Mismo detalle documentado en
    # ur5_dyn_control/include/ur5_dyn_control/csv_logger.hpp.
    dt = 0.002
    n_meta = 0
    with open(path) as fh:
        for line in fh:
            if line.startswith("#"):
                n_meta += 1
                if line.startswith("# dt="):
                    dt = float(line.split("=")[1])
            else:
                break
    d = np.genfromtxt(path, delimiter=",", names=True, dtype=None,
                      encoding="utf-8", skip_header=n_meta)
    col = lambda p: np.column_stack([d[p % i] for i in range(1, 7)])  # noqa: E731
    return Reference(dt=dt, q=col("q%d"), dq=col("dq%d"), ddq=col("ddq%d"),
                     phase=np.asarray(d["phase"], dtype=str))


# ─────────────────────────────────────────────────────────────────────────────
# Perfil de fuerza de corte
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class CuttingForce:
    """
    `F_ext(t)` en el TCP durante la incisión.

    Sintético por ahora: cero en aproximación, rampa durante la penetración,
    meseta con ruido durante el corte, y caída en la retirada. El plan es
    REEMPLAZARLO por el perfil medido con `ft_data` en la FASE 9; hasta
    entonces cualquier conclusión cuantitativa sobre robustez frente a la fuerza
    de corte es provisional y así hay que reportarla.
    """

    f_cut: float = 5.0          # [N] meseta de fuerza de corte
    noise_std: float = 0.5      # [N] rugosidad del material
    seed: int = 0
    t_penetration: tuple = (11.60, 13.60)
    t_cut: tuple = (13.90, 25.90)
    # Fuerza que el ROBOT ejerce sobre el tejido (`simulate` aplica la reacción
    # −Jᵀw sobre el brazo): normal hacia dentro del tejido (−z, mantiene la
    # profundidad) más arrastre a lo largo del avance (+y, el eje del trazo).
    direction: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 1.0, -1.0]) / np.sqrt(2.0))

    def wrench(self, t: float, rng: np.random.Generator) -> np.ndarray:
        """Wrench 6D en el TCP (LOCAL_WORLD_ALIGNED), [f; tau]."""
        f = 0.0
        if self.t_penetration[0] <= t < self.t_penetration[1]:
            a, b = self.t_penetration
            f = self.f_cut * (t - a) / (b - a)
        elif self.t_cut[0] <= t <= self.t_cut[1]:
            f = self.f_cut + self.noise_std * rng.standard_normal()
        w = np.zeros(6)
        w[:3] = f * self.direction
        return w


@dataclass
class JointFriction:
    """
    Fricción articular aplicada a nivel de VELOCIDAD, no de par.

    Por qué no como par
    -------------------
    Sumar `−f_v·q̇ − f_c·tanh(q̇/ε)` al par e integrar explícitamente es
    inestable en este robot, y por dos vías distintas (medido 2026-08-04,
    docs/05_smc.md §7.5):

      - **Viscoso**: el factor de amplificación de ese modo es `1 − f_v·dt/M_ii`.
        En `wrist_3` (M = 2.6e-4, f_v = 2.87) con dt = 2 ms vale **−21.3**:
        diverge. Gazebo, con dt = 1 ms y tratamiento implícito, no diverge pero
        CONGELA la junta, que es el mismo problema con otra cara.
      - **Coulomb suavizado**: `f_c·tanh(q̇/ε)` tiene pendiente `f_c/ε` en el
        origen — 3180 N·m·s/rad con ε = 1e-3 — o sea un número de rigidez de
        **24 693**. Cuatro órdenes peor que el viscoso. No hay ε utilizable: para
        domarlo haría falta ε ≈ 24.7 rad/s, más que cualquier velocidad de
        trabajo.

    Qué se hace en su lugar
    -----------------------
    El viscoso, implícito (incondicionalmente estable, sin coste: es una
    división). El Coulomb, como PROYECCIÓN de velocidad: se resta el impulso
    `f_c·dt/M` y se satura en cero, de modo que la junta nunca cruza el cero
    empujada por su propia fricción. Eso es exactamente la regla física —se pega
    si su cantidad de movimiento no supera el impulso de fricción— y es la misma
    familia de esquemas que usan los motores de física para el contacto.

    Ambos son de primer orden en `dt`, igual que el integrador que los rodea, así
    que no degradan el orden del método.

    ESTADO: SIN VALIDAR — no usar para optimizar
    --------------------------------------------
    Contra la corrida `smc_403` de Gazebo, con la MISMA configuración (mismas
    ganancias, misma referencia, misma fricción inyectada, feedforward por
    velocidad deseada), el error articular sale así:

        junta          evaluador   Gazebo 403     razón
        shoulder_pan     0.39900      0.00066    604.55
        shoulder_lift    0.01917      0.00014    136.90
        elbow            0.09118      0.00028    325.64
        wrist_1          1.19845      0.00099   1210.56
        wrist_3          0.01092      0.15994      0.07

    `wrist_3` está frozen en Gazebo por un artefacto propio (docs/05_smc.md
    §7.5), así que ahí el que miente es Gazebo. En las otras cuatro miente esto.

    La causa está identificada pero no resuelta: casar un integrador propio con
    un motor de física basado en LCP sobre fricción rígida no es un ajuste, es
    un problema de método. La versión desacoplada anterior era peor (hasta
    2195×); pasar a la métrica de M arregló el codo 7× y empeoró `wrist_1`.

    Alternativa que probablemente sea la correcta, y que está sin decidir: el
    optimizador NO necesita simular stick-slip. Necesita dimensionar `K` contra
    la INCERTIDUMBRE que queda tras compensar, que es el error de
    identificación (~5 %) y es una perturbación ACOTADA y bien condicionada —
    justo lo que la teoría de SMC pide y lo que ya hace `disturbance_bound` con
    la fuerza de corte. El stick-slip completo es cosa de Gazebo (FASE 8).
    """

    f_v: np.ndarray          # [N·m·s/rad]
    f_c: np.ndarray          # [N·m]

    def __post_init__(self):
        self.f_v = np.broadcast_to(np.asarray(self.f_v, float), (6,)).copy()
        self.f_c = np.broadcast_to(np.asarray(self.f_c, float), (6,)).copy()
        if np.any(self.f_v < 0.0) or np.any(self.f_c < 0.0):
            raise ValueError("f_v y f_c son magnitudes: no pueden ser negativas")

    def feedforward(self, dq_ref: np.ndarray, eps: float = 1e-3) -> np.ndarray:
        """
        Par de COMPENSACIÓN que el nodo suma al mando, espejo de
        `frictionFeedforward` en `torque_command.hpp`.

        Se evalúa sobre la velocidad DESEADA, no la medida, igual que el nodo
        con `friction.dq_source = desired`: con la medida, `tanh(q̇/ε)` se anula
        justo cuando la junta está clavada y no la desbloquea nunca
        (docs/05_smc.md §7.1 — 314× de diferencia medidos en Gazebo).

        Aquí sí es un PAR y no una operación sobre la velocidad, y el `tanh` no
        da problemas de rigidez porque `dq_ref` viene de la tabla de referencia:
        es una entrada conocida, no realimentación.
        """
        return self.f_v * dq_ref + self.f_c * np.tanh(dq_ref / eps)

    def apply(self, dq_free: np.ndarray, M: np.ndarray, dt: float) -> np.ndarray:
        """
        Velocidad tras un paso `dt`, partiendo de la libre de fricción.

        Se resuelve en la MÉTRICA DE M, no junta a junta. La versión desacoplada
        —dividir por `1 + f_v·dt/M_ii` y restar `f_c·dt/M_ii`— parece razonable y
        está mal: el par de compensación que manda el controlador entra en la
        planta por `M⁻¹`, que está acoplada, mientras que una fricción diagonal
        sale por `M_ii`. Las dos no pueden cancelarse. Medido en `q_init`: en
        `shoulder_pan` el feedforward aporta −0.0096 rad/s por paso y la
        proyección diagonal quitaba 0.0138, un frenado espurio de 0.023 rad/s
        que en 14 717 pasos da 0.4 rad de error — con Gazebo dando 0.0007 sobre
        la misma configuración.

        Viscoso, implícito y exacto:  `(M + dt·F_v) q̇⁺ = M q̇_libre`.
        Coulomb, como impulso acotado en esa misma métrica, con pegado si
        invertiría el signo: la junta no cruza el cero empujada por su fricción.
        """
        A = M + dt * np.diag(self.f_v)
        dq = np.linalg.solve(A, M @ dq_free)
        salto = np.linalg.solve(A, self.f_c * dt)
        dq_frenada = dq - np.sign(dq) * np.abs(salto)
        return np.where(np.sign(dq_frenada) != np.sign(dq), 0.0, dq_frenada)


# ─────────────────────────────────────────────────────────────────────────────
# Leyes de control (espejo exacto de los nodos C++)
# ─────────────────────────────────────────────────────────────────────────────

class SmcLaw:
    """
    Espejo de gz_smc_control_node.cpp. Ver docs/05_smc.md §1.

    Devuelve `(tau, s, info)`; `info["chi"]` es el número de estabilidad
    discreta por junta SIN el `dt` (lo aplica `simulate`, que es quien lo
    conoce):

        chi_i = (K_i / phi_i) / M_ii      [1/s]  →  ·dt es adimensional

    Dentro de la capa límite `sat(s/phi)` no conmuta: actúa como una ganancia
    proporcional `K/phi`, así que el polo del lazo es `(K/phi)/M_ii` y el lazo
    discreto se vuelve inestable —ciclo límite— cuando `chi_i·dt` se acerca a
    la unidad. La FASE 5 midió que ese umbral, y no el compromiso clásico de
    `phi`, es lo que gobierna el chattering (docs/05_smc.md §4-5), así que
    se instrumenta aquí para poder imponerlo como restricción.

    `phi` admite un escalar o seis valores. El nodo lo expone como el parametro
    escalar `phi` mas un `phi_joint` opcional de seis; aqui basta un array
    porque numpy difunde el escalar. Hace falta por junta porque el umbral
    discreto va por junta y la inercia del UR5e abarca cuatro ordenes de
    magnitud: un phi comun razonable en el hombro deja chi ~ 515 en wrist_3.
    """

    def __init__(self, lam, eta, phi, alpha=0.3, use_sat=True):
        self.lam = np.asarray(lam, float)
        self.eta = np.asarray(eta, float)
        self.phi = np.broadcast_to(np.asarray(phi, float), (6,)).copy()
        if not np.all(self.phi > 0.0):
            raise ValueError("phi debe ser > 0 en las seis juntas")
        self.alpha = float(alpha)
        self.use_sat = bool(use_sat)

    def __call__(self, model, data, q, dq, ref_q, ref_dq, ref_ddq):
        q_e, dq_e = q - ref_q, dq - ref_dq
        s = dq_e + self.lam * q_e
        dq_r = ref_dq - self.lam * q_e
        ddq_r = ref_ddq - self.lam * dq_e

        M = pin.crba(model, data, q)
        M = np.triu(M) + np.triu(M, 1).T
        b = pin.nonLinearEffects(model, data, q, dq)
        C = pin.computeCoriolisMatrix(model, data, q, dq)
        dM = C + C.T

        K = self.eta + np.abs(self.alpha * (M @ ddq_r) + self.alpha * b +
                              (1.0 - self.alpha) * (dM @ dq_r))
        rho = np.clip(s / self.phi, -1.0, 1.0) if self.use_sat else np.sign(s)
        tau = b + M @ ddq_r - K * rho
        # `M_diag` lo consume `simulate` para el paso implicito de friccion.
        # Se devuelve aqui porque la ley YA calculo M: recalcularla en el
        # integrador duplicaria una crba por ciclo sin necesidad.
        # `M` la consume `simulate` para el paso implicito de friccion. Se
        # devuelve aqui porque la ley YA la calculo: recalcularla en el
        # integrador duplicaria una crba por ciclo. Esta evaluada en el estado
        # RETARDADO que ve la ley, un paso por detras del de la planta; a 2 ms
        # la diferencia en M es despreciable frente a lo que cuesta repetirla.
        return tau, s, {"chi": (K / self.phi) / np.diag(M), "M": M}


# ─────────────────────────────────────────────────────────────────────────────
# Integración de lazo cerrado
# ─────────────────────────────────────────────────────────────────────────────

class Plant:
    """
    Modelo de Pinocchio del UR5e con el frame `gripper_tcp`, construido UNA vez.

    Existe porque `simulate` se llama miles de veces desde el optimizador:
    reconstruir el modelo desde el URDF en cada evaluación costaba más que
    varios pasos de integración. Es el mismo modelo, la misma gravedad y el
    mismo offset de TCP que `Ur5Dynamics` en C++.
    """

    def __init__(self, urdf: str, gravity: float = 9.8,
                 tcp_frame: str = "gripper_tcp", tcp_offset: float = 0.141):
        self.model = pin.buildModelFromUrdf(urdf)
        self.model.gravity.linear = np.array([0.0, 0.0, -gravity])
        f0 = self.model.frames[self.model.getFrameId("tool0")]
        self.tcp = self.model.addFrame(pin.Frame(
            tcp_frame, f0.parentJoint, self.model.getFrameId("tool0"),
            f0.placement * pin.SE3(np.eye(3), np.array([0, 0, tcp_offset])),
            pin.OP_FRAME))
        self.data = self.model.createData()

    def inertia_diag(self, q) -> np.ndarray:
        """diag(M(q)) — la escala con la que hay que ponderar cualquier ganancia."""
        M = pin.crba(self.model, self.data, np.asarray(q, float))
        return np.diag(np.triu(M) + np.triu(M, 1).T).copy()


@dataclass
class EvalResult:
    f1_iae: float           # ∫‖e_p‖ dt  [m·s]  (IAE cartesiano)
    f2_effort: float        # ∫Σ tau² dt [N²m²s]
    f3_chatter: float       # ∫Σ|dtau/dt| dt = TV(tau)
    g1_tau: float           # max|tau_i| − tau_max_i   (≤ 0 factible)
    g2_dq: float            # max|dq_i| − dq_max_i
    rmse_q: float
    rmse_tcp_mm: float
    s_max: float
    diverged: bool
    n_sat: int
    chi_max: float = 0.0    # max_t max_i (K_i/phi)·dt/M_ii  (adimensional)
    #: `chi_max` POR JUNTA. Hace falta porque el umbral de ciclo límite NO es el
    #: mismo en todas: medido con fricción, va de 0.22 en `shoulder_lift` a 0.99
    #: en `wrist_1`, un factor 5 (docs/05_smc.md §7.6). Con solo el máximo global
    #: no se puede imponer un límite por junta, y un escalar único es a la vez
    #: demasiado permisivo para las grandes y demasiado restrictivo para la
    #: muñeca.
    chi_joint: np.ndarray = field(
        default_factory=lambda: np.zeros(6))


#: Desviación típica del ruido de velocidad articular [rad/s], MEDIDA sobre las
#: corridas limpias de la FASE 5 en Gazebo (`smc_522`, `smc_523`; φ = 0.10 y
#: 0.20): contenido de `q̇ − q̇_d` por encima de 50 Hz en la meseta del corte,
#: entre 1e-6 y 5e-6 rad/s según la junta. Es el suelo numérico del simulador,
#: no ruido de sensor.
#:
#: PENDIENTE: en el UR5e REAL este valor es otro y NO está caracterizado. Como
#: `λ` está limitada precisamente por el ruido de `q̇` que amplifica, la `λ`
#: que salga de esta fase vale para la campaña de Gazebo (FASE 8) y hay que
#: re-validarla con datos del robot antes de la FASE 9.
DQ_NOISE_STD_GAZEBO = 5.0e-6


#: Retardo de tubería, en pasos de control. El lazo real no es instantáneo: el
#: nodo lee `/joint_states` publicado en el ciclo anterior, calcula el par y lo
#: publica, y el simulador lo aplica en el siguiente paso. La FASE 2 lo midió
#: como ~1 ms de desfase (docs/02_friction.md), del orden de medio ciclo a 500
#: Hz; se modela **un** paso completo, que es la hipótesis conservadora.
#:
#: NO es un detalle: sin él el evaluador premia `λ` arbitrariamente grande. Con
#: las ganancias que salieron de la FASE 7 sin modelarlo (λ·dt hasta 0.60), la
#: predicción offline daba 0.02 mm de error de TCP y Gazebo midió 29.7 mm.
PIPELINE_DELAY_STEPS = 1


def simulate(law, ref: Reference, plant: Plant,
             force: CuttingForce | None = None,
             friction: "JointFriction | None" = None,
             friction_ff: "JointFriction | None" = None,
             dq_noise_std: float = DQ_NOISE_STD_GAZEBO,
             delay_steps: int = PIPELINE_DELAY_STEPS,
             seed: int = 0) -> EvalResult:
    model, data, tcp = plant.model, plant.data, plant.tcp
    rng = np.random.default_rng(seed)
    delay_steps = max(0, int(delay_steps))

    dt, n = ref.dt, ref.n
    q, dq = ref.q[0].copy(), ref.dq[0].copy()

    e_p = np.zeros(n)
    e_q = np.zeros((n, 6))
    taus = np.zeros((n, 6))
    s_hist = np.zeros((n, 6))
    dq_hist = np.zeros((n, 6))
    n_sat = 0

    chi_max = 0.0
    chi_joint = np.zeros(6)
    # Cola del retardo de tubería: la ley ve el estado de hace `delay_steps`.
    from collections import deque
    hist: deque = deque([(q.copy(), dq.copy())] * (delay_steps + 1),
                        maxlen=delay_steps + 1)

    for k in range(n):
        t = k * dt
        # La ley ve la velocidad MEDIDA; la planta integra la verdadera.
        dq_meas = (dq + dq_noise_std * rng.standard_normal(6)
                   if dq_noise_std > 0.0 else dq)
        hist.append((q.copy(), dq_meas))
        q_seen, dq_seen = hist[0]
        tau_law, s, info = law(model, data, q_seen, dq_seen,
                               ref.q[k], ref.dq[k], ref.ddq[k])
        if "chi" in info:
            chi_i = np.asarray(info["chi"], dtype=float) * dt
            chi_joint = np.maximum(chi_joint, chi_i)
            chi_max = max(chi_max, float(chi_i.max()))
        # `n_sat` mide cuando el CONTROLADOR pide mas par del que el actuador
        # entrega, asi que se evalua sobre la salida de la ley y nada mas. La
        # friccion no interviene aqui: no la manda nadie y ningun limite de
        # actuador la recorta. Ver `JointFriction`, que la aplica a nivel de
        # velocidad despues de integrar.
        # La COMPENSACION de friccion si es par de mando: la pide el controlador
        # y el actuador tiene que entregarla, asi que entra antes del recorte y
        # cuenta para `n_sat`. Es lo contrario de la friccion de la PLANTA, que
        # no la manda nadie y se aplica sobre la velocidad. Confundirlas es el
        # error que hay que evitar aqui.
        if friction_ff is not None:
            tau_law = tau_law + friction_ff.feedforward(ref.dq[k])
        tau = np.clip(tau_law, -TAU_MAX, TAU_MAX)
        if np.any(np.abs(tau_law) > TAU_MAX):
            n_sat += 1

        # Fuerza externa: se mapea al espacio articular con Jᵀ.
        tau_ext = np.zeros(6)
        if force is not None:
            J = pin.computeFrameJacobian(model, data, q, tcp,
                                         pin.LOCAL_WORLD_ALIGNED)
            tau_ext = J.T @ force.wrench(t, rng)

        ddq = pin.aba(model, data, q, dq, tau - tau_ext)
        if not np.all(np.isfinite(ddq)) or np.abs(dq).max() > 1e3:
            return EvalResult(np.inf, np.inf, np.inf, np.inf, np.inf,
                              np.inf, np.inf, np.inf, True, n_sat, chi_max,
                              chi_joint)
        # Semi-implícito (simpléctico): mismo esquema que un integrador de
        # física, más estable que Euler explícito al mismo dt.
        dq = dq + ddq * dt
        if friction is not None:
            M_pl = info.get("M")
            if M_pl is None:
                M_pl = pin.crba(model, data, q)
                M_pl = np.triu(M_pl) + np.triu(M_pl, 1).T
            dq = friction.apply(dq, M_pl, dt)
        q = q + dq * dt

        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        p = data.oMf[tcp].translation.copy()
        pin.forwardKinematics(model, data, ref.q[k])
        pin.updateFramePlacements(model, data)
        e_p[k] = np.linalg.norm(p - data.oMf[tcp].translation)

        e_q[k] = q - ref.q[k]
        taus[k] = tau
        s_hist[k] = s
        dq_hist[k] = dq

    m = ref.cut_plateau_mask()
    if not m.any():
        # Una referencia más corta que la ventana de la meseta dejaría las
        # métricas de régimen en NaN, y un NaN en `g4` se propaga a la
        # comprobación de factibilidad SIN levantar ningún error: el optimizador
        # aceptaría como factible algo que no lo es. Se cae a la trayectoria
        # completa, que para una referencia así es lo único que hay.
        m = np.ones(n, dtype=bool)
    dtau = np.diff(taus, axis=0)
    return EvalResult(
        f1_iae=float(np.sum(e_p) * dt),
        f2_effort=float(np.sum(taus ** 2) * dt),
        f3_chatter=float(np.abs(dtau).sum()),
        g1_tau=float((np.abs(taus).max(axis=0) - TAU_MAX).max()),
        g2_dq=float((np.abs(dq_hist).max(axis=0) - DQ_MAX).max()),
        rmse_q=float(np.sqrt((e_q[m] ** 2).mean())),
        rmse_tcp_mm=float(1e3 * np.sqrt((e_p[m] ** 2).mean())),
        s_max=float(np.abs(s_hist[m]).max()),
        diverged=False, n_sat=n_sat, chi_max=chi_max, chi_joint=chi_joint)


def default_urdf() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("ur5_kinematics"), "ur5e.urdf")
