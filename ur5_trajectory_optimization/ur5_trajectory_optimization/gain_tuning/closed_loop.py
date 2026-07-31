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
    fricción como el Gazebo actual.
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


# ─────────────────────────────────────────────────────────────────────────────
# Leyes de control (espejo exacto de los nodos C++)
# ─────────────────────────────────────────────────────────────────────────────

class SmcLaw:
    """
    Espejo de gz_smc_control_node.cpp. Ver docs/05_smc.md §1.

    Devuelve `(tau, s, info)`; `info["chi"]` es el número de estabilidad
    discreta por junta SIN el `dt` (lo aplica `simulate`, que es quien lo
    conoce):

        chi_i = (K_i / phi) / M_ii        [1/s]  →  ·dt es adimensional

    Dentro de la capa límite `sat(s/phi)` no conmuta: actúa como una ganancia
    proporcional `K/phi`, así que el polo del lazo es `(K/phi)/M_ii` y el lazo
    discreto se vuelve inestable —ciclo límite— cuando `chi_i·dt` se acerca a
    la unidad. La FASE 5 midió que ese umbral, y no el compromiso clásico de
    `phi`, es lo que gobierna el chattering (docs/05_smc.md §4-5), así que
    se instrumenta aquí para poder imponerlo como restricción.
    """

    def __init__(self, lam, eta, phi, alpha=0.3, use_sat=True):
        self.lam = np.asarray(lam, float)
        self.eta = np.asarray(eta, float)
        self.phi = float(phi)
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
        return tau, s, {"chi": (K / self.phi) / np.diag(M)}


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


def simulate(law, ref: Reference, plant: Plant,
             force: CuttingForce | None = None, friction=None,
             dq_noise_std: float = DQ_NOISE_STD_GAZEBO,
             seed: int = 0) -> EvalResult:
    model, data, tcp = plant.model, plant.data, plant.tcp
    rng = np.random.default_rng(seed)

    dt, n = ref.dt, ref.n
    q, dq = ref.q[0].copy(), ref.dq[0].copy()

    e_p = np.zeros(n)
    e_q = np.zeros((n, 6))
    taus = np.zeros((n, 6))
    s_hist = np.zeros((n, 6))
    dq_hist = np.zeros((n, 6))
    n_sat = 0

    chi_max = 0.0

    for k in range(n):
        t = k * dt
        # La ley ve la velocidad MEDIDA; la planta integra la verdadera.
        dq_meas = (dq + dq_noise_std * rng.standard_normal(6)
                   if dq_noise_std > 0.0 else dq)
        tau_law, s, info = law(model, data, q, dq_meas, ref.q[k], ref.dq[k], ref.ddq[k])
        if "chi" in info:
            chi_max = max(chi_max, float(np.max(info["chi"])) * dt)
        if friction is not None:
            tau_law = tau_law + friction(dq)
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
                              np.inf, np.inf, np.inf, True, n_sat, chi_max)
        # Semi-implícito (simpléctico): mismo esquema que un integrador de
        # física, más estable que Euler explícito al mismo dt.
        dq = dq + ddq * dt
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
        diverged=False, n_sat=n_sat, chi_max=chi_max)


def default_urdf() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("ur5_kinematics"), "ur5e.urdf")
