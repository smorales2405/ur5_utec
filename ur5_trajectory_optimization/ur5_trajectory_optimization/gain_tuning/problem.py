"""
Problema de sintonía de ganancias del SMC (FASE 7).

Convierte un vector de decisión `x` en una ley de control, la simula con el
evaluador de lazo cerrado (`closed_loop.simulate`) y devuelve los tres
objetivos y las restricciones duras del plan.

Parametrizaciones
-----------------
El plan pide `[λ, η, φ]` escalar con extensión a `[λ_1..λ_6, η_1..η_6, φ]`. Se
implementan las dos:

  * ``scalar`` (3 vars): ``[log10 λ, log10 a_reach, φ]`` con
    ``η_i = M_ii(q_init)·a_reach``. Es la parametrización de la FASE 5 —
    escalada por inercia— y sirve de **caso base reproducible**.
  * ``full`` (13 vars): ``[log10 λ_1..6, log10 η_1..6, φ]``, las variables que
    pide el plan.

Por qué `η` en log10: la diagonal de inercia del UR5e recorre CUATRO órdenes de
magnitud (2.59 kg·m² en shoulder_lift frente a 2.6e-4 en wrist_3), así que las
ganancias útiles también. Un muestreo lineal gastaría casi toda la población en
la década alta y jamás resolvería las muñecas. Mismo motivo para λ.

Restricciones (≤ 0 factible)
----------------------------
  g1: max_t max_i |τ_i| − τ_max_i           límite de actuador
  g2: max_t max_i |q̇_i| − q̇_max_i           límite de velocidad
  g3: max_i(χ_i/umbral_i) − chi_safety      umbral de chattering, POR JUNTA
  g4: RMSE_TCP(meseta) − tcp_tol            la incisión se ejecuta de verdad

`g4` no está en la lista del plan y se añade con motivo. Sin él, el frente de
Pareto contiene soluciones con 228 mm de error de TCP que dominan en esfuerzo y
chattering, y la selección por punto de rodilla —que minimiza distancia a la
utopía en el espacio NORMALIZADO— las elige: f2 solo varía un 22 % a lo largo
del frente mientras f1 varía un factor 557, así que sacrificar f1 sale
«barato» en distancia normalizada. Pero una incisión que se desvía 228 mm no es
un compromiso peor, es una tarea NO EJECUTADA. Eso pertenece al conjunto
factible, no a la función objetivo.

`tcp_tol` es una **cota declarada**, no un dato de hardware: 1.0 mm, el 20 % de
la profundidad de corte de 5 mm (`incision.cut_depth` en `smc_params.yaml`). Se
reporta como tal, igual que el plan hace con `ddq_max`.

`g3` es la contribución de la FASE 5 a esta fase. Dentro de la capa límite
`sat(s/φ)` no conmuta: actúa como ganancia proporcional `K/φ`, el polo del lazo
es `(K/φ)/M_ii` y el lazo DISCRETO entra en ciclo límite cuando
`χ = (K/φ)·dt/M_ii` se acerca a 1.

**El umbral NO es universal, y esto se midió** (docs/05_smc.md §7.6). Barriendo
φ en Gazebo con la fricción real inyectada, la transición cae en:

    shoulder_lift   0.22 – 0.36
    elbow           0.32 – 0.53
    wrist_1         0.99 – 1.67

Un factor 5 entre la primera y la última. El argumento de estabilidad discreta
predice un umbral común cercano a 1, y los datos no lo respaldan: la fricción de
Coulomb no solo disipa, también es discontinua en el cruce por cero, y el
pegado-deslizado induce ciclo límite a ganancias más bajas en las juntas que más
Coulomb tienen.

Por eso `g3` se normaliza POR JUNTA. El valor anterior —`chi_limit = 0.8`
escalar, calibrado en la FASE 5 sobre planta SIN fricción— era a la vez
demasiado permisivo para hombro y codo y demasiado restrictivo para la muñeca:
un escalar conservador al umbral más bajo obligaría a `wrist_1` a φ = 0.32, con
0.92° de error permanente, más del doble de la tolerancia de TCP.

`g4` del plan (positividad de λ, η, φ) se satisface POR CONSTRUCCIÓN: las dos
primeras se buscan en log10 y φ tiene cota inferior positiva. `g3` del plan
—la condición de alcance K ≥ η + |·|— también, porque el nodo C++ calcula K en
cada ciclo desde la cota (docs/05_smc.md §1); aquí se replica igual.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass

import numpy as np
from pymoo.core.problem import ElementwiseProblem

from .closed_loop import (TAU_MAX, CuttingForce, EvalResult, JointFriction,
                          Plant, Q_INIT, Reference, SmcLaw, simulate)

#: Umbral de χ POR JUNTA, MEDIDO en Gazebo con la fricción real inyectada
#: (docs/05_smc.md §7.6). Es el borde inferior de la banda de transición: el
#: último χ en el que la energía por encima de 20 Hz sigue siendo despreciable.
#:
#: No es el mismo en todas las juntas, y por eso un escalar no sirve: va de 0.22
#: en `shoulder_lift` a 0.99 en `wrist_1`, un factor 5. Con 0.8 —el valor
#: anterior, heredado de la FASE 5 sobre planta SIN fricción— se aceptarían
#: ganancias que hacen chatear al hombro y al codo, y se rechazarían ganancias
#: válidas en la muñeca.
#:
#: SUPUESTOS declarados: `shoulder_pan` nunca cruzó en el barrido (su χ no pasó
#: de 0.20), y `wrist_2` no se mueve en esta trayectoria mientras Gazebo congela
#: a `wrist_3` (§7.5). A los tres se les asigna el de su familia — `pan` como
#: `lift`, las muñecas como `wrist_1` — y eso es suposición, no medida.
CHI_THRESHOLD = np.array([0.22, 0.22, 0.32, 0.99, 0.99, 0.99])

#: Fracción del umbral en la que se permite operar. Es el ÚNICO número de
#: ingeniería aquí: los seis de arriba son medidas.
#:
#: 0.75 no es arbitrario. Las `phi` de `smc_params.yaml` se eligieron por otra
#: vía —buscando el óptimo de error de TCP en Gazebo, §7.7— y quedaron a
#: 1.40× del umbral en las tres juntas medidas, o sea un factor 0.714. Dos
#: criterios independientes convergen ahí, así que 0.75 deja la configuración
#: actual factible con algo de holgura.
CHI_SAFETY_DEFAULT = 0.75

#: Tolerancia de seguimiento del TCP en la meseta del corte [mm]. COTA
#: DECLARADA (20 % de los 5 mm de profundidad de corte), no medida.
TCP_TOL_MM_DEFAULT = 1.0

#: Penalización devuelta cuando la simulación diverge.
PENALTY = 1e9


# ─────────────────────────────────────────────────────────────────────────────
# Parametrización
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class SmcParameterization:
    """Traduce el vector de decisión del optimizador a ganancias del SMC."""

    inertia: np.ndarray               # diag M(q_init), escala de referencia
    mode: str = "scalar"              # "scalar" | "full"
    phi_bounds: tuple = (0.02, 1.0)
    #: λ convierte una `s` acotada en error de posición (q_e ≈ s/λ en régimen),
    #: así que el óptimo la empuja hacia arriba. Quien la limita de verdad es el
    #: ruido de `q̇`, que λ amplifica; en Gazebo ese suelo es ~5e-6 rad/s
    #: (medido, ver `DQ_NOISE_STD_GAZEBO`) y no muerde hasta λ muy alta. La cota
    #: de 300 evita extrapolar a un régimen que no se ha validado.
    lam_bounds: tuple = (2.0, 300.0)          # [1/s]
    a_reach_bounds: tuple = (0.1, 500.0)      # [rad/s²]  (modo scalar)
    #: Cotas de η ancladas al ACTUADOR, no a un número elegido a mano: entre
    #: 1e-4 y 0.2 del par máximo de cada junta. El intervalo cubre a la vez la
    #: escala inercial (I·a_reach) y la de la perturbación de corte, que no
    #: escalan igual — que es justo lo que esta fase tiene que resolver.
    eta_frac_bounds: tuple = (1e-4, 0.2)

    def __post_init__(self):
        if self.mode not in ("scalar", "full"):
            raise ValueError(f"parametrización desconocida: {self.mode!r}")

    @property
    def n_var(self) -> int:
        return 3 if self.mode == "scalar" else 13

    @property
    def names(self) -> list:
        if self.mode == "scalar":
            return ["log10_lambda", "log10_a_reach", "phi"]
        return ([f"log10_lambda{i}" for i in range(1, 7)] +
                [f"log10_eta{i}" for i in range(1, 7)] + ["phi"])

    def bounds(self) -> tuple:
        """Devuelve (xl, xu) en el espacio de BÚSQUEDA (log10 donde aplica)."""
        lo_lam, hi_lam = np.log10(self.lam_bounds)
        lo_phi, hi_phi = self.phi_bounds
        if self.mode == "scalar":
            lo_a, hi_a = np.log10(self.a_reach_bounds)
            return (np.array([lo_lam, lo_a, lo_phi]),
                    np.array([hi_lam, hi_a, hi_phi]))
        lo_eta = np.log10(self.eta_frac_bounds[0] * TAU_MAX)
        hi_eta = np.log10(self.eta_frac_bounds[1] * TAU_MAX)
        return (np.concatenate([np.full(6, lo_lam), lo_eta, [lo_phi]]),
                np.concatenate([np.full(6, hi_lam), hi_eta, [hi_phi]]))

    def gains(self, x: np.ndarray) -> tuple:
        """`x` → `(lambda[6], eta[6], phi)` en unidades físicas."""
        x = np.asarray(x, dtype=float)
        if self.mode == "scalar":
            lam = np.full(6, 10.0 ** x[0])
            eta = self.inertia * (10.0 ** x[1])
            phi = float(x[2])
        else:
            lam = 10.0 ** x[:6]
            eta = 10.0 ** x[6:12]
            phi = float(x[12])
        return lam, eta, phi

    def encode(self, lam, eta, phi) -> np.ndarray:
        """Inversa de `gains`, para sembrar el optimizador con las ganancias
        de la FASE 5 (y poder comprobar que las reproduce)."""
        lam = np.atleast_1d(np.asarray(lam, dtype=float))
        eta = np.atleast_1d(np.asarray(eta, dtype=float))
        if self.mode == "scalar":
            a_reach = float(np.median(eta / self.inertia))
            return np.array([np.log10(float(lam[0])), np.log10(a_reach), phi])
        if lam.size == 1:
            lam = np.full(6, lam[0])
        return np.concatenate([np.log10(lam), np.log10(eta), [phi]])


# ─────────────────────────────────────────────────────────────────────────────
# Evaluador con caché
# ─────────────────────────────────────────────────────────────────────────────

class GainEvaluator:
    """
    Envuelve `simulate` con una caché LRU.

    La caché no es un adorno: SLSQP pide el objetivo y CADA restricción por
    separado en el mismo punto, y las diferencias finitas repiten puntos entre
    iteraciones. Sin caché, una corrida de ε-restricción costaría cuatro veces
    lo necesario a ~0.9 s la evaluación.
    """

    def __init__(self, param: SmcParameterization, ref: Reference, plant: Plant,
                 force: CuttingForce | None = None, alpha: float = 0.3,
                 chi_safety: float = CHI_SAFETY_DEFAULT,
                 tcp_tol_mm: float = TCP_TOL_MM_DEFAULT, cache_size: int = 4096,
                 friction: JointFriction | None = None):
        self.param = param
        self.ref = ref
        self.plant = plant
        self.force = force
        self.friction = friction
        self.alpha = float(alpha)
        self.chi_safety = float(chi_safety)
        self.tcp_tol_mm = float(tcp_tol_mm)
        self._cache: OrderedDict = OrderedDict()
        self._cache_size = cache_size
        self.n_eval = 0
        self.n_hit = 0
        self.n_diverged = 0

    def _key(self, x: np.ndarray) -> bytes:
        # Redondeo a 1e-9: SLSQP re-pide puntos con ruido de último bit.
        return np.round(np.asarray(x, dtype=float), 9).tobytes()

    def result(self, x: np.ndarray) -> EvalResult:
        key = self._key(x)
        hit = self._cache.get(key)
        if hit is not None:
            self._cache.move_to_end(key)
            self.n_hit += 1
            return hit

        lam, eta, phi = self.param.gains(x)
        law = SmcLaw(lam=lam, eta=eta, phi=phi, alpha=self.alpha, use_sat=True)
        try:
            res = simulate(law, self.ref, self.plant, force=self.force,
                           friction=self.friction)
        except Exception:
            # Una ley absurda puede reventar la dinámica; para el optimizador
            # eso es simplemente un punto infactible, no un fallo de la corrida.
            res = EvalResult(PENALTY, PENALTY, PENALTY, PENALTY, PENALTY,
                             PENALTY, PENALTY, PENALTY, True, 0, PENALTY)
        if res.diverged:
            self.n_diverged += 1

        self.n_eval += 1
        self._cache[key] = res
        if len(self._cache) > self._cache_size:
            self._cache.popitem(last=False)
        return res

    def objectives(self, x: np.ndarray) -> np.ndarray:
        r = self.result(x)
        if r.diverged:
            return np.full(3, PENALTY)
        return np.array([r.f1_iae, r.f2_effort, r.f3_chatter])

    N_CON = 4

    def constraints(self, x: np.ndarray) -> np.ndarray:
        """[g1, g2, g3, g4] ≤ 0 factible."""
        r = self.result(x)
        if r.diverged:
            return np.full(self.N_CON, PENALTY)
        # g3 NORMALIZADO por el umbral de CADA junta: lo que se acota es la
        # fraccion del umbral en la que se opera, no un chi absoluto que
        # significaria cosas distintas en el hombro y en la muñeca.
        chi_rel = float(np.max(np.asarray(r.chi_joint) / CHI_THRESHOLD))
        return np.array([r.g1_tau, r.g2_dq, chi_rel - self.chi_safety,
                         r.rmse_tcp_mm - self.tcp_tol_mm])

    def evaluate(self, x: np.ndarray) -> tuple:
        return self.objectives(x), self.constraints(x)

    def is_feasible(self, x: np.ndarray) -> bool:
        return bool(np.all(self.constraints(x) <= 0.0))

    def stats(self) -> dict:
        return {"n_eval": self.n_eval, "n_cache_hit": self.n_hit,
                "n_diverged": self.n_diverged}


# ─────────────────────────────────────────────────────────────────────────────
# Problema pymoo
# ─────────────────────────────────────────────────────────────────────────────

class GainTuningProblem(ElementwiseProblem):
    """min [f1, f2, f3] s.a. [g1, g2, g3] ≤ 0 sobre la caja de la parametrización."""

    def __init__(self, evaluator: GainEvaluator):
        xl, xu = evaluator.param.bounds()
        super().__init__(n_var=evaluator.param.n_var, n_obj=3,
                         n_ieq_constr=GainEvaluator.N_CON, xl=xl, xu=xu)
        self.ev = evaluator

    def _evaluate(self, x, out, *args, **kwargs):
        F, G = self.ev.evaluate(x)
        out["F"] = F
        out["G"] = G


#: Fricción articular del UR5e REAL, medida (docs/02_friction_real.md §3.1).
#:
#: Son los valores FÍSICOS, que es lo que el mando debe vencer con la
#: compensación interna del driver APAGADA (G4 = 0.0). Al nivel `default` el
#: robot aporta parte y el residual es otro —mucho menor y muy desigual entre
#: juntas—, así que estos números solo valen si la campaña real corre a 0.0.
#: Ver `docs/02_friction_real.md` §4: las dos vías miden magnitudes distintas y
#: solo coinciden en el nivel 0.0.
#:
#: ACTUALIZADO 2026-08-20 con las `k` medidas por `tau_phys/cur`
#: (`02_friction_real.md` §8.3), que ya NO dependen del modelo. Las anteriores
#: venían del método de gravedad y llevaban un sesgo sistemático del ~6 %; en
#: `wrist_2` el error era del 37 %, y en `wrist_3` su `k` era directamente una
#: hipótesis. Cambios: −5/−6 % en las cuatro primeras juntas, −27 % en
#: `wrist_2` y `wrist_3`.
#:
#: INCERTIDUMBRE que hay que arrastrar: `F_v` depende del estado TÉRMICO. Medido
#: el 2026-08-20 sobre `shoulder_lift`, primera corrida del día contra la
#: séptima: 13.71 -> 12.44, un **9.3 %**, mientras `F_c` solo se movió 0.76 % y
#: `k` 0.05 %. Estos valores son los de RÉGIMEN (robot rodado), que es la
#: condición en la que ocurre la incisión.
FRICTION_REAL_G4_0 = JointFriction(
    f_v=np.array([13.64, 11.55, 13.74, 1.28, 1.35, 2.12]),
    f_c=np.array([6.85, 6.76, 7.46, 1.78, 1.96, 2.35]))


def make_evaluator(ref: Reference, plant: Plant, mode: str = "scalar",
                   alpha: float = 0.3, f_cut: float = 5.0,
                   chi_safety: float = CHI_SAFETY_DEFAULT,
                   tcp_tol_mm: float = TCP_TOL_MM_DEFAULT,
                   q_ref: np.ndarray | None = None,
                   friction: JointFriction | None = None) -> GainEvaluator:
    """Atajo: construye parametrización + evaluador con los valores del plan."""
    inertia = plant.inertia_diag(Q_INIT if q_ref is None else q_ref)
    param = SmcParameterization(inertia=inertia, mode=mode)
    force = CuttingForce(f_cut=f_cut) if f_cut > 0 else None
    return GainEvaluator(param, ref, plant, force=force, alpha=alpha,
                         chi_safety=chi_safety, tcp_tol_mm=tcp_tol_mm,
                         friction=friction)


#: Dispersión relativa de la identificación entre las TRES campañas reales
#: independientes (docs/02_friction_real.md §3.2). Es el error de los parámetros,
#: no una incertidumbre inventada.
FRICTION_REL_ERR_F_V = np.array([0.081, 0.050, 0.042, 0.052, 0.051, 0.044])
FRICTION_REL_ERR_F_C = np.array([0.023, 0.006, 0.044, 0.007, 0.005, 0.009])

#: `wrist_3` arrastra ADEMÁS la incertidumbre de su `k`, que no es identificable
#: y se declaró en el rango [9.8, 11.7] N·m/A: ±8.8 % sobre la media. Sus dos
#: coeficientes escalan linealmente con ella, así que el error relativo se suma
#: en cuadratura al de la campaña.
FRICTION_REL_ERR_K_W3 = 0.088


def friction_residual_bound(ref: Reference, friction: JointFriction,
                            dq_eps: float = 1e-3) -> np.ndarray:
    """
    Cota por junta del par de fricción que el feedforward NO cancela.

    Es lo que `η` tiene que cubrir de verdad, y sustituye a simular el stick-slip
    dentro del evaluador: una perturbación ACOTADA está bien condicionada, es lo
    que pide la teoría de SMC, y no depende de casar un integrador propio con un
    motor de física (ver `closed_loop.JointFriction`, que quedó sin validar).

    Dos regímenes, y el que manda no es el que uno esperaría:

    - **En movimiento**: el feedforward acierta salvo el error de los parámetros,
      `δf_v·|q̇| + δf_c`. Son décimas de N·m.
    - **Cerca de una inversión de sentido**: `f_c·tanh(q̇_d/ε)` tiende a cero, así
      que la compensación se apaga y queda la fricción estática **ENTERA**. Y no
      es un caso de borde: sobre la trayectoria de incisión cada junta invierte
      1–3 veces y pasa el 15–40 % del tiempo por debajo de `ε`.

    Domina el segundo, así que la cota es esencialmente `f_c`. Eso NO es un
    defecto de esta función: es el mismo hecho que docs/05_smc.md §7.4 midió en
    Gazebo —el feedforward tiende a `f_c` por debajo y nunca lo supera, así que
    el margen de arranque solo puede darlo el término conmutado—, aquí expresado
    de forma analítica y sin depender del simulador.
    """
    rel_v = FRICTION_REL_ERR_F_V.copy()
    rel_c = FRICTION_REL_ERR_F_C.copy()
    rel_v[5] = np.hypot(rel_v[5], FRICTION_REL_ERR_K_W3)
    rel_c[5] = np.hypot(rel_c[5], FRICTION_REL_ERR_K_W3)

    dq_max = np.abs(ref.dq).max(axis=0)
    en_marcha = rel_v * friction.f_v * dq_max + rel_c * friction.f_c
    # La fricción estática entera solo cuenta donde la junta tiene que ARRANCAR:
    # pasa por debajo de `dq_eps` y ADEMÁS se mueve en algún otro tramo. Una
    # junta que no se mueve en toda la trayectoria —`wrist_2` en la incisión
    # recta— no necesita romper nada: allí la fricción la ayuda a sostenerse, y
    # cargarle `f_c` de perturbación le pediría al optimizador una `η` para un
    # problema que no tiene.
    # El criterio primario es el CAMBIO DE SIGNO, no «hay una muestra pequeña»:
    # una junta puede cruzar el cero ENTRE muestras y entonces la segunda prueba
    # no lo ve. Con el trazo de incisión y dt = 2 ms pasa de verdad — una rampa
    # de −0.5 a 0.5 rad/s en 100 muestras no deja ninguna por debajo de 1e-3.
    sg = np.sign(ref.dq)
    invierte = np.array([
        bool(np.any(np.diff(sg[sg[:, j] != 0, j]) != 0)) for j in range(6)])
    lenta = (np.abs(ref.dq) < dq_eps).any(axis=0)
    arranca = (invierte | lenta) & (dq_max > dq_eps)
    return np.where(arranca, np.maximum(en_marcha, friction.f_c), en_marcha)


def disturbance_bound(plant: Plant, ref: Reference, force: CuttingForce,
                      sigma: float = 3.0, stride: int = 10) -> np.ndarray:
    """
    Cota por junta del par que induce la fuerza de corte: `max_t |Jᵀ w|`.

    Es la escala que de verdad tiene que cubrir `η`: con `|d_i| > K_i` la
    función `sat` satura, `ṡ` no cambia de signo y el modo deslizante no se
    establece. Sirve para SEMBRAR el optimizador con un punto físicamente
    razonable en vez de dejarle descubrir esa escala por muestreo aleatorio en
    13 dimensiones.
    """
    import pinocchio as pin
    w = np.zeros(6)
    w[:3] = (force.f_cut + sigma * force.noise_std) * force.direction
    d = np.zeros(6)
    for k in range(0, ref.n, stride):
        J = pin.computeFrameJacobian(plant.model, plant.data, ref.q[k],
                                     plant.tcp, pin.LOCAL_WORLD_ALIGNED)
        d = np.maximum(d, np.abs(J.T @ w))
    return d
