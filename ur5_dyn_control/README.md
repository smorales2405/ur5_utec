# ur5_dyn_control

Control **basado en torque** (interfaz `effort`) del UR5e en Gazebo Fortress
(ROS 2 Humble), con miras a su implementación en el robot real vía
`ur_robot_driver`. Controladores implementados: **Feedback Linearization**
(computed torque), **Sliding Mode** (`sgn` / `sat`) y **LQR-SDRE**; la
arquitectura permite añadir ASTSMC, etc. como nodos delgados que solo
implementan su ley de control.

## Arquitectura

```
CartesianTrajectory (interfaz)   p(t), ṗ(t), p̈(t), p⃛(t) analíticas;
        │                        orientación TCP constante (A4)
        ├── CartesianSplineTrajectory   cúbico clamped — jerk DISCONTINUO
        ├── QuinticSplineTrajectory     quíntico C⁴ — jerk CONTINUO
        └── IncisionTrajectory          5 fases, geometría ∘ arco ∘ S-curve
        │
JointReferenceTable  (base)  tabla {q,dq,ddq} + diagnósticos + phaseLabel(k)
   ┌────┴──────────────────┐
JointReferenceGenerator     JointSweepGenerator
IK QP (ur5_kinematics) +    barrido de excitación por junta (FASE 2),
refinamiento de Newton +    SIN IK: se escribe la referencia articular
dq = J⁻¹ẋ, ddq = J⁻¹(ẍ−J̇q̇)  directamente
Aborta si: salto de rama IK · σ_min(J) bajo umbral · dq/ddq fuera de límites
        │
TorqueControlNodeBase       máquina de estados PRE_HOLD → WAIT → HOLD_START →
        │                   RAMP (quíntica) → TRACK → HOLD_END → (SAFE_HOLD)
        │                   CSV · G3 · saturación · límite de tasa · watchdog
   ┌────┼─────────────┬────────────────┬───────────────┐  τ ∈ ±[150,…,28] N·m
gz_gravity_comp  gz_fl_control  gz_smc_control  gz_lqr_sdre_control  (futuro: astsmc)
τ=g+Kp e−Kd dq   τ = M(q̈d+Kp e   τ = b̂ + M̂ q̈_r    τ = M q̈_d + C q̇_d + g
                 +Kd ė)+C q̇+g       − K⊙ρ(s)          − K x_e,  x_e = [q_e; q̇_e]
                                  s = ė + Λe          K = R⁻¹BᵀP,  CARE por
                                  ρ = sgn | sat(s/φ)  función signo (SDRE)
```

### Trayectorias

`trajectory_type` selecciona cuál se usa:

| Valor | Clase | Uso |
|---|---|---|
| `cubic_spline` | `CartesianSplineTrajectory` | Histórico. Jerk constante a trozos y discontinuo en los nudos; aceleración no nula en los extremos. Se conserva para la comparación cúbico↔quíntico del paper. |
| `quintic_spline` | `QuinticSplineTrajectory` | Spline quíntico C⁴ sobre `waypoints_xyz`/`waypoint_times`. `v=a=0` en los extremos, jerk continuo. Sustituto directo del cúbico. |
| `incision` | `IncisionTrajectory` | Las 5 fases de la incisión (`config/incision_params.yaml`). |
| `joint_sweep` | `JointSweepGenerator` | Barrido de excitación de UNA junta a varias velocidades, con las demás fijas (FASE 2, identificación de fricción). No pasa por IK: la referencia es articular. `config/sweep_params.yaml`. |

**Separación geometría / temporización** (`IncisionTrajectory`):

```
geometría   QuinticSpline3d p(u), u ∈ [0,1]      (CHORD_TANGENT: |p'| > 0)
   ∘        ArcLength: s(u) por Gauss-Legendre → u(s)   [port de numerical_integration.py del CU3]
   ∘        ScurveProfile: s(t) con jerk acotado y MESETA de velocidad constante
   =        p(t) = p(u(s(t)))   ⟹   |ṗ| = ṡ  (feed exacto en la meseta)
```

Una fase de dos waypoints es **exactamente** un segmento recto (el Hermite
quíntico con tangente de cuerda degenera en la interpolación lineal), así que
`contact`, `penetration`, `cut` y `withdraw` son rectas por construcción y la
profundidad se mantiene constante durante el corte.

> ⚠️ `QuinticBoundary::CLAMPED_REST` (`v=a=0`) es correcto para una trayectoria
> parametrizada en **tiempo**, pero hace `|p'| = 0` en los extremos y por tanto
> **no sirve como geometría** para el arco (`du/ds = 1/|p'|` divergería).
> `ArcLength` lo detecta y lanza excepción en vez de propagar `1/0`.

- **Modelo dinámico**: `ur5_kinematics/share/ur5e.urdf` (brazo solo, sin masa
  de gripper — coincide con la planta de Gazebo sin Robotiq), gravedad 9.8
  (la del mundo), frame `gripper_tcp` a 0.141 m de `tool0`.
- **Comandos**: `Float64MultiArray` en `/forward_effort_controller/commands`
  (`effort_controllers/JointGroupEffortController`) — mismo contrato que el
  `forward_effort_controller` del driver real de UR → los nodos son
  portables sim ↔ real.
- **Reloj**: lazo de PARED a `control_rate` (500 Hz); fases y trayectoria
  indexadas por tiempo de SIMULACIÓN (robusto al RTF bajo del mundo del
  laboratorio, ~0.07 por las colisiones trimesh de las mesas).

## Arranque sin caída (evaluación realizada)

Se evaluó el esquema "controlador de posición activo al inicio → switch a
effort al ejecutar el nodo". **Resultado: NO es viable en simulación** con
`gz_ros2_control` 0.7.19: al declarar la command interface `effort` en el
URDF, `initSim` (gz_system.cpp:476) activa el modo effort del joint desde el
arranque y los comandos de posición dejan de aplicarse a la física
(verificado empíricamente; además `perform_command_mode_switch` tiene un bug
de máscaras: usa `&` en vez de `|`). En el robot real el switch
JTC ↔ `forward_effort_controller` SÍ está soportado por el driver.

Esquema adoptado (validado, el robot **nunca cae**):

1. Gazebo arranca **PAUSADO**; el robot spawnea en la pose inicial
   (`initial_value`): `[π/2, −π/2, π/2, −π/2, −π/2, 0]` — la muñeca queda
   apuntando hacia +y; es la pose en la que hay que colocar el UR5e **real**
   para que coincida con la escena de Gazebo.
2. Los spawners cargan+configuran (`--inactive`) `joint_state_broadcaster` y
   `forward_effort_controller` (activar en pausa no es posible: el
   controller_manager solo actualiza en cada paso de física).
3. El **nodo de control** construye la tabla de referencias (IK offline, sin
   prisa), publica `g(q_init)` a ciegas, pide el switch de activación
   (asíncrono, queda pendiente) y **despausa**: la activación se consuma en
   el primer paso de física con el torque de sostén ya disponible.

## Uso

```bash
# Smoke test: compensación de gravedad (regulación en q_init)
ros2 launch ur5_dyn_control gravity_comp.launch.py gazebo_gui:=false

# Feedback Linearization siguiendo la trayectoria cartesiana spline
ros2 launch ur5_dyn_control fl_control.launch.py test_num:=1

# Incisión de 5 fases (corte de 80 mm a 10 mm/s, profundidad 5 mm)
ros2 launch ur5_dyn_control fl_control.launch.py test_num:=1 \
    params_file:=$(ros2 pkg prefix ur5_dyn_control)/share/ur5_dyn_control/config/incision_params.yaml

# Desarrollo rápido (mundo sin mesas, RTF ~1):
ros2 launch ur5_dyn_control fl_control.launch.py \
    world:=$(ros2 pkg prefix ur5_dyn_control)/share/ur5_dyn_control/worlds/empty_test_world.sdf

# Bringup solo (robot congelado en q_init hasta despausar; sin nodo el brazo
# cae al despausar porque la única interfaz es effort):
ros2 launch ur5_dyn_control ur5e_effort_gz.launch.py auto_start:=false

# SMC sobre la incisión; sign vs sat sin duplicar YAMLs
ros2 launch ur5_dyn_control smc_control.launch.py test_num:=510 switching_function:=sat
ros2 launch ur5_dyn_control smc_control.launch.py test_num:=511 switching_function:=sign

# SMC con las ganancias que produce la FASE 7 (sin editarlas a mano)
ros2 launch ur5_dyn_control smc_control.launch.py test_num:=704 \
    gains_file:=<...>/ur5_trajectory_optimization/results/gain_tuning/smc/test3/selected_gains.yaml

# Ensayo de tiempo de alcance: error inicial deliberado (FASE 5 §6)
ros2 launch ur5_dyn_control smc_control.launch.py test_num:=540 \
    switching_function:=sign initial_offset:="0.05 0.05 0.05 0.05 0.05 0.05"

# LQR-SDRE sobre la incisión (FASE 4). Escribe además lqr_diag_<n>.csv
ros2 launch ur5_dyn_control lqr_sdre_control.launch.py test_num:=10

# Barrido de diseño sin duplicar YAMLs; Q_mode=scheduled resintetiza Q(q)
ros2 launch ur5_dyn_control lqr_sdre_control.launch.py test_num:=11 \
    wn:=35 zeta:=1.0 Q_mode:=scheduled

# Estudio de decimación: la CARE se resuelve a 50 Hz y K se mantiene con ZOH
ros2 launch ur5_dyn_control lqr_sdre_control.launch.py test_num:=12 \
    care_update_rate:=50
```

Análisis: `ur5_identification/scripts/analyze_lqr.py --test 10 --plot fig.png`
evalúa los tres criterios de aceptación de la FASE 4 (estabilidad del esquema
congelado en el 100 % de los pasos, tiempo de cómputo bajo presupuesto,
seguimiento sin saturación sostenida).

Parámetros (waypoints, tiempos, ganancias, tasas, IK, CSV):
`config/fl_control_params.yaml`, `config/gravity_comp_params.yaml`,
`config/smc_params.yaml`, `config/incision_params.yaml`, `config/sweep_params.yaml`.
CSV de resultados: `~/.ros/ur5_dyn_control/<prefijo>_<test_num>.csv`, con
cabecera de trazabilidad (`#` con SHA de git y hash FNV-1a de los parámetros
efectivos) y esquema unificado `t, q, dq, q_des, dq_des, ddq_des, tau,
tau_sat, s, xyz, xyz_des, wrench, state`.

> ⚠️ `numpy.genfromtxt(names=True)` **no** salta las líneas `#`: toma la primera
> del fichero como cabecera de columnas. Hay que contarlas y pasar
> `skip_header`. Ha mordido dos veces en este proyecto; está documentado en
> `csv_logger.hpp` y cubierto por un test de regresión.

### Parámetros añadidos por las fases posteriores

| Parámetro | Fase | Para qué |
|---|---|---|
| `gravity_in_command` | 0 (G3) | `false` en el robot real: `direct_torque` ya compensa `g(q)`. |
| `friction_compensation` / `friction.f_v` / `friction.f_c` | 2 | Feedforward de fricción identificada (`none`\|`viscous`\|`viscous_coulomb`). |
| `tau_rate_max`, `watchdog.*` | 3 | Límite de tasa del comando y vigilancia del lazo → `SAFE_HOLD`. |
| `initial_offset[6]` | 5 | Error inicial **deliberado** para cronometrar el alcance. Desplaza el destino de la rampa, así que TRACK arranca en reposo con `s(0) = Λ·offset` conocido. Default cero. |
| `reference_table_out` | 7 | Vuelca la tabla `{q,dq,ddq}` a CSV para el evaluador offline de sintonía. La referencia **no se reimplementa** en Python: optimizador y robot comparten la misma tabla. |
| `lqr.wn`, `lqr.zeta`, `lqr.r`, `lqr.Q_mode` | 4 | Diseño del LQR-SDRE. `Q` **no** se da en bruto: se sintetiza `Qp = r·wn⁴·M²`, `Qv = r·wn²(4ζ²−2)·M²`. `ζ ≥ 1/√2` es obligatorio (con menos, `Qv ≺ 0`). |
| `lqr.care_update_rate`, `lqr.*_tol`, `lqr.max_consecutive_failures` | 4 | Decimación (ZOH sobre `K`), aceptación de la solución de la CARE y política de `SAFE_HOLD`. |

## Mundos

- **`lab_incision_world.sdf`** — **el default de los cuatro launches**. Mesa
  base + surgery table, física a **1 kHz** (paso 1 ms), **sin** el obstáculo del
  caso de uso de pick & place.
- `lab_torque_world.sdf` — el anterior, que al copiarse de `lab_base_world.sdf`
  heredó ese obstáculo (caja AABB en (0.85, 0, 0.73)). Se conserva **intacto**
  para reproducir campañas anteriores:
  `world:=$(ros2 pkg prefix --share ur5_dyn_control)/worlds/lab_torque_world.sdf`
- `empty_test_world.sdf` — solo ground plane, física 1 kHz, RTF ~1
  (desarrollo y tuning).

Por qué se retiró el obstáculo: ningún código de control por torque lo
referencia, y la holgura mínima medida sobre la trayectoria de incisión (todos
los orígenes de junta y el TCP, tabla completa) es de **0.150 m**, así que nada
de lo ya registrado está afectado. Se quita mirando a la FASE 8, donde el
desajuste de modelo de ±50 % y los 4 kg en el TCP desvían el brazo mucho más: una
colisión contra un objeto ajeno al experimento no dejaría rastro evidente en el
CSV.

El default vive en **un solo sitio**, `launch/world_defaults.py`, del que
importan los cuatro launches. Que dos controladores acabasen corriendo en
escenas distintas invalidaría la comparación del paper sin dar ningún síntoma.

## Añadir un controlador nuevo (ASTSMC, ...)

Subclasear `TorqueControlNodeBase`, implementar `computeTau(q, dq, ref, dt)`
y `csvPrefix()`, declarar las ganancias propias y llamar `start()` al final
del constructor (ver `src/gz_fl_control_node.cpp`, ~60 líneas; el SMC completo
son ~160 con toda la formulación documentada). El acceso a la dinámica es
`dyn()` (M, nle, gravity, `coriolis`, `dM = C + Cᵀ`, J, J̇q̇, FK).

Ganchos opcionales:

- `slidingVariable()` — vuelca `s` a la columna del CSV unificado.
- `onSaturation()` — congela integradores cuando una junta queda recortada
  (anti-windup; lo necesita el super-twisting del ASTSMC).
- `requestSafeHold(reason)` — parada segura pedida por la **ley**, no por el
  watchdog. El par de ese ciclo **no** se publica. La usa el LQR-SDRE cuando el
  par `(A,B)` pierde la certificación de estabilizabilidad o la CARE falla
  repetidamente.
- `DiagLogger` + `traceMetadata()`, `csvDir()`, `testNum()` — CSV de diagnóstico
  **aparte** con columnas propias de la ley, con la misma cabecera de
  trazabilidad. El esquema del CSV unificado es común a los cuatro
  controladores y no debe bifurcarse: lo que solo existe en una ley va aquí.
- `solveCare(A, B, Q, R)` (`care_solver.hpp`) — CARE de tiempo continuo por
  función signo del hamiltoniano, con balanceado simpléctico. **Medido: 57 µs**
  para 12 estados y 6 entradas.

> **Escala por inercia siempre.** Este robot tiene cuatro órdenes de magnitud de
> dispersión en inercia articular (2.59 kg·m² en `shoulder_lift` frente a
> 2.6e-4 en `wrist_3`). Cualquier ganancia uniforme rompe en las muñecas: ya
> pasó en `gravity_comp`, en el `η` del SMC, en el `a_reach` del alcance y en la
> `Q` del LQR-SDRE. La regla práctica es referir la ganancia a `M_ii` y vigilar
> el número de estabilidad discreta correspondiente (`kd·dt/I` para un PD,
> `(K/φ)·dt/M` para la capa límite del SMC, `max|λ|·dt` para el LQR): por encima
> de ~1 hay ciclo límite.
>
> **Y ojo: `M_ii` no siempre basta.** `M(q_init)` está lejos de ser diagonal —
> `M(1,5) = 2.8e-2` frente a `M(5,5) = 5.4e-3`, un acoplo **cinco veces** la
> propia diagonal. En el LQR-SDRE, sintetizar `Q` con `diag(M)²` deja los polos
> entre 8.4 y 290 rad/s (34× de dispersión, `max|λ|·dt = 0.58`); con `M²`
> completa, los doce caen exactamente en `−wn`. Ver `docs/04_lqr_sdre.md` §2.2.

## Hacia el robot real (compuertas en `docs/00_prereqs.md`)

Instalar `ros-humble-ur` (metapaquete; en Humble la versión **2.13.2** ya trae
`forward_effort_controller`, `friction_model_controller` y
`force_torque_sensor_broadcaster` — verificado, ver G2). El mismo nodo corre
con: `use_sim_time:=false`, `perform_unpause:=false`,
**`gravity_in_command:=false`**, `activate_controllers:=[forward_effort_controller]`,
`deactivate_controllers:=[scaled_joint_trajectory_controller]` (mutuamente
excluyentes en el driver). Acompañar con `friction_model_controller`,
**fijando sus escalas explícitamente por servicio** (si no se llaman, el robot
usa defaults no documentados y la campaña no es reproducible — ver G4).

> `gravity_update_controller` **no existe en Humble 2.13.2** (solo desde Jazzy).
> Solo hace falta con montaje en orientación no estándar; el UR5e de este
> laboratorio está sobre mesa en orientación estándar, así que no bloquea nada.

### ⛔ Gravedad en el comando (compuerta G3)

`computeTau()` devuelve siempre el torque **físico completo** (con gravedad).
Lo que se comanda depende del parámetro `gravity_in_command`:

| | `gravity_in_command` | Comando |
|---|---|---|
| Gazebo | `true` (default) | `tau_cmd = tau_ley` |
| UR5e real | **`false`** | `tau_cmd = tau_ley − g(q)` |

En el robot real, `direct_torque(...)` compensa la gravedad **dentro** del
robot: comandarla otra vez la duplica y el brazo se acelera hacia arriba. La
regla vive en `torque_command.hpp` (funciones puras) y está cubierta por
`test/test_gravity_policy.cpp`. **Sin ese test en verde, prohibido tocar el
robot.**

## Tests

```bash
colcon build --packages-select ur5_dyn_control
colcon test --packages-select ur5_dyn_control --event-handlers console_direct+
```

- `test_gravity_policy` (14) — compuerta G3, saturación y feedforward de fricción.
- `test_tool_inertia_hook` (4) — supuestos A1 (herramienta en el TCP) y A2
  (offset del TCP configurable).
- `test_quintic_spline` (9) — derivadas analíticas vs diferencias finitas
  (< 1e-6), continuidad de jerk en los nudos, y los mismos tests sobre el
  cúbico demostrando que **falla** (evidencia de la figura cúbico↔quíntico).
- `test_incision_trajectory` (23) — Gauss-Legendre, longitud de arco, perfil
  S-curve, feed constante en el corte y geometría de las 5 fases.
- `test_limits_and_traceability` (13) — límite de tasa del comando, marcas de
  saturación, hash de trazabilidad y compatibilidad de nombres de columna del CSV.
- `test_care_solver` (9) — solver CARE contra la solución analítica del doble
  integrador, el caso **defectivo** (`ζ = 1`, polo doble) que rompe el método de
  autovectores, MIMO aleatorio, estabilizabilidad del par SDC del UR5e y la
  regresión del escalado por inercia de `Q`.

Total: **72 tests** en 6 ejecutables de `ur5_dyn_control`, + 19 pytest en
`ur5_identification` y 18 en `ur5_trajectory_optimization` (**109** en el
workspace).

## Seguridad del lazo (FASE 3)

`watchdog.*` vigila el ritmo del lazo (`dt` de simulación > k× el nominal) y la
llegada de `/joint_states`. Al dispararse entra en **`SAFE_HOLD`**, un estado
terminal que sostiene la última pose conocida con un PD sobre la gravedad —
deliberadamente **sin** llamar a `computeTau()`: si el lazo dejó de ser fiable,
la ley del controlador es justo lo que no hay que seguir ejecutando. El timeout
de `/joint_states` se mide con reloj de **pared**, porque si la fuente de estado
muere el reloj de simulación puede congelarse y un timeout en tiempo de sim no
se dispararía nunca.

`tau_rate_max` acota `|Δτ/Δt|` por junta, y `onSaturation()` avisa a las
subclases de qué juntas quedaron recortadas para que congelen sus integradores
(anti-windup; lo necesita el super-twisting del ASTSMC).

### Ganancias escaladas por inercia

Las ganancias PD de `gravity_comp` son `kp_j = I_jj·ω_n²`, `kd_j = 2ζ·I_jj·ω_n`
con `ω_n = 20 rad/s`, `ζ = 1`. Con ganancias uniformes por bloques el ancho de
banda iba de 5 a 88 rad/s y en `wrist_3` (I = 2.6e-4 kg·m², 10 000× menor que el
hombro) el número de estabilidad discreta `kd·dt/I` valía **1.55**: con el
retardo de una muestra del lazo, ciclo límite con la velocidad saturando en
±π rad/s. Escalado por inercia vale **0.080 en las seis juntas**.

## Documentación del proyecto

- [`docs/00_prereqs.md`](../docs/00_prereqs.md) — compuertas G1–G6 con evidencia
  y acciones pendientes del operador.
- [`docs/00_assumptions.md`](../docs/00_assumptions.md) — supuestos A1–A4 y su
  trazabilidad al código.
- [`docs/01_trajectory.md`](../docs/01_trajectory.md) — trayectoria de incisión:
  geometría ∘ arco ∘ S-curve, y la ventana de meseta donde se miden las métricas.
- [`docs/02_friction.md`](../docs/02_friction.md) — identificación de fricción y
  el control negativo: en una planta sin fricción, las juntas cargadas por
  gravedad muestran fricción viscosa **aparente** por el desfase de ~1 ms.
- [`docs/04_lqr_sdre.md`](../docs/04_lqr_sdre.md) — LQR-SDRE: la discrepancia
  del plan entre `τ` y `A` (resuelta a favor de `A`), la síntesis de `Q ∝ M²` y
  por qué la CARE se resuelve por función signo y no por autovectores (con
  `ζ = 1` el lazo cerrado es **defectivo**).
- [`docs/05_smc.md`](../docs/05_smc.md) — `sgn` vs `sat`, barridos de φ y α (el
  compromiso clásico está **invertido** bajo el umbral discreto) y el ensayo de
  tiempo de alcance.
- [`docs/07_gain_tuning.md`](../docs/07_gain_tuning.md) — sintonía multiobjetivo
  (NSGA-II + ε-restricción) y por qué el retardo de tubería no era opcional: sin
  modelarlo, las ganancias «óptimas» daban 29.7 mm de error en Gazebo frente a
  los 0.02 mm predichos.
