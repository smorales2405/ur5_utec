# ur5_dyn_control

Control **basado en torque** (interfaz `effort`) del UR5e en Gazebo Fortress
(ROS 2 Humble), con miras a su implementación en el robot real vía
`ur_robot_driver`. Primer controlador: **Feedback Linearization** (computed
torque); la arquitectura permite añadir Sliding Mode, MRAC, etc. como nodos
delgados que solo implementan su ley de control.

## Arquitectura

```
CartesianTrajectory (interfaz)   p(t), ṗ(t), p̈(t), p⃛(t) analíticas;
        │                        orientación TCP constante (A4)
        ├── CartesianSplineTrajectory   cúbico clamped — jerk DISCONTINUO
        ├── QuinticSplineTrajectory     quíntico C⁴ — jerk CONTINUO
        └── IncisionTrajectory          5 fases, geometría ∘ arco ∘ S-curve
        │
JointReferenceGenerator     IK QP (ur5_kinematics) + refinamiento de Newton +
                            dq = J⁻¹ẋ, ddq = J⁻¹(ẍ − J̇q̇)  →  tabla {q,dq,ddq}
        │                   Aborta si: salto de rama IK · σ_min(J) bajo umbral ·
        │                   dq/ddq fuera de límites del UR5e
TorqueControlNodeBase       máquina de estados PRE_HOLD → WAIT → HOLD_START →
        │                   RAMP (quíntica) → TRACK → HOLD_END; CSV; G3; saturación
   ┌────┴─────────┐         τ ∈ ±[150,150,150,28,28,28] N·m
gz_gravity_comp   gz_fl_control_node        (futuros: gz_lqr_sdre, gz_smc, gz_astsmc)
τ=g+Kp e−Kd dq    τ = M(q)(q̈d+Kp e+Kd ė) + C q̇ + g      [Pinocchio: crba+nle]
```

### Trayectorias

`trajectory_type` selecciona cuál se usa:

| Valor | Clase | Uso |
|---|---|---|
| `cubic_spline` | `CartesianSplineTrajectory` | Histórico. Jerk constante a trozos y discontinuo en los nudos; aceleración no nula en los extremos. Se conserva para la comparación cúbico↔quíntico del paper. |
| `quintic_spline` | `QuinticSplineTrajectory` | Spline quíntico C⁴ sobre `waypoints_xyz`/`waypoint_times`. `v=a=0` en los extremos, jerk continuo. Sustituto directo del cúbico. |
| `incision` | `IncisionTrajectory` | Las 5 fases de la incisión (`config/incision_params.yaml`). |

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
   (`initial_value`): `[0, −π/2, π/2, −π/2, −π/2, 0]`.
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
```

Parámetros (waypoints, tiempos, ganancias, tasas, IK, CSV):
`config/fl_control_params.yaml` y `config/gravity_comp_params.yaml`.
CSV de resultados: `~/.ros/ur5_dyn_control/<prefijo>_<test_num>.csv`
(columnas: `t, q, dq, q_des, dq_des, ddq_des, tau, xyz, xyz_des, state`).

## Mundos

- `lab_torque_world.sdf` — contenido de `lab_base_world.sdf` (mesa base,
  surgery table, obstáculo) con física a **1 kHz** (paso 1 ms). RTF ~0.07
  en esta máquina por las colisiones malla-malla (igual que el pick-place);
  todo corre en tiempo de sim, así que solo alarga el tiempo de pared.
- `empty_test_world.sdf` — solo ground plane, física 1 kHz, RTF ~1
  (desarrollo y tuning).

## Añadir un controlador nuevo (SMC, MRAC, ...)

Subclasear `TorqueControlNodeBase`, implementar `computeTau(q, dq, ref, dt)`
y `csvPrefix()`, declarar las ganancias propias y llamar `start()` al final
del constructor (ver `src/gz_fl_control_node.cpp`, ~60 líneas). El acceso a
la dinámica es `dyn()` (M, nle, gravity, J, J̇q̇, FK).

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

- `test_gravity_policy` (8) — compuerta G3 y saturación.
- `test_tool_inertia_hook` (4) — supuestos A1 (herramienta en el TCP) y A2
  (offset del TCP configurable).
- `test_quintic_spline` (9) — derivadas analíticas vs diferencias finitas
  (< 1e-6), continuidad de jerk en los nudos, y los mismos tests sobre el
  cúbico demostrando que **falla** (evidencia de la figura cúbico↔quíntico).
- `test_incision_trajectory` (23) — Gauss-Legendre, longitud de arco, perfil
  S-curve, feed constante en el corte y geometría de las 5 fases.
- `test_limits_and_traceability` (13) — límite de tasa del comando, marcas de
  saturación, hash de trazabilidad y compatibilidad de nombres de columna del CSV.

Total: **68 tests** en `ur5_dyn_control` + 19 pytest en `ur5_identification`.

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
