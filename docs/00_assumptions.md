# FASE 0 — Supuestos declarados (A1–A4)

Supuestos bajo los que se produce **toda** la evidencia experimental del
artículo. Se declaran aquí una sola vez, se hacen explícitos en el código
(parámetro o *hook*, nunca un número cosido) y **deben aparecer en la sección
de *Limitations* del paper**.

- **Fecha:** 2026-07-30
- **Plan de referencia:** `PLAN_INCISION_UR5e.md`, §1
- **Compuertas asociadas:** [`docs/00_prereqs.md`](00_prereqs.md)

> El plan dice *"Estos tres supuestos"* pero enumera cuatro (A1–A4). Aquí se
> tratan **los cuatro** como declarables en *Limitations*; A4 es además una
> decisión de diseño de la trayectoria, no solo una simplificación.

| ID | Supuesto | Estado | Dónde vive en el código |
|---|---|---|---|
| A1 | Masa del acople del bisturí despreciable | ✅ Vigente, con *hook* listo y testeado | `tool_mass` / `tool_com` / `tool_inertia` |
| A2 | Punta de la hoja ≡ `gripper_tcp` (0.141 m desde `tool0`) | ✅ Vigente, offset parametrizado | `tcp_offset_z` |
| A3 | Métricas cartesianas en `gripper_tcp` respecto a `base_link` | ✅ Vigente | `Ur5Dynamics::fk()`, columnas `x,y,z` del CSV |
| A4 | Orientación del TCP constante `rpy = [π, 0, −π/2]` | ✅ Vigente | `tcp_orientation_rpy` |

---

## A1 — Se desprecia la masa del acople del bisturí

**Enunciado.** El modelo dinámico usado por todas las leyes de control es el del
**brazo solo** (`ur5_kinematics/share/ur5e.urdf`, sin gripper Robotiq y sin
herramienta). La masa e inercia del acople impreso del bisturí y de la propia
hoja se desprecian frente a las de los eslabones del UR5e.

**Por qué es aceptable de partida.** El acople es una pieza impresa pequeña
montada en la muñeca; su contribución a `M(q)` y a `g(q)` es de segundo orden
frente a los ~20 N·m de gravedad que ya soportan `shoulder_lift` y `elbow` en la
pose de trabajo. Además, en Gazebo la planta también está sin herramienta, así
que **modelo y planta coinciden exactamente** en las FASES 1–8 y el supuesto no
introduce error ahí.

**Dónde sí importa.** En el **robot real** (FASE 9) la planta sí lleva el acople
y el modelo no: el residuo aparecerá como perturbación no modelada. Esto es
justamente lo que los controladores robustos (SMC, ASTSMC) deben absorber, así
que **hay que declararlo** para no atribuir a robustez lo que es masa no
modelada — el mismo argumento que motiva la identificación de fricción de la
FASE 2.

### Hook implementado (requisito del plan: *"dejar el hook listo desde ahora"*)

```yaml
# config/fl_control_params.yaml (y gravity_comp_params.yaml)
tool_mass: 0.0                        # [kg]        PENDIENTE de medir
tool_com: [0.0, 0.0, 0.0]             # [m] en el frame TCP
tool_inertia: [0.0, 0.0, 0.0,         # [kg·m²] respecto al CoM:
               0.0, 0.0, 0.0]         #   [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
```

- `ur5_dyn_control::ToolInertia` (en `ur5_dynamics.hpp`) y el 4.º argumento del
  constructor de `Ur5Dynamics`. Con `mass = 0` **el modelo no se toca**: el
  supuesto vigente se mantiene bit a bit.
- Con `mass > 0`, `Ur5Dynamics` ancla el cuerpo a `wrist_3_joint` en la
  colocación del TCP (`appendBodyToJoint`) y `M`, `nle` y `gravity` lo recogen
  automáticamente — no hay que tocar ninguna ley de control.
- El nodo emite un **`RCLCPP_WARN` explícito** cuando `tool_mass > 0`
  (*"A1 LEVANTADO"*): levantar el supuesto no puede pasar en silencio.

**Tests** (`test/test_tool_inertia_hook.cpp`, 4/4 en verde):

| Test | Qué garantiza |
|---|---|
| `ZeroMassLeavesModelUntouched` | Con `mass = 0`, `g(q)` y `M(q)` son idénticos al modelo sin herramienta (< 1e-12) |
| `PointMassMatchesJacobianDerivedGravity` | Con masa puntual `m` en el TCP, `Δg(q) = m·g·Jᵥ,z(q)ᵀ` exactamente (< 1e-9): la masa queda anclada en el frame correcto |
| `PointMassIncreasesInertia` | `tr(M)` aumenta: guarda contra un signo invertido |
| `IsHonouredByForwardKinematics` | (A2) El offset del TCP es efectivo en la FK |

### 🔲 PENDIENTE para levantar A1

No se inventan valores (§5 del plan). Antes de la FASE 9 conviene obtener:

```
Masa del acople + hoja:        ________ kg     (báscula de precisión o CAD)
CoM respecto al gripper_tcp:   [__, __, __] m  (CAD)
Inercia en el CoM:             [Ixx,Iyy,Izz,Ixy,Ixz,Iyz]  (CAD)
```

Basta rellenar el YAML: no hay cambios de código pendientes. Si se levanta A1,
hay que **rehacer la identificación de fricción de la FASE 2**, porque el
residuo `tau_cmd − RNEA(q,q̇,q̈)` cambia.

---

## A2 — La punta de la hoja coincide con `gripper_tcp`

**Enunciado.** El punto controlado es el frame `gripper_tcp`, situado a
**0.141 m** de `tool0` a lo largo del eje Z local. Se asume que la punta de la
hoja del bisturí está en ese punto.

**Origen del valor.** Es la misma cadena TCP que define
`ur5_pick_place/urdf/ur5_robotiq_2f85.urdf.xacro` para el gripper Robotiq, y se
conserva para que las trayectorias y la cinemática sean comparables entre el
trabajo previo (pick-and-place, CU3) y este.

**Regla de código (exigida por el plan): no hardcodear 0.141 en ningún sitio
nuevo.** Cumplida y además reforzada:

- El nodo declara el parámetro **`tcp_offset_z`** (default `0.141`).
- `Ur5Dynamics` lo guarda y lo expone con `tcpOffsetZ()`.
- `JointReferenceGenerator` **ya no repite el número**: registra el frame de la
  IK con `dyn_->tcpFrameName()` y `dyn_->tcpOffsetZ()`. Antes tenía un `0.141`
  literal duplicado (`joint_reference_generator.cpp:38`), de modo que un cambio
  del offset habría hecho **divergir la IK de la dinámica en silencio**. Ahora
  hay una sola fuente de verdad.
- Los ficheros URDF/xacro de simulación (`ur5e_effort.urdf.xacro`) siguen
  llevando el valor porque definen el frame para Gazebo/TF; si se cambia el
  offset hay que cambiarlo **en el xacro y en el YAML a la vez**. Es el único
  acoplamiento restante y queda anotado aquí.

### 🔲 PENDIENTE al montar el bisturí

```
Distancia real tool0 -> punta de la hoja: ________ m   (calibrador)
Desviación respecto a 0.141 m:            ________ m
```

Si la desviación es apreciable frente a la precisión de corte que se quiere
reportar, se actualiza `tcp_offset_z` (y el xacro). Si no, se declara el sesgo
en *Limitations* junto con la repetibilidad nominal del UR5e (±0.03 mm).

---

## A3 — Todas las métricas cartesianas: `gripper_tcp` respecto a `base_link`

**Enunciado.** Posición, error de posición, velocidad de avance y profundidad de
corte se expresan como la pose del frame `gripper_tcp` en el frame `base_link`
del robot. Ningún resultado se reporta en el frame del mundo de Gazebo ni en el
frame `base` de UR.

**Consecuencias operativas:**

- En Gazebo el robot está montado a **z = 0.63 m** sobre la mesa
  (`ur5e_effort_gz.launch.py`). Por tanto `z_Gazebo = z_base_link + 0.63`. Los
  waypoints del YAML están en `base_link`; ninguna figura del paper debe usar la
  coordenada de Gazebo.
- ⚠️ **Ojo con `base_link` vs `base`:** UR define `base_link` (convención ROS,
  Z arriba) y `base` (convención UR, rotada π alrededor de Z). El sensor F/T se
  publica en `tool0_controller` (ver `docs/00_prereqs.md`, G5) y
  `tcp_pose_broadcaster` usa `base`. **Toda comparación entre `ft_data` y
  cantidades del CSV requiere una transformación explícita**, que hay que
  implementar y verificar en la FASE 3 (suscripción a `ft_data` con timestamp
  alineado), no darla por hecha.
- El error de orientación se reportará como `theta_err = ‖log(R_dᵀ R)‖`
  (FASE 10), **nunca** con ángulos de Euler, para evitar el *wrap*.

**Dónde vive:** `Ur5Dynamics::fk(q)` devuelve `oMf[gripper_tcp]`, que Pinocchio
expresa en el frame raíz del modelo. En `ur5_kinematics/share/ur5e.urdf` la raíz
es `world`, unida a `base_link` por `base_joint` con `origin rpy="0 0 0"
xyz="0 0 0"` — **identidad**, así que el frame raíz de Pinocchio y `base_link`
coinciden exactamente y A3 se cumple sin transformación adicional. Las columnas
`x, y, z` y `x_des, y_des, z_des` del CSV son exactamente eso.

*(Verificado: `base` está definido como `base_link` rotado π alrededor de Z
—`base_link-base_fixed_joint`— precisamente porque `base_link` es REP-103 y los
frames internos del controlador de UR no lo son. De ahí la advertencia de
arriba.)*

---

## A4 — Orientación del TCP constante `rpy = [π, 0, −π/2]`

**Enunciado.** Durante toda la incisión el TCP mantiene orientación fija, con la
herramienta apuntando hacia abajo (−Z del mundo). Por tanto
`ω_des = ω̇_des = 0` en toda la trayectoria.

**Por qué.** Es una incisión recta sobre una superficie horizontal: la
orientación no aporta grados de libertad útiles y fijarla (i) simplifica la
generación de referencias — el spline solo interpola posición y la IK resuelve
con orientación constante — y (ii) elimina el acoplamiento posición/orientación
de las métricas de precisión de corte.

**Dónde vive:** parámetro `tcp_orientation_rpy` en los YAML;
`CartesianSplineTrajectory::orientation()` devuelve la `R` constante y
`JointReferenceGenerator` pone a cero el bloque angular de `ẋ` y `ẍ`.

**Limitación que genera (para el paper):** el trabajo **no** demuestra
seguimiento de orientación bajo torque. Cualquier afirmación sobre control de
orientación queda fuera del alcance. Además, con orientación fija los 6 DOF se
consumen en 3 de posición + 3 de orientación, así que **no hay redundancia
disponible** para evitar singularidades: de ahí que la FASE 1 exija el chequeo
de `sigma_min(J)` y `w = sqrt(det(J·Jᵀ))` a lo largo del trazo.

---

## Texto propuesto para *Limitations* del paper

> The dynamic model used by all controllers is that of the bare UR5e arm: the
> mass and inertia of the 3D-printed scalpel coupler and blade are neglected
> (A1). The blade tip is assumed to coincide with the tool centre point located
> 0.141 m from `tool0` along its Z axis (A2); no independent calibration of the
> tip offset was performed. All Cartesian quantities are reported for that TCP
> frame expressed in the robot `base_link` frame (A3). The TCP orientation is
> held constant at `rpy = [π, 0, −π/2]` throughout the incision (A4), so the
> results characterise position tracking under torque control and do not
> demonstrate orientation tracking. In the Gazebo campaign the plant is likewise
> modelled without a tool, so A1 introduces no model–plant mismatch there; on
> the physical robot it contributes an unmodelled disturbance that is absorbed
> by the controllers and must not be interpreted as robustness to tissue
> properties.

*(Redacción preliminar; se ajusta al cerrar la FASE 10.)*

---

## Trazabilidad

| Supuesto | Ficheros |
|---|---|
| A1 | `ur5_dyn_control/include/ur5_dyn_control/ur5_dynamics.hpp` (`ToolInertia`), `src/ur5_dynamics.cpp`, `src/torque_control_node_base.cpp`, `test/test_tool_inertia_hook.cpp`, `config/*_params.yaml` |
| A2 | ídem + `src/joint_reference_generator.cpp`, `urdf/ur5e_effort.urdf.xacro` |
| A3 | `src/ur5_dynamics.cpp` (`fk`), `include/ur5_dyn_control/csv_logger.hpp` |
| A4 | `include/ur5_dyn_control/cartesian_spline_trajectory.hpp`, `src/joint_reference_generator.cpp`, `config/*_params.yaml` |
