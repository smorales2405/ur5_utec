# FASE 0 — Compuertas previas (G1–G6)

Documento de cierre de la FASE 0 del plan `PLAN_INCISION_UR5e.md`.
Ninguna fase posterior puede empezar sin cerrar estas compuertas.

- **Fecha de verificación:** 2026-07-30
- **Repositorio:** `smorales2405/ur5_utec`, rama `main`, commit base `1fc9134`
- **Máquina de verificación:** Ubuntu 22.04 (Jammy), kernel 6.8.0-124-generic, ROS 2 Humble

**Leyenda de estado**

| Símbolo | Significado |
|---|---|
| ✅ | Cerrada con evidencia verificable en este documento |
| ⚠️ | Cerrada con condiciones / decisión tomada que hay que respetar |
| 🔲 **PENDIENTE** | Requiere una acción del operador (hardware o sesión con el robot) |

**Resumen — TODAS LAS COMPUERTAS CERRADAS (2026-07-30)**

| Compuerta | Estado | Bloquea |
|---|---|---|
| G1 — PolyScope | ✅ **5.25.2** — cumple todos los umbrales | FASE 9 |
| G2 — Distro y driver | ✅ Ruta (a): `ros-humble-ur` **2.13.2** instalado y verificado | FASE 9 |
| G3 — Gravedad fuera del comando | ✅ Implementada y testeada (8/8) | FASE 9 (y §7) |
| G4 — Fricción interna | ✅ Ajuste de operación decidido: `1.0 / 1.0` (ver aviso) | FASES 2, 9 |
| G5 — Qué se puede medir | ✅ Verificado en el sistema instalado | FASES 2, 9, 10 |
| G6 — Seguridad (§7) | ⚠️ Firmado salvo el watchdog (entregable de la FASE 3) | FASE 9 |

**Habilitado:** FASES 1–8. La FASE 9 queda bloqueada solo por el watchdog, que
se entrega en la FASE 3.

### Identificación del robot

| Campo | Valor |
|---|---|
| Modelo | UR5e |
| PolyScope | **5.25.2** |
| Número de serie | **20245500119** |
| IP | **192.168.0.102** |

---

## G1 — Versión de PolyScope del UR5e físico

**Estado:** ✅ **CERRADA. PolyScope 5.25.2**, por encima de todos los umbrales.

| Capacidad | Umbral PS5 | 5.25.2 |
|---|---|---|
| `direct_torque(...)` / interfaz `effort` | ≥ 5.23.0 | ✅ |
| Escalas de fricción (`viscous_scale`, `coulomb_scale`) | ≥ 5.25.1 | ✅ |
| `forward_effort_controller` según la doc oficial | ≥ 5.25.1 | ✅ |

Como 5.25.2 ≥ 5.25.1, **no aplica** el popup bloqueante del teach pendant al
llamar a `set_friction_model_parameters` (ver la advertencia más abajo, que se
conserva por trazabilidad pero no afecta a este robot).

### Umbrales reales (verificados en el código fuente, no en el plan)

El plan original atribuía `5.25.1 / 10.12.1` a `forward_effort_controller` y
`5.23.0 / 10.10.0` a `direct_torque(...)`. Ambos números son correctos pero
corresponden a **capas distintas**, y la diferencia importa para G4:

| Capacidad | PolyScope 5 | PolyScope X | Fuente |
|---|---|---|---|
| `direct_torque(...)` — modo `TORQUE` del `ReverseInterface` | ≥ 5.23.0 | ≥ 10.10.0 | `Universal_Robots_Client_Library/doc/polyscope_compatibility.rst`, sección *Torque control (From version 2.4.0)* |
| Chequeo que hace el driver antes de aceptar la interfaz `effort` | ≥ 5.23.0 | ≥ 10.10.0 | `ur_robot_driver/src/hardware_interface.cpp:1252-1265` (tag `2.13.2`) |
| Popup de error del script embarcado | ≥ 5.23.0 | ≥ **10.11.0** | `Universal_Robots_Client_Library/resources/external_control.urscript:296,311` |
| **Escalas de fricción** (`viscous_scale`, `coulomb_scale`) = lo que usa `friction_model_controller` | ≥ **5.25.1** | ≥ **10.12.1** | `external_control.urscript:286,301` (popup bloqueante si no se cumple) |
| Documentación oficial de `forward_effort_controller` | ≥ 5.25.1 | ≥ 10.12.1 | `ur_robot_driver/doc/usage/force_torque_control.rst` |

**Consecuencia práctica:** con PolyScope entre 5.23.0 y 5.25.0 el torque directo
funciona, pero **cualquier llamada a `setFrictionScales()` abre un popup
bloqueante en el teach pendant**. Como `friction_model_controller` arranca
activo por defecto en el launch del driver (ver G4), con esas versiones hay que
**no llamar al servicio** o desactivar el controlador. Con ≥ 5.25.1 no hay
restricción.

### Datos registrados (para la sección de setup experimental del paper)

```
PolyScope (versión exacta): 5.25.2
Número de serie del UR5e:   20245500119
IP del robot:               192.168.0.102
```

Comando de re-verificación (dashboard server, puerto 29999), por si se
actualiza el software del robot durante la campaña:

```bash
export UR_IP=192.168.0.102
printf 'PolyscopeVersion\nget serial number\nquit\n' | nc -q 2 $UR_IP 29999
```

> Si la versión cambia a mitad de campaña, **hay que reportarlo**: el ajuste de
> compensación de fricción interna del robot depende de ella.

> **Aviso importante para la planificación de la FASE 9:**
> *"On URSim torque commands don't have any effect."*
> (`Universal_Robots_Client_Library/doc/examples/direct_torque_control.rst`).
> El control por torque **no se puede ensayar en URSim**: o Gazebo (FASES 1–8) o
> el robot físico (FASE 9). No hay escalón intermedio.

---

## G2 — Distro y versión del driver

**Estado:** ✅ **Cerrada. Decisión: ruta (a) — el paquete de Humble ya trae todo
lo necesario.** No hay que compilar de fuente ni migrar a Jazzy.

El plan asumía que la documentación de *Force and Torque Control*, publicada
solo para Rolling/Jazzy/Kilted, implicaba que las capacidades no estaban en
Humble. **Es falso:** la documentación no se retro-publicó, pero el código sí
está backporteado. Verificado contra el **tag exacto que empaqueta el debian de
Humble**, no contra la rama.

### Evidencia — versiones INSTALADAS en este sistema

`sudo apt install -y ros-humble-ur` ejecutado el 2026-07-30:

```
ros-humble-ur-robot-driver     2.13.2-1jammy.20260625.112711
ros-humble-ur-controllers      2.13.2-1jammy.20260625.111916
ros-humble-ur-client-library   2.13.0-1jammy.20260619.115341
ros-humble-ur-dashboard-msgs   2.13.2-1jammy.20260625.111113
ros-humble-ur-calibration      2.13.2-1jammy.20260625.113543
ros-humble-ur-msgs             2.5.0-1jammy.20260605.131141
ros-humble-ur-description      2.10.0-1jammy.20260422.111151
```

Verificación funcional sobre los ficheros instalados (no sobre el repositorio):

| Requisito | Verificado en | Resultado |
|---|---|---|
| `forward_effort_controller` con `interface_name: effort` | `/opt/ros/humble/share/ur_robot_driver/config/ur_controllers.yaml` | ✅ tipo `effort_controllers/JointGroupEffortController` |
| `friction_model_controller` cargable | `/opt/ros/humble/share/ur_controllers/controller_plugins.xml` | ✅ `ur_controllers/FrictionModelController` |
| Servicio de escalas de fricción | `/opt/ros/humble/share/ur_msgs/srv/SetFrictionModelParameters.srv` | ✅ `FrictionModelParameters parameters → bool success` |
| `force_torque_sensor_broadcaster` → `ft_data` | `ur_controllers.yaml` | ✅ `topic_name: ft_data`, `frame_id: tool0_controller` |
| Interfaz de comando `effort` por junta (robot real) | `/opt/ros/humble/share/ur_description/urdf/ur.ros2_control.xacro:69-74` | ✅ dentro de `<xacro:unless value="${sim_ignition}">` |
| Sensor F/T declarado | ídem `:149` | ✅ `<sensor name="${tf_prefix}tcp_fts_sensor">` |
| GPIO de escalas de fricción | ídem `:294` | ✅ `<gpio name="${tf_prefix}friction_model">` (`viscous_0..5`) |
| GPIO de `zero_ftsensor` | ídem `:320-322` | ✅ `zero_ftsensor_cmd`, `zero_ftsensor_async_success` |

> El `<xacro:unless value="${sim_ignition}">` de la línea 72 es exactamente la
> razón por la que `ur5_dyn_control` necesita su propio bloque `ros2_control`
> para Gazebo: el xacro estándar **suprime** la interfaz `effort` en simulación.
> En el robot real no hace falta: el bloque del driver ya la trae.

Coincidencia con el tag `2.13.2` del repositorio
`UniversalRobots/Universal_Robots_ROS2_Driver` (verificado antes de instalar):

| Requisito del plan | Presente en 2.13.2 | Evidencia |
|---|---|---|
| `forward_effort_controller` (`effort_controllers/JointGroupEffortController`, `interface_name: effort`) | ✅ | `ur_robot_driver/config/ur_controllers.yaml` |
| `friction_model_controller` (`ur_controllers/FrictionModelController`, servicio `~/set_friction_model_parameters`) | ✅ | `ur_controllers/controller_plugins.xml` + `ur_controllers/src/friction_model_controller.cpp:113` |
| `force_torque_sensor_broadcaster` publicando en `ft_data` | ✅ | `ur_robot_driver/config/ur_controllers.yaml` (`topic_name: ft_data`) |
| Interfaz de comando `effort` exportada por el hardware | ✅ | `ur_robot_driver/src/hardware_interface.cpp:373-375` |
| `zero_ftsensor` en `io_and_status_controller` | ✅ | `ur_controllers/GPIOController` |

**No disponible en Humble 2.13.2:** `gravity_update_controller`
(`ur_controllers/GravityUpdateController`), que sí existe en Jazzy. Solo hace
falta si el robot se monta en orientación no estándar; el UR5e de este
laboratorio está montado sobre mesa en orientación estándar, así que **no
bloquea nada**. Queda anotado como limitación si en algún momento se cambia el
montaje. *(El `README.md` de `ur5_dyn_control` lo recomendaba; se corrige.)*

### Controladores activos por defecto en `ur_control.launch.py` (2.13.2)

```
activos:   joint_state_broadcaster, io_and_status_controller,
           speed_scaling_state_broadcaster, force_torque_sensor_broadcaster,
           tcp_pose_broadcaster, ur_configuration_controller,
           friction_model_controller
inactivos: scaled_joint_trajectory_controller, joint_trajectory_controller,
           forward_velocity_controller, forward_position_controller,
           forward_effort_controller, force_mode_controller,
           passthrough_trajectory_controller, freedrive_mode_controller,
           tool_contact_controller
```

`forward_effort_controller` arranca **inactivo**: hay que activarlo
explícitamente, que es exactamente lo que hace `ControllerSwitcher` del paquete.

### Exclusión mutua (confirmada en la tabla de compatibilidad del hardware)

`hardware_interface.cpp:63-106` declara `effort` **incompatible** con
`position`, `velocity`, `force_mode`, `passthrough` y `freedrive`; solo es
compatible con `tool_contact`. Confirma el `deactivate_controllers:=[scaled_joint_trajectory_controller]`
que el nodo ya soporta.

### Re-verificación (si se actualiza el sistema)

```bash
source /opt/ros/humble/setup.bash
dpkg -l | grep -E "ros-humble-ur-(robot-driver|controllers|client-library|msgs)"
grep -A2 "^    forward_effort_controller:" /opt/ros/humble/share/ur_robot_driver/config/ur_controllers.yaml
grep -c FrictionModelController /opt/ros/humble/share/ur_controllers/controller_plugins.xml
```

---

## G3 — Gravedad: NO enviarla en el comando  ✅ **CERRADA**

**Prioridad absoluta de la FASE 0. Implementada, construida y testeada.**

### Hecho verificado

> *"This controller streams target joint efforts (torques) directly to the robot
> using the URScript function `direct_torque(...)`. **The robot automatically
> compensates for gravity, so the provided target torques should not include
> gravity compensation.** The user is responsible for sending commands that are
> safe and achievable."*
>
> — `ur_robot_driver/doc/usage/force_torque_control.rst`, sección
> `forward_effort_controller`

### Implementación

Parámetro `gravity_in_command` (bool) en `TorqueControlNodeBase`:

```
gravity_in_command = true   (Gazebo)      ->  tau_cmd = tau_ley
gravity_in_command = false  (UR5e real)   ->  tau_cmd = tau_ley - g(q)
```

- **Default `true`** — el comportamiento en Gazebo no cambia (regresión nula:
  con la política de paso directo el comando es bit a bit el de antes).
- La regla vive en funciones **puras y sin ROS** en
  `ur5_dyn_control/include/ur5_dyn_control/torque_command.hpp`
  (`applyGravityPolicy`, `saturate`, `torqueCommand`), y `publishTau()` es la
  **única** ruta por la que el nodo publica torque, de modo que el test cubre
  literalmente el código de producción.
- **Orden fijado y testeado: primero se resta `g(q)`, después se satura.**
  `tau_max` limita lo que se *envía* al hardware; en el robot real el comando es
  un *feedforward* que se suma a la compensación interna, así que saturar antes
  de restar limitaría una magnitud que nunca se transmite. Esto importa
  directamente para el §7 (`tau_max` conservador al 30 % en el primer ensayo).
- `computeTau()` de las subclases **no cambia**: sigue devolviendo el torque
  físico completo (con gravedad). Se respeta la prohibición de bifurcar la ley
  de control por entorno — lo único que cambia entre sim y real es este
  parámetro.

### Test unitario (requisito bloqueante del plan)

`ur5_dyn_control/test/test_gravity_policy.cpp` — 8 tests, **8/8 en verde**:

| Test | Qué demuestra |
|---|---|
| `PureGravityLawCommandsZeroOnRealRobot` | **El caso exigido por el plan**: ley de gravedad pura (`tau_ley = g(q)`) con `gravity_in_command=false` ⟹ `‖tau_cmd‖∞ < 1e-9` en 5 configuraciones |
| `FeedbackLinearizationAtRestCommandsZeroOnRealRobot` | Lo mismo para FL en reposo sobre referencia estática (`M·0 + n(q,0) = g(q)`): `gz_fl_control_node` tampoco duplica |
| `KeepsNonGravityTermsUntouched` | Solo se quita la gravedad; la acción de control sobrevive intacta (no se pierde ganancia en el real) |
| `GazeboPolicyIsPassThrough` | Regresión de Gazebo: con `true` el comando es la ley **bit a bit** |
| `ModelHonoursConfiguredGravity` | El modelo Pinocchio honra la `g` configurada (guarda contra mismatch modelo↔planta) |
| `IsSymmetricAndComponentWise` | Saturación simétrica por componente |
| `AppliesToCommandNotToLaw` | El orden política→saturación es el implementado, con `tau_max` conservador del §7 |
| `ApplyGravityPolicyMatchesDefinition` | La política coincide con su definición algebraica |

```bash
cd ~/ur5_ws
colcon build --packages-select ur5_dyn_control
colcon test --packages-select ur5_dyn_control --event-handlers console_direct+
# -> 8 tests from 2 test suites ran. [ PASSED ] 8 tests.
```

### Regresión en Gazebo (sin cambios de comportamiento)

| Corrida | Resultado |
|---|---|
| `gravity_comp` (mundo vacío, 15 s de sim) | Regulación en `q_init`; `tau` en régimen = `[0, −20.244, −20.244, −1.823, 0, 0]` N·m = `g(q_init)` exacto |
| `fl_control` #1 (mundo vacío, TRACK completo, 3751 muestras) | RMS articular **0.000059 rad**, máx 0.000395 rad; TCP RMS **0.037 mm**, máx 0.241 mm; `\|tau\|` máx 35.1 N·m (sin saturación) |
| `fl_control` #2 (tras el refactor A1/A2, 3750 muestras) | RMS articular **0.000127 rad**, máx 0.000740 rad; TCP RMS **0.111 mm**, máx 0.394 mm |

Ambas corridas quedan **dos órdenes de magnitud por debajo** del umbral del plan
original (RMS < 0.02 rad, TCP < 5 mm). La diferencia entre las dos **no es una
regresión**: alineando las series por tiempo, el desfase óptimo entre ellas es
de **+1 ms** — exactamente un paso de física de 1 kHz, es decir medio periodo
del lazo de 500 Hz. Es *jitter* de fase entre el timer de pared y los pasos de
física, que varía de corrida a corrida. La tabla de referencias no cambió: el
*hook* de herramienta con `mass = 0` es neutro a 1e-12 (test
`ZeroMassLeavesModelUntouched`) y `tcp_offset_z` vale el mismo `0.141`.

> Anotado para la **FASE 3**: conviene registrar el histograma de `dt` real del
> lazo (el plan ya lo pide en su tabla de riesgos, *"jitter del lazo en Linux
> no-RT"*). Con la evidencia de arriba, ese jitter es hoy la fuente dominante de
> variación entre corridas nominales, y hay que caracterizarlo antes de atribuir
> diferencias entre controladores en la FASE 8.

**Observación registrada (no es regresión, es física):** en `gravity_comp` la
`wrist_3` deriva ~0.02 rad en los últimos 5 s. La planta de Gazebo no tiene
fricción y la ganancia de esa junta es `kp=2` — es deriva de integrador libre.
El criterio *"`gravity_comp` regula en `q_init` sin deriva durante 60 s"* es de
la **FASE 3** y se aborda allí (compensación de fricción de la FASE 2 +
endurecimiento del lazo). Queda anotado como entrada pendiente de la FASE 3, no
de la FASE 0.

### ⛔ Regla operativa

Con `gravity_in_command:=true` en el robot real, la gravedad se compensa **dos
veces** y el brazo se acelera hacia arriba. En cualquier fichero de parámetros
destinado al UR5e físico, `gravity_in_command` **debe** valer `false`; el nodo
lo registra en el log al arrancar:

```
[gz_fl_control_node]: G3 gravity_in_command=false (UR5e real) -> tau_cmd = tau_ley - g(q)
```

---

## G4 — Fricción interna del robot

**Estado:** ✅ **CERRADA. Ajuste de operación decidido: `viscous_scale =
coulomb_scale = 1.0` en las 6 juntas** (compensación interna completa),
fijado explícitamente por servicio en cada sesión.

> ### ⚠️ Dato nuevo tras instalar el driver: el robot NO usa 1.0 por defecto
>
> Al inspeccionar el paquete instalado apareció la documentación de los valores
> por defecto, que **no estaba disponible antes de la instalación** y corrige el
> hallazgo original de esta compuerta (*"defaults no documentados"*):
>
> ```
> # /opt/ros/humble/share/ur_msgs/msg/FrictionModelParameters.msg
> # Default: [0.9, 0.9, 0.8, 0.9, 0.9, 0.9]   <- viscous_scale
> # Default: [0.8, 0.8, 0.7, 0.8, 0.8, 0.8]   <- coulomb_scale
> ```
>
> Es decir: UR **no** aplica compensación completa por defecto, y baja aún más
> la del **codo** (`elbow`, índice 2: 0.8 viscoso / 0.7 Coulomb). Eso sugiere
> que 1.0 puede sobre-compensar en esa junta.
>
> **Riesgo de la decisión adoptada:** un compensador de fricción que
> sobre-estima puede producir *stick-slip* / ciclos límite a baja velocidad,
> justo el régimen del tramo `cut`. Con 1.0/1.0 se opera por encima del ajuste
> que UR entrega de fábrica.
>
> **Mitigación acordada (sin cambiar la decisión):** el barrido de
> caracterización de la FASE 2 pasa de `{0.0, 1.0}` a **`{0.0, default, 1.0}`**
> (tres niveles). Si el nivel `1.0` muestra oscilación a baja velocidad que el
> `default` no muestra, se reconsidera el ajuste de operación **con datos**, y
> el cambio se reporta. Hasta entonces, 1.0/1.0 queda como valor de campaña.

### Hechos verificados (tag 2.13.2, confirmados sobre el sistema instalado)

1. `friction_model_controller` **arranca activo por defecto**
   (`ur_control.launch.py:382-389`).
2. **Pero no impone ninguna escala hasta que se llama al servicio.** El
   controlador inicializa `viscous_scale` y `coulomb_scale` a **NaN**
   (`friction_model_controller.cpp:97-110`) y solo escribe a las command
   interfaces cuando `change_requested_` es `true`
   (`friction_model_controller.cpp:165-185`), es decir tras un
   `~/set_friction_model_parameters`. Antes de esa llamada el robot usa sus
   **valores internos por defecto, no documentados**.
3. El script embarcado distingue tres modos
   (`external_control.urscript:99-101, 284-311`):
   `FRICTION_COMP_MODE_NOT_SET` → `direct_torque(torque)` con los *defaults del
   robot*; `..._FRICTION_SCALES` → `direct_torque(torque, viscous_scale=…,
   coulomb_scale=…)`; `..._FRICTION_LEGACY` → `direct_torque(torque,
   friction_comp=…)`.
4. Las escalas por junta están en `[0, 1]`; `0` = sin compensación interna,
   `1` = compensación completa.

**Consecuencia metodológica:** si no se llama al servicio, el ajuste de fricción
usado es *desconocido* y la campaña **no es reproducible**. Hay que llamarlo
siempre y de forma explícita, aunque sea para fijar `1.0`.

### Decisión adoptada (confirmada por el usuario, 2026-07-30)

- **Operación (FASES 2 y 9):** `viscous_scale = coulomb_scale = 1.0` en las 6
  juntas. La identificación de la FASE 2 captura entonces solo la fricción
  **residual**, y es lo que menos carga deja al término discontinuo del SMC.
- **Caracterización (FASE 2):** barrido a **tres** niveles — `0.0` (sin
  compensación interna), `default` (`[0.9,0.9,0.8,0.9,0.9,0.9]` /
  `[0.8,0.8,0.7,0.8,0.8,0.8]`) y `1.0` — por el motivo del aviso de arriba.
- El ajuste se **fija siempre de forma explícita** al inicio de cada sesión,
  aunque coincida con el default: sin la llamada al servicio el controlador no
  impone nada y el valor efectivo no queda registrado.
- Los valores se escriben en el YAML de campaña y se reportan en el paper.

### Comandos de sesión (robot en marcha)

```bash
# Ajuste de OPERACIÓN acordado: compensación completa en las 6 juntas
ros2 service call /friction_model_controller/set_friction_model_parameters \
  ur_msgs/srv/SetFrictionModelParameters \
  "{parameters: {viscous_scale: [1.0,1.0,1.0,1.0,1.0,1.0],
                 coulomb_scale: [1.0,1.0,1.0,1.0,1.0,1.0]}}"

# Nivel 'default' del barrido de caracterización (FASE 2)
ros2 service call /friction_model_controller/set_friction_model_parameters \
  ur_msgs/srv/SetFrictionModelParameters \
  "{parameters: {viscous_scale: [0.9,0.9,0.8,0.9,0.9,0.9],
                 coulomb_scale: [0.8,0.8,0.7,0.8,0.8,0.8]}}"

# Nivel 0.0 del barrido: sin compensación interna
ros2 service call /friction_model_controller/set_friction_model_parameters \
  ur_msgs/srv/SetFrictionModelParameters \
  "{parameters: {viscous_scale: [0.0,0.0,0.0,0.0,0.0,0.0],
                 coulomb_scale: [0.0,0.0,0.0,0.0,0.0,0.0]}}"
```

Verificar que la llamada tuvo efecto (`success: true`) y anotarlo en la hoja de
sesión. Con PolyScope 5.25.2 **no** aparece el popup bloqueante del teach
pendant (requiere ≥ 5.25.1, ver G1).

---

## G5 — Qué se puede medir y qué no  ✅ **CERRADA**

Los tres puntos del plan quedan verificados en el código del driver:

| Afirmación | Estado | Evidencia |
|---|---|---|
| El campo `effort` de `/joint_states` son **corrientes de motor**, no torques físicos | ✅ Confirmado | `hardware_interface.cpp:800` lee el campo RTDE `actual_current` en `urcl_joint_efforts_`, que es lo que alimenta la state interface `effort` (`:235`). También lo dice la nota de `doc/usage/controllers.rst:53`. |
| El torque de referencia es el **comando** | ✅ Adoptado | El CSV registra `tau1..tau6` = torque **comandado** (post-G3, post-saturación). Documentado en `csv_logger.hpp`. |
| El wrench del TCP se publica en `ft_data` | ✅ Confirmado | `force_torque_sensor_broadcaster` con `topic_name: ft_data`, `frame_id: tool0_controller`; solo lectura, combinable con cualquier controlador |
| El sensor F/T es **relativo**: `zero_ftsensor` antes de cada corte y antes de tocar nada | ✅ Confirmado | Servicio `~/zero_ftsensor` de `io_and_status_controller`; la propia doc del driver lo recomienda antes del contacto |

**Consecuencia para la FASE 9:** la validación cruzada del estimador de fuerza
`F̂_ext = (Jᵀ)⁻¹(tau_cmd − tau_modelo)` usa el **comando**, nunca el campo
`effort`. Cualquier figura o tabla del paper que hable de "torque medido" sería
incorrecta: se dice **torque comandado**.

---

## G6 — Seguridad (§7)  ⚠️ **Firmado salvo el watchdog**

El checklist del §7 del plan se firma **por sesión**, no una sola vez. Sin él,
prohibido activar `forward_effort_controller` en el robot real.

**Estado 2026-07-30 (firma del usuario):** el checklist queda firmado **hasta el
punto del watchdog, no incluido**. Los puntos posteriores son de sesión y se
firman el día del ensayo.

- [x] Modo de velocidad reducida activo en el teach pendant
- [x] Paro de emergencia al alcance del operador
- [x] Planos de seguridad / límites de espacio alrededor de la mesa de corte
- [x] `tau_max` conservador para el primer ensayo (30 % del nominal:
      `[45, 45, 45, 8.4, 8.4, 8.4]` N·m) y subida gradual
- [ ] **Watchdog probado** — ⛔ **BLOQUEANTE. No existe todavía: es entregable
      de la FASE 3.** Mientras no esté, no se activa torque en el robot real.
- [ ] `RobotReceiveTimeout` en decenas de ms *(se configura con el bring-up del
      driver, FASE 9)*
- [ ] Primer ensayo sin bisturí montado y sin material *(por sesión)*
- [ ] Segundo ensayo con bisturí al aire, verificando `ft_data ≈ 0` *(por sesión)*
- [ ] `zero_ftsensor` ejecutado y verificado antes de aproximar *(por corte)*
- [ ] Protocolo de corte-punzante para la hoja; contenedor rígido a mano *(por sesión)*
- [ ] Nadie dentro del espacio de trabajo durante los ensayos con torque *(por sesión)*

```
Firma de los puntos previos al watchdog: 2026-07-30
Operador responsable:                    Sergio Morales
```

**Consecuencia sobre el orden de fases:** el watchdog convierte la dependencia
**FASE 3 → FASE 9** en dura. El grafo del plan (§10) ya la tenía, pero por otra
razón; aquí queda además como requisito de seguridad firmado.

---

## Resumen de acciones — todas resueltas (2026-07-30)

| # | Acción | Compuerta | Estado |
|---|---|---|---|
| 1 | Registrar versión de PolyScope, número de serie e IP | G1 | ✅ 5.25.2 / 20245500119 / 192.168.0.102 |
| 2 | `sudo apt install ros-humble-ur` y verificar | G2 | ✅ 2.13.2, verificado sobre los ficheros instalados |
| 3 | Fijar el ajuste de fricción de operación | G4 | ✅ `1.0 / 1.0` (con barrido a 3 niveles en la FASE 2) |
| 4 | Firmar el §7 | G6 | ⚠️ Firmado salvo el watchdog (FASE 3) |

**FASE 0 CERRADA. Habilitadas las FASES 1–8.** La FASE 9 queda bloqueada
únicamente por el watchdog, entregable de la FASE 3.

### Pendientes que se resuelven solos al avanzar

| Qué | Dónde se resuelve |
|---|---|
| Watchdog probado (§7) | FASE 3 |
| `RobotReceiveTimeout` en decenas de ms | FASE 9 (bring-up del driver) |
| Puntos de sesión del §7 (bisturí, `zero_ftsensor`, espacio libre) | FASE 9, por sesión/corte |
| Masa/CoM/inercia del acople del bisturí (A1) | Antes de la FASE 9; hook ya listo |
| Distancia real `tool0` → punta de la hoja (A2) | Al montar el bisturí |

---

## Procedimiento de sesión con torque en el UR5e real

Orden fijo. Los pasos 1–5 son del operador y hay que **verificarlos** antes de
que nada comande par; por eso `ur5e_real.launch.py` **no** levanta el driver:
encadenarlo significaría que el primer torque sale en cuanto el driver esté
listo, sin ventana para abortar.

**1. Driver** (terminal aparte; robot encendido, sin protective stop)

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e robot_ip:=192.168.0.102 launch_rviz:=false \
    initial_joint_controller:=scaled_joint_trajectory_controller
```

**2. Pendant**: cargar el programa con `External Control` y darle a play.

**3. Verificar**

```bash
ros2 control list_controllers      # forward_effort_controller debe estar cargado
ros2 topic hz /joint_states        # ~500 Hz
```

**4. Escalas de fricción interna** (G4) — fijarlas **siempre** de forma
explícita, aunque coincidan con el default: sin la llamada el valor efectivo no
queda registrado y la campaña no es reproducible. Comandos en §G4.

**5. `zero_ftsensor`** antes de aproximar, y firmar el checklist §7 del plan.

**6. Primer ensayo: el brazo solo se sostiene** (sin moverse). Valida G3, el
watchdog y el switch sin que el robot recorra nada:

```bash
ros2 launch ur5_dyn_control ur5e_real.launch.py \
    controller:=gravity_comp skip_trajectory:=true tau_scale:=0.30 test_num:=900
```

Si la gravedad estuviera mal compensada se ve **aquí**, con el robot quieto.

**7. Campaña de fricción** — 6 juntas × 3 niveles de compensación interna
(`0.0`, `default`, `1.0`) = **18 corridas**. Usar el runner semiautomático, que
las encadena pero **no comanda par sin confirmación escrita** en cada una:

```bash
ros2 run ur5_identification run_friction_campaign_real.py --test-base 900
# reanudar tras una interrupción (salta las que ya tienen CSV):
ros2 run ur5_identification run_friction_campaign_real.py --test-base 900 --resume
```

Antes de **cada** corrida comprueba que llega `/joint_states`, que el robot está
dentro de la tolerancia de `q_init` (si no, dice qué junta y cuánto se desvía) y
fija las escalas de fricción por servicio **verificando `success`** — sin esa
llamada el valor efectivo no queda registrado y la campaña no es reproducible
(G4). Aborta la campaña entera si el nodo entra en SAFE_HOLD, y deja un YAML de
sesión con lo que se fijó y lo que se ejecutó.

La confirmación es teclear el **nombre de la junta**, no Enter: Enter se pulsa
por inercia, un nombre hay que leerlo.

**Las muñecas no admiten el nivel `0.0`.** La ley FL entrega `M_jj·kp_j·e` de par
por radián de error, y con `kp` uniforme eso recorre cuatro órdenes de magnitud:
105.8 N·m/rad en `shoulder_pan` frente a **0.03** en `wrist_3`. Sin la
compensación interna del robot, la muñeca se queda parada donde el PD iguala su
fricción estática — medido en `wrist_1`: 44° de error, 1.85 N·m de par, cero
movimiento. Subir `kp` no lo arregla: `wrist_3` necesitaría ~194 000, o sea
`ω_n·dt = 0.88`, fuera del rango de estabilidad discreta a 500 Hz.

Es un resultado, no un fallo: es *por qué existe* el `friction_model_controller`
de UR. Se salta con `--skip` y queda registrado en el YAML de sesión:

```bash
ros2 run ur5_identification run_friction_campaign_real.py --test-base 900 \
    --skip 0.0:wrist_1 0.0:wrist_2 0.0:wrist_3
```

`--skip` admite índice o nombre de junta y `*` como comodín (`0.0:*`, `*:5`).

Una corrida suelta, si hace falta repetir solo una:

```bash
ros2 launch ur5_dyn_control ur5e_real.launch.py \
    controller:=fl trajectory_type:=joint_sweep sweep_joint:=0 \
    tau_scale:=0.30 test_num:=901
```

### Vía alternativa: barrido por CONTROL DE POSICIÓN

El barrido por par **no puede mover las muñecas**. La ley FL entrega
`M_jj·kp_j·e` de par por radián de error, y aun subiendo `kp` hasta el límite de
estabilidad discreta, `wrist_2` saturó en los 8.4 N·m de `tau_scale = 0.30` sin
llegar a moverse: su fricción estática es mayor.

El servo interno del robot sí tiene autoridad. La alternativa manda la
trayectoria al `scaled_joint_trajectory_controller` y registra la corriente:

La campaña entera va por esta vía **por defecto**:

```bash
ros2 run ur5_identification run_friction_campaign_real.py --test-base 950
# equivale a --method position; --method torque es la vía original
```

O una junta suelta:

```bash
ros2 run ur5_identification run_current_sweep.py --joint 4 --test-num 954 \
    --friction-level 0.0
ros2 run ur5_identification calibrate_current.py \
    --csv ~/.ros/ur5_dyn_control/cur_954.csv --joint 4 \
    --out ~/.ros/ur5_dyn_control/fl_954.csv
```

`--friction-level` fija las escalas por servicio, verifica la respuesta y lo
anota en la cabecera del CSV (G4). Sin él no se toca nada y el CSV lo registra
como «no fijado en esta corrida», que es honesto pero no reproducible.

**El campo `effort` es corriente, no par** (G5). La conversión sale del propio
barrido: en la meseta, a la misma postura y misma rapidez,

```
i(+v)·k = g(q) + C·v + f_v·v + f_c
i(−v)·k = g(q) + C·v − f_v·v − f_c
```

la **suma** deja `g(q) + C·v`, que el modelo conoce → de ahí `k` por junta; la
**diferencia** deja la fricción. No hacen falta las constantes de motor de UR.

**Contrastado contra el método por par en la misma junta y el mismo nivel**
(`shoulder_lift`, compensación `0.0`):

| | por par (`fl_901`) | por posición (`fl_951`) |
|---|---|---|
| F_v [N·m·s/rad] | 12.47 ± 1.75 | **12.77 ± 0.61** |
| F_c [N·m] | 6.98 ± 0.86 | **7.21 ± 0.30** |
| R² | 0.9693 | **0.9965** |
| CV leave-one-out RMSE | 2.28 | **1.17** |

Concuerdan al 2.5 % y 3.3 %, cada uno dentro del IC95 del otro, y el método por
posición sale con **la mitad de incertidumbre**: la junta la mueve el servo del
robot en vez de un controlador peleando contra la fricción. La calibración dio
`k = 11.8332 N·m/A` con residuo relativo del **0.294 %** y dispersión entre los
8 niveles de velocidad de 0.0965 N·m/A (0.8 %) — si la relación corriente-par no
fuese lineal, `k` derivaría entre niveles.

Validado además contra verdad sintética conocida: `k` recuperado exacto (0.8500 frente
a 0.850) con residuo relativo del 0.000 %, y `F_v = 2.5000`, `F_c = 4.0000`
frente a los 2.50 / 4.00 verdaderos.

`calibrate_current.py` deja el CSV con el esquema del barrido por par, así que
`run_identification` lo consume sin cambios y **los dos métodos quedan
comparables sobre las mismas juntas** — validación cruzada limpia para el paper.

**8. Identificación**

```bash
ros2 run ur5_identification run_identification \
    --csv ~/.ros/ur5_dyn_control/fl_90*.csv \
    --models viscous_coulomb stribeck --out ~/friction_real.yaml
```

**9. Los coeficientes van a DOS sitios**, no a uno:

| Destino | Para qué | Si se olvida |
|---|---|---|
| Controlador (`friction_compensation`, `friction.f_v/f_c`) | Feedforward en el comando | El SMC tiene que absorber la fricción con el término discontinuo |
| **URDF/xacro** (`<dynamics damping= friction=>`) | Planta de Gazebo | **La campaña de la FASE 8 compara controladores sobre una planta SIN fricción**, que no existe |

`ur_macro.xacro` genera ambos a **cero**, así que el segundo destino es fácil de
pasar por alto y no da ningún síntoma.

### Lo que hace `ur5e_real.launch.py` por ti

- `gravity_in_command:=false` — **forzado, no expuesto como argumento**, para
  que no pueda quedarse a `true` por descuido (G3).
- `use_sim_time:=false`, `perform_unpause:=false`.
- `activate_controllers:=[forward_effort_controller]` (el único que arranca
  inactivo) y `deactivate_controllers:=[scaled_joint_trajectory_controller]`
  (mutuamente excluyentes, ver arriba). **Las dos listas se filtran** contra el
  estado real antes del switch: pedir que se desactive algo ya inactivo hace
  fallar un switch `STRICT` entero.
- `tau_scale` con **default 0.30**, no 1.0 — el §7 pide empezar conservador y
  subir gradualmente, así que subirlo es una decisión consciente.

### ⛔ La columna `tau_phys` del CSV

La identificación **no** debe usar la columna `tau` en el robot real. `tau` es
el par **comandado**, y con `gravity_in_command=false` vale `tau_ley − g(q)`. El
residuo se calcula contra `rnea()`, que **sí** lleva gravedad: usar `tau`
dejaría un sesgo de exactamente `g(q)` —varios N·m en hombro y codo— que el
ajuste atribuiría a **fricción de Coulomb**, con buen R² y coeficientes
inventados.

Por eso el CSV registra `tau<i>_phys` = par **físico** entregado
(`tau_cmd + g(q)` en el real, `tau_cmd` en Gazebo), calculado post-saturación, y
`residual.py` la usa cuando existe. Los CSV antiguos de Gazebo no la tienen;
allí `gravity_in_command=true` y las dos coinciden, así que el fallback es
exacto y se avisa por consola.

---

## Referencias

- `UniversalRobots/Universal_Robots_ROS2_Driver`, tag `2.13.2` (el que empaqueta
  el debian de Humble) — `ur_robot_driver/src/hardware_interface.cpp`,
  `ur_robot_driver/config/ur_controllers.yaml`,
  `ur_robot_driver/launch/ur_control.launch.py`,
  `ur_controllers/controller_plugins.xml`,
  `ur_controllers/src/friction_model_controller.cpp`
- `UniversalRobots/Universal_Robots_ROS2_Driver`, rama `main` —
  `ur_robot_driver/doc/usage/force_torque_control.rst` (única fuente que
  documenta explícitamente la compensación interna de gravedad, G3)
- `UniversalRobots/Universal_Robots_Client_Library`, rama `master` —
  `doc/polyscope_compatibility.rst`, `doc/examples/direct_torque_control.rst`,
  `resources/external_control.urscript`
- Este repositorio — `ur5_dyn_control/include/ur5_dyn_control/torque_command.hpp`,
  `ur5_dyn_control/test/test_gravity_policy.cpp`, [`docs/00_assumptions.md`](00_assumptions.md)
