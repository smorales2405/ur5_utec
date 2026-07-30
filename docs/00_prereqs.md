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

**Resumen**

| Compuerta | Estado | Bloquea |
|---|---|---|
| G1 — PolyScope | ⚠️ Cerrada por el usuario; falta registrar la versión exacta | FASE 9 |
| G2 — Distro y driver | ✅ Ruta (a): el paquete Humble ya lo trae todo | FASE 9 |
| G3 — Gravedad fuera del comando | ✅ Implementada y testeada | FASE 9 (y §7) |
| G4 — Fricción interna | ⚠️ Caracterizada; falta fijar el ajuste de campaña | FASE 2, 9 |
| G5 — Qué se puede medir | ✅ Verificado en el código del driver | FASES 2, 9, 10 |
| G6 — Seguridad (§7) | 🔲 **PENDIENTE** (firma por sesión) | FASE 9 |

---

## G1 — Versión de PolyScope del UR5e físico

**Estado:** ⚠️ El usuario confirma que la versión instalada cumple los requisitos.
Falta **registrar la cadena de versión exacta**, que el paper necesita en la
sección de setup experimental y que condiciona G4.

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

### 🔲 PENDIENTE — ejecutar con el robot encendido y en red

```bash
# Opción A (sin ROS): dashboard server, puerto 29999.  Sustituye la IP.
export UR_IP=192.168.1.102        # <-- ajustar a la IP real del UR5e
printf 'PolyscopeVersion\nversion\nget serial number\nquit\n' | nc -q 2 $UR_IP 29999
```

```bash
# Opción B (con el driver instalado y corriendo, ver G2)
ros2 service call /dashboard_client/get_robot_mode ur_dashboard_msgs/srv/GetRobotMode
ros2 topic echo /io_and_status_controller/robot_program_running --once
```

Anotar aquí el resultado literal:

```
PolyScope (versión exacta): ______________________     🔲 PENDIENTE
Número de serie del UR5e:   ______________________     🔲 PENDIENTE
IP del robot:               ______________________     🔲 PENDIENTE
```

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

### Evidencia

Versión candidata en este sistema (`apt-cache policy`):

```
ros-humble-ur-robot-driver    2.13.2-1jammy.20260625.112711   (no instalado)
ros-humble-ur-controllers     2.13.2-1jammy.20260625.111916   (no instalado)
ros-humble-ur-client-library  2.13.0-1jammy.20260619.115341   (no instalado)
ros-humble-ur                 2.13.2-1jammy.20260625.114324   (no instalado)
```

Contenido del tag `2.13.2` del repositorio `UniversalRobots/Universal_Robots_ROS2_Driver`:

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

### 🔲 PENDIENTE — instalar el driver (aún no está en este sistema)

```bash
sudo apt update
sudo apt install -y ros-humble-ur          # metapaquete: driver + controllers + moveit config
# Verificación posterior:
source /opt/ros/humble/setup.bash
ros2 pkg xml ur_robot_driver | grep -E "<name>|<version>"
ros2 pkg xml ur_controllers  | grep -E "<name>|<version>"
grep -c "forward_effort_controller" /opt/ros/humble/share/ur_robot_driver/config/ur_controllers.yaml
```

Anotar el resultado:

```
ur_robot_driver instalado, versión: ______________     🔲 PENDIENTE
ur_controllers  instalado, versión: ______________     🔲 PENDIENTE
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

**Estado:** ⚠️ Caracterizada en el código; falta **fijar y registrar el ajuste de
operación**, que es lo que exige el plan para la reproducibilidad del paper.

### Hechos verificados (tag 2.13.2)

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

### Decisión propuesta (⚠️ requiere confirmación del usuario)

- **Caracterización (FASE 2):** barrer `scale ∈ {0.0, 1.0}` en ambos coeficientes
  para acotar la fricción residual en los dos extremos.
- **Operación (FASES 2 y 9):** fijar `viscous_scale = coulomb_scale = 1.0` en las
  6 juntas (compensación interna completa) y dejar que la identificación de la
  FASE 2 capture solo la **residual**. Es el ajuste más estable y el que menos
  carga deja al término discontinuo del SMC.
- Los valores usados se escriben en el YAML de campaña y se reportan en el paper.

```
Ajuste de operación acordado: viscous_scale = ______  coulomb_scale = ______
                                                                🔲 PENDIENTE (confirmar)
```

### 🔲 PENDIENTE — comandos con el robot en marcha

```bash
# Fijar escalas (ejemplo: compensación completa en las 6 juntas)
ros2 service call /friction_model_controller/set_friction_model_parameters \
  ur_msgs/srv/SetFrictionModelParameters \
  "{parameters: {viscous_scale: [1.0,1.0,1.0,1.0,1.0,1.0],
                 coulomb_scale: [1.0,1.0,1.0,1.0,1.0,1.0]}}"

# Extremo opuesto para la caracterización
ros2 service call /friction_model_controller/set_friction_model_parameters \
  ur_msgs/srv/SetFrictionModelParameters \
  "{parameters: {viscous_scale: [0.0,0.0,0.0,0.0,0.0,0.0],
                 coulomb_scale: [0.0,0.0,0.0,0.0,0.0,0.0]}}"
```

> Con PolyScope 5 < 5.25.1 este servicio provoca un **popup bloqueante** en el
> teach pendant (ver G1). Confirmar la versión antes de llamarlo.

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

## G6 — Seguridad (§7)  🔲 **PENDIENTE**

El checklist del §7 del plan se firma **por sesión**, no una sola vez. Sin él,
prohibido activar `forward_effort_controller` en el robot real.

Como G1–G5 ya están cerradas o acotadas, lo que falta es material y de
procedimiento. Antes de la primera sesión de FASE 9 hay que tener resueltos:

- [ ] Modo de velocidad reducida activo en el teach pendant
- [ ] Paro de emergencia al alcance del operador
- [ ] Planos de seguridad / límites de espacio alrededor de la mesa de corte
- [ ] `tau_max` conservador para el primer ensayo (30 % del nominal:
      `[45, 45, 45, 8.4, 8.4, 8.4]` N·m) y subida gradual
- [ ] Watchdog probado (⚠️ **no existe todavía** — es entregable de la **FASE 3**;
      la FASE 9 no puede empezar antes que la FASE 3)
- [ ] `RobotReceiveTimeout` en decenas de ms
- [ ] Primer ensayo sin bisturí montado y sin material
- [ ] Segundo ensayo con bisturí al aire, verificando `ft_data ≈ 0`
- [ ] `zero_ftsensor` ejecutado y verificado antes de aproximar
- [ ] Protocolo de corte-punzante para la hoja; contenedor rígido a mano
- [ ] Nadie dentro del espacio de trabajo durante los ensayos con torque

```
Fecha de firma de la primera sesión: ____________       🔲 PENDIENTE
Operador responsable:                ____________       🔲 PENDIENTE
```

---

## Resumen de acciones para el usuario

| # | Acción | Compuerta | Bloquea |
|---|---|---|---|
| 1 | Registrar la versión exacta de PolyScope, número de serie e IP | G1 | FASE 9 |
| 2 | `sudo apt install ros-humble-ur` y anotar versiones | G2 | FASE 9 |
| 3 | Confirmar el ajuste de fricción de operación propuesto (`1.0/1.0`) | G4 | FASES 2, 9 |
| 4 | Firmar el §7 antes de la primera sesión con torque | G6 | FASE 9 |

**Ninguna de las cuatro bloquea las FASES 1–8** (todas en Gazebo). El trabajo
puede continuar por la FASE 1 en cuanto se confirme este documento.

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
