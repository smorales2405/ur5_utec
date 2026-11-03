# Protocolo de sesión con par en el UR5e real — B7.0 (seguridad) y B4.4 (α_g)

**Estado:** **ejecutado el 2026-11-02**. Plan: B4.4 y B7.0 de
`Journal_UR5eControl_plan.md`. B7.0 va **antes** de B4.4 en la misma sesión:
B4.4 es el primer uso del par con el acople montado.

Números de corrida: `gravity_comp_1203`–`1204` (lo que queda de B7.0) y `fl_1101`–`1133`
(B4.4: `1100 + 10·postura + junta`). Los scripts y `ur5e_real.launch.py` se niegan a
sobrescribir un CSV con datos.

**Resultado de la sesión del 2026-11-02 (§2 y §3 completas, `α_g` = 0.0543):**

| Corrida | Qué es | Estado |
|---|---|---|
| `gravity_comp_1200` | Sostén 221 s | **Perdida:** la sobrescribió una prueba del asistente, solo cabecera. Quedan los valores: `g_robot` −21.06 / −21.01 / −1.98 N·m y deriva ≤ 0.0001 rad |
| `gravity_comp_1201` | Fin limpio, 2.3 | Válida |
| `gravity_comp_1202` | Red, 2.4 | **Sin valor:** se quedó en `PRE_HOLD`, sin par; un relanzamiento pisó el primer intento |
| `gravity_comp_1203` | Sostén y vigilante, 2.1 y 2.2 | Válida. El vigilante cortó en 105 ms |
| `gravity_comp_1204` | Red, 2.4 | Válida. «Socket timed out» en el Registro; el brazo se quedó quieto (lo confirmó el operador) |
| `fl_1101`–`1133` | Barridos, §3 | Válidas, 12 de 12. `α_g` en `~/.ros/gravity_sweeps/alpha_g.yaml` |
| `fl_1111.stub` | Primer intento | Sin datos: cuelgue en `PRE_HOLD`, ya corregido |

---

## 0. Preparación (robot apagado o en paro)

1. **Acople montado SIN la hoja.** Es el primer ensayo con par y acople
   (checklist §7: «primer ensayo sin bisturí»). Mesa despejada: sin muestras ni
   fijación.
2. **Masa del acople sin bisturí ni hoja: 0.155 kg** (0.179 kg con ellos, menos los
   23.9 g del bisturí y la hoja, pesados el 2026-11-02). Para los cortes, con la hoja
   puesta, el payload vuelve a ser 0.179 kg.
3. **Pendant → Instalación → Carga útil:** masa **0.155 kg**; centro de gravedad
   **(−2.2, 2.1, 29.4) mm** (CAD, desde la brida). **TCP en (0, 0, 0)**. En la
   instalación, no solo en el programa: el payload vuelve solo al recargar
   (`02_friction_real.md` §8.7).

   Ese CoG es **con** el bisturí (el CAD da 0.182 kg, como la pesada con él); sin él
   `z` bajaría a ~14–25 mm, sin medir. No importa aquí: con la herramienta vertical el
   error es ≤ 0.0011 N·m en todos los barridos.

   *Por qué no payload 0:* en el real la gravedad la compensa el ROBOT (G3). El
   nodo manda la ley menos su `ĝ` y el robot suma la suya con el payload
   configurado; con payload 0 el peso del acople queda sin compensar (0.88 N·m en
   hombro y codo en `q_init`). El payload 0 de antes era para la brida desnuda.
4. Velocidad reducida activa, paro de emergencia a mano, nadie dentro.

## 1. Arranque

**Terminal A — driver** (y External Control en *play* en el pendant):

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e robot_ip:=192.168.0.102 launch_rviz:=false \
    initial_joint_controller:=scaled_joint_trajectory_controller
```

**Terminal B** (y C, para las pruebas de desconexión):

```bash
source ~/ur5_ws/install/setup.bash

# G4 a cero. NO persiste: repetir cada vez que se relance el driver.
ros2 service call /friction_model_controller/set_friction_model_parameters \
  ur_msgs/srv/SetFrictionModelParameters \
  "{parameters: {viscous_scale: [0.0,0.0,0.0,0.0,0.0,0.0],
                 coulomb_scale: [0.0,0.0,0.0,0.0,0.0,0.0]}}"

ros2 control list_controllers      # scaled_joint_trajectory_controller activo
```

Robot en `q_init` = (90°, −90°, 90°, −90°, −90°, 180°): con el pendant, o desde
cerca con

```bash
ros2 action send_goal /scaled_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
                               wrist_1_joint, wrist_2_joint, wrist_3_joint],
    points: [{positions: [1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 3.1416],
              velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              time_from_start: {sec: 10}}]}}"
```

## 2. B7.0 — seguridad

### 2.1 Sostén por par y payload

```bash
ros2 launch ur5_dyn_control ur5e_real.launch.py controller:=gravity_comp \
    tool_mounted:=true test_num:=1203
```

- El banner tiene que decir `herramienta: MONTADA`, `tau_max` al 30 % y
  `deadman: si el nodo calla 100 ms, par cero y vuelta al JTC`.
- **No seguir hasta ver en el log `command_deadman ... armado`.** Antes de esa línea no
  hay par aplicado, y las pruebas de 2.2 y 2.4 no probarían nada.
- El brazo se queda quieto en `q_init`. Dejarlo ~20 s.

Seguir con 2.2 **sin parar este sostén**. Al final de la sesión:

```bash
ros2 run ur5_identification check_session_config.py --payload-kg 0.155 \
    ~/.ros/ur5_dyn_control/gravity_comp_1203.csv
```

Esperado: `shoulder_lift` dentro de ±0.5 N·m de la referencia (modelo sin herramienta +
firma del acople, −0.75 N·m). Con payload 0 saldría ~+0.75 fuera. Medido por RTDE el
2026-11-02: 0.155 kg activos, TCP 0.

### 2.2 El nodo muere sin limpiar (`kill -9`) → vigilante

Con el sostén de 2.1 en marcha, **terminal C**:

```bash
pkill -9 -f lib/ur5_dyn_control/gz_gravity_comp_node
```

**Esperado** (medido en Gazebo: 100 ms tras el `kill -9`):

- en el terminal del launch: `DEADMAN: ~100 ms sin comandos. Par cero publicado
  ...` y `controlador de trayectoria activo; el brazo se sostiene en posicion`;
- el brazo **no se mueve** (el JTC lo sostiene en posición);
- `ros2 control list_controllers`: `scaled_joint_trajectory_controller` activo,
  `forward_effort_controller` inactivo.

Si el brazo se mueve de forma apreciable: **paro de emergencia** y avisar. Sin el
vigilante el último par quedaría aplicado indefinidamente (así giró 4.7 rad
`shoulder_pan` en agosto). Después, Ctrl-C en el terminal del launch.

### 2.3 Fin limpio (Ctrl-C)

```bash
ros2 launch ur5_dyn_control ur5e_real.launch.py controller:=gravity_comp \
    tool_mounted:=true test_num:=1201
# ~10 s de sostén y Ctrl-C en ese terminal
```

**Esperado:** el destructor del nodo publica par cero; el brazo queda flotando
compensado por el robot (puede derivar despacio). Devolver el mando al JTC:

```bash
ros2 control switch_controllers --deactivate forward_effort_controller \
    --activate scaled_joint_trajectory_controller
```

### 2.4 `RobotReceiveTimeout` — cae el driver o la red

En ur_robot_driver 2.14 está **fijo en 20 ms** (`hardware_interface.hpp`:
`RobotReceiveTimeout::millisec(20)`), no es un parámetro: no hay nada que
configurar, solo comprobarlo. Protege de la caída del **driver o de la red**, no
de la del nodo de control (eso es 2.2).

```bash
ros2 launch ur5_dyn_control ur5e_real.launch.py controller:=gravity_comp \
    tool_mounted:=true test_num:=1204
```

Con la línea `armado` ya en el log, **desenchufar el cable Ethernet del robot** (es el
fallo real).

**Esperado:** en ~20 ms el programa del robot sale con `stopj`; la pestaña **Registro**
del pendant (no sale como aviso emergente) muestra «Socket timed out waiting for command
on reverse_socket. The script will exit now.» y External Control se detiene; el brazo
frena y queda quieto.

Recuperación: reconectar, Ctrl-C en todos los terminales, relanzar el driver,
*play* en External Control y **G4 a cero otra vez** (§1).

### 2.5 Retardo de tubería

Ya medido con datos existentes: **5 ciclos (10 ms)** (`analyze_pipeline_delay.py`
sobre `fl_500`–`507`; plan V15). Los barridos de B4.4 **no** sirven para volver a medirlo,
como decía antes este protocolo: el limitador de tasa del feedforward deja saltos de
0.3 N·m y el detector necesita 2 N·m (`fl_1101`–`1103`: 0 eventos, correlación 0.08). El
retardo es de la tubería de comunicación, así que el acople no debería cambiarlo: se
mantienen los 5 ciclos.

## 3. B4.4 — α_g

Con el JTC activo (tras 2.4, después de recuperar). El script fija G4 a cero y coloca el
robot; `--resume` salta lo que ya tiene datos y aparta el CSV vacío de `fl_1111`.

```bash
# Posturas, holguras y comandos, sin mover nada:
ros2 run ur5_identification run_gravity_sweeps_real.py --dry-run --test-base 1100

# La sesión (pide «ACEPTO», el payload tecleado y el nombre de cada junta):
ros2 run ur5_identification run_gravity_sweeps_real.py --test-base 1100 \
    --payload-kg 0.155 --resume
```

- 4 posturas (inicio de la aproximación y tres sobre el trazo a 60 mm de la cota
  del corte) × hombro, codo y `wrist_1`: **12 corridas** de ~26 s, ±0.05 rad a
  0.05 y 0.10 rad/s. La punta nunca baja a menos de 71 mm del tablero.
- Entre corridas el script devuelve el mando al JTC y recoloca; aborta si el
  nodo entra en SAFE_HOLD o el brazo no se detiene. `--resume` continúa.
- CT con 100/20 en las juntas grandes y 900/60 en las muñecas: margen ×2.2 con
  los 5 ciclos de retardo (las 7500/173 de `sweep_params.yaml` serían inestables
  en `wrist_2`). El banner de cada corrida tiene que decir `lazo discreto (MIMO)
  con 5 ciclo(s) de retardo ... margen de ganancia x2.19`; desde R10 el nodo no
  arranca en el real si una ley sale inestable con ese retardo (plan §2.7).

Análisis:

```bash
ros2 run ur5_identification analyze_gravity_sweeps.py \
    --out ~/.ros/gravity_sweeps/alpha_g.yaml ~/.ros/ur5_dyn_control/fl_11??.csv
```

Por corrida: `ĝ`, `g_real − ĝ` a cada velocidad (tienen que coincidir), `g_UR − ĝ`,
`Δ = g_real − g_UR` y `α = |Δ|/|ĝ|`; al final `α_g` por junta. El modelo del nodo
sigue con 0.182 kg y el acople real tiene 0.155: `g_real − ĝ` y `g_UR − ĝ` llevan por
eso +0.13–0.19 N·m en hombro, +0.11–0.13 en codo y +0.03 en `wrist_1`, sin que sea un
error. `Δ` no lo nota porque no usa `ĝ`, y `α` cambia un 0.4 %. Verificado en
Gazebo con la fricción real inyectada (`fl_98831`–`98833`): `g_real − ĝ` =
0.004 / 0.002 / 0.0002 N·m en hombro / codo / `wrist_1` (en Gazebo `g_UR = 0` y
α = 1 por construcción: allí solo se prueba el método).

## 4. Qué devolver

Los CSV (`gravity_comp_1203`–`1204`, `fl_1101`–`1133`) con sus `.params.yaml`,
el registro de sesión (`~/.ros/gravity_sweeps/session_*.yaml`), lo observado en
2.2–2.4 (¿se movió el brazo?, mensajes del pendant).
