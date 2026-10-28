# Protocolo de sesión con par en el UR5e real — B7.0 (seguridad) y B4.4 (α_g)

**Estado:** preparado el 2026-10-28, **sin ejecutar**. Plan: B4.4 y B7.0 de
`Journal_UR5eControl_plan.md`. B7.0 va **antes** de B4.4 en la misma sesión:
B4.4 es el primer uso del par con el acople montado.

Números de corrida reservados: `gravity_comp_1200`–`1202` (B7.0) y
`fl_1101`–`1133` (B4.4: `1100 + 10·postura + junta`). Ninguno existe hoy; los
scripts se niegan a sobrescribir un CSV.

---

## 0. Preparación (robot apagado o en paro)

1. **Acople montado SIN la hoja.** Es el primer ensayo con par y acople
   (checklist §7: «primer ensayo sin bisturí»). Mesa despejada: sin muestras ni
   fijación.
2. **Pesar el acople** tal como va montado (con tornillos, sin hoja). El CAD dice
   0.182 kg; se usa el peso medido.
3. **Pendant → Instalación → Carga útil:** masa = la pesada; centro de gravedad
   **(−2.2, 2.1, 29.4) mm** (CAD, desde la brida). **TCP en (0, 0, 0)**. En la
   instalación, no solo en el programa: el payload vuelve solo al recargar
   (`02_friction_real.md` §8.7).

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
    tool_mounted:=true test_num:=1200
```

- El banner tiene que decir `herramienta: MONTADA`, `tau_max` al 30 % y
  `deadman: si el nodo calla 100 ms, par cero y vuelta al JTC`.
- En el log: `command_deadman ... armado` al empezar el sostén.
- El brazo se queda quieto en `q_init`. Dejarlo ~20 s.

Seguir con 2.2 **sin parar este sostén**. Al final de la sesión:

```bash
ros2 run ur5_identification check_session_config.py --payload-kg <masa pesada> \
    ~/.ros/ur5_dyn_control/gravity_comp_1200.csv
```

Esperado: `shoulder_lift` dentro de ±0.5 N·m de la referencia (sin carga +
firma del acople). Con payload 0 saldría ~+0.9 fuera.

### 2.2 El nodo muere sin limpiar (`kill -9`) → vigilante

Con el sostén de 2.1 en marcha, **terminal C**:

```bash
pgrep -af lib/ur5_dyn_control/gz_gravity_comp_node     # anotar el PID del nodo
kill -9 <PID>
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
    tool_mounted:=true test_num:=1202
```

Con el sostén en marcha, **desenchufar el cable Ethernet del robot** (es el fallo
real). Alternativa: `pgrep -af ur_ros2_control_node` y `kill -9 <PID>`.

**Esperado:** en ~20 ms el programa del robot sale con `stopj`; el log del pendant
muestra «Socket timed out waiting for command on reverse_socket. The script will
exit now.» y External Control se detiene; el brazo frena y queda quieto.

Recuperación: reconectar, Ctrl-C en todos los terminales, relanzar el driver,
*play* en External Control y **G4 a cero otra vez** (§1).

### 2.5 Retardo de tubería

Ya medido con datos existentes: **5 ciclos (10 ms)** (`analyze_pipeline_delay.py`
sobre `fl_500`–`507`; plan V15). No hace falta un ensayo propio: los barridos de
B4.4 tienen los mismos escalones de Coulomb y lo re-miden con el acople:

```bash
ros2 run ur5_identification analyze_pipeline_delay.py ~/.ros/ur5_dyn_control/fl_11??.csv
```

Si sale lejos de 5 ciclos (más de ±1), avisar: cambia el evaluador de B5.

## 3. B4.4 — α_g

Robot en `q_init` con el JTC activo (tras 2.4, después de recuperar).

```bash
# Posturas, holguras y comandos, sin mover nada:
ros2 run ur5_identification run_gravity_sweeps_real.py --dry-run --test-base 1100

# La sesión (pide «ACEPTO», el payload tecleado y el nombre de cada junta):
ros2 run ur5_identification run_gravity_sweeps_real.py --test-base 1100 \
    --payload-kg <masa pesada>
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
`Δ = g_real − g_UR` y `α = |Δ|/|ĝ|`; al final `α_g` por junta. Verificado en
Gazebo con la fricción real inyectada (`fl_98831`–`98833`): `g_real − ĝ` =
0.004 / 0.002 / 0.0002 N·m en hombro / codo / `wrist_1` (en Gazebo `g_UR = 0` y
α = 1 por construcción: allí solo se prueba el método).

## 4. Qué devolver

Los CSV (`gravity_comp_1200`–`1202`, `fl_1101`–`1133`) con sus `.params.yaml`,
el registro de sesión (`~/.ros/gravity_sweeps/session_*.yaml`), lo observado en
2.2–2.4 (¿se movió el brazo?, mensajes del pendant) y la masa pesada del acople.
