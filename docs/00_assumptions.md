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
| A1 | Masa del acople del bisturí despreciable | ❌ **LEVANTADO** (2026-08-03): el acople se modela con sus valores de CAD | `tool_mass` / `tool_com` / `tool_inertia` |
| A2 | Punta de la hoja ≡ `gripper_tcp` (**0.162686 m** desde `tool0`) | ✅ Vigente, offset parametrizado y medido sobre la malla | `tcp_offset_z` |
| A3 | Métricas cartesianas en `gripper_tcp` respecto a `base_link` | ✅ Vigente | `Ur5Dynamics::fk()`, columnas `x,y,z` del CSV |
| A4 | Orientación del TCP constante `rpy = [π, 0, π]` | ✅ Vigente | `tcp_orientation_rpy` |

---

## A1 — Se desprecia la masa del acople del bisturí

> ## ❌ A1 LEVANTADO el 2026-08-03
>
> El acople porta-bisturí **ya no se desprecia**: se modela con sus valores de
> CAD tanto en la planta de Gazebo (`urdf/scalpel_tool.xacro`) como en el
> modelo de Pinocchio del controlador (`tool_mass` / `tool_com` /
> `tool_inertia`). El *hook* descrito más abajo es exactamente el mecanismo que
> se ha usado; no hizo falta tocar ninguna ley de control.
>
> | | Valor | Fuente |
> |---|---|---|
> | Masa | **0.182 kg** | `physical_properites_toolscalpel.png` (Inventor) |
> | CoG (desde `tool0`) | (−2.227, 2.134, 29.398) mm | ídem |
> | Inercia en el CoG | Ixx 239.199, Iyy 254.858, Izz 119.620 kg·mm²<br>Ixy −0.880, Ixz −3.820, Iyz −7.570 | ídem |
>
> El `tool_com` del YAML va **en el frame TCP**, o sea el CoG anterior menos
> `[0, 0, tcp_offset_z]` → `(−0.002227, 0.002134, −0.133288)` m.
>
> **Verificado en simulación** (`gravity_comp`, 28 s de sostén en `q_init`):
> error de regulación **0.000000 rad en las seis juntas**. Es la prueba de que
> planta y modelo llevan la MISMA herramienta; si sólo la llevara la planta, la
> muñeca caería.
>
> **Consecuencia que hay que respetar:** los dos lados van juntos. Si alguna vez
> se quita `scalpel_tool.xacro` del URDF hay que poner `tool_mass: 0.0`, y al
> revés. Desparejarlos hace que el controlador compense una gravedad que no es
> la de la planta, y el error resultante se confundiría con error de control.
>
> **Campañas anteriores.** Las FASES 2 y 5 (barridos de fricción, `sign`/`sat`,
> φ, α) se corrieron **sin** herramienta. Sus números siguen siendo válidos para
> lo que medían, pero **no son comparables cuantitativamente** con corridas
> nuevas: la muñeca ahora carga 0.182 kg más.

**Enunciado (histórico).** El modelo dinámico usado por todas las leyes de
control era el del **brazo solo** (`ur5_kinematics/share/ur5e.urdf`, sin gripper
Robotiq y sin herramienta). La masa e inercia del acople impreso del bisturí y
de la propia hoja se despreciaban frente a las de los eslabones del UR5e.

**Por qué era aceptable de partida.** El acople es una pieza pequeña montada en
la muñeca; su contribución a `M(q)` y a `g(q)` es de segundo orden frente a los
~20 N·m de gravedad que ya soportan `shoulder_lift` y `elbow` en la pose de
trabajo. Además, en Gazebo la planta también estaba sin herramienta, así que
**modelo y planta coincidían exactamente** y el supuesto no introducía error.

**Por qué se levanta igualmente.** En el **robot real** (FASE 9) la planta sí
lleva el acople, y dejar el modelo sin él convierte 0.182 kg en una perturbación
no modelada que los controladores robustos (SMC, ASTSMC) absorberían — se
atribuiría a robustez lo que es masa conocida. Con los datos de CAD disponibles
no hay razón para pagar ese precio.

### Hook implementado (requisito del plan: *"dejar el hook listo desde ahora"*)

```yaml
# config/fl_control_params.yaml (y los otros cinco YAML del paquete)
tool_mass: 0.182                      # [kg]
tool_com: [-0.002227, 0.002134, -0.133288]           # [m] en el frame TCP
tool_inertia: [2.39199e-4, 2.54858e-4, 1.19620e-4,   # [kg·m²] respecto al CoM
               -8.80e-7, -3.820e-6, -7.570e-6]       # [Ixx,Iyy,Izz,Ixy,Ixz,Iyz]
```

Con `mass = 0` (el valor histórico) el modelo no se toca y se recupera el
supuesto original bit a bit.

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

### ✅ A1 levantado — lo que arrastra

Los valores salieron del CAD del acople, no de una báscula, y bastó rellenar el
YAML: cero cambios de código. Queda pendiente:

- **Contrastar la masa con una báscula** antes de la FASE 9. 0.182 kg es el
  valor de Inventor, que supone material y relleno homogéneos; una pieza impresa
  real se desvía.
- **Rehacer la identificación de fricción de la FASE 2 con la herramienta
  montada**, porque el residuo `tau_cmd − RNEA(q,q̇,q̈)` cambia. La campaña en el
  robot real todavía no se ha corrido, así que no hay nada que invalidar: sólo
  hay que correrla con el acople puesto, que es como estará el robot.

---

## A2 — La punta de la hoja coincide con `gripper_tcp`

**Enunciado.** El punto controlado es el frame `gripper_tcp`, situado a
**0.162686 m** de `tool0` a lo largo del eje Z local. Ahí está la punta de la
hoja del bisturí.

**Origen del valor (2026-08-03).** Vértice de z máximo de la malla del acople,
`meshes/scalpel_tool/ur5_scalpel_coupling.obj`: **(0.316, 4.280, 162.686) mm**
en el frame del propio modelo. Ese frame es `tool0`, y no por suposición: el
acople tiene un rebaje de Ø63.4 mm cuyo fondo es una cara plana exactamente en
`z = 0` (anillo de radio 5.550–31.700 mm), que es la que apoya contra la brida
Ø63 × 6.5 mm del UR5e. Como el rebaje mide 7.0 mm y el resalte 6.5, el acople
hace tope por esa cara.

> ### ⚠️ Discrepancia de 2.5 mm con el cálculo de `mounting_measurements.png`
>
> Restar los 6.5 mm del resalte a la cota de 171.686 mm da **165.186 mm**, no
> 162.686. La diferencia está en el datum: la punta de flecha de esa cota cae
> **9.0 mm** por detrás de `tool0` (medido sobre la imagen: 2.0 mm por detrás de
> la cara trasera del acople, que está en z = −7), no 6.5 mm.
>
> Se adopta **162.686 mm** porque (a) sale de la malla que efectivamente se
> monta en Gazebo, y (b) es independiente de la altura del resalte de la brida,
> ya que el acople siempre hace tope contra la cara de la brida.
>
> Para volver a 165.186 basta cambiar `scalpel_tcp_offset_z` en
> `urdf/scalpel_tool.xacro` y los `tcp_offset_z` de `config/*.yaml`. **Los dos a
> la vez.**

**Antes de 2026-08-03** el offset era 0.141 m: la misma cadena TCP que define
`ur5_pick_place/urdf/ur5_robotiq_2f85.urdf.xacro` para el gripper Robotiq. Se
conservaba para que trayectorias y cinemática fueran comparables con el trabajo
previo (pick-and-place, CU3). Ese gripper ya no se monta.

**Regla de código (exigida por el plan): no hardcodear el offset en ningún
sitio nuevo.** Cumplida y además reforzada:

- El nodo declara el parámetro **`tcp_offset_z`** (default `0.141`; todos los
  `config/*.yaml` lo fijan explícitamente a `0.162686`).
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

### 🔲 PENDIENTE: contrastar el CAD contra el robot montado

El valor viene del CAD, no de una medida sobre el robot real:

```
Distancia real tool0 -> punta de la hoja: ________ m   (calibrador)
Desviación respecto a 0.162686 m:         ________ m
```

Si la desviación es apreciable frente a la precisión de corte que se quiere
reportar, se actualiza `tcp_offset_z` (y el xacro). Si no, se declara el sesgo
en *Limitations* junto con la repetibilidad nominal del UR5e (±0.03 mm).
Sirve además para zanjar la discrepancia de 2.5 mm de arriba con una medida
directa.

### Holgura contra la mesa de cirugía

La punta es el punto de z **máximo** de la malla, y la orientación del TCP es
constante con `Z_tool = −Z_base` (A4), así que la punta es siempre el punto más
bajo de la herramienta. Medido sobre una corrida completa de la incisión
(`fl_91.csv`, 16 298 muestras): **z mínima de la punta = 0.015000 m**, es decir
**15.00 mm** sobre la mesa, que está en `z = 0` de `base_link`. Sin cambiar la
trayectoria.

Con el offset antiguo de 0.141 m la punta real habría bajado a **−6.7 mm**:
6.7 mm *dentro* de la mesa.

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

## A4 — Orientación del TCP constante `rpy = [π, 0, π]`

> El valor era `[π, 0, −π/2]` hasta que la pose inicial pasó a llevar `wrist_3`
> a π (commit `0ecb25c`). Los YAML llevan `[π, 0, π]` desde entonces; este
> documento se había quedado con el valor viejo.

**Enunciado.** Durante toda la incisión el TCP mantiene orientación fija, con la
herramienta apuntando hacia abajo (−Z del mundo). Por tanto
`ω_des = ω̇_des = 0` en toda la trayectoria.

**Y además orienta el filo.** Con `[π, 0, π]` la rotación del TCP en
`base_link` es `diag(−1, +1, −1)`: el eje **Y de la herramienta coincide con el
+Y de `base_link`**. El filo de la hoja mira hacia −Y de la herramienta
(`urdf/scalpel_tool.xacro`) y el trazo recorre Y con `cut_direction = -1`, o
sea hacia −Y de `base_link`: **el filo va por delante del movimiento y la hoja
corta en su propio plano**, sin componente lateral. No es casualidad que salga
bien, pero tampoco estaba comprobado hasta montar el acople.

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

> The dynamic model used by all controllers is that of the UR5e arm carrying the
> scalpel coupler, whose mass (0.182 kg), centre of gravity and inertia tensor
> are taken from the CAD model of the tool and included in both the simulated
> plant and the controller model (A1). The blade tip is taken to coincide with
> the tool centre point located 0.162686 m from `tool0` along its Z axis, a
> distance measured on the CAD mesh of the coupler rather than calibrated on the
> physical robot (A2); residual tip-offset error is therefore bounded by the
> agreement between the printed part and its CAD model. All Cartesian quantities
> are reported for that TCP frame expressed in the robot `base_link` frame (A3).
> The TCP orientation is held constant at `rpy = [π, 0, π]` throughout the
> incision (A4), which aligns the cutting edge with the direction of travel so
> that the blade cuts in its own plane; the results therefore characterise
> position tracking under torque control and do not demonstrate orientation
> tracking.

*(Redacción preliminar; se ajusta al cerrar la FASE 10.)*

---

## Trazabilidad

| Supuesto | Ficheros |
|---|---|
| A1 | `ur5_dyn_control/include/ur5_dyn_control/ur5_dynamics.hpp` (`ToolInertia`), `src/ur5_dynamics.cpp`, `src/torque_control_node_base.cpp`, `test/test_tool_inertia_hook.cpp`, `config/*_params.yaml` |
| A2 | ídem + `src/joint_reference_generator.cpp`, `urdf/ur5e_effort.urdf.xacro` |
| A3 | `src/ur5_dynamics.cpp` (`fk`), `include/ur5_dyn_control/csv_logger.hpp` |
| A4 | `include/ur5_dyn_control/cartesian_spline_trajectory.hpp`, `src/joint_reference_generator.cpp`, `config/*_params.yaml` |
