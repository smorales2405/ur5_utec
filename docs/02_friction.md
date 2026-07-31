# FASE 2 — Identificación offline de fricción articular

Cierre de la FASE 2 del plan `PLAN_INCISION_UR5e.md`.

- **Fecha:** 2026-07-30
- **Prerrequisitos:** [FASE 0](00_prereqs.md) (G4 en particular) y [FASE 1](01_trajectory.md)
- **Paquete nuevo:** `ur5_identification` (ament_python)

---

## 1. Diseño de la excitación

Barrido de **una junta a velocidad constante**, con las demás sostenidas en
`q_fixed = [0, −π/2, π/2, −π/2, −π/2, 0]` (la configuración indicada por el
usuario), **±45° desde su posición fija**, en **ambos sentidos** y a **8 niveles
de velocidad**.

| | |
|---|---|
| Niveles de \|q̇\| | 0.02, 0.05, 0.10, 0.20, 0.35, 0.50, 0.75, 1.00 rad/s |
| Amplitud | ±45°, salvo los dos niveles más lentos (ver abajo) |
| Perfil | S-curve con jerk acotado (el mismo `ScurveProfile` de la FASE 1) |
| Duración por junta | **273.6 s** |

**Por qué una meseta de velocidad constante.** El identificador necesita tramos
con `q̈ = 0`: allí el residuo de par es fricción pura, sin términos inerciales
que se confundan con ella. El perfil S-curve garantiza esa meseta por
construcción, y el generador la marca en la columna `state` del CSV como
`SWEEP_<v>_POS` / `SWEEP_<v>_NEG` (frente a `SWEEP_RAMP` y `SWEEP_MOVE`), de modo
que el estimador filtra sin recalcular la temporización.

**Amplitud vs. velocidad.** Con `max_sweep_duration = 40 s`, un nivel lento que
no quepa **reduce su amplitud, nunca su velocidad** — la velocidad es la variable
independiente del ajuste y recortarla destruiría el diseño experimental. Efecto:

| v [rad/s] | amplitud efectiva | meseta útil |
|---|---|---|
| 0.02 | ±17.6° | 21.5 s |
| 0.05 | ±44.1° | 21.5 s |
| ≥ 0.10 | **±45.0°** | 11.0 s → 1.1 s |

Confirmado por el usuario: a 0.02 rad/s un barrido de ±45° tardaría 204 s y la
campaña por junta pasaría de ~4.5 a ~13 min.

Un nivel cuya rampa exigiría más de `ddq_max` se **rechaza en construcción**, no
se recorta en silencio. Con estos valores el pico es 3.98 rad/s² (límite
declarado 5.0, ver FASE 1).

---

## 2. Inyección de fricción conocida en Gazebo

`ur_macro.xacro` emite `<dynamics damping="0" friction="0"/>` en las 6 juntas y
no expone parámetros. El launch parchea el URDF **tras** ejecutar xacro:

```bash
ros2 launch ur5_dyn_control fl_control.launch.py \
    joint_damping:=1.5 joint_friction:=2.5     # escalar o 6 valores
```

Con `0` (default) el URDF queda intacto y las FASES 0–1 no cambian.

> ### ⚠️ Trampa: hay que pasar por `shlex.split`
>
> `Command` (la substitución de launch) hace `shlex.split` internamente. Al
> generar el URDF con `subprocess` hay que hacerlo también: sin ello,
> `tf_prefix:=""` llega a xacro con las **comillas literales** y todas las juntas
> salen nombradas `""shoulder_pan_joint`. El robot spawnea igual, pero el
> `controller_manager` no encuentra `shoulder_pan_joint/effort` y el switch se
> rechaza con *"Not acceptable command interfaces combination"*.
>
> Se aisló comprobando que `damping=1e-6` —físicamente nulo— fallaba idéntico:
> no era la física, era el tipado de los argumentos.

---

## 3. Estimador

`tau_residual = tau_cmd − RNEA(q, q̇, q̈)` con Pinocchio. Tres cuidados que el
plan exige:

1. **`q̈` filtrado con fase cero.** El CSV no registra aceleración medida, así
   que se deriva de `q̇`. Se usa `filtfilt` (Butterworth de 4.º orden, corte
   10 Hz configurable): un filtro causal metería un retardo que se traduce en un
   sesgo **dependiente de la velocidad**, justo la variable independiente del
   ajuste.
2. **Descarte de tramos de aceleración**: solo las mesetas, y recortando un 10 %
   en cada extremo para que el lazo cerrado asiente tras la rampa.
3. **El par es el COMANDO, nunca el campo `effort`** — en el UR5e real ese campo
   son corrientes de motor (compuerta G5).

### Decisión metodológica: la unidad de observación es la meseta

Dentro de una meseta hay miles de muestras a 500 Hz, pero **no son
observaciones independientes**: es la misma condición experimental repetida, con
residuos fuertemente autocorrelacionados. Ajustar sobre las muestras crudas daría
bandas de confianza absurdamente estrechas (n ≈ 5·10⁴ cuando la información real
es n = 16). Por eso cada meseta se agrega a **una** observación y el ajuste, la
covarianza y los IC se calculan sobre esas 16. La dispersión intra-meseta se
reporta aparte, como medida de ruido, no como grados de libertad.

### Validación cruzada: se retira un NIVEL DE VELOCIDAD completo

Dejar fuera mesetas sueltas sería optimista — el sentido opuesto del mismo nivel
lleva casi la misma información. Retirando el nivel entero se mide de verdad si
el modelo **extrapola a velocidades no vistas**, que es lo que pide el plan.

### Limpieza del eje temporal

Un 0.026 % de las filas comparten instante (el reloj de simulación de 1 ms no
avanzó entre dos ticks del lazo de pared). `np.gradient` divide por cero ahí y el
NaN se propaga por `filtfilt` contaminando **toda** la serie. Esas filas se
descartan y se reporta cuántas.

---

## 4. Resultados

### Validación contra la verdad inyectada (criterio del plan: ≤ 10 %)

Junta 0 (`shoulder_pan_joint`), 16 mesetas, 53 987 muestras útiles:

| | Verdad | Identificado | Error | IC 95 % |
|---|---|---|---|---|
| `F_v` [N·m·s/rad] | 1.5 | **1.4982** | **0.12 %** | [1.4971, 1.4993] |
| `F_c` [N·m] | 2.5 | **2.5038** | **0.15 %** | [2.5033, 2.5044] |

`R² = 1.0000`, `RMSE = 0.0007 N·m`; validación cruzada `R² = 1.0000`,
`RMSE = 0.0010 N·m`. Ruido intra-meseta σ = 0.0198 N·m. ✅ **CUMPLE**

El modelo **solo viscoso** valida claramente peor sobre los mismos datos
(`R²_cv = 0.64`, `RMSE_cv = 1.85 N·m` frente a 0.0010): la validación cruzada
discrimina el modelo, no premia cualquier ajuste.

### Control negativo (planta sin fricción)

| | Identificado |
|---|---|
| `F_v` | −0.0001 ± 0.0001 |
| `F_c` | +0.0000 ± 0.0000 |

El residuo es ~0 en los 16 niveles. Esto verifica que **no hay sesgo sistemático
en el cálculo del residuo**: el modelo Pinocchio y la planta de Gazebo coinciden
a nivel de 0.1 mN·m. Sin este control, un error de modelado se habría
confundido con fricción.

> El `R²` sale negativo aquí y eso es lo correcto: cuando la señal verdadera es
> cero no hay nada que explicar y el `R²` deja de tener sentido. Solo es
> interpretable cuando hay fricción real.

### Efecto de la compensación (criterio del plan: mejora medible)

Misma planta con fricción inyectada, misma trayectoria, con y sin
`friction_compensation: viscous_coulomb` usando los coeficientes identificados:

| v [rad/s] | sin compensar | con compensar | reducción |
|---|---|---|---|
| ±0.020 | 0.0239 rad | 0.000012 rad | ~2000× |
| ±0.100 | 0.0251 rad | 0.000033 rad | ~900× |
| ±0.500 | 0.0307 rad | 0.000316 rad | ~120× |
| ±1.000 | 0.0362 rad | 0.000156 rad | ~600× |

**RMS global de la fase de barrido: 0.024228 → 0.002482 rad, una reducción de
9.8× (89.8 %).** Máximo: 0.0514 → 0.0214 rad. ✅ **CUMPLE**

El RMS global mejora menos que las mesetas porque incluye rampas e inversiones
de sentido, donde la compensación es menos efectiva (ver §5).

---

## 5. Limitación declarada: la fricción estática no se compensa

El término de Coulomb usa `tanh(q̇/eps)` en vez de `sgn(q̇)`. Un escalón
discontinuo en `q̇ = 0` haría saltar el comando ±F_c cada vez que el ruido de
velocidad cruza cero, metiendo un ciclo límite.

**Consecuencia física:** cerca de `q̇ = 0` la compensación tiende a cero, así que
**no cancela la fricción estática**. Eso es correcto —un modelo dependiente de
la velocidad no puede hacerlo— pero hay que tenerlo presente:

- el error residual máximo (0.021 rad) ocurre en las **inversiones de sentido**;
- el criterio del plan *"`gravity_comp` regula sin deriva"* no mejora con esta
  compensación, porque en reposo `q̇ ≈ 0`. Para el error de regulación en reposo
  hace falta un término integral, que es entregable de fases posteriores;
- en el tramo de corte (feed constante de 10 mm/s, sin inversiones) la
  compensación opera en su régimen favorable.

---

## 6. Uso

```bash
# 1. Campaña de excitación (una corrida por junta; sweep.joint de 0 a 5)
ros2 launch ur5_dyn_control fl_control.launch.py gazebo_gui:=false \
    params_file:=$(ros2 pkg prefix ur5_dyn_control)/share/ur5_dyn_control/config/sweep_params.yaml \
    test_num:=200 world:=<...>/empty_test_world.sdf

# 2. Identificación (--truth solo para validar en Gazebo)
ros2 run ur5_identification run_identification \
    --csv ~/.ros/ur5_dyn_control/fl_200.csv \
    --models viscous viscous_coulomb stribeck \
    --out ~/.ros/ur5_dyn_control/friction_identified.yaml

# 3. Compensar con lo identificado
ros2 launch ur5_dyn_control fl_control.launch.py \
    friction_compensation:=viscous_coulomb \
    friction_f_v:="1.4982,0,0,0,0,0" friction_f_c:="2.5038,0,0,0,0,0"
```

El YAML de salida trae `friction.f_v` y `friction.f_c` como listas de 6, en el
orden canónico, consumibles directamente por `ur5_dyn_control`. Las juntas sin
identificar quedan a 0, que equivale a no compensar.

---

## 7. Pendiente antes de cerrar la fase del todo

| Qué | Por qué |
|---|---|
| **Campaña de las 6 juntas** | Solo se identificó la junta 0. Es repetir el paso 1 con `sweep.joint` de 1 a 5 (~28 min de simulación en total). El pipeline ya está validado. |
| **Campaña en el robot real** | Requiere el ajuste de fricción de G4 (`1.0/1.0`) fijado por servicio y **registrado**; criterio del plan: `R² > 0.9` por junta en validación. Bloqueado por la FASE 3 (watchdog). |
| **Barrido de 3 niveles del `friction_model_controller`** | `{0.0, default, 1.0}` según lo acordado en G4, para acotar si `1.0` sobre-compensa. |
| **Figura de Stribeck** | El modelo está implementado y testeado, pero en Gazebo no hay efecto Stribeck que mostrar (la planta es viscoso + Coulomb puro). La figura sale de los datos reales. |

---

## 8. Archivos

**Paquete nuevo `ur5_identification`**: `friction_models.py` (modelos y
regresores lineales), `residual.py` (RNEA + filtrado + ventanas), `estimator.py`
(LS, validación cruzada, bandas), `run_identification.py` (entry point),
`test/test_estimator.py` (10 tests sobre datos sintéticos).

**En `ur5_dyn_control`**: `joint_sweep_generator.{hpp,cpp}`,
`joint_reference_table.hpp` (base común de los generadores),
`config/sweep_params.yaml`, `friction_compensation` en `torque_command.hpp` y
`torque_control_node_base.{hpp,cpp}`, inyección de fricción en
`ur5e_effort_gz.launch.py` y overrides en `fl_control.launch.py`.

**Tests:** 64 en total (50 gtest en `ur5_dyn_control`, incluidos 6 nuevos de
compensación de fricción, + 10 pytest en `ur5_identification`).
