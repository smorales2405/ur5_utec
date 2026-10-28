# Plan actualizado — Incisión con UR5e por par: LQR-SDRE vs SMC optimizado

- **Fecha:** 2026-10-28 · **Revisión 8** — retardo de tubería del real medido
  (**5 ciclos**, §2.7, V15); B4.4 y B7.0 preparados (protocolo en
  `docs/09a_session_protocol.md`); B4.4 corregido (qué gravedad entra en el
  lazo); payload con acople; `RobotReceiveTimeout` aclarado. Rev. 7: B0 cerrado
  (§2.6), rondas de respuestas (§9), escena real (§2.5)
- **Sustituye como hoja de ruta a:** `Documents/UTEC/Journal Q2/PLAN_INCISION_UR5e.md`
  (se conserva como histórico; la numeración de FASES original se cita para
  trazabilidad con `docs/00`–`09` y con los commits).
- **Base revisada:** commit `7e4526c` + cambios sin commitear (§2.3).
- **Alcance de la revisión:** `ur5_dyn_control` (código, YAML, launch, tests),
  `ur5_identification/scripts`, `ur5_trajectory_optimization/gain_tuning`,
  `docs/00`–`09`, los CSV de `~/.ros/ur5_dyn_control/` y las fotos de las
  muestras (`Documents/UTEC/Journal Q2/images`).
- **Fecha objetivo de envío:** fin de 2026 → **~9 semanas** (calendario en §10).

---

## 0. Objetivo, decisiones y cambios de alcance

**Objetivo principal.** Comparar el desempeño de un **LQR-SDRE** frente a un
**SMC optimizado** en la tarea de incisión con un bisturí montado en el UR5e,
con el **Computed Torque (CT)** como línea base natural.

| # | Entregable | Qué significa operativamente |
|---|---|---|
| E1 | Gemelo de simulación en Gazebo | Planta lo más parecida posible al robot real, con parámetros **obtenidos por ensayo** (fricción, herramienta, sensado, contacto/corte), de modo que la diferencia sim↔real sea mínima y **medida** |
| E2 | Formulación de las leyes | CT, LQR-SDRE y SMC escritas tal como van al paper, con su prueba de estabilidad y su restricción de implementación discreta |
| E3 | Simulación de la incisión | Incisión **con tejido** en Gazebo con el controlador seleccionado, como predicción verificable de la implementación real |
| E4 | Comparación en 4 materiales | Carne de cerdo (panceta) · espuma EVA (tabla de natación) · Ecoflex 00-30 · zapallo |

### 0.1 Decisiones tomadas (respuestas del usuario, 2026-10-27)

| # | decisión | consecuencia en el plan |
|---|---|---|
| D1 | La mesa real queda en `base_y ≈ 0.85`; **se acercará la base del robot a la mesa**, sin que el mounting base choque con la mesa | Con la escena real y la holgura medida (D16) el borde de la mesa ya está a 0.426 m: **no hace falta mover el robot**. `cut_center_y` = **0.65 m** (V12, V13); se revierte el 0.85 del YAML (B0.0) |
| D2 | Leyes: **CT-PD** (línea base), **LQR-SDRE con integral y pesos físicos**, **SMC `sat`** con R1–R3 | §4. `sign` queda como resultado de simulación de la FASE 5; ASTSMC fuera (B9) |
| D3 | **Un juego de ganancias por ley** para los 4 materiales | B5: el material es desconocido para el controlador |
| D4 | Solo el sensor F/T **integrado** del UR5e | §3.3: promediado en meseta + estimador por corriente |
| D5 | **Una muestra por material** (dimensiones por confirmar; fotos en `images/`) | §3.5: carriles de corte, fijación indexada, sesiones únicas para perecibles |
| D6 | Medición con **cámara calibrada** junto a la mesa | B7.4: vista superior por corte + sección transversal final para la profundidad |
| D7 | Criterios sim↔real de §5.3 aceptados, ajustables a lo alcanzable en el real | §5.3 |
| D8 | Envío a **fin de 2026** | §10. Alcance recortado: ASTSMC fuera, N limitado por la muestra |
| D9 | Hay **pieza aparte** de panceta y de zapallo para los pilotos | B2 no consume carriles de la muestra de campaña en esos dos materiales |
| D10 | Dimensiones (largo × ancho × espesor): **panceta 200 × 50 × 40 mm**, **EVA 400 × 250 × 40 mm**; Ecoflex y zapallo por confirmar | §3.5 a: la EVA sobra; la panceta de 50 mm de ancho no admitía 9 carriles → D14 |
| D11 | Panceta **con la piel arriba**; panceta y zapallo **recortados a cara superior plana** | §3.5 c |
| D12 | **Sección transversal destructiva** al final de cada material para medir la profundidad | B7.4 |
| D13 | Se puede fabricar una **fijación indexada** | §3.5 b: la trayectoria es la misma en todos los cortes |
| D14 | Panceta en **bloque de al menos 150 × 130 mm** de cara útil, de la misma pieza | §3.5 a: 9 carriles con el trazo de 100 mm |
| D15 | Los modelos de pedestal y mesa del mundo **estaban desactualizados**; se sustituyen por los medidos sobre los reales (`ur5_base_table_dimensions.pdf` + OBJ del CAD) | §2.5 y V13: pedestal de 0.816 m (spawn a `z` = 0.816), tablero a `z` = −0.046 en `base_link`; hecho antes de B0 |
| D16 | El robot está **centrado** sobre el mounting base; la cara del mounting base que mira a la mesa está a **5 cm** de ella (modificable si conviene) | Borde interior del fondo de la U a `base_y` = 0.376 + 0.050 = **0.426 m**; así está en `lab_incision_world.sdf` (V13) |
| D17 | Bandeja de la fijación de **2 mm** | Altura común `surface_z` = −0.046 + 0.002 + 0.040 = **−0.004 m** (§3.5 c) |
| D18 | La dirección del corte la fija el **filo**: con `wrist_3` en π el filo mira a la base, así que el corte va de lejos a cerca de la base | Verificado (malla + FK): filo y avance alineados a 0.000°. Recta del corte en `x` = −0.1333 = −d4: `wrist_3` = π y la base **quieta** en todo el corte (V13, §2.6). **Confirmado** por el usuario |
| D19 | El mounting base está **centrado lateralmente** en la abertura de la U | La escena (`u_table` en `y` = 0) coincide con el laboratorio |
| D20 | Pinocchio 4.1 (actualizado desde 3.1 hace unos meses) se **retiene** | Hecho: `apt-mark hold` de `robotpkg-{,py310-}pinocchio` 4.1.0 y de lo que lleva compilado dentro, `robotpkg-{,py310-}coal` 3.0.4, `robotpkg-py310-eigenpy` 3.13.0, `robotpkg-{,py310-}casadi` 3.7.2. Para actualizar: `sudo apt-mark unhold <paquete>` y re-verificar (test cruzado de B0.4) |
| D21 | Muestras aún sin comprar: el usuario las conseguirá según las **dimensiones recomendadas** | §3.5 (h) |
| D22 | **Trabajo futuro**, tras el corte recto: corte **curvo** con la hoja tangente a la trayectoria | B10 |

### 0.2 Cambios respecto al plan original

| Plan original | Este plan |
|---|---|
| 4 controladores (CT, LQR-SDRE, OSMC, ASTSMC) + SMC-`sign` | **CT, LQR-SDRE, SMC optimizado (`sat`)** |
| Gazebo sin tejido; `F_ext` sintético en el evaluador | **Modelo de corte por material**, identificado en el robot real y llevado a Gazebo y al evaluador (B2–B3) |
| Sintonía multiobjetivo solo del SMC | **Misma sintonía, mismas restricciones y mismo presupuesto** para los tres (B5) |
| LQR-SDRE con `Q ∝ M²` | **Reformulado** (§4.3): la versión actual es CT con otra ganancia |
| 5 controladores × 4 materiales × N ≥ 10 | 3 controladores × 4 materiales × **N limitado por los carriles de una muestra** (objetivo N = 3, §3.5) |

**Reglas de trabajo que se mantienen:** un commit por bloque; build + tests
verdes antes de cerrar; resumen al cierre; **no se avanza de bloque sin
confirmación explícita**; no se inventan parámetros físicos (se marcan `TODO` o
se miden); no se borran CSV de campañas; el mismo nodo corre en sim y en real
cambiando solo parámetros.

---

## 1. Resumen ejecutivo

**Lo que está sólido y no hay que rehacer**

- Infraestructura de par: G3 (gravedad fuera del comando, testeada), watchdog,
  `SAFE_HOLD`, guardas de seguimiento y de vibración (validadas en Gazebo,
  `smc_730/731`), límite de tasa, `onSaturation()`, `dry_run`, CSV unificado.
- Trayectoria de incisión quíntica con feed constante e IK refinada (FASE 1).
- Fricción **física** de las 6 juntas medida en el robot con `k` por diferencia
  entre sentidos (dispersión ≤ 0.24 %), en régimen térmico (`02_friction_real`
  §8.4–8.5). Ajuste interno G4 = 0.0. Es propiedad de las juntas: **no cambia
  al reubicar el robot**.
- SMC: implementado, sintonizado (`smc_v5_g6`) y verificado en Gazebo con la
  fricción real (TCP 0.37 mm en aire). Límite real medido: ganancia derivativa
  efectiva `G ≤ 120 N·m/(rad/s)` (cuantización + modo estructural de 25–35 Hz).
- **151 resultados de test, 0 fallos** (V1).

**Lo que hoy impide cumplir el objetivo (por orden de gravedad)**

1. **La comparación todavía no discrimina nada.** En aire los tres controladores
   dan ~0.04 mm; bajo la fuerza de corte sintética de 5 N, CT y LQR-SDRE dan
   **~228 mm** y el SMC **1.76 mm** (V3). El LQR-SDRE actual (`Q ∝ M²`) es, por
   construcción, un CT con `kp = ωn²`, `kd = 2ζωn` (§4.3).
2. **Gazebo no tiene tejido y no hay ninguna fuerza de corte medida**: el
   suscriptor a `ft_data` no existe (V7) y solo se dispone del sensor integrado.
3. **La cadena referencia → evaluador → Gazebo → robot no es coherente hoy**
   (V4–V6), y la escena va a cambiar con la reubicación del robot (D1).
4. **CT y LQR-SDRE nunca se han validado en la configuración actual** (V9) ni en
   el robot; la incisión del SMC en el robot tampoco (V10).
5. **Una sola muestra por material** fija el número de cortes posibles. La
   panceta irá en bloque (D14); falta la capacidad de Ecoflex y zapallo antes de
   sintonizar (§9.7 R2).

**Ruta crítica** (calendario en §10)

```
B0 reubicación + coherencia ─┬─> B4 leyes reformuladas ───────────────┐
                             ├─> B1 ft_data + estimador de fuerza ────┤
                             └─> B2 cortes piloto en POSICIÓN ──> B3 gemelo con tejido ──> B5 sintonía equitativa
                                  (robot real, sin par)                                         │
        B7.1 bring-up en aire en la escena nueva (en paralelo) ────────────────────────────────┤
                                                              B6 campaña Gazebo <───────────────┘
                                                                   │
                                                              B7 campaña real ──> B8 análisis/paper
```

---

## 2. Estado verificado

### 2.1 Estado por fase (numeración del plan original)

| FASE | Estado | Evidencia | Validez hoy / salvedades |
|---|---|---|---|
| 0 Compuertas | ✅ | `00_prereqs.md` | G6 parcial: `RobotReceiveTimeout` es fijo (20 ms, driver 2.14) y solo cubre la caída del driver o de la red; la muerte del nodo la cubre ahora `command_deadman.py` (B7.0). Las dos pruebas en el real, preparadas y sin ejecutar (`09a_session_protocol.md` §2). Pinocchio 4.1.0 (cambió de 3.9.0 el 2026-09-01). **Los planos de seguridad del pendant hay que rehacerlos tras la reubicación.** |
| 1 Trayectoria | ✅ | `01_trajectory.md`, 26 tests | **Doc desactualizado**: describe el trazo en `x = 0.50`, superficie 0.02, σ_min 0.2168, 27.60 s. Hoy: trazo en `base_y` 0.80→0.70 con `x = 0`, superficie 0.03, TCP 0.162686, **σ_min 0.155**, 29.43 s (V6). |
| 2 Fricción (Gazebo) | ✅ | `02_friction.md` | Validación del 0.12 % solo en `shoulder_pan` (inmune al artefacto de retardo). |
| 2 Fricción (real) | ✅ | `02_friction_real.md` | Pendientes: residual en muñecas, stiction/Stribeck (< 0.02 rad/s), validar la inyección en `wrist_1`/`wrist_2` de Gazebo. |
| 3 Infraestructura | ✅ salvo `ft_data` | README, tests | No hay suscriptor `ft_data` (V7). No existe `docs/03_*.md`. |
| 4 LQR-SDRE | ⚠️ implementado, **obsoleto** | `04_lqr_sdre.md`, `lqr_10–12` | Validado solo en la configuración del 31-jul (V9). Formulación equivalente a CT (V3). Nunca sintonizado ni llevado al real. |
| 5 SMC | ✅ en Gazebo | `05_smc.md` | `sign`/`sat`, φ, α medidos **sin herramienta**. No existe el dataset "en los 4 materiales" (no hay tejido). La prueba de estabilidad del nodo tiene un paso incorrecto (V11). |
| 6 ASTSMC | ⏸ no iniciado | — | Fuera del alcance de esta fecha (B9). |
| 7 Sintonía | ⚠️ solo SMC | `07_gain_tuning.md`, `smc_v5_g6` | Evaluador con desalineaciones (V5). ε-restricción 0/12 factibles en 18 variables. CT y LQR sin sintonizar. Todo depende de la escena: **se rehace tras B0.0**. |
| 8 Campaña Gazebo | ❌ | — | Sin runner, sin mecanismo de desajuste, sin tejido. |
| 9 Campaña real | 🔶 puesta a punto del SMC | `09_real_bringup.md` | Barridos 710–723, incidentes 710/712, guardas. Incisión real no ejecutada. CT/LQR nunca con par en el real (salvo FL en barridos de fricción). |
| 10 Análisis | ❌ | — | — |

### 2.2 Verificaciones realizadas en esta revisión (2026-10-27)

Todas reproducibles; el método está en el Anexo A.

**V1 — Tests.** `colcon test` sobre `ur5_dyn_control`, `ur5_identification` y
`ur5_trajectory_optimization`: **151 resultados, 0 errores, 0 fallos** (80 gtest
+ 15 pytest de launch + 19 + 30 pytest + 7 envoltorios CTest). El README sigue
diciendo 72/109: desactualizado.

**V2 — Ganancia derivativa efectiva `G_i = −∂τ_i/∂q̇_i`** (máximo sobre
`incision_ref_v2`, modelo con el acople):

| ley | pan | lift | codo | w1 | w2 | w3 |
|---|---|---|---|---|---|---|
| CT (`kp=100, kd=20`): `M_ii·kd` | 66.9 | 72.8 | 18.8 | 0.6 | 0.2 | 0.0 |
| LQR-SDRE `fixed` (ωn=20, ζ=1): `(K_D)_ii` | 44.5 | 106.8 | 38.4 | 1.6 | 1.2 | 0.0 |
| LQR-SDRE `scheduled` | **133.8** | **145.7** | 37.8 | 1.2 | 0.4 | 0.0 |
| SMC `smc_v5_g6`: `M_ii λ_i + K_i/φ_i` | 104.8 | **122.1** | 81.4 | 4.5 | 2.2 | 0.0 |

- La cota medida `G ≤ 120` (`09` §6.8) **nunca se aplicó a CT ni a LQR**. El LQR
  `scheduled` la viola en base y hombro; el SMC v5 la excede un 1.8 % en el
  hombro con el bisturí (ya anotado en `07` §5.8.3).
- **La gravedad infla `G` del SMC.** La parte `α·|ĝ|` de `K` vale hasta
  15.6 N·m en `shoulder_lift` y 6.3 en el codo, es decir `K/φ` = 68.9 y 27.2:
  **el 56 % de la `G` del hombro y el 33 % de la del codo vienen de tratar la
  gravedad como incierta al 30 %**, cuando la discrepancia medida entre el
  modelo de UR y el nuestro es del orden del 5–7 % (`02_friction_real` §8.2–8.3).
  Base de la reformulación R2 (§4.4).

**V3 — CT vs LQR-SDRE vs SMC bajo la fuerza de corte** (evaluador
`closed_loop.simulate` tal como lo usa el optimizador: retardo de 1 paso, ruido
de Gazebo, sin fricción de planta; TCP en la punta y ventanas alineadas con
`incision_ref_v2`):

| ley | aire: RMSE TCP meseta | corte 5 N: RMSE TCP meseta | corte 5 N: RMSE q |
|---|---|---|---|
| CT (`kp=100, kd=20`) | 0.043 mm | **228.3 mm** | 0.459 rad |
| LQR-SDRE `fixed` (ωn=20, ζ=1) | 0.039 mm | **227.5 mm** | 0.877 rad |
| SMC `smc_v5_g6` | 0.037 mm | **1.755 mm** | 0.0020 rad |

- En aire —que es todo lo que Gazebo puede simular hoy— **los tres son
  indistinguibles**.
- Bajo carga, CT y LQR-SDRE fallan igual: ambos tienen rigidez `∝ M_ii`, y en
  las muñecas (`M_55 ≈ 0.0054 kg·m²`) eso son ~0.5–2 N·m/rad frente a ~1 N·m de
  momento de corte. El SMC aguanta porque su `η` por junta **no** escala con la
  inercia (restricción `g5`), que es exactamente la lección de la FASE 7 §3.
- Con el modelo de fricción de planta del evaluador activado, **los tres fallan
  incluso en aire** (70–447 mm), mientras Gazebo con la misma fricción da
  0.37 mm (`smc_604`): confirma que ese modelo no valida (commit `408581a`) y
  que cualquier conclusión con fricción debe salir de Gazebo.

**V4 — El cambio sin commitear `cut_center_y: 0.75 → 0.85`** (`smc_params.yaml`).
`dry_run` del nodo con ambos valores:

| | 0.75 (commiteado) | 0.85 (sin commitear) |
|---|---|---|
| duración / muestras | 29.43 s / 14 717 | 30.93 s / 15 465 |
| σ_min(J) (umbral 0.05) | 0.1550 | **0.0513** — casi singular |
| w_min | 0.0819 | 0.0476 |
| `G` máx (pan, lift, codo) | 104.8, 122.1, 81.4 | **135.5, 142.2**, 81.5 |

`smc_605` (Gazebo, **2026-10-27 de 12:01 a 13:45**; la cabecera dice
`git a6381f2`, pero es el SHA rancio de configure-time que B0.5 corrigió: el HEAD
era `7e4526c`) corrió esa geometría: **RMSE de TCP 132.5 mm** en TRACK, frente a
0.343 mm de `smc_604` con 0.75. (La Rev. 1 la fechaba el 1-oct por error.) Las demás YAML (FL, LQR, `sweep_smc`) siguen en 0.75.

**V5 — Desalineaciones del evaluador offline (FASE 7).**

- `CuttingForce` y `Reference.cut_plateau_mask()` tienen **cosidos** los tiempos
  de la trayectoria de julio (penetración 11.60–13.60, corte 13.90–25.90, meseta
  15.90–23.90 s). En `incision_ref_v2` la penetración termina en **15.336 s**:
  desfase **+1.736 s**. La fuerza sintética empieza con la hoja aún 8 mm por
  encima de la superficie (z 38→30 mm) y la ventana de `g4` incluye 1.7 s de la
  rampa de aceleración y pierde los 1.7 s finales de la meseta. (El commit
  `8056f04` corrigió esto en `analyze_smc.py`, no en el evaluador.)
- `Plant(tcp_offset=0.141)` por defecto: fuerza y error de TCP en un punto
  21.7 mm por encima de la punta; **sin la inercia del acople**; `Q_INIT` es el
  antiguo `[0, …, 0]`.
- Efecto medido sobre `smc_v5_g6`: 1.755 mm con todo alineado frente a los
  1.877 mm publicados. Pequeño, pero todas las cifras de la FASE 7 deben
  recalcularse antes de ir al paper.

**V6 — Los controladores no ejecutan la misma referencia.**
`incision_params.yaml` (FL) usa `start_pose` z = 0.3252 (= FK(q_init) con el
TCP nuevo); `smc_params`, `lqr_sdre_params` y `sweep_smc_params` conservan
0.3469 (valor con el TCP antiguo). Resultado: FL 14 580 muestras (29.16 s) que
arrancan en `q_init`; SMC/LQR 14 717 (29.43 s) que arrancan 21.7 mm más arriba.
**Fases desplazadas 0.27 s entre controladores.** Además la sección `friction:`
solo existe en las YAML del SMC: FL y LQR no compensan fricción salvo por
argumento de launch.

**V7 — `ft_data` no se registra.** `last_wrench_` se declara y se vuelca al CSV
pero ninguna suscripción lo escribe: la columna `wrench` es 0 también en el robot.

**V8 — Trazabilidad incompleta.** La cabecera del CSV lleva `git_sha` y
`params_hash`, pero no los valores ni un indicador de árbol sucio. `smc_605`
corrió con un YAML sin commitear: su SHA no reproduce la corrida y el hash no se
puede invertir.

**V9 — LQR-SDRE obsoleto.** `lqr_10–12` (2026-07-31) son anteriores al acople
(08-03), al cambio de `q_init` (08-01), a `surface_z = 0.03` (08-03) y a la
fricción real. Nada de `04_lqr_sdre.md` es comparable con la configuración
actual, incluidos los tiempos de cómputo de la CARE.

**V10 — Seguridad / real.** `RobotReceiveTimeout` sin verificar (aclarado en
B7.0: fijo en 20 ms, y no protege si muere el nodo; §2.7). Pasos 740–742
(SMC v5 en aire en el robot) sin ejecutar (no hay CSV). La base solo está
probada en el real hasta `G = 30`, y las ganancias v5 le piden 105 (`09` §7.1).

**V11 — Prueba de estabilidad del SMC.** La cabecera de `gz_smc_control_node.cpp`
afirma `ṡ = −M⁻¹K ρ(s)  ⟹  sᵀṡ < 0`. **No es cierto** con `M` acoplada y `K` no
uniforme (contraejemplo: `M⁻¹ = [[1, .9],[.9, 1]]`, `K = (1, 10)`,
`s = (1, −0.01)` da `sᵀṡ > 0`), y es justo lo que se vio en `05` §6.4 (juntas
4–5 alejándose de la superficie antes de converger). Con la ley actual
(`b̂ = Ĉq̇ + ĝ`) y `V = ½ sᵀMs` queda `V̇ = −sᵀKρ + ½ sᵀṀs`, negativa solo
localmente. La forma de Slotine–Li (`Ĉ q̇_r`) da `V̇ = −sᵀKρ` exacta (R1, §4.4).

**V12 — ¿Cuánto acercar el robot a la mesa?** `dry_run` del SMC (`smc_v5_g6`,
trazo de 100 mm en `base_y`, `x = 0`) barriendo el centro del corte:

| `cut_center_y` [m] | σ_min(J) | w_min | `G` pan | `G` lift | `G` codo |
|---|---|---|---|---|---|
| 0.50 | 0.2106 † | **0.0557** | 47.9 | 77.9 | 81.4 |
| 0.55 | 0.2106 † | 0.0687 | 57.2 | 86.4 | 81.4 |
| **0.60** | 0.2106 † | **0.0818** | 67.5 | 95.0 | 81.4 |
| **0.65** | **0.2030** | **0.0819** | 78.9 | 103.8 | 81.4 |
| 0.70 | 0.1821 | 0.0819 | 91.3 | 112.8 | 81.4 |
| 0.75 | 0.1550 | 0.0819 | 104.8 | 122.1 | 81.4 |
| 0.85 | **0.0513** | 0.0476 | **135.5** | **142.2** | 81.5 |

† el mínimo está en la pose de salida (t = 0), no en el corte.

**Recomendación: centro del corte a 0.60–0.65 m del eje de la base**, es decir
acercar el robot ~0.20–0.25 m hacia la mesa. Ahí la manipulabilidad es máxima
(w_min 0.082), σ_min ≥ 0.20 y la `G` de la base baja de 105 a 68–79 con las
mismas ganancias: el brazo menos extendido tiene menos inercia en la base, y ese
margen frente a `G ≤ 120` es precisión que el optimizador podrá comprar con `λ`.
Por debajo de 0.55 el brazo se pliega y w_min cae.

**V13 — Escena real: cuánto se puede acercar sin colisión, y la geometría
resultante** (rehecho en la Rev. 4 con los modelos reales, D15). Cotas de
`ur5_base_table_dimensions.pdf` y de los OBJ del CAD; `base_y = world_x` por el
giro de −90° del robot:

| elemento | escena anterior (desactualizada) | **escena real** |
|---|---|---|
| pedestal | ±0.45 m, alto 0.63 m | **±0.376 m (752 mm), alto 0.816 m** |
| mesa en U | tablero a 0.63 m, fondo de `x` = 0.60 a 1.20 m | **2.205 × 1.80 m, tablero de 35 mm a 0.770 m**; hueco de 1.005 × 1.10 m; fondo de la U de 0.70 m |
| tablero en `base_link` | `z` = 0 | **`z` = −0.046** |
| borde interior del fondo de la U | `base_y` = 0.600 | **`base_y` = 0.426, medido** (5 cm de holgura, D16) |
| holgura mounting base ↔ mesa | 0.150 m delante, 0.10 a los lados | **0.050 m delante (medida)**, 0.1265 a los lados (centrado) |

La muestra tiene que quedar entera sobre la mesa: borde ≤ `cut_center_y` − L/2 −
margen. Con el corte en 0.65, la EVA (L = 250 mm en la dirección del corte) y
25 mm de margen, el borde tiene que estar en **≤ 0.50 m**. Con el borde en 0.600
el corte tendría que ir a ≥ 0.745 m, donde `G` lift = 122 > 120. **Con la
holgura medida (5 cm, D16) el borde está en 0.426: la condición se cumple sin
mover el robot**, con 0.099 m de margen para la EVA y el corte en 0.65.
(Terminología: *pedestal* = `ur5_mounting_base`; *abertura de la U* = el
entrante de 1005 × 1100 mm donde va el mounting base.)

`dry_run` sobre el tablero real con muestras de 40 mm (bandeja de 5 mm supuesta
en la Rev. 4, `surface_z` = −0.001; con la de 2 mm medida, D17, −0.004):

| `cut_center_y` | `surface_z` | duración | σ_min(J) | w_min | `G` pan / lift / codo |
|---|---|---|---|---|---|
| 0.60 | −0.001 | 28.19 s | 0.2106 † | 0.0810 | 68.7 / 95.8 / 81.4 |
| **0.65** | **−0.001** | 28.66 s | **0.2058** | **0.0819** | **80.0 / 104.4 / 81.4** |
| 0.70 | −0.001 | 29.22 s | 0.1854 | 0.0819 | 92.3 / 113.3 / 81.4 |
| 0.75 | −0.001 | 29.85 s | 0.1590 | 0.0819 | 105.6 / 122.4 / 81.4 |
| 0.60 | −0.004 | 28.24 s | 0.2106 † | 0.0810 | 68.8 / 95.9 / 81.4 |
| 0.65 | −0.004 | 28.70 s | 0.2061 | 0.0819 | 80.1 / 104.5 / 81.4 (recta `x` = 0: q6 = 2.92–2.95, la base gira) |
| **0.65, `x` = −0.1333** | **−0.004** | **28.25 s** | **0.2014** | **0.0819** | **83.1 / 106.7 / 81.4** — **referencia de la campaña** (D18) |
| 0.65 | 0.045 (Rev. 3) | 27.95 s | 0.2015 | 0.0819 | 78.4 / 103.5 / 81.5 |

Bajar la superficie 46 mm alarga el descenso (+0.7 s) y apenas mueve σ_min y `G`;
la distancia radial es lo que manda. En Gazebo la `x` de `u_table` en
`lab_incision_world.sdf` es justamente la distancia del eje del robot al borde
interior del fondo de la U: **0.426** (D16).

### 2.3 Cambios sin commitear en el árbol

| fichero | cambio | valoración |
|---|---|---|
| `ur5_dyn_control/config/smc_params.yaml` | `cut_center_y` 0.75 → 0.85 | Se sustituye por el valor medido tras la reubicación (D1, B0.0). |
| `docs/100_ur5e_real_commands.txt` | notas del operador (719, `lambda_joint`) | Inocuo. Las `G=`/`P=` apuntan a `smc_v4_g5`, **retiradas del robot** (`07` §5.8): corregir. |
| `ur5_pick_place/*` | rviz, params, `.m` | Fuera de alcance; no se toca. |

### 2.4 Corridas que no estaban documentadas (B0.0)

Todas de **Gazebo** (sin wrench; en simulación la columna `cur` lleva el par de
la junta). Lo que dicen sus CSV y el commit compilado:

| corrida | fecha | commit | qué es |
|---|---|---|---|
| `smc_605` | 2026-10-27 12:01 | (`7e4526c`; cabecera rancia `a6381f2`) | la lanzó el usuario **solo para ver el robot en la escena de Gazebo** (no es un experimento). Corrió con el `cut_center_y` 0.85 sin commitear: TCP 132.3 mm, casi singular (V4), y quedó 100 min en HOLD_END. **Descartada** |
| `smc_732` | 2026-09-10 14:43 | `7b12351` | barrido a 0.05 rad/s (`SWEEP_0.050_POS`), tras el arreglo de Pinocchio 4.1 |
| `smc_733`, `734` | 2026-09-10 14:46 | `9bc216a` | el mismo barrido con la guarda de vibración |
| `smc_735` | 2026-09-10 14:50 | `8f17876` | el mismo barrido con el banner de `λ` y `G` corregido |
| `smc_736` | 2026-09-24 11:44 | `43c25a4` | incisión en 0.75: TCP 0.030 mm en TRACK |
| `smc_737` | 2026-09-24 11:45 | `43c25a4` | barrido a 0.05 rad/s en los dos sentidos |

`smc_732`–`737` son comprobaciones en Gazebo de cada commit de la FASE 9: no
alimentan ningún resultado del paper. La intención se deduce del commit
adyacente; no hay nota del operador.

### 2.5 Escena real del laboratorio (2026-10-27, antes de B0)

Los modelos de pedestal y mesa del mundo (`model://ur5_base` y
`model://surgery_table` de `ur5_pick_place`) no coincidían con el laboratorio
(D15). En `lab_incision_world.sdf` se sustituyeron por los medidos sobre los
reales; `ur5_pick_place` no se toca y `lab_torque_world.sdf` sigue con los
antiguos para reproducir campañas anteriores.

| | escena real | dónde |
|---|---|---|
| pedestal | `ur5_mounting_base`: 752 × 752 × 816 mm; placas negras, patas y marco gris | `ur5_dyn_control/meshes/ur5_mounting_base/` |
| mesa | `u_table`: 2205 × 1800 × 770 mm, tablero de caoba de 35 mm con textura, hueco de 1005 × 1100 mm | `ur5_dyn_control/meshes/u_table/` |
| spawn | `z` = 0.816 (`world_defaults.mount_z`); `lab_torque_world.sdf` conserva 0.63 | `launch/world_defaults.py`, `ur5e_effort_gz.launch.py` |
| colisión | cajas, no la malla: pedestal entero **con su cara superior 10 mm por debajo de la base del robot** (V14), tablero en 3 piezas | `model.sdf` de cada modelo |
| posición de la mesa | origen del modelo = borde interior del fondo de la U; `x` = **0.426**: 5 cm de holgura medidos (D16). Hasta la Rev. 4 era 0.600, heredado sin medir; las corridas de V14 son de entonces | `lab_incision_world.sdf` |

Dos correcciones sobre los ficheros del CAD, documentadas en sus cabeceras: el
OBJ de la mesa viene **invertido en z** (tablero en z 0–35 mm, regatones en 770),
y se endereza con un giro propio en el `<pose>` del visual (roll π, yaw π/2,
det +1). Además, sus coordenadas de textura están a 1 unidad por 10 mm (la
caoba se repetía cada 1 cm y se veía como un color liso): se dividieron por 30.
La textura se renombró a `mahogany.png` (el nombre original tenía espacios y `&`).

**Verificado:** render offscreen con 4 cámaras (forma en U, pedestal en el hueco,
pedestal 46 mm por encima del tablero, veta visible); el robot spawnea a 0.816
sobre el pedestal; `gravity_comp` 70 s sin disparos del watchdog (error máx.
4.5 mrad transitorio en `wrist_1`, 0 al final); 102 tests de `ur5_dyn_control`
en verde.

**Lo que cambia para el resto del plan:** el tablero queda a `z` = **−0.046** en
`base_link` (antes 0), así que `surface_z` = espesor de muestra + bandeja −
0.046. Los YAML conservan su `surface_z` actual hasta B0.1 (solo se corrigieron
los comentarios).

**V14 — La escena nueva no degrada el lazo (tras corregir la colisión del
pedestal).** Misma incisión SMC (`smc_params.yaml` con `cut_center_y` = 0.75,
`gains_file` = `smc_v5_g6`, sin fricción en la planta ni compensación), ventana
del corte de `analyze_smc.py`:

| escena | corridas | TCP en el corte [mm] | TV(τ) en el corte | |
|---|---|---|---|---|
| anterior (`lab_incision_world` de HEAD), robot a 0.63 | `smc_98806`, `98809` | 0.0046, 0.0051 | 26, 67 | referencia |
| anterior, robot a 0.816 (sin pedestal debajo) | `smc_98811` | 0.0039 | 36 | la altura no influye |
| real, caja del pedestal a **1 mm** del `base_link` | `smc_98807`, `98810` | 0.0082, 0.0077 | **303, 398** | defecto |
| real sin colisiones de pedestal ni mesa | `smc_98812` | 0.0052 | 90 | aísla las cajas |
| real, caja del pedestal a 10 mm (scratch / modelo final) | `smc_98813`, `98814` | 0.0037, 0.0038 | **12, 11** | **corregido** |
| **escena final** (mesa a 0.426, D16) con la geometría de B0.1 (corte en 0.65, `surface_z` −0.004) | `smc_98815` | **0.0023** | **12** | `G` = 80 / 104 / 81 |

La caja a 1 mm quedaba dentro del margen de contacto de la física frente al
`base_link` del UR5e: un contacto permanente que no mueve nada (la base está
soldada al mundo) pero multiplica ×10 la actividad del par. Bajada a 10 mm
desaparece. Las métricas sobre TRACK completo varían entre corridas mucho más que
entre escenas (TV 218–1624 con las mismas ganancias), porque las dominan los
transitorios entre fases.

Tres hallazgos de esta verificación que afectan al resto del plan:

- **El RTF ya era 1.0.** Las corridas de septiembre (`smc_601`–`604`) y las de
  hoy en ambas escenas van a RTF 1.000 (medido con `t_wall`/`t_sim` del CSV). El
  "RTF ≈ 0.07 por el trimesh" de `04_lqr_sdre.md` §5.2 era de agosto: el coste de
  la CARE dentro del lazo medido entonces (348 µs) es pesimista para hoy.
- **Con las ganancias propias de `smc_params.yaml`** (λ = 20, `G` lift = 332;
  no son `smc_v5_g6`, que entran por `gains_file`) el resultado es **bimodal entre
  corridas en cualquier escena**: TV(τ) del corte 111 en una corrida y 11 263 en
  la siguiente, en la misma escena real. Una sola corrida no basta para comparar
  chattering: B3 y B6 deben repetir cada caso y medir la dispersión (el
  temporizador sincronizado con la sim de B3 ataca la causa).
- **`lab_torque_world.sdf` ya no sirve con la trayectoria actual:** su caja
  (pick & place) cae sobre el trazo en `base_y` 0.70–0.80; la punta se apoya en
  ella a t = 8.5 s y el watchdog salta en `wrist_3` (`smc_98802`). Avisado en
  `world_defaults.py`.



### 2.6 B0 — cerrado (2026-10-27)

| # | hecho | verificación |
|---|---|---|
| B0.0 | Escena real (§2.5); el borde de la mesa ya está a 0.426 (D16): **no hay reubicación**. Corridas documentadas (§2.4). `100_ur5e_real_commands.txt` apunta a `smc_v5_g6` (en el árbol, sin commitear: va sobre tus notas). | — |
| B0.1 | Geometría congelada en `config/incision_common.yaml`: recta `x` = −0.1333 (D18), centro 0.65, `surface_z` −0.004, trazo de 100 mm (80 medidos a 10 mm/s), 5 mm. | σ_min 0.2014, w_min 0.0819, 28.25 s, meseta 8.00 s |
| B0.2 | YAML común `/**:` + YAML por ley solo con ganancias; `launch/param_layers.py` lo antepone si el YAML declara `trajectory_type: incision` (sim y real); el nodo **se niega** a arrancar una incisión sin él. | `test_incision_common.py`: los tres nodos producen la misma tabla **byte a byte** |
| B0.3 | La tabla exportada lleva `phase` (`approach` … `withdraw`) y `plateau`; el evaluador lee de ahí la fuerza (`CuttingForce.from_reference`) y la meseta. Fuera los tiempos cosidos; una tabla sin etiquetas es un error. La columna `state` del CSV sigue en TRACK. | fases en orden; meseta contigua de cut_length/v_cut |
| B0.4 | `Plant` con TCP 0.162686, el acople y `Q_INIT` actual (la de la FASE 5, explícita). | `test_dynamics_cross.py`: `M`, `g`, `C` de C++ y Python **idénticos** (diferencia 0.0 en 200 configuraciones); el evaluador da ahora la `G` que imprime el nodo (186.4 / 240.0 / 134.5) |
| B0.5 | `<prefijo>_<n>.params.yaml` con todos los parámetros efectivos, recargable; `git_sha` y `git_dirty` regenerados en **cada build** (antes el SHA se fijaba al configurar y quedaba rancio, §2.4); campos de sesión como argumentos de launch, iguales en sim y real. | recargar el fichero lateral reproduce la tabla exacta |
| B0.6 | `00_assumptions` (A3 altura, A4 filo/D18), `01_trajectory` (geometría vigente), README (bloque común, trazabilidad, tests). | — |

**De punta a punta en Gazebo** (launches normales, aire, sin fricción): las tres
leyes cargan el común, recorren base_y 0.700 → 0.600 con `wrist_3` = π y la
base quieta, y acaban en el mismo instante. TCP en la meseta: SMC (`smc_v5_g6`)
**0.0030 mm**, CT (kp 100, kd 20) **0.0068 mm**, LQR-SDRE (wn 20, fixed)
**0.049 mm** (`smc_98816`, `fl_98818`, `lqr_98817`). En aire no discriminan:
lo hará la fuerza de corte (B2–B3), como ya decía V3.

**Tests:** 164 en verde (80 gtest + 24 pytest en `ur5_dyn_control`, 19 en
`ur5_identification`, 32 en `ur5_trajectory_optimization`, + 9 envoltorios).

**Hallazgos de B0 que afectan a lo que sigue:**

- `ur5e_real.launch.py` **no tiene `lqr_sdre`** (solo `gravity_comp`, `fl`,
  `smc`): el LQR-SDRE no se puede correr hoy en el robot. Entra en B4/B7.1.
- Pinocchio no estaba retenido: ya lo está, con sus dependencias (D20).
- `smc_1.csv` quedó sobrescrito por los `dry_run` de esta sesión (era un artefacto, §9.6). Los
  tests y `dry_run` escriben ahora en directorios temporales con `test_num`
  propios.
- `closed_loop.py` conserva en su docstring "sin retardo de tubería / estado
  exacto", que ya no es cierto (`PIPELINE_DELAY_STEPS` = 1, ruido de `q̇`): se
  corrige con B4.3.

### 2.7 Retardo de tubería del real (B7.0, 2026-10-28)

**V15 — El lazo real tiene 5 ciclos (10 ms) de retardo, no 1.** Medido con los
CSV que ya había (`analyze_pipeline_delay.py`, `pipeline_delay.py`): en los
barridos FL reales `fl_500`/`502`/`506`/`507` el feedforward de Coulomb cambia de
signo en cada inversión, un escalón de ~7 N·m en 2–3 muestras. Promediando la
respuesta alineada en 71 escalones y ajustando `k⊙i(k) = a·τ_cmd(k − D)`:

| origen | corridas | escalones | D [ciclos] | D [ms] | ganancia `a` |
|---|---|---|---|---|---|
| **real** | `fl_500`, `502`, `506`, `507` | 71 | **5.02** (4.85–5.49) | **10.0** | 0.98 |
| Gazebo | `smc_98801` | 21 | 1.00 | 2.0 | 0.94 |

- No es un filtro de la medida de corriente: la **velocidad medida arranca en la
  misma muestra** que la corriente. La respuesta asoma a las 2 muestras (~7 %) y
  el grueso llega a las 5; el mejor modelo es un retardo puro de 5 con una
  constante de tiempo de ~0.5 ms. `a` = 0.98 confirma de paso la `k` de corriente
  a par (§8.4 de `02_friction_real.md`).
- `D` es exactamente el `delay_steps` del evaluador (`closed_loop.simulate`,
  hoy **1**, tomado de Gazebo) y del chequeo discreto del banner
  (`pdDiscreteSpectralRadius`, también con 1).
- **Consecuencia (análisis lineal por junta, dentro de la capa límite):** margen
  de ganancia discreto con 1 → 5 ciclos:

  | ley | D = 1 | D = 5 |
  |---|---|---|
  | CT nominal (100/20) | ×24.9 | ×7.0 |
  | semilla LQI (ωn = 10) | ×16.5 | ×4.5 |
  | `smc_v5_g6` (la vigente, nunca probada en el real) | todas > ×1.8 | **inestable en las tres muñecas** (hombro ×3.6, codo ×1.4) |
  | `smc_v4_g5` (la de `smc_712`) | todas > ×1.6 | inestable en base, codo y muñecas |

  En el evaluador completo, D = 3 o 5 vuelve inestables los dos juegos del SMC
  (TV(τ) ×10⁴); con D = 1 los dos son estables. Coincide en lo cualitativo con
  `smc_712` (ciclo límite con `smc_v4_g5`), **pero no en la frecuencia**: el
  modelo rígido con D = 5 predice ~20 Hz y se midieron 37 Hz en las cinco juntas.
  El retardo no lo explica todo: el modo estructural (25–35 Hz) y la no
  linealidad de `sat` también intervienen.
- **Qué cambia:** `smc_v5_g6` **no se lleva al robot** (B7.1) sin re-sintonizar
  con D = 5; B5 debería usar `delay_steps` = 5 y la restricción discreta con 5
  ciclos (§9.8, R10). Los barridos de B4.4 re-miden D con el acople.

**`RobotReceiveTimeout` y el par latcheado.** En ur_robot_driver 2.14 el timeout
es fijo (20 ms, `hardware_interface.hpp`), no un parámetro. El script del robot
(`external_control.urscript`) sale con `stopj` si en ese plazo no llega ningún
paquete: cubre la caída del **driver o de la red**. Pero `forward_effort_controller`
conserva el último comando y el driver lo reenvía en **cada** ciclo, así que si
muere el **nodo de control** sin pasar por su destructor (fallo, `kill -9`) o se
queda en DONE, el último par sigue aplicándose y el timeout no vence nunca. Lo
cierra `ur5_dyn_control/scripts/command_deadman.py`, en su propio proceso dentro
de `ur5e_real.launch.py` (`deadman:=true` por defecto): con 100 ms sin comandos
publica par cero y devuelve el mando al JTC. Medido en Gazebo con `kill -9` al
nodo: salta a los 100 ms y el controller_manager hace el switch. Mayor hueco entre
ciclos del nodo en 21 corridas reales: 26.6 ms.

---

## 3. Diagnóstico: qué falta para el objetivo

### 3.1 La comparación (E2, E4)

Un CT o un LQR cuyas ganancias escalan con `M(q)` coloca todos los polos en el
mismo sitio, pero la **rigidez** frente a una perturbación de par es
`K_P,ii ≈ ωn² M_ii`. La fuerza de corte no escala con la inercia: el momento en
las muñecas (~1 N·m con 5 N y 0.16 m de brazo) es del orden del de las juntas
grandes, y su inercia es cientos de veces menor. Eso hace que **cualquier ley
lineal con ganancias proporcionales a M** pierda la incisión en las muñecas
(V3). El SMC ya resolvió esto en la FASE 7 (`η` por junta + `g5`); el CT y el
LQR no.

Para que el paper compare **leyes** y no **sintonías**:

1. El LQR-SDRE debe ser un controlador distinto del CT: pesos con significado
   físico (no `∝ M²`) y acción integral (§4.3).
2. Los tres se sintonizan con **el mismo optimizador, los mismos objetivos, las
   mismas restricciones (incluida `G ≤ 120`) y el mismo presupuesto** (B5).
3. Se compara bajo la **fuerza de corte de cada material** y bajo desajuste.

### 3.2 El gemelo digital (E1)

| fuente de discrepancia | robot real | Gazebo hoy | acción |
|---|---|---|---|
| fricción articular | F_v, F_c medidos (§8.4) | inyectada; validada en `shoulder_lift` (−0.47 % / +0.44 %) | validar también `wrist_1`/`wrist_2`; **`wrist_3` solo Coulomb** (B3.4) |
| herramienta | 0.182 kg (CAD) | ídem | **pesar** el acople y medir `tool0`→punta (A1/A2) |
| escena | mesa en `base_y` ≈ 0.85 (hoy) | mesa en world x = 0.3 | reubicar el robot, **medir** la escena y replicarla (B0.0) |
| gravedad | modelo interno de UR | exacta (mismo modelo) | medir la discrepancia `α_g` (B4.4) y emularla en el desajuste |
| sensado de velocidad | encoder de 5.72 µrad → `Δq̇` = 2.86e-3 rad/s | suelo ~5e-6 rad/s (570× menor) | **relé de sensor** que cuantiza y deriva en sim (B3.2) |
| modo estructural 25–35 / 48 Hz | medido (`smc_712/718`) | brazo rígido | no modelable en Gazebo → restricción empírica `g6` **común a las tres leyes**; declararlo |
| retardo de tubería | **5 ciclos = 10 ms** (V15) | 1 ciclo | llevarlo al evaluador y a la restricción discreta (B5, R10) |
| temporización | ciclo del driver a 500 Hz | timer de pared contra reloj de sim (jitter) | timer en tiempo de simulación (B3.3) |
| estado térmico | F_v −9.3 % frío→rodado | — | calentamiento obligatorio; ±10 % de F_v en el desajuste |
| **tejido / corte** | real | **ausente** | plugin de corte por material (B3.1) |

### 3.3 La fuerza de corte (E3, E4) — solo con el sensor integrado (D4)

No hay ni un dato medido, y el sensor F/T integrado del UR5e es de baja
resolución para fuerzas de pocos newtons (del orden de 3–4 N de precisión según
la hoja técnica — **verificar** en la del equipo). Lo que el modelo de tejido
necesita, sin embargo, son sobre todo **niveles medios** en la meseta del corte,
no el contenido de alta frecuencia. Estrategia:

1. **Cero inmediatamente antes del contacto** (`zero_ftsensor` al final de la
   aproximación, hoja al aire): elimina sesgo y deriva lenta, que son lo que más
   pesa en la especificación. Con orientación constante (A4) el peso del acople
   queda cancelado por el mismo cero.
2. **Promediado sobre la meseta** (8 s, ~4 000 muestras): si el ruido es
   aproximadamente blanco, la media se resuelve muy por debajo de la precisión
   por muestra. B1 lo cuantifica con pesos conocidos antes de cortar nada.
3. **Estimador por corriente** `F̂ = J⁻ᵀ(k⊙i − τ̂ − τ̂_f)`, con `k` medida al
   0.1–0.2 %: independiente del mando y del sensor. **Solo valen las juntas en
   movimiento** (una parada tiene fricción indeterminada en ±F_c). En la recta
   de D18 se mueven hombro, codo y `wrist_1`: **F_y (arrastre) y F_z (normal)
   son observables, F_x (lateral) no** (B1, `force_estimator.py`). Validación
   cruzada con el F/T = resultado propio (FASE 9 del plan original).
4. El **zapallo** es mucho más duro que los otros tres; los cortes piloto en
   posición (B2) confirman que hoja y acople impreso aguantan 5 mm a 10 mm/s.

### 3.4 Coherencia de la cadena

Hoy cada eslabón tiene su propia copia de la geometría: tres YAML con valores
distintos (V6), un evaluador con tiempos cosidos (V5) y un análisis que deduce
ventanas de los datos (`8056f04`). Hace falta **una única fuente**: un YAML
común de incisión/herramienta/fricción, y una tabla de referencia que lleve la
**etiqueta de fase** por muestra, consumida por el evaluador y por el análisis.

### 3.5 Una sola muestra por material (D5)

Fotos revisadas: el **Ecoflex** es una placa colada de superficie plana (parece
la más pequeña); la **tabla de EVA** es grande y plana; la **panceta** lleva la
piel arriba, con superficie ondulada e irregular; el **zapallo** es una cuña con
la pulpa a la vista y superficie cóncava. Consecuencias:

**(a) Capacidad: carriles de corte.** Todos los cortes de un material salen de la
misma pieza, en carriles paralelos separados un paso `p`. El trazo (100 mm) va
en la dirección radial de la base; los carriles se suceden en perpendicular.
Con N = 3: 3 leyes × 3 = **9 cortes de campaña** por material, más los pilotos
que caigan en la misma pieza:

| material | carriles necesarios | pilotos en la misma pieza |
|---|---|---|
| panceta | 9 | ninguno (pieza aparte, D9) |
| zapallo | 9 | ninguno (pieza aparte, D9) |
| Ecoflex | 11 | 2 en posición (B2) |
| EVA | 14 | 2 en posición (B2) + 3 con par (B7.2) |

Con margen al borde `m = 15 mm` y `p = 15 mm` (3× la profundidad; se valida en
los pilotos: el corte anterior no debe alterar el siguiente), `n` carriles en una
fila piden **`2m + (n−1)p` en la dirección de los carriles** y **`100 + 2m = 130
mm` en la del corte**:

| material | dimensiones (D10) | orientación | carriles posibles | necesarios | |
|---|---|---|---|---|---|
| EVA | 400 × 250 × 40 | corte a lo largo de 250, carriles a lo largo de 400 (fuera de la zona de las asas) | ~19 | 14 | ✅ |
| panceta | ~~200 × 50 × 40~~ → **bloque ≥ 150 × 130 × 40** (D14; p. ej. 200 × 150) | carriles a lo largo de ≥ 150, corte en ≥ 130 | 9 (12 con 200) | 9 | ✅ |
| Ecoflex | **250 × 150 × 40** recomendado (§3.5 h) | corte a lo largo de 150, carriles a lo largo de 250 | 15 | 11 | ✅ |
| zapallo | **2 bloques de 150 × 105 × 40** recomendados (§3.5 h) | corte a lo largo de 150, carriles a lo largo de 2 × 105 | 2 × 6 = 12 | 9 | ✅ |

**Panceta resuelta con un bloque (D14).** Las alternativas (tiras de 200 × 50
lado a lado, o acortar el trazo para todos los materiales a 30–40 mm medidos)
quedan descartadas. Falta la capacidad de Ecoflex y zapallo (§9.7 R2), que hay
que tener **antes de B5**, porque la sintonía depende de la referencia.

**(b) La misma referencia para todos los cortes.** **Fijación indexada** (D13)
que desplaza la muestra un paso `p` entre cortes en la dirección de los carriles
(perpendicular al trazo, `base_x`), con pasadores o topes cada 15 mm: la
trayectoria del robot es **byte a byte la misma** en todos los cortes, todas las
leyes y —con la altura común de (c)— todos los materiales. Recorrido necesario,
con los carriles de reserva de (h): hasta 16 × 15 = 240 mm (EVA), así que la
placa base de la fijación mide al menos el ancho de la muestra más ese
recorrido. Hay espacio de sobra: el fondo de la U
mide 2.205 m en esa dirección (V13). La posición del carril se registra en el CSV
(B0.5).

**(c) Superficie plana y a la misma altura.** La referencia supone una
superficie horizontal a `surface_z`. Panceta (con la piel arriba) y zapallo se
**recortan a un bloque de cara superior plana** (D11). Además, para que la
trayectoria sea **la misma en los cuatro materiales**, todas las caras superiores
se llevan a una **altura común**: espesor de 40 mm (el de la panceta y la EVA)
sobre una bandeja de espesor conocido, con suplementos debajo de las muestras
más delgadas o recortando las más gruesas. El tablero real está a `z` = −0.046
en `base_link` (V13), así que con la bandeja de 2 mm (D17) `surface_z` =
**−0.004 m**. Antes de cada carril se toca la superficie con la punta en
3 puntos (freedrive) y se registra `surface_z` del carril; si se aparta más de
±0.5 mm de la común, se re-recorta o se suplementa.

**(d) Fijación.** La muestra no puede deslizar con la fuerza tangencial: bandeja
con paredes + cinta doble cara (Ecoflex, EVA) o pasadores/mordazas laterales
fuera de los carriles (panceta, zapallo).

**(h) Dimensiones recomendadas para conseguir las muestras (D21).** Salen de
(a): el trazo de 100 mm más 15 mm de margen a cada lado pide **130 mm** en la
dirección del corte (se recomiendan **150**, para ±10 mm de colocación), y `n`
carriles piden `2·15 + (n−1)·15` mm en la otra. A los carriles de campaña se
suman **3 de reserva** por material (una corrida que aborta, un carril que el
corte anterior deforma). Espesor **40 mm** = la altura común de (c); los
naturales se compran más gruesos y se recortan planos.

| material | conseguir (largo × ancho × espesor) | carriles | notas |
|---|---|---|---|
| **panceta** | bloque de **200 × 150**, espesor ≥ 45 mm para dejarlo en 40, de **una sola pieza**; + una pieza aparte de ~150 × 100 para los pilotos (D9) | 12 (9 + 3) | Piel arriba, cara superior recortada plana (D11). Si sale más delgada (30–40 mm), sirve con suplementos debajo: la hoja solo entra 5 mm. Todos los cortes en la misma dirección respecto a la pieza (anotar si van a lo largo o a lo ancho de la fibra) |
| **zapallo** | **dos bloques de 150 × 105 × 40** de pulpa, sin cáscara y con caras planas, del **mismo** zapallo; + uno aparte para los pilotos | 2 × 6 = 12 | Un único bloque de 200 mm de ancho no sale de una pared curva: al aplanar se pierde la flecha (21 mm para 150 × 105 en un zapallo de Ø 40 cm). Conviene un **zapallo macre grande (Ø ≥ 40 cm) con pulpa ≥ 6 cm**. Si la pulpa no da, bloques de 25–30 mm sobre suplementos |
| **Ecoflex 00-30** | placa colada de **250 × 150 × 40** (≈ 1.6 kg); + una de 150 × 100 × 40 para los pilotos (≈ 0.65 kg) | 15 (11 + 4) | Total ≈ 2.3 kg: basta un kit de 1 galón (≈ 3.6 kg). Molde nivelado para que la superficie libre, que es la de corte, salga plana; el molde puede ser la propia bandeja de la fijación |
| **EVA** | la tabla que ya hay, **400 × 250 × 40** | hasta 25 a lo largo de 400 (17 = 14 + 3 necesarios) | Corte a lo largo de 250; carriles fuera de la zona de las asas: hacen falta 270 mm libres |

**(e) Perecibles.** Panceta y zapallo cambian con las horas (secado,
temperatura). Toda su campaña en **una sola sesión**, el día de compra o
refrigerados con hora registrada; orden de materiales: EVA → Ecoflex → zapallo
→ panceta. Metadatos por corte: temperatura, minutos desde la apertura, usos de
la hoja.

**(f) Aleatorización.** Rondas: en cada ronda un corte por ley, en orden
aleatorio y en carriles asignados al azar (semilla registrada). Así la posición
en la muestra, el desgaste de la hoja y el tiempo no se confunden con la ley.

**(g) Alcance estadístico.** Las réplicas son carriles de **una** muestra: la
inferencia es sobre las leyes en esa muestra, no sobre el material en general.
Se declara en *Limitations*; la variabilidad entre muestras se cubre en parte
con el desajuste de ±30 % de las propiedades del tejido en el gemelo (B6).

---

## 4. Formulación de las leyes (confirmada, D2)

### 4.1 Planta y estructura común

```
M(q) q̈ + C(q,q̇) q̇ + g(q) + τ_f(q̇) + Jᵀ(q) F_ext = τ
```

Común a las tres leyes (y ya implementado en `TorqueControlNodeBase`):

- **Referencia** `{q_d, q̇_d, q̈_d}` idéntica (B0.2).
- **Feedforward de fricción** `τ̂_f(q̇_d) = F̂_v q̇_d + F̂_c tanh(q̇_d/ε)`, con la
  velocidad **deseada** (`05` §7.3) y `ε = 1e-5`, límite `ff_dv_max`.
- **G3**: la ley devuelve el par físico; en el real se resta `ĝ(q)`.
- Saturación, límite de tasa, watchdog y guardas: idénticos.
- **Misma fuente de velocidad** `q̇` (la del driver; si se filtra, se filtra para
  las tres).

### 4.2 CT-PD — línea base

```
τ = M̂(q) [ q̈_d − K_P q_e − K_D q̇_e ] + Ĉ(q,q̇) q̇ + ĝ(q) + τ̂_f(q̇_d)
q_e = q − q_d,   K_P = diag(kp_i),   K_D = diag(kd_i)
```

- Ley **sin cambios** (`gz_fl_control_node`). Lo que cambia es que `kp_i, kd_i`
  se sintonizan por junta con el mismo optimizador (B5). Rigidez efectiva frente
  a perturbación: `M_ii·kp_i`; `G_i = M_ii·kd_i`.
- Restricción discreta: radio espectral del lazo discretizado `< 1 − margen`
  en toda la referencia (`g3_lin`, B5), con el retardo **real** de 5 ciclos
  (V15; hasta la Rev. 7 se suponía 1).

### 4.3 LQR-SDRE con acción integral (LQI-SDRE)

**Por qué cambiar.** Con `Q = blkdiag(r ωn⁴ M², r ωn²(4ζ²−2) M²)`, `R = rI`, la
CARE da en el límite desacoplado `K = [ωn² M, 2ζωn M]`: la ley resultante
`M q̈_d + C q̇_d + g − M(ωn² q_e + 2ζωn q̇_e)` es un CT con `kp = ωn²`,
`kd = 2ζωn` más un término `C q̇_e` despreciable a las velocidades de la
incisión (`|q̇| ≤ 0.2 rad/s`). V3 lo confirma numéricamente (227.5 vs 228.3 mm).
Es una observación analítica válida para *Methods*, pero no un controlador
distinto que comparar.

**Formulación:**

```
σ̇ = q_e                                       (estado integral)
x  = [σ ; q_e ; q̇_e] ∈ R¹⁸

A(q,q̇) = [ 0  I  0 ;  0  0  I ;  0  0  −M⁻¹C ]      B(q) = [ 0 ; 0 ; M⁻¹ ]

J = ∫ (xᵀ Q x + uᵀ R u) dt,    Q = blkdiag(Q_σ, Q_p, Q_v)  CONSTANTES,   R = diag(r_i)
AᵀP + PA − P B R⁻¹ Bᵀ P + Q = 0,     [K_I  K_P  K_D] = R⁻¹ Bᵀ P

τ = M̂ q̈_d + Ĉ q̇_d + ĝ + τ̂_f(q̇_d) − K_I σ − K_P q_e − K_D q̇_e
```

- **Pesos en unidades físicas**, diagonales y constantes (Bryson como punto de
  partida: `Q_p,ii = 1/e_max,i²`, `R_ii = 1/τ_max,i²`), optimizados en B5. La
  rigidez de cada junta sale del compromiso par/error y **no** queda atada a
  `M_ii`; la dependencia de `B(q) = M(q)⁻¹` hace que `K(q)` se re-programe con la
  configuración: eso es lo que el SDRE aporta realmente aquí. Desaparece la
  dicotomía `fixed`/`scheduled` (era un artefacto de `Q ∝ M²`).
- **Integral**: rechaza la parte constante de la fuerza de corte, el residual de
  fricción y el error de gravedad respecto al modelo de UR. Se habilita al entrar
  en TRACK, se congela con `onSaturation()` (anti-windup) y en `SAFE_HOLD`, y se
  acota `|σ_i| ≤ σ_max,i`.
- **Semilla y regresión:** la síntesis por polinomio generaliza la actual. Para
  un polinomio deseado por modo `a(s) = s³ + a₂s² + a₁s + a₀` y `R = rI`,
  `Q_σ = r a₀² M²`, `Q_p = r (a₁² − 2a₀a₂) M²`, `Q_v = r (a₂² − 2a₁) M²`
  (exige `a₁² ≥ 2a₀a₂`, `a₂² ≥ 2a₁`; con `n = 2` recupera la fórmula vigente).
  Sirve de semilla y de test (18 polos en la raíz deseada), **no** de diseño final.
- **Ablación para el paper:** LQR-SDRE sin integral (= CT, ya implementado).
- **Coste:** la CARE pasa de 12 a 18 estados (estimado ≈ 3.4× en banco: ~200 µs
  frente a 57; hay que medirlo). En lazo, con Gazebo cargando la máquina, el pico
  de 12 estados ya fue 1371 µs: si no cabe, decimar a 100 Hz con ZOH (`04` §5.4:
  −20 % de precisión, estabilidad intacta). En el real no hay Gazebo cargando la
  CPU.
- Siguen vigentes: solver por función signo con balanceado simpléctico,
  certificación de estabilizabilidad, `max Re(eig(A − BK)) < 0` por paso, CSV de
  diagnóstico.

### 4.4 SMC optimizado — tres correcciones

```
s    = q̇_e + Λ q_e,     q̇_r = q̇_d − Λ q_e,     q̈_r = q̈_d − Λ q̇_e
τ    = M̂ q̈_r + Ĉ(q,q̇) q̇_r + ĝ + τ̂_f(q̇_d) − K ⊙ sat(s/φ)                 (R1)
K_i  = η_i + α_M |(M̂ q̈_r)_i| + α_C |(Ĉ q̇_r)_i| + α_g |ĝ_i|                (R2)
η_i  ≥ d̄_f,i + d̄_cut,i                                                     (R3, g5 ampliada)
```

- **R1 — Slotine–Li exacto.** `Ĉ q̇_r` en vez de `b̂ = Ĉ q̇`. Con `Ṁ − 2C`
  antisimétrica (`Ṁ = C + Cᵀ`, ya usada en el código), `V = ½ sᵀMs` da
  `V̇ = −sᵀKρ − sᵀΔ`: la condición de alcance `K_i ≥ |Δ_i| + η_i` pasa a ser una
  prueba global y no local (V11). Diferencia numérica `Ĉ s`: despreciable a estas
  velocidades; el cambio es de rigor, y corrige el comentario del nodo.
- **R2 — Cota de incertidumbre estructurada.** Hoy un único `α = 0.3` multiplica
  `b̂`, que incluye la gravedad. Separar `α_M`, `α_C`, `α_g` con valores
  **medidos o declarados**: `α_g` de barridos en varias posturas (B4.4), `α_M`
  de la incertidumbre de masas (acople pesado, URDF). **Qué gravedad:** en el
  real el nodo manda la ley menos `ĝ` y el robot suma su `g_UR` (G3), así que `ĝ`
  se cancela y el error que ve el lazo es `g_real − g_UR`, no `g_UR − ĝ` (lo que
  decía la Rev. 7: la diferencia entre dos modelos). Con `α_g ≈ 0.07` se liberan ~50 N·m/(rad/s) de
  `G` en el hombro (V2) que el optimizador puede gastar en `λ` dentro de
  `G ≤ 120`.
- **R3 — `η` cubre la fricción residual y el corte.** `g5` ya impone
  `η ≥ d̄_f`; se amplía con la cota del momento de corte por junta, `d̄_cut`,
  sacada del modelo de cada material (B2) — **envolvente de los cuatro**, porque
  el material es desconocido para el controlador (D3).
- Se mantiene: `sat` con φ por junta, `G_i = M_ii λ_i + K_i/φ_i ≤ 120`, χ por
  junta normalizado (`g3`), tolerancia de TCP 2.0 mm (`g4`).
- `sign` queda como resultado de simulación (FASE 5). Al robot no se lleva: su
  energía por encima de 20 Hz es la que excita el modo de 25–35 Hz.

### 4.5 La restricción común `G ≤ 120`

`G_i := Σ_j |∂τ_i/∂q̇_j|` evaluada sobre la referencia (conservadora: incluye
los acoplos de `K_D` del LQR). Para CT: `M_ii·kd_i` (+ acoplo por `M̂`); LQR:
fila `i` de `K_D`; SMC: `M_ii λ_i + K_i/φ_i`. **El banner del nodo y el
optimizador deben calcularla para las tres leyes.**

Antes de fijarla como cota de diseño común hay que medir la base en la pose de
la incisión **de la escena nueva** (equivalente al paso 741 de `09` §7.1): hoy
solo está probada hasta 30. Con el corte a 0.60–0.65 m la base pide menos `G`
(V12), lo que reduce este riesgo.

`G ≤ 120` es empírica. Con el retardo real medido (V15, 5 ciclos) la
restricción discreta por junta con D = 5 es una cota **de modelo** que explica
parte de lo mismo; se propone usar las dos en B5 (R10).

### 4.6 Protocolo de equidad (va a *Methods*)

| elemento | regla |
|---|---|
| referencia | la misma tabla, byte a byte (hash en la cabecera); la muestra se mueve, no el trazo (§3.5 b) |
| modelo | mismo URDF + acople, mismo `tcp_offset_z` |
| feedforward | misma compensación de fricción, misma `q̇` |
| actuador | mismo `tau_max`, `tau_scale`, límite de tasa |
| sintonía | mismo NSGA-II (pop, gen, semilla), mismos objetivos `f1–f3`, mismas restricciones `g1, g2, g4, g6` + la discreta de cada ley, mismo selector (rodilla), mismo presupuesto de evaluaciones, **un juego por ley para los 4 materiales** |
| verificación | toda ganancia se re-verifica en el gemelo antes de usarse |
| campaña | rondas aleatorizadas por carril y orden (§3.5 f) |
| reporte | se reporta la sintonía de cada ley, no solo su resultado |

### 4.7 ASTSMC

Fuera del alcance para el envío de fin de 2026 (D8). Si se retoma, sin cambios
respecto al plan original (§FASE 6) y sobre la misma infraestructura
(`onSaturation()`, `requestSafeHold()`).

---

## 5. Gemelo digital en Gazebo

### 5.1 Ya calibrado (no se rehace)

Fricción física por junta (inyección validada), masa/CoM/inercia del acople
(CAD), offset del TCP (malla), G3, pose inicial, payload del pendant controlado
por `check_session_config.py` (con el acople, `--payload-kg`: B7.0). La **escena** sí se rehace (B0.0).

### 5.2 Modelo de tejido y corte por material

Modelo fenomenológico en la punta de la hoja `p`, con velocidad `v`, superficie
`h_s` y profundidad `δ = h_s − p_z`:

```
δ ≤ 0                         F = 0
indentación (no perforado)    F_n = k δⁿ + c δⁿ δ̇            (Hunt–Crossley)   hasta F_n ≥ F_p
corte (perforado, δ > 0)      F_n = κ_n δ + c_n δ̇
                              F_t = −(κ_t δ + μ F_n) v̂_t + ruido(σ)
                              F_lat = −k_lat Δ_lat            (resistencia a salirse del plano de corte)
```

- **Identificables con los pilotos a feed nominal (B2):** `F_p`, `k`, `n`,
  `κ_n`, `κ_t`, `σ` (niveles medios y dispersión en la meseta).
- **No identificables con un solo feed y sin excitación lateral:** dependencia
  con la velocidad (se fija nula y se declara) y `k_lat` (se toma de la rigidez
  de indentación bajo hipótesis de isotropía, declarada). Ambos entran en el
  desajuste ±30 % de B6. Nada se toma de la literatura sin marcarlo.
- `F_lat` importa porque la **rectitud** es una métrica del corte.
- Implementación preferida: **plugin de sistema de Gazebo Fortress** que aplica
  la fuerza en la punta del eslabón de la herramienta a 1 kHz
  (`Link::AddWorldWrench`) y publica el wrench verdadero para registro (nunca al
  controlador). Alternativa: nodo relé que suma `JᵀF` al comando (un ciclo de
  retardo).
- La muestra aparece en el mundo como un bloque con las dimensiones medidas, en
  la posición de la escena real (B0.0).
- El evaluador offline sustituye `CuttingForce` por el **mismo modelo** (puerto
  Python con test de valores dorados contra el plugin).
- Emulación de `ft_data` en simulación (resolución del sensor del UR5e, medida
  en B1) para que registro y análisis sean los mismos en sim y en real.

### 5.3 Criterios de aceptación sim↔real (aceptados, D7)

Se adoptan estos umbrales. Si el robot real no permite alcanzarlos, se reportan
los alcanzados con su causa (p. ej. el modo estructural, que Gazebo no tiene).

| comparación | métrica | criterio |
|---|---|---|
| incisión en aire, misma ley y ganancias | RMSE de TCP en meseta, RMSE por junta | diferencia relativa ≤ 30 % |
| ídem | `G` efectiva, saturación | idénticas por construcción; sin saturación en ninguno |
| corte (por material), con CT ya validado en aire | perfil de fuerza `F_t(t)`, `F_n(t)` | R² ≥ 0.8 en la meseta; media dentro de ±15 % |
| corte, cada ley | RMSE de TCP en meseta | el orden entre leyes se conserva; diferencia relativa ≤ 40 % |

---

## 6. Plan de trabajo

Cada bloque: entregables · criterio de cierre · dependencias. Un commit por
bloque (o sub-bloque si es grande).

### B0 — Reubicación, escena y coherencia (**cerrado el 2026-10-27: §2.6**)

| # | entregable |
|---|---|
| B0.0 | *(Previo, hecho el 2026-10-27: escena con los modelos reales y la holgura medida, §2.5, D16.)* **No hay reubicación**: el borde de la mesa está a 0.426 m. `cut_center_y` = 0.65 en el YAML común (sustituye al 0.85). En el laboratorio: comprobar que los planos de seguridad del pendant admiten el corte en 0.65 y la superficie en −0.004. La muestra y la fijación **no** se añaden a Gazebo como bloques rígidos (la hoja penetra 5 mm y chocaría): van con el modelo de contacto de B3. Documentar `smc_605`, `smc_732–737`; corregir `100_ur5e_real_commands.txt`. |
| B0.1 | **Geometría del corte congelada:** `cut_center_y` 0.65, `surface_z` −0.004 (D17), trazo de 100 mm, paso `p` y margen `m` de 15 mm. Con la fijación indexada la referencia no depende del material: las medidas de Ecoflex y zapallo (§9.7 R2) solo fijan cuántos carriles caben, salvo que obliguen a acortar el trazo para todos. Esto fija la referencia de B5. |
| B0.2 | **YAML común** `config/incision_common.yaml` (`/**:`: trayectoria, herramienta, fricción, límites, IK) cargado por las tres leyes; los YAML por ley quedan solo con ganancias. Test: los tres nodos producen la **misma** tabla (hash). |
| B0.3 | La tabla de referencia lleva la **etiqueta de fase** por muestra (`approach/contact/penetration/cut/withdraw` + `plateau`). Evaluador y análisis leen ventanas de ahí; fuera los tiempos cosidos de `closed_loop.py`. *(Toca `IncisionTrajectory`/`JointReferenceTable`: cambio de interfaz que se avisa.)* |
| B0.4 | Evaluador: `Plant` con `tcp_offset_z = 0.162686` y la inercia del acople (igual que `Ur5Dynamics::appendBodyToJoint`), `Q_INIT` actual. Test cruzado C++↔Python de `M`, `g`, `C` en configuraciones aleatorias (< 1e-9). |
| B0.5 | Trazabilidad: fichero lateral `<prefijo>_<n>.params.yaml` con **todos** los parámetros efectivos + `git_dirty` en la cabecera. Campos de sesión: material, carril, `surface_z` medida, minutos desde apertura, usos de hoja. |
| B0.6 | Docs: `01_trajectory.md` (geometría nueva), README (tests, LQR), `00_assumptions.md` (la altura del montaje ya se corrigió con la escena, §2.5). `apt-mark hold` de Pinocchio verificado. |

**Cierre:** tests verdes; hash de referencia idéntico en FL/LQR/SMC; escena de
Gazebo = escena medida; `smc_v5_g6` re-evaluado sobre la referencia nueva
(cifras de partida, no ganancias finales).

### B1 — Medida de fuerza (resto de la FASE 3) — código hecho (2026-10-27); falta el cierre en el robot

**Hecho:** el nodo (las tres leyes) se suscribe al wrench del driver, lo lleva a
la punta y a `base_link` (`ft_wrench.hpp`, con tests) y lo escribe en
`wrench1..6` con su sello (`ft_stamp`); en el real (`ur5e_real.launch.py`) pide
`zero_ftsensor` al pasar de la aproximación al contacto y cuenta los ceros
confirmados (`ft_zero`). Estimador por corriente offline
(`ur5_identification/force_estimator.py`, con tests) y `mean_resolution` (error
típico de la media con la autocorrelación corregida). Runner de la incisión por
**posición** (`scripts/run_position_incision.py`) para el cierre y para B2.
Verificado en Gazebo con un F/T falso (`smc_98821`): wrench rotado y trasladado
exacto, cero en `z` = 0.026 (fin de la aproximación); estimador sin fuerza
externa: F_y −0.0002 N, F_z 0.0006 N.

**Corrección a G5:** el `force_torque_sensor_broadcaster` 2.53.3 instalado solo
crea `~/wrench`, es decir `/force_torque_sensor_broadcaster/wrench`; el
`topic_name: ft_data` del YAML de UR no aparece en su librería. Es el default de
`ft.topic`; **confirmar en el robot** con `ros2 topic list | grep -i wrench`.

**Cierre en el robot (pendiente; lo ejecuta el usuario):**

1. Driver arriba, escalas G4 a 0.0, robot en `q_ref[0]` (lo imprime
   `run_position_incision.py --solo-tabla`). Confirmar el tópico del F/T.
   Payload 0 y **TCP en (0, 0, 0)** en las tres corridas: con payload el robot
   descuenta el peso de la lectura; con TCP el wrench llega en otro frame.
2. Tres masas con la misma trayectoria articular (2026-10-28, por la dificultad
   de colgar un peso del acople): `--test-num 800` **brida libre**; `801` con el
   **Robotiq 2F-85** (`--peso-kg` = gripper + acople de montaje **pesados**, sin
   cable ni activación: es solo una masa, ~9–10 N, centrada en el eje); `802`
   con el **acople y la hoja** (~1.8 N, del orden de los materiales blandos; lo
   pesa y cierra de paso A1). Lectura `ros2 topic echo --once` del wrench antes
   y después de cada montaje, sin hacer cero en medio. El 2F-85 mide ~0.17 m
   desde la brida (los dedos bajan ~8 mm más que la hoja): ~29 mm al tablero en
   el punto más bajo. La corrida pide el cero sola.
3. `force_estimator.estimate_from_csv` sobre las dos: `ΔF_z` = `m·g` por las dos
   vías, y el SIGNO del wrench del driver queda fijado; `mean_resolution` sobre
   la meseta da la resolución efectiva de la media del F/T (decide qué materiales
   se pueden caracterizar con él, D4).

Lo que pedía el plan:

- Suscriptor `ft_data` con sello de tiempo, transformación `tool0_controller` →
  `base_link` (A3) y traslado del momento a la punta.
- `zero_ftsensor` llamado por el nodo en un punto fijo (fin de la aproximación,
  hoja en el aire), registrado en el CSV.
- Estimador por corriente `F̂ = J⁻ᵀ(k⊙i − τ̂ − τ̂_f)` (offline, `ur5_identification`).
- **Cierre:** en el real, hoja al aire, `|ft_data| ≈ 0` tras el cero; pesos
  conocidos colgados de la punta (en posición, sin par) reproducen `m·g` por las
  dos vías; **se mide la resolución efectiva de la media sobre 8 s**, que decide
  qué materiales se pueden caracterizar con el sensor integrado (D4).

### B2 — Caracterización del corte por material (robot real, control de POSICIÓN)

**Sin control de par**: el `scaled_joint_trajectory_controller` sigue la misma
tabla de la incisión (reutilizando la vía de `run_current_sweep.py`). Es el modo
más seguro y desacopla la medida de fuerza de cualquier ley propia.

- Registro: `ros2 bag` de `ft_data` + `/joint_states` (corriente) — no depende de
  B1. Foto calibrada de cada corte (prueba del protocolo de B7.4).
- **1–2 cortes por material a feed y profundidad nominales**: panceta y zapallo
  sobre su **pieza aparte** (D9), recortada igual que la de campaña; EVA y
  Ecoflex en carriles de su propia muestra (§3.5 a). Con la pieza aparte de
  panceta y zapallo pueden hacerse 3–4 cortes, lo que mejora la identificación.
- Salidas: perfiles de fuerza, parámetros identificables de §5.2, cota `d̄_cut`
  por junta (para `g5`), validación de `p = 15 mm` entre carriles y viabilidad
  del zapallo (fuerza máxima frente a la resistencia de hoja/acople).
- Subproducto publicable: el controlador de posición nativo de UR como
  **referencia industrial** de calidad de corte (opcional en el paper).
- Prerrequisitos: fijación, muestras recortadas planas, `surface_z` por carril
  (§3.5 c–d), protocolo de hoja (§7 del plan original).

### B3 — Gemelo con tejido y sensado realista

| # | entregable |
|---|---|
| B3.1 | Plugin de corte (§5.2) con los parámetros de B2; reproduce los perfiles medidos (criterio §5.3). |
| B3.2 | Relé de sensor en sim: `q` cuantizada a 2²⁰ cuentas/vuelta y `q̇` por diferencia finita a 500 Hz. Comprobar que `σ(q̇ − q̇_d)` en sim ≈ 2.9e-3 rad/s como en `smc_712`. |
| B3.3 | Control disparado por tiempo de simulación (lockstep a 500 Hz) como opción; histograma de `dt`. |
| B3.4 | `wrist_3` con solo Coulomb: su viscoso aporta ≤ 0.088 N·m en toda la incisión (≤ 0.006 en la meseta) frente a 2.35 de Coulomb, y es el término que congela el integrador (`b·dt/I` = 11). Declararlo. Validar la inyección en `wrist_1`/`wrist_2`. |
| B3.5 | Validación sim↔real en aire con la escena nueva (B7.1) y criterio §5.3. |

### B4 — Leyes reformuladas (sin robot, en paralelo con B1–B3)

| # | entregable |
|---|---|
| B4.1 | ✅ (`9ccb9be`) CT: ganancias por junta desde YAML, `G` y chequeo discreto en el banner. `G` común a las tres leyes (`reportLoopGain`: diagonal y suma por fila) y `g_loop_max` en el bloque común. Referencia congelada: CT nominal G = 52/60/19, ρ = 0.983, margen ×25; SMC `smc_v5_g6` 83.1/106.7/81.4 en diagonal pero **101/168/98 con acoplos**. |
| B4.2 | ✅ LQI-SDRE (§4.3) en `lqr_sdre_law.hpp` + nodo: `lqr.integral` (18 estados; `false` = la FASE 4, ablación), `lqr.weights` `polynomial` (semilla: K = [a₀M, a₁M, a₂M], un CT con integral) o `physical` (Q, R diagonales constantes, para B5), integral solo en TRACK/HOLD_END, congelada por junta con `onSaturation` y acotada a `lqr.integral_tau_frac`·τ_max/K_I. Tests: 18 polos en las raíces (< 1e-6), K analítica (< 1e-8), CARE = scipy (< 1e-8), anti-windup, CARE de 18 estados en **176 µs** en banco. Semilla por defecto ωn = 10 (con 20, G = 160 en el hombro): G = 33.8/80.3/28.8. Gazebo (`lqr_98823`): corte **0.0133 mm** frente a 0.0425 del LQR de 12 estados, max Re(λ) ≤ −3.4, \|σ\| ≤ 1.2e-3. **Cómputo con Gazebo cargando la CPU: ley media 807 µs, p99 1673, > 2 ms en el 0.20 % de los ciclos** (la CARE sola nunca; el resto es la vigilancia de autovalores y controlabilidad de 18 estados). En el real, sin Gazebo, sobra; si no, `lqr.care_update_rate`. |
| B4.3 | ✅ R1 y R2 (§4.4) en `smc_law.hpp` (ley pura, la usa el nodo) **y** en `SmcLaw` del evaluador; `alpha_M`/`alpha_C`/`alpha_g` (= `alpha` hasta medir `α_g` en B4.4). Test dorado C++↔Python: mismo par en 200 casos (< 1e-9). Con `smc_v5_g6` sin re-sintonizar: G 83.1/106.7/81.6, corte 0.0025 mm (antes 0.0036) y TV(τ) 21 (antes 50) en Gazebo (`smc_98822`). **R3**: `g5` ya exige η ≥ d̄_f + d̄_cut (`run_gain_tuning`); falta sustituir la `d̄_cut` del corte sintético por la envolvente medida de los cuatro materiales (B2). |
| ⚠️ | **Regresión de B0 encontrada y corregida (`7bfcf55`)**: con el común `/**` delante y los YAML de ley con sección `<nodo>:`, los overrides de los launches (comodín en `launch_ros` si el `Node` no lleva nombre) se ignoraban en silencio en las incisiones (`lambda_joint`, `phi`, `alpha`, `wn`, `kp`/`kd`), también en el real. Ahora los YAML de ley van en `/**` y los launches nombran el `Node`; dos tests lo fijan. |
| B4.4 | 🔶 **Preparado (2026-10-28), sin ejecutar.** `α_g` = `|g_real − g_UR| / |ĝ|` (§4.4), con **barridos** y no con sostén: quieto, la fricción estática de hombro y codo (±7 N·m) tapa un error de ~1.5. Por junta y postura, `g_real = ½[k⊙i(+v) + k⊙i(−v)] − Cq̇` a q emparejada (la fricción es impar) y `g_UR = k⊙i − τ_cmd(k − D)` con D = 5 (V15). 4 posturas sobre el trazo (inicio y tres a 60 mm de la cota del corte) × hombro, codo, `wrist_1`, ±0.05 rad a 0.05 y 0.10 rad/s, CT por par con acople (sin hoja) y payload = acople pesado. Ganancias 100/20 y 900/60 en muñecas (margen ×2.2 con D = 5). Herramientas: `config/gravity_sweep_params.yaml`, `run_gravity_sweeps_real.py` (semiautomático, holgura ≥ 71 mm al tablero comprobada, nunca sobrescribe), `analyze_gravity_sweeps.py`, `gravity_sweeps.py` (6 tests: IK, holgura, cancelación de fricción, D y `k` recuperados de un CSV sintético). **Gazebo con la fricción real inyectada** (`fl_98831`–`98833`, `corte_medio`): `g_real − ĝ` = 0.004 / 0.002 / 0.0002 N·m, igual a las dos velocidades. Protocolo: `09a_session_protocol.md` §3. |
| B4.5 | Las tres leyes en el gemelo (aire + fuerza sintética corregida). |

### B5 — Sintonía equitativa (FASE 7 ampliada)

- Parametrizaciones: CT `[kp_i, kd_i]` (12) · LQI-SDRE `[Q_σ,ii, Q_p,ii, Q_v,ii]`
  con `R` fija por Bryson (18, escala logarítmica) · SMC `full_phi` (18).
- Objetivos `f1` IAE cartesiano, `f2` esfuerzo, `f3` TV(τ); restricciones
  `g1` par, `g2` velocidad, `g4` TCP ≤ 2 mm, `g6` `G ≤ 120`, más la discreta de
  cada ley (`χ` normalizado para el SMC; radio espectral con retardo para CT y
  LQR) y `g5` (SMC).
- **Un juego por ley para los 4 materiales (D3):** objetivos promediados sobre
  los 4 modelos de tejido y restricciones en el peor caso. Coste estimado ≈ 4×
  el de `smc_v5_g6` por ley (~132 min × 4 con `-j 8`): se corren de noche.
- Método: NSGA-II como principal; certificación KKT por refinamiento local
  SLSQP **desde la rodilla** (la ε-restricción desde cero no converge en 18
  variables: 0/12 en `smc_v5_g6`). La comparación de métodos se reporta solo
  donde converge (parametrización escalar) o se retira.
- **Cierre:** tres `selected_gains.yaml` factibles, cada uno re-verificado en el
  gemelo con tejido.

### B6 — Campaña en el gemelo (FASE 8)

- Runner desatendido (`scripts/run_campaign.py`) con limpieza de Gazebo por PID
  (trampas conocidas de huérfanos, `/clock` viejo).
- Matriz: 3 leyes (+ ablación LQR sin integral) × {aire, 4 materiales} × 4
  condiciones: nominal; modelo del controlador con gravedad ±7 % y acople ±20 %;
  fricción F_v ±10 % / F_c ±5 % (térmico); propiedades del tejido ±30 %.
  Implementación: parámetros que alteran **solo el modelo del controlador** (o
  solo la planta), nunca la ley.
- Repeticiones N = 5 con **fuentes de variación sembradas** (ruido del tejido,
  altura de superficie ±0.5 mm, fase de cuantización): sin ellas Gazebo es
  determinista y N > 1 no significa nada.
- **Cierre:** ninguna corrida nominal abortada; tabla de métricas por celda;
  selección del controlador (E3) y **predicción** cuantitativa para B7.

### B7 — Campaña real (FASE 9)

| # | paso |
|---|---|
| B7.0 | 🔶 **Preparado (2026-10-28).** Sesión: **payload = la configuración montada** (brida desnuda → 0; acople → masa pesada y CoG del CAD (−2.2, 2.1, 29.4) mm; la gravedad la compensa el robot), G4 = 0.0, calentamiento, `check_session_config.py --payload-kg`, guardas activas, §7 firmado. **Retardo: medido con datos existentes, 5 ciclos** (§2.7, V15); B4.4 lo re-mide con el acople. **Seguridad** (`09a_session_protocol.md` §2): `kill -9` al nodo en sostén → el vigilante `command_deadman.py` (nuevo, 3 tests, probado en Gazebo: 100 ms) corta y devuelve el mando al JTC; Ctrl-C → par cero; desenchufar Ethernet → `RobotReceiveTimeout` (20 ms, fijo) para el robot. |
| B7.1 | Bring-up en aire **por ley** en la escena nueva (sin hoja → hoja al aire con `ft_data ≈ 0`): barrido en `q_init`, barrido de la base en la pose de la incisión, incisión en aire. Empezar por el SMC (equivale a 740–742) en cuanto exista la escena: valida la `G` de la base que usan las tres sintonías. **No con `smc_v5_g6`:** con el retardo real (V15) sus muñecas son inestables en el análisis lineal; antes, re-sintonizar con D = 5 (R10). El CT nominal (×7.0) sí puede ir. |
| B7.2 | Corte piloto con par en un carril de EVA (la muestra con más área), una vez por ley. |
| B7.3 | Campaña: por material, en **una sesión**, rondas aleatorizadas (§3.5 f) hasta N = 3 por ley; CSV + `.params.yaml` + `ft_data` + foto + metadatos. |
| B7.4 | **Medida del corte (D6), protocolo escrito antes de cortar** (`docs/09b_cut_protocol.md`): cámara calibrada (intrínsecos con tablero; escala con una referencia en el plano de la superficie de cada muestra) en vista superior → longitud, rectitud (desviación máxima a la recta objetivo) y ancho, foto tras cada corte. **Profundidad**: una vista superior no la da; se propone una **sección transversal al final de cada material** (corte perpendicular a todos los carriles en 3 estaciones, fotografiado con la misma cámara), que mide la profundidad de todos los cortes a la vez. |
| B7.5 | Validación del estimador de fuerza: `F̂` por corriente vs `ft_data` (correlación, sesgo). |

### B8 — Análisis y paper (FASE 10)

Métricas con una sola definición (RMSE/máx/IAE de TCP en la meseta,
`θ_err = ‖log(R_dᵀR)‖`, TV(τ), ∫Στ², tiempo de cómputo, métricas del corte),
contraste con la repetibilidad del UR5e (±0.03 mm), brecha sim↔real como
resultado propio, y `docs/paper_artifacts.md` (figura → script → CSV → bloque).

Estadística con el diseño de §3.5: Friedman entre las 3 leyes con bloques =
material × ronda (4 × 3 = 12 bloques), post-hoc de Wilcoxon con Holm y tamaño
del efecto; con N = 3 por celda el peso descansa en las comparaciones pareadas
dentro de cada ronda y en la consistencia del orden entre materiales, no en
p-valores por celda.

**Hipótesis que la matriz debe poder confirmar o refutar:**

- H1: el SMC optimizado da mejor calidad de corte que el LQR-SDRE con material desconocido.
- H2: el LQR-SDRE da menor esfuerzo y menor TV(τ) que el SMC.
- H3: ambos mejoran al CT con la misma restricción de `G`.
- H4: el gemelo predice el orden y la magnitud de las métricas reales (§5.3).

### B9 — ASTSMC

Fuera de esta fecha (D8).

### B10 — Trabajo futuro: corte curvo con la hoja tangente (D22)

Después de cerrar la campaña del corte recto. El trazo pasa a ser una curva
plana sobre la superficie (arco, clotoide o spline) y la hoja tiene que ir
**tangente a la trayectoria** con el filo por delante, como hoy en la recta
(D18): el ángulo de guiñada del TCP sigue al de la tangente, `ψ(s)`, y lo
absorbe sobre todo `wrist_3`.

Lo que cambia respecto a lo que hay:

- **Levanta el supuesto A4** (orientación constante). `IncisionTrajectory`
  necesita `orientation(t)`, y `JointReferenceGenerator` el bloque angular de
  `ẋ` y `ẍ`, que hoy fija a cero (`ω = ψ̇ ẑ`, `ω̇ = ψ̈ ẑ`). La IK ya resuelve con
  la orientación de cada muestra.
- **Métricas nuevas:** error de orientación del filo respecto a la tangente
  (guiñada), además del de posición; error lateral respecto a la curva.
- **Límites de la geometría:** el giro total de `wrist_3` (±2π en el UR5e) y la
  **curvatura mínima**: la hoja, con longitud `l` dentro del material, barre
  lateralmente `l²/(8R)`; con `l` ≈ 10 mm y `R` = 50 mm son 0.25 mm. Hay que
  fijar `R_min` con pilotos.
- **Control:** `wrist_3` deja de estar parada en el corte, así que vuelve su
  fricción a baja velocidad (la junta que más costó en la FASE 2 real). La
  comparación de las tres leyes se repite con la misma metodología (B5–B8).
- **Paper:** hoy se declara que el trabajo no demuestra seguimiento de
  orientación bajo par (A4); B10 sería esa demostración.

---

## 7. Datos físicos pendientes (no se inventan)

| dato | para qué | cómo |
|---|---|---|
| ~~distancia del eje del robot a la mesa~~ | — | **medida: 5 cm de holgura, borde a 0.426 (D16)** |
| dimensiones de Ecoflex y zapallo | capacidad de carriles (§3.5 a) | regla/calibrador (§9.7 R2) |
| ~~espesor de la bandeja~~ | — | **2 mm (D17)** |
| `surface_z` de cada carril | profundidad real del corte | toque con la punta en 3 puntos antes de cortar |
| masa del acople | A1, gravedad en muñeca | báscula |
| `tool0` → punta de la hoja | A2, error de TCP | calibrador (resuelve también la discrepancia de 2.5 mm de `00_assumptions`) |
| intrínsecos y escala de la cámara | B7.4 | tablero de calibración + referencia en el plano de cada muestra |
| retardo de tubería real | evaluador | B7.0 |
| `α_g` | SMC R2 | B4.4 |
| fuerzas de corte por material | modelo de tejido, `d̄_cut` | B2 |
| resolución efectiva del F/T integrado | qué se puede identificar con él | hoja técnica + pesos conocidos (B1) |

---

## 8. Riesgos

| riesgo | consecuencia | mitigación |
|---|---|---|
| **calendario** (9 semanas con fiestas) | campaña real comprimida | §10: escritura en paralelo desde la semana 1; sintonías de noche; ASTSMC fuera |
| Ecoflex y zapallo sin medir (la panceta ya va en bloque, D14) | N < 3 o trazo más corto | decidir en B0.1; el trazo se acorta para todos, nunca por material |
| perecibles (panceta, zapallo) | propiedades cambian durante la sesión | sesión única, orden aleatorizado, minutos desde apertura registrados |
| superficie irregular | profundidad real ≠ 5 mm | recorte plano + `surface_z` por carril |
| fuerzas de Ecoflex/EVA bajo la resolución del F/T | modelo de tejido mal identificado | cero antes del contacto, promediado en meseta, estimador por corriente (§3.3) |
| zapallo demasiado duro | rotura de hoja/acople, paro protector | pilotos en posición (B2); reducir profundidad para ese material y declararlo |
| cambio de la holgura en el laboratorio (es modificable, D16) | escena de Gazebo distinta de la real | la holgura se registra por sesión (B0.5) y la `x` de `u_table` se actualiza si cambia |
| CT/LQR no alcanzan 2 mm bajo carga dentro de `G ≤ 120` | infactibilidad en B5 | resultado reportable; se relaja `g4`, nunca `g6` |
| CARE de 18 estados fuera de presupuesto | lazo lento | decimar a 100 Hz con ZOH (cuantificado en `04`) |
| la base no aguanta la `G` pedida en la pose de la incisión | rehacer sintonías | B7.1 con el SMC en cuanto exista la escena |
| stiction/Stribeck no identificados | picos de error en arranques y cruces por cero | limitación declarada |
| modo estructural no simulable | brecha sim↔real en chattering | `g6` común; declararlo en *Limitations* |

---

## 9. Respuestas recibidas y lo que aún necesito

### 9.1 Respuestas (2026-10-27) → §0.1

P1 mesa en 0.85 y se acerca el robot (D1) · P2 leyes confirmadas (D2) · P3 un
juego por ley (D3) · P4 solo F/T integrado (D4) · P5 una muestra por material,
dimensiones después (D5) · P6 cámara calibrada (D6) · P7 umbrales aceptados
(D7) · P8 envío a fin de 2026 (D8).

### 9.2 Segunda ronda (2026-10-27) → §0.1

Q1 pieza aparte de panceta y zapallo: sí (D9) · Q2 panceta 200 × 50 × 40, EVA
400 × 250 × 40; Ecoflex y zapallo después (D10) · Q3 piel arriba y recorte plano:
sí (D11) · Q4 sección transversal: sí (D12) · Q5 se acerca el pedestal sin
chocar con la mesa, geometría en `lab_incision_world.sdf` (D1, V13) · Q6
fijación indexada: sí (D13).

### 9.3 Tercera ronda (2026-10-27) → §0.1

R1 panceta en bloque de ≥ 150 × 130 mm: sí (D14) · R3 los modelos de pedestal y
mesa estaban desactualizados; cotas reales y modelos nuevos entregados →
sustituidos en el mundo antes de B0 (D15, §2.5, V13).

### 9.4 Cuarta ronda (2026-10-27) → §0.1

R3 robot centrado en el mounting base; 5 cm de holgura medida con la mesa,
modificable (D16) · R4 bandeja de 2 mm (D17) · R5 iniciar B0 tras las
modificaciones, con el cambio de escena en un commit aparte · R2 aún sin medidas.

### 9.5 Quinta ronda (2026-10-27) → §0.1

R7 retener Pinocchio si hace falta: retenido con sus dependencias (D20) · R6
centrado lateralmente: sí (D19) · R2 sin materiales aún: se piden dimensiones
recomendadas para los cuatro (D21, §3.5 h) · R9 trazo confirmado; el corte curvo
con la hoja tangente queda como trabajo futuro (D22, B10).

### 9.6 Sexta ronda (2026-10-27)

R8 `smc_605` la lanzó el usuario para visualizar el robot en Gazebo (§2.4); no
hay corrida suya en `smc_1.csv`, así que lo que sobrescribieron los `dry_run`
era un artefacto de desarrollo · commit y push del txt corregido · arrancar S2.

### 9.7 Séptima ronda (2026-10-28)

Push de los 6 commits de S2 · B1 se cierra con brida libre (800), Robotiq 2F-85
(801) y acople (802), no colgando un peso del acople · `α_g` con **nuevos
barridos** · el retardo, con los datos que ya hay · preparar scripts y protocolo
de lo que falte · corregir B4.4, el payload con acople y `RobotReceiveTimeout`.

### 9.8 Pendiente

| # | pregunta | por qué importa |
|---|---|---|
| **R2** | Conseguir las muestras según §3.5 (h) y confirmar sus medidas reales al tenerlas. | Fija N por material; la referencia no cambia salvo que obligue a acortar el trazo |
| **R10** | ¿Llevar D = 5 (V15) al evaluador (`PIPELINE_DELAY_STEPS`) y a la restricción discreta de las tres leyes en B5, y re-sintonizar el SMC antes de B7.1? | Con D = 1 el optimizador premia ganancias que el real no tolera; `smc_v5_g6` es inestable en las muñecas con D = 5 |

---

## 10. Calendario hasta fin de 2026

| semana | fechas | bloques | hito |
|---|---|---|---|
| S1 | 27-oct → 1-nov | ~~B0~~ (**cerrado el 27-oct**, §2.6); arranca B4.2 | referencia única en la escena nueva |
| S2 | 2 → 8-nov | B1; B4.1–B4.3; **B0.1 (geometría congelada: R2, R4)**; B7.0 (retardo, `RobotReceiveTimeout`); B4.4; fabricación de la fijación | `ft_data` en el CSV; leyes reformuladas con tests; referencia definitiva. **Hecho sin robot:** B1, B4.1–B4.3, retardo (V15), herramientas y protocolo de B4.4 y B7.0. **Falta en el robot:** cierre de B1, B7.0, B4.4 |
| S3 | 9 → 15-nov | B2 pilotos en posición (4 materiales); B3.2–B3.4; B7.1 SMC en aire | primeros perfiles de fuerza reales |
| S4 | 16 → 22-nov | B3.1 plugin + validación; B4.5; arranque de B5 (noches) | gemelo con tejido validado |
| S5 | 23 → 29-nov | B5 (3 leyes) + re-verificación en el gemelo; B7.1 CT y LQR en aire | tres juegos de ganancias aprobados |
| S6 | 30-nov → 6-dic | B6 (noches); B7.2; fijación y protocolo B7.4 escritos | predicción para el real; go/no-go de la campaña |
| S7 | 7 → 13-dic | **B7.3 campaña real** (EVA, Ecoflex, zapallo, panceta) + secciones transversales | datos reales completos |
| S8 | 14 → 20-dic | B8: métricas, estadística, figuras, tablas | resultados cerrados |
| S9 | 21 → 31-dic | redacción final, revisión interna, envío | **envío** |

La redacción de *Introduction*, *Related work* y *Methods* (leyes, gemelo,
protocolo) va en paralelo desde S2; con este plan son estables.

**Puntos de decisión:** fin de S2 (capacidad de muestras → largo del trazo
congelado), fin de S4 (gemelo validado o no → si no, B5/B6 con la fuerza
sintética recalibrada a los niveles medidos y se declara), fin de S6 (go/no-go
de la campaña real).

---

## Anexo A — Cómo se obtuvieron las verificaciones

| | método |
|---|---|
| V1 | `colcon test --packages-select ur5_dyn_control ur5_identification ur5_trajectory_optimization` + `colcon test-result --all` |
| V2 | Pinocchio sobre `ur5_kinematics/urdf/ur5e.urdf` + acople (`appendBodyToJoint`, mismos valores que el YAML), cada 20 muestras de `~/.ros/ur5_dyn_control/incision_ref_v2.csv`; LQR con `scipy.linalg.solve_continuous_are` y la `Q` de `buildQ()`. El SMC reproduce el banner del nodo (104.8 / 122.1 / 81.4). |
| V3 | `gain_tuning.closed_loop.simulate` sin modificar, con `Plant(tcp_offset=0.162686)`, `CuttingForce` y máscara de meseta desplazadas +1.736 s, leyes CT y LQR-SDRE escritas con la misma firma que `SmcLaw`. |
| V4, V6, V12, V13 | `gz_*_control_node` con `dry_run:=true perform_switch:=false perform_unpause:=false use_sim_time:=false reference_table_out:=…` (y `incision.cut_center_y:=…`, `incision.surface_z:=…`) en un `ROS_DOMAIN_ID` aislado (no se publica par). |
| V13 (escena) | Rev. 3: vértices de `surgery_table.dae` y `ur5_base.dae` de `ur5_pick_place` (desactualizados). Rev. 4: envolventes por grupo de los OBJ del CAD (`ur5_dyn_control/meshes/{ur5_mounting_base,u_table}/meshes/*.obj`, mm), contrastadas con las cotas del PDF; transformación malla → modelo comprobada numéricamente y por render (§2.5). |
| V5 | FK de `incision_ref_v2` con TCP 0.162686: penetración hasta 15.336 s, `z_cut` = 0.025 m. |
| V7, V8, V11 | lectura de `torque_control_node_base.{hpp,cpp}`, cabeceras de `smc_605.csv`, `gz_smc_control_node.cpp`. |

Los scripts fueron ad hoc; si se quieren versionados, van a
`ur5_identification/scripts/` en B0.
