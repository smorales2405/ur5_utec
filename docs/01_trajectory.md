# FASE 1 — Trayectoria de incisión: cúbico → quíntico, feed constante, 5 fases

Cierre de la FASE 1 del plan `PLAN_INCISION_UR5e.md`.

- **Fecha:** 2026-07-30
- **Prerrequisito:** [FASE 0](00_prereqs.md) cerrada (G1–G6)
- **Supuestos vigentes:** [A1–A4](00_assumptions.md)

---

## 1. Parámetros de la incisión (decisiones del usuario)

| Parámetro | Valor | Justificación |
|---|---|---|
| Superficie del tejido | `z = 0.02 m` (base_link) | La mesa de cirugía queda exactamente en `z = 0.00`: pedestal del robot y mesa miden ambos 0.63 m. Es el espesor de una muestra fina apoyada sobre ella. |
| Longitud del corte **medida** | **80 mm** | Largo frente a la repetibilidad del UR5e (±0.03 mm) y a los ~0.03 mm de RMSE del lazo. Se recorre **íntegra** a feed constante. |
| Entrada / salida (`cut_lead`) | **10 mm** a cada lado | Las rampas de aceleración y frenado ocurren aquí, fuera del tramo medido. Trazo total de material: **100 mm**. |
| Profundidad | **5 mm** | Medible con calibrador; alcanzable en los 4 materiales. |
| Velocidad de avance (feed) | **10 mm/s** | Régimen cuasi-estático de la literatura de corte de tejido blando. |
| Dirección | a lo largo de **Y**, `x = 0.50 m` | σ_min(J) varía 1e-4 a lo largo del trazo (frente a 1e-2 si el corte fuera radial): el brazo ve la **misma planta** durante todo el corte, así que las diferencias entre controladores no se confunden con cambios de condicionamiento. |
| Cota de aproximación | 30 mm sobre la superficie | |

```
   z [m, base_link]
  0.35 ┤ ●  start_pose = FK(q_init) = [0.49, 0.13, 0.35]
       │  ╲  approach (100 mm/s)
  0.05 ┤   ●───────────────────────●   z_above = 0.050
       │   │ contact (15 mm/s)     ↑ withdraw (50 mm/s)
  0.02 ┤   ●━━━━━━━━━━━━━━━━━━━━━━━┿━━━  superficie del tejido
       │   │ penetration (5 mm/s)  │
 0.015 ┤   ●──═══════════════════──●   z_cut = 0.015
       └───┴──┴───────────────┴──┴──────→ y [m]
        −0.05 −0.04         +0.04 +0.05
             │←── 80 mm MEDIDOS ──→│      a feed EXACTAMENTE constante
        │←────── 100 mm de trazo ──────→│  material cortado (rampas incluidas)
```

| Fase | t [s] | Longitud | v [mm/s] | Meseta de feed constante |
|---|---|---|---|---|
| `approach` | 0.00 – 6.90 | 345.0 mm | 100 | — |
| `contact` | 7.20 – 11.20 | 30.0 mm | 15 | — |
| `penetration` | 11.50 – 13.50 | 5.0 mm | 5 | — |
| `cut` | 13.80 – 23.40 | **80.0 mm** | **10** | **15.40 – 21.80 s → 64 mm** |
| `withdraw` | 23.70 – 25.10 | 35.0 mm | 50 | — |

Duración total **27.60 s** (incluye 0.3 s de reposo entre fases). Config en
[`ur5_dyn_control/config/incision_params.yaml`](../ur5_dyn_control/config/incision_params.yaml).

> **Los 80 mm que se reportan se recorren íntegros a feed constante.** Las
> rampas de aceleración y frenado son físicamente inevitables —el bisturí
> arranca en reposo tras la penetración— así que se sacan del tramo medido:
> el trazo se alarga 10 mm por cada lado (`cut_lead`) y se corta 100 mm de
> material para medir 80. La fracción de rampa del perfil **no es un parámetro
> libre**: se deriva de `cut_lead` para que la meseta coincida exactamente con
> `cut_length`.
>
> Un perfil quíntico puro —la otra opción que contempla el plan— **no tendría
> ninguna región de feed constante**, así que la S-curve con meseta es la única
> forma de cumplir el criterio.

---

## 2. Criterios de aceptación

| Criterio del plan | Resultado | Estado |
|---|---|---|
| Derivadas analíticas vs diferencias finitas: error < 1e-6 | quíntico y trayectoria compuesta: **< 1e-6** | ✅ `test_quintic_spline`, `test_incision_trajectory` |
| Continuidad de jerk en los nudos (el cúbico falla, el quíntico pasa) | quíntico: salto **< 1e-6**; cúbico: salto **> 1e-3** | ✅ ambos verificados |
| Feed constante dentro de ±2 % en el tramo `cut` | referencia analítica **< 1e-9**; ejecutado en Gazebo **0.58 %** máx sobre los 80 mm medidos íntegros | ✅ |
| `q̇`, `q̈` dentro de límites del UR5e | `\|q̇\|` máx 0.238 rad/s (**margen 92.4 %**), `\|q̈\|` máx 0.383 rad/s² (**margen 92.3 %**) | ✅ |
| σ_min(J) por encima del umbral | **0.2168** (umbral 0.05); `w_min` = 0.0677 | ✅ |
| Regresión: `gz_fl_control_node` sigue funcionando | corrida completa en `lab_torque_world` | ✅ |

### Corridas de regresión

Trazo definitivo (100 mm de material, 80 mm medidos), 13 803 muestras de TRACK:

```
RMS error articular   0.000054 rad
RMS error TCP         0.0273 mm         máx 0.1632 mm

En la incisión MEDIDA (80 mm, toda ella en la meseta):
  feed referencia     desviación media 0.005 %, p99 0.11 %, MÁX 0.23 %
  feed ejecutado      desviación media 0.042 %, p99 0.22 %, MÁX 0.58 %   (criterio < 2 %)
  profundidad         desviación máx 0.00008 mm
  rectitud (en x)     desviación máx 0.00088 mm
  longitud medida     79.979 mm   (el déficit de 21 µm es un paso de muestreo)
  trazo total         100.000 mm
```

Corrida previa en `lab_torque_world.sdf` con el trazo de 80 mm (64 medidos):
RMS articular 0.000059 rad, TCP RMS 0.0286 mm, `|tau|` máx
`[0.08, 27.35, 20.24, 1.85, 0, 0]` N·m frente a límites `150/150/150/28/28/28`.

`~/.ros/ur5_dyn_control/fl_104.csv` (lab world) y `fl_105.csv` (trazo definitivo).

---

## 3. Dos defectos encontrados y corregidos

### 3.1 La referencia articular avanzaba a escalones de 100 µm

`UR5Kinematics::inverseKinematicsQP2` **para cuando `‖error‖ < 1e-4`**
(criterio fijo dentro de `ur5_kinematics`, `src/kinematics.cpp`). A 10 mm/s y
500 Hz el avance por muestra es de **20 µm**, cinco veces menor que esa
tolerancia: el solver devolvía **la misma `q` durante 4–5 muestras seguidas** y
la referencia cartesiana avanzaba a saltos de 100 µm.

Consecuencias medidas en Gazebo antes del arreglo:

- rizado de **±25 %** en la velocidad de avance ejecutada (criterio: ±2 %);
- escalón de 100 µm = **3× la repetibilidad nominal del UR5e** (±0.03 mm) y del
  mismo orden que el RMSE que hay que reportar;
- incoherencia interna de la referencia: `q_des` en escalera pero `dq_des` y
  `ddq_des` suaves (se calculan analíticamente del Jacobiano), así que el
  término de *feedforward* del FL peleaba contra el de realimentación.

**Corrección:** refinamiento de Newton amortiguado (`JointReferenceGenerator::refineIk`)
sobre el error de pose 6D, partiendo de la solución del QP y usando el
Jacobiano que ya se calcula para `dq`/`ddq`. Baja el error a ~1e-12 m.
**No se tocó `ur5_kinematics`** (§5 del plan). Verificado: la referencia avanza
ahora **20.00 µm por muestra exactos** en la meseta.

### 3.2 El CSV tenía 6 decimales (1 µm)

Insuficiente para la evidencia de esta fase: 1 µm es el 5 % del avance por
muestra y el 2.5 % del RMSE a reportar. Al derivar el CSV para medir el feed, el
ruido de cuantización tapaba la señal (la referencia —exacta por construcción—
aparentaba un 10 % de desviación en ventanas de 0.1 s). Subido a **9 decimales**
(1 nm) en `CsvLogger`. El esquema de columnas no cambia; el esquema unificado de
trazabilidad es entregable de la FASE 3.

---

## 4. Observación para las FASES 3 y 8: jitter del lazo

El lazo de control es un timer de **pared** a 500 Hz mientras el reloj de
**simulación** avanza en pasos de 1 ms (mundo a 1 kHz). El índice de la tabla se
calcula con el tiempo de simulación, así que en un 0.1 %–16 % de los ciclos
—según el RTF de la corrida— el índice se repite o salta uno.

Esto **no** afecta a la trayectoria (que es exacta) ni al criterio del feed
medido con ajuste por mínimos cuadrados, pero:

- es la fuente dominante de variación entre corridas nominales (ya se observó en
  la FASE 0: dos corridas idénticas de FL diferían en un desfase de +1 ms);
- hay que **caracterizarlo con un histograma de `dt`** antes de comparar
  controladores en la FASE 8, para no atribuir a un controlador lo que es
  temporización;
- en el robot real no existe en esta forma: el lazo lo marca el ciclo de 500 Hz
  del driver. La tabla de riesgos del plan ya lo anticipa ("jitter del lazo en
  Linux no-RT"), y el watchdog de la FASE 3 es el sitio natural para medirlo.

**Al medir el feed desde un CSV hay que usar ajuste por mínimos cuadrados sobre
una ventana** (que usa los instantes reales), no `np.gradient` muestra a
muestra: con diferencias simples, unas pocas filas con `dt ≠ 2 ms` dominan el
`max` y dan desviaciones aparentes del 25 %.

---

## 5. Archivos

**Nuevos**

| Archivo | Contenido |
|---|---|
| `include/ur5_dyn_control/cartesian_trajectory.hpp` | Interfaz abstracta `CartesianTrajectory` |
| `include/…/quintic_spline.hpp` · `src/quintic_spline.cpp` | `QuinticSpline3d`: Hermite quíntico, continuidad C³/C⁴ en nudos interiores, nudos no uniformes, contorno `CLAMPED_REST` / `CHORD_TANGENT` |
| `include/…/quintic_spline_trajectory.hpp` · `.cpp` | `QuinticSplineTrajectory` (drop-in del cúbico) |
| `include/…/gauss_legendre.hpp` | Cuadratura de Gauss-Legendre — port de `numerical_integration.py` del CU3 (nodos por Newton sobre P_n, pesos de Bonnet) |
| `include/…/arc_length.hpp` · `src/arc_length.cpp` | `s(u)`, `u(s)`, y `du/ds`, `d²u/ds²`, `d³u/ds³` analíticas |
| `include/…/time_profile.hpp` · `src/time_profile.cpp` | `ScurveProfile` con jerk acotado y meseta |
| `include/…/incision_trajectory.hpp` · `src/incision_trajectory.cpp` | Las 5 fases + regla de la cadena hasta jerk |
| `config/incision_params.yaml` | Parámetros de la campaña |
| `test/test_quintic_spline.cpp` (9) · `test/test_incision_trajectory.cpp` (22) | Criterios de aceptación |

**Modificados:** `cartesian_spline_trajectory.{hpp,cpp}` (implementa la interfaz,
expone `jerk()`), `joint_reference_generator.{hpp,cpp}` (interfaz genérica,
refinamiento de IK, manipulabilidad, límites articulares, diagnóstico),
`torque_control_node_base.{hpp,cpp}` (`trajectory_type`, bloque `incision.*`,
log de diagnóstico), `csv_logger.cpp` (precisión), `CMakeLists.txt`, `README.md`.

---

## 6. Artefactos de paper que habilita esta fase

- **Figura**: trayectoria de incisión por fases (perfil `z(t)` y planta `x-y`),
  con las 5 fases marcadas y la meseta de feed constante sombreada.
- **Tabla**: waypoints, tiempos, longitud, profundidad y velocidad por fase
  (§1 de este documento).
- **Figura**: comparación cúbico vs quíntico en jerk. Los datos están en
  `test_quintic_spline` (salto de jerk en los nudos: > 1e-3 vs < 1e-6) y la
  clase `CartesianSplineTrajectory` se conserva a propósito para generarla.
- **Dato para *Methods***: σ_min(J) = 0.2168 y márgenes de q̇/q̈ del 92 %,
  es decir la incisión se ejecuta lejos de singularidades y de los límites del
  robot.
