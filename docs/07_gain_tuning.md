# FASE 7 — Sintonía multiobjetivo de ganancias

Cierre de la FASE 7 del plan `PLAN_INCISION_UR5e.md`.

- **Fecha:** 2026-07-30
- **Módulo:** `ur5_trajectory_optimization/ur5_trajectory_optimization/gain_tuning/`
- **Entrada:** `reference_table_out` del nodo · **Salida:** `results/gain_tuning/smc/`
- **Prerrequisito:** FASE 5 (un controlador terminado con el que validar el evaluador)

---

## 1. Arquitectura

```
closed_loop.py   Plant (Pinocchio, construido una vez) + simulate() con ABA
                 CuttingForce (F_ext sintética)  ·  SmcLaw (espejo del nodo C++)
problem.py       SmcParameterization (scalar | full) · GainEvaluator (con caché)
                 GainTuningProblem (pymoo)
optimize.py      NSGA-II · ε-restricción+SLSQP · certify_kkt · línea base
run_gain_tuning.py  orquestador y salidas
```

### 1.1 La referencia no se reimplementa en Python

El evaluador carga la tabla `{q, q̇, q̈}` que **vuelca el propio nodo** con el
parámetro `reference_table_out`, producida por `IncisionTrajectory` + IK QP +
refinamiento de Newton en C++. Una segunda implementación en Python divergiría
en silencio y se estarían optimizando ganancias para una trayectoria distinta de
la que va al robot. Con el volcado, optimizador y nodo comparten referencia byte
a byte.

### 1.2 Coste por evaluación

**0.70 s** por evaluación de lazo cerrado (13 802 pasos a `dt` = 2 ms), frente a
~90 s de una corrida de Gazebo: **≈ 130× más barato**. Con 16 procesos bajan a
0.13 s efectivos, y NSGA-II de 7 200 evaluaciones cabe en un cuarto de hora.

Dos decisiones dieron la mayor parte de ese margen:

- **`Plant` construye el modelo una sola vez.** Reconstruir el modelo desde el
  URDF en cada evaluación costaba más que varios pasos de integración.
- **Caché LRU en `GainEvaluator`.** SLSQP pide el objetivo y *cada* restricción
  por separado en el mismo punto, y las diferencias finitas repiten puntos entre
  iteraciones; sin caché la ε-restricción costaría cuatro veces lo necesario.

---

## 2. La restricción que aporta la FASE 5: umbral de chattering discreto

La FASE 5 dejó escrito que *«la restricción se escribe sobre `(K/φ)·dt/M`, no
sobre φ ni α por separado»* (docs/05_smc.md §5). Aquí se implementa:

```
χ = max_t max_i (K_i/φ) · dt / M_ii   ≤   chi_limit
```

Dentro de la capa límite `sat(s/φ)` **no conmuta**: actúa como ganancia
proporcional `K/φ`, el polo del lazo es `(K/φ)/M_ii`, y el lazo **discreto**
entra en ciclo límite cuando `χ` se acerca a 1.

### 2.1 Calibración con datos, no con un número elegido

Se midió `χ` con este mismo evaluador sobre los dos puntos de operación que la
FASE 5 corrió en Gazebo:

| φ | χ medido | Gazebo (FASE 5) | veredicto |
|---|---|---|---|
| 0.05 | **1.139** | TV = 8 117, 99 % de energía > 20 Hz | chattering |
| 0.10 | **0.561** | TV = 1, 0 % de energía > 20 Hz | limpio |

El límite teórico `χ = 1` queda **bracketado** por las dos medidas. Se adopta
`chi_limit = 0.8`, el centro del intervalo, que deja margen para el retardo de
un ciclo de la tubería real — que este evaluador no modela y que reduce el
margen de estabilidad.

> Es el mismo patrón que ya apareció en `gravity_comp` y en el `η` de la FASE 5:
> **en este robot toda ganancia hay que referirla a la inercia de su junta**, que
> recorre cuatro órdenes de magnitud (2.59 kg·m² en `shoulder_lift` frente a
> 2.6e-4 en `wrist_3`).

---

## 3. Hallazgo: las ganancias de la FASE 5 no rechazan la fuerza de corte

La FASE 5 sintonizó el SMC sobre una planta **sin perturbación**. Al añadir el
perfil de fuerza de corte, con una meseta de 5 N, los pares articulares que
induce sobre la trayectoria son:

| junta | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|
| cota de `\|Jᵀw\|` [N·m] | 2.298 | 2.471 | 2.576 | 0.856 | **1.106** | 0.000 |
| `η` de la FASE 5 [N·m] | 1.058 | 2.591 | 0.881 | 0.023 | **0.005** | 0.0003 |

En muñeca 1 y 2 la perturbación es **37× y 207×** mayor que la ganancia de
conmutación. Con `|d| > K` la función `sat` satura, `ṡ` no cambia de signo, el
modo deslizante **no se establece** y el error deriva sin cota:

```
FASE 5 (λ=20, η=I·1.0, φ=0.05) bajo carga de corte:
  RMSE TCP = 457.7 mm      χ = 1.775      INFACTIBLE (g2 y g3 violadas)
```

Es un resultado reportable, no un fallo del método: dice que **la sintonía sin
perturbación no transfiere**, y justifica que esta fase exista.

### 3.1 Qué le cuesta a la parametrización escalar

En el modo `scalar` se impone `η_i = M_ii · a_reach`, que es la corrección
inercial de la FASE 5. Pero la **perturbación no escala con la inercia**: las
muñecas tienen inercia minúscula y sin embargo soportan par de corte apreciable,
así que para cubrir `wrist_2` hay que subir `a_reach` hasta que el hombro queda
enormemente sobre-ganado.

El optimizador **sí** encuentra un punto factible en modo escalar, pero el
precio es el que anticipa ese razonamiento: `η` de hasta ~476 N·m en
`shoulder_lift`, con el esfuerzo y la variación total disparados frente a lo que
consigue la parametrización por junta. La comparación cuantitativa está en §5.

> Corrección de una lectura anterior: la parametrización escalar no es
> *estructuralmente incapaz* bajo carga, como supuse antes de medirlo. Es
> **factible pero cara**. La conclusión operativa —usar la extensión
> `[λ_1..λ_6, η_1..η_6, φ]` que pide el plan— no cambia; el motivo sí.

### 3.2 Qué sí es factible, y quién manda

Imponiendo `χ ≤ 0.8` por junta, el φ mínimo admisible es `d_i·dt/(0.8·M_ii)`:

| junta | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|
| φ mínimo | 0.005 | 0.002 | 0.007 | 0.092 | **0.517** | 0 |

**Manda `wrist_2`**: exige `φ ≳ 0.52`, dos órdenes de magnitud por encima del
φ = 0.05 de la FASE 5. Y una vez `φ` es grande, `|s|` queda acotada pero grande,
así que quien recupera la precisión es **`λ`**, porque en régimen `q_e ≈ s/λ`.
Esa es la estructura del compromiso que resuelve el optimizador.

### 3.3 Cota superior de `λ`

`λ` amplifica el ruido de `q̇`. Ese suelo se **midió** sobre las corridas limpias
de la FASE 5 (`smc_522`, `smc_523`; contenido por encima de 50 Hz de `q̇ − q̇_d`
en la meseta): entre **1e-6 y 5e-6 rad/s** según la junta. Es el suelo numérico
del simulador, no ruido de sensor, y no muerde hasta `λ` muy alta. El evaluador
lo inyecta (`DQ_NOISE_STD_GAZEBO`) para no premiar `λ → ∞`.

> **PENDIENTE (FASE 9).** En el UR5e **real** ese ruido es otro y **no está
> caracterizado**. La `λ` que sale de esta fase vale para la campaña de Gazebo
> (FASE 8); antes del robot real hay que medir el ruido de `q̇` y re-validarla.

---

## 4. La restricción `g4`, y por qué el plan se queda corto sin ella

La primera campaña (13 variables, 60×100, 6 000 evaluaciones) dejó un frente que
**sí contenía** soluciones excelentes —`f1` mínimo de 0.0212 frente a 3.37 en la
rodilla, 160× mejor— pero la **selección por punto de rodilla eligió una
solución con 228 mm de error de TCP**.

No es un fallo del optimizador sino del planteamiento. A lo largo del frente:

| objetivo | mínimo | máximo | factor |
|---|---|---|---|
| f1 (IAE) | 0.0212 | 11.81 | **557×** |
| f2 (esfuerzo) | 19 168 | 23 480 | 1.22× |
| f3 (TV) | 91 | 12 927 | 142× |

La rodilla minimiza la distancia a la utopía en el espacio **normalizado**, así
que un 22 % de mejora en `f2` compensa perder tres órdenes de magnitud en `f1`.
Con los tres objetivos normalizados al mismo rango, el que apenas varía acaba
mandando.

La corrección **no** es cambiar de selector, es reconocer que una incisión con
228 mm de desvío no es un compromiso peor: es la **tarea no ejecutada**. Eso
pertenece al conjunto factible, no a la función objetivo. De ahí `g4`:

```
g4:  RMSE_TCP(meseta del corte) − tcp_tol ≤ 0        tcp_tol = 1.0 mm
```

`tcp_tol` es una **cota declarada** (20 % de los 5 mm de profundidad de corte),
no un dato de hardware — se reporta como tal, igual que el plan hace con
`ddq_max`. Es exigente pero alcanzable: el suelo teórico lo fija `wrist_2`,

```
|s_i| ≥ d_i·dt/(χ_lím·M_ii)        q_e,i ≈ |s_i|/λ_i
```

que con `λ = 300` da ≈ 0.25 mm de contribución al TCP.

---

## 5. Resultados

Corridas reproducibles: `seed = 42`, `α = 0.3`, `F_cut = 5 N`, `χ_lím = 0.8`,
`tcp_tol = 1.0 mm`. `results/gain_tuning/smc/test1` (`full`) y `test2`
(`scalar`).

| | pop × gen | evaluaciones | NSGA-II | ε-restricción | total |
|---|---|---|---|---|---|
| `full` (13 var) | 60 × 120 | 7 200 | 15.3 min | 53 min | **69.5 min** |
| `scalar` (3 var) | 40 × 80 | 3 200 | 6.9 min | 20 min | **29.0 min** |

Coste por evaluación **0.70 s** en serie, **0.13 s** efectivos con 16 procesos.
El criterio del plan («presupuesto total < unas horas en una máquina de
escritorio») se cumple con holgura.

### 5.1 Comparación de métodos de sintonía

| método | f1 (IAE) | f2 (esfuerzo) | f3 (TV) | RMSE TCP | χ | factible |
|---|---|---|---|---|---|---|
| FASE 5, a mano | 6.734 | 22 803 | 20 090 | 457.7 mm | 1.775 | **no** (g2,g3,g4) |
| suma ponderada + sustituto cúbico | 5.657 | 22 666 | 2 177 | 429.1 mm | 0.096 | **no** (g4) |
| NSGA-II `scalar` (rodilla) | 0.01476 | 119 673 | 948 337 | 0.823 mm | 0.471 | sí |
| **NSGA-II `full` (rodilla)** | **0.01304** | **22 377** | **2 542** | **0.931 mm** | 0.800 | sí |

Dos lecturas:

- **Frente al método previo.** La suma ponderada sobre sustituto polinómico
  cúbico ajusta el sustituto perfectamente (`R² = 1.000`) y aun así devuelve un
  punto **infactible**: 429 mm de error de TCP. No es mala suerte, es el defecto
  estructural del método — el escalarizado no distingue factible de infactible,
  así que **no se reporta un porcentaje de mejora**, que sería falsa precisión
  al comparar contra algo que no cumple las restricciones.
- **`full` frente a `scalar`.** Mismo seguimiento (0.93 vs 0.82 mm) con
  **5.3× menos esfuerzo y 373× menos chattering**. La parametrización escalar
  compra su precisión con `η` de 407 N·m en `shoulder_lift` —casi 3× el par
  máximo de la junta, que es 150 N·m— y saturando el actuador (`g1 = 0`
  exactamente, restricción activa). Es la confirmación cuantitativa de §3.1.

### 5.2 Ganancias seleccionadas (`full`, punto de rodilla)

```
λ = [255.5, 64.3, 299.9, 171.3, 300.0,  88.7]
η = [ 30.0, 2.415,  30.0, 1.410, 0.830, 0.00297]      φ = 0.4588
```

`g = [−26.1, −2.80, −1.0e−6, −0.069]` → **factible**, con `g3` (chattering
discreto) **activa**: `χ = 0.800` justo en su límite. Que la restricción que
manda sea el umbral discreto de la FASE 5, y no el par ni la velocidad, es el
resultado de diseño de esta fase.

Varias variables quedan en el borde de la caja: `λ_3`, `λ_5` en 300 y `η_1`,
`η_3` en 30 N·m (0.2·τ_max). Relajar la caja daría mejor `f1`, pero el tope de
`λ` está puesto donde acaba lo validado (§3.3), no donde acaba lo posible. Se
reporta como límite del estudio, no como óptimo interior.

El extremo de máxima precisión del frente llega a **0.443 mm** de TCP
(`f1 = 0.00665`) manteniendo `χ = 0.799`, a costa de más esfuerzo. Ahí está el
margen si la tolerancia de la tarea se aprieta.

### 5.3 Métricas de frente

| | HV | IGD | cobertura sobre el otro |
|---|---|---|---|
| NSGA-II | 149 455 | 1.055 | 0.000 |
| ε-restricción | 145 249 | 1 145.9 | 0.233 |
| combinado | 151 982 | — | — |

**La ε-restricción no compite con NSGA-II en 13 variables**: sólo **2 de 8**
niveles dieron soluciones factibles y ninguno convergió (todos agotaron el
límite de iteraciones). En 3 variables sí converge, y en 5 iteraciones. El
motivo es el coste del gradiente numérico: `n_var + 1` simulaciones por
iteración, en serie porque SLSQP pide los puntos de uno en uno. Es un resultado
sobre el **método**, reportable como tal.

> Aviso de escala: `hv_*` de `full` y de `scalar` **no son comparables entre
> sí**. El punto de referencia se deriva del nadir de cada frente y las escalas
> de f2/f3 difieren en dos órdenes de magnitud entre parametrizaciones. Los
> puntos de referencia van en `metrics.yaml`; sin ellos un HV no significa nada.

### 5.4 Certificación KKT

Sobre el punto SLSQP del nivel ε más exigente, para el problema de
ε-restricción que SLSQP resuelve realmente:

| | `full` | `scalar` |
|---|---|---|
| residuo de estacionariedad (relativo) | **0.0169** | 0.0434 |
| violación máxima | 3.1e-5 | 9.6e-2 |
| complementariedad | 2.3e-4 | 1.15 |
| restricciones activas | `ε, g3` + 6 cotas de caja | `ε, g1` + 1 cota |

En `full` el certificado es bueno: estacionariedad al 1.7 % del gradiente,
factibilidad y complementariedad a nivel de ruido numérico. En `scalar` es
claramente peor porque el punto está saturando el actuador y contra el borde de
`φ`. Se reporta el residuo, no un veredicto binario: con gradientes numéricos
sobre un lazo de 13 802 pasos, exigir residuo cero sería teatro.

### 5.5 Sensibilidad a `α`

Ganancias fijas del punto de rodilla `full`, variando la incertidumbre asumida:

| α | RMSE TCP | max\|s\| | χ | TV(τ) | factible |
|---|---|---|---|---|---|
| 0.1 | 1.590 mm | 0.872 | 0.753 | 2 331 | sí |
| **0.3** | **0.931 mm** | 0.809 | 0.800 | 2 542 | sí |
| 0.5 | 0.720 mm | 0.754 | 0.883 | 2 835 | **no** (χ) |
| 1.0 | 0.532 mm | 0.653 | 1.237 | 246 597 | **no** (χ) |

Subir `α` mejora el seguimiento —más ganancia de conmutación— pero empuja `χ`
por encima del umbral: en α = 1.0 la variación total se multiplica por 100. **Las
ganancias seleccionadas sólo son factibles hasta α = 0.3**, que es justamente el
valor con el que se optimizaron. Si la campaña de la FASE 8 encuentra que hace
falta más `α` bajo desajuste, hay que **reoptimizar**, no subir `α` sobre estas
ganancias.

> Sigue vigente el aviso de la FASE 5: en Gazebo el modelo es perfecto, así que
> un `α` pequeño sale «mejor» porque no hay incertidumbre que dominar. Este
> barrido sólo es interpretable junto con el de desajuste paramétrico.

---

## 5.6 Re-verificación en Gazebo: el retardo de tubería no era opcional

La primera tanda de ganancias (`test1`, evaluador **sin** retardo) **no
transfirió**:

| | RMSE q | TCP | max\|s\|/φ | TV(τ) | >20 Hz |
|---|---|---|---|---|---|
| predicción offline | 2.3e-5 | 0.0198 mm | — | 120 | — |
| **Gazebo medido** | 2.4e-2 | **29.72 mm** | 7.52 | 3 798 | 18.1 % |

Factor **1 500×**. La causa se aisló midiendo: con un paso de retardo añadido al
evaluador, esas ganancias se degradan 72× mientras las de la FASE 5 solo 2.7×.
Sin retardo modelado, subir `λ` es gratis y el optimizador la lleva al borde de
la caja; el lazo real, que sí tiene el desfase de ~1 ms que midió la FASE 2, no
lo aguanta.

Corregido el evaluador (`PIPELINE_DELAY_STEPS`) y reoptimizado (`test3`):

| | RMSE q | TCP | max\|s\|/φ | TV(τ) | >20 Hz |
|---|---|---|---|---|---|
| `test1` (sin retardo) | 2.4e-2 | 29.72 mm | 7.52 | 3 798 | 18.1 % |
| **`test3` (con retardo)** | **1.3e-5** | **0.0081 mm** | **0.08** | 963 | 4.8 % |

**3 670× mejor en TCP**, y ahora sí cumple el criterio del plan `|s| = O(φ)`.

### 5.6.1 El evaluador sirve para buscar, no para predecir

El acuerdo cuantitativo **sigue sin ser bueno**, y en las dos direcciones:

| ganancias | offline sin retardo | offline con retardo | Gazebo |
|---|---|---|---|
| `test1` | 0.0198 mm | 0.3921→1.418 mm | **29.72 mm** (peor que ambas) |
| `test3` | 0.0196 mm | 0.3921 mm | **0.0081 mm** (mejor que ambas) |

El modelo de un paso resultó **21× optimista** para unas ganancias y **48×
pesimista** para otras, así que el sesgo no es una simple cuestión de «cuánto
retardo». La FASE 2 midió ~1 ms, medio ciclo a 500 Hz, y el lazo real encadena
además el paso de 1 ms del simulador, el retén de orden cero del publicador y el
temporizador de pared del nodo.

Conclusión operativa, que es la que hay que escribir en *Methods*: el evaluador
offline es un **sustituto de búsqueda** válido —encuentra ganancias buenas 130×
más barato que Gazebo— pero **no un predictor de prestaciones**. Cada juego de
ganancias que salga de esta fase se re-verifica en Gazebo antes de usarlo. Eso
ya no es una precaución teórica: aquí atrapó un fallo de 1 500×.

### 5.6.2 Escena

Las corridas de re-verificación usan `lab_incision_world.sdf`, que **no** lleva
el obstáculo del caso de uso de pick & place (caja AABB en (0.85, 0, 0.73)) que
`lab_torque_world.sdf` había heredado al copiarse. No afecta a nada ya medido
—la holgura mínima sobre la trayectoria nominal es de 0.150 m— pero se retira de
cara a la FASE 8, donde el desajuste de ±50 % y la carga de 4 kg desvían el
brazo mucho más. Ver `ur5_dyn_control/launch/world_defaults.py`.

---

## 6. Límites de validez del evaluador

Declarados en el encabezado de `closed_loop.py` y verificados contra Gazebo:

- **Sin retardo de tubería.** El lazo real tiene ~1 ms de desfase entre medir el
  estado y aplicar el par (medido en la FASE 2). Aquí el par actúa en el mismo
  paso, lo que hace el evaluador **optimista en chattering**.
- **Perfil de `F_ext` sintético.** Rampa de penetración + meseta con ruido. El
  plan lo reemplaza por el perfil medido con `ft_data` en la FASE 9; hasta
  entonces **toda conclusión cuantitativa sobre robustez frente a la fuerza de
  corte es provisional** y así hay que reportarla.
- **Planta sin fricción**, como el Gazebo actual (la identificación real sigue
  pendiente en la FASE 2).

Validación contra Gazebo (mismas ganancias, sin fuerza de corte):

| | RMSE q | max\|s\| | TV(τ) |
|---|---|---|---|
| offline φ=0.05 | 2.2e-5 | 0.0001 | 15 487 |
| Gazebo φ=0.05 | 1.1e-4 | 0.173 | 8 117 |
| offline φ=0.10 | 2.2e-5 | 0.0002 | 21 |
| Gazebo φ=0.10 | 4.0e-6 | 0.00076 | 1 |

El evaluador **reproduce el umbral** (TV cae tres órdenes de magnitud entre 0.05
y 0.10, igual que en Gazebo), que es lo que se necesita para optimizar `φ`, pero
`max|s|` sale ~1 000× optimista porque no hay ruido ni retardo. **Por eso las
ganancias seleccionadas se re-verifican en Gazebo antes de la FASE 8**, y el
`selected_gains.yaml` lleva ese aviso escrito.

---

## 7. Artefactos de paper que habilita esta fase

- **Frentes de Pareto** NSGA-II vs ε-restricción con el *knee* marcado.
- **Tabla** HV / IGD / cobertura / tiempo por método.
- **Tabla de ganancias seleccionadas** por controlador.
- **Figura del fallo bajo carga**: la sintonía de la FASE 5 frente a la
  optimizada, que es el argumento de por qué hace falta la fase.
- **Dato para *Methods***: la restricción real de sintonía es `(K/φ)·dt/M ≤ 1`,
  calibrada contra dos puntos medidos en Gazebo.
