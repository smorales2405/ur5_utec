# FASE 5 — SMC clásico: `sgn` vs `sat(s/φ)`

Cierre de la FASE 5 del plan `PLAN_INCISION_UR5e.md`.

- **Fecha:** 2026-07-30
- **Nodo:** `gz_smc_control_node` · **Config:** `config/smc_params.yaml`
- **Prerrequisitos:** FASES 1 (trayectoria) y 3 (infraestructura)

---

## 1. Formulación

Estructura de Slotine-Li:

```
q_e = q − q_d          dq_e = q̇ − q̇_d
s   = dq_e + Λ q_e                          (superficie deslizante)
q̇_r = q̇_d − Λ q_e      q̈_r = q̈_d − Λ dq_e    ⟹  s = q̇ − q̇_r

tau = b̂(q,q̇) + M̂(q) q̈_r − K ⊙ ρ(s)          ρ = sgn(s) | sat(s/φ)
```

`K` se calcula **en cada ciclo** desde la cota de alcance

```
K_i ≥ η_i + | α M̂ q̈_r + α b̂ + (1−α) Ṁ̂ q̇_r |_i
```

en vez de dejarla como parámetro libre, para que la FASE 7 optimice sobre
`[Λ, η, φ]` con la condición de alcance satisfecha **por construcción**
(restricción `g3` del plan).

> Es **`≥`**, no `≤`. La tesis de partida lo tenía invertido; con `≤` la
> ganancia podría quedar por debajo de la incertidumbre y el modo deslizante no
> se establecería.

`Ṁ̂ = C + Cᵀ` (antisimetría de `Ṁ − 2C`), añadido a `Ur5Dynamics`.

---

## 2. Defecto corregido: `η` sin escalar por inercia

Con `η` uniforme (`[5,5,5,1,1,1]`) el SMC daba `max|s| = 4.09 rad/s = 82·φ` y un
error RMS **150× peor que el FL** sobre la misma trayectoria.

Dentro de la capa límite `sat(s/φ)` actúa como una ganancia `K/φ`, así que el
polo del lazo es `(K/φ)/I_ii`. Con `η` uniforme iba de 39 rad/s en el hombro a
**77 652 rad/s en `wrist_3`** (I = 2.6e-4 kg·m²), con número de estabilidad
discreta 155.

Corrección: `η_i = I_ii · a_reach` con `a_reach = 1.0 rad/s²`. El polo queda en
`a_reach/φ = 20 rad/s` en las seis juntas.

> **Es el tercer sitio donde aparece el mismo patrón** (tras `gravity_comp` y el
> retardo de la FASE 2): este robot tiene cuatro órdenes de magnitud de
> dispersión en inercia articular, y **cualquier ganancia no escalada por
> inercia rompe en las muñecas**. `Λ` no lo sufre porque `M·Λ` ya escala con la
> inercia por construcción.

---

## 3. `sign` vs `sat` (meseta del corte, régimen)

| | RMSE q [rad] | TCP [mm] | max\|s\| | TV(τ) | RMS Δτ |
|---|---|---|---|---|---|
| `sign` | 0.000546 | 0.337 | 0.356 | 68 576 | 6.860 |
| `sat` (φ=0.05) | **0.000113** | **0.064** | 0.173 | **8 117** | **0.568** |

`sat` gana en todo: **4.8× mejor seguimiento y 12× menos chattering**.

> Las métricas se miden en la **meseta del corte**, no en toda la fase TRACK. La
> incisión encadena cinco fases con reposos, y en cada arranque `s` salta por el
> transitorio de alcance: eso no es régimen deslizante. Restringir al régimen
> baja `max|s|` de 7.16·φ a 3.47·φ.

---

## 4. Barrido de φ — el compromiso clásico está **invertido** bajo umbral

| φ | RMSE q | TCP [mm] | max\|s\| | TV(τ) | RMS Δτ | energía >20 Hz |
|---|---|---|---|---|---|---|
| 0.01 | 3.9e-4 | 0.254 | 0.287 | 41 369 | 3.743 | 99.9 % |
| 0.02 | 2.8e-4 | 0.174 | 0.253 | 15 301 | 1.311 | 99.5 % |
| 0.05 | 1.1e-4 | 0.064 | 0.173 | 8 117 | 0.568 | 99.3 % |
| **0.10** | **4.0e-6** | **0.0039** | **0.00076** | **1** | **0.0005** | **0 %** |
| 0.20 | 8.0e-6 | 0.0068 | 0.00125 | 2 | 0.0003 | 0 % |

**Transición brusca entre 0.05 y 0.10**: `|s|` cae 230×, `TV` cae 8000×, y la
energía por encima de 20 Hz pasa de 99 % a **cero**. Es el umbral de chattering
del lazo discreto: por debajo, el sistema está en ciclo límite y la teoría
continua no aplica.

**Consecuencia para el paper.** La teoría dice *"φ menor = más precisión, más
chattering"*. Aquí, bajar φ de 0.10 a 0.01 empeora **la precisión 100× y el
chattering 40 000× a la vez**. En esa región no hay compromiso que negociar,
solo una implementación fuera de su rango de validez.

El criterio del plan **sí se cumple**, pero solo por encima del umbral: con
φ=0.10, `max|s|/φ = 0.0076`.

---

## 5. Barrido de α

| α | RMSE q | TCP [mm] | max\|s\| | TV(τ) | RMS Δτ | >20 Hz |
|---|---|---|---|---|---|---|
| 0.1 | 4.0e-6 | 0.0038 | 0.00089 | 8 | 0.0016 | 0 % |
| 0.3 | 1.1e-4 | 0.064 | 0.173 | 8 117 | 0.568 | 99 % |
| 0.5 | 5.9e-4 | 0.378 | 0.285 | 12 734 | 1.042 | 99 % |
| 1.0 | 1.3e-3 | 0.854 | 0.880 | 61 247 | 5.415 | 99 % |

**El mismo mecanismo explica los dos barridos**: lo que manda es la razón `K/φ`
frente al límite de estabilidad discreta, con `K = η + α·|cota|`. Bajar φ y
subir α son dos formas de aumentar `K/φ`. Comprobación: `α=0.1, φ=0.05` y
`α=0.3, φ=0.10` dan el mismo resultado (4e-6 rad los dos).

**Para la FASE 7**: la restricción se escribe sobre `(K/φ)·dt/M`, no sobre φ ni
α por separado.

> ### ⚠️ Caveat obligatorio en el paper
> `α` es la incertidumbre **asumida**. En Gazebo el modelo es perfecto, así que
> α=0.1 sale "mejor" porque **no hay incertidumbre que dominar**. Bajo desajuste
> paramétrico (FASE 8), un α pequeño dejaría `K` por debajo de la incertidumbre
> real y el modo deslizante no se establecería. **No debe leerse como "usar
> α=0.1"**: el barrido de α solo es interpretable junto con el de desajuste.

---

## 6. Ensayo de tiempo de alcance

El criterio *"con `sign`, `s` alcanza el entorno de cero en tiempo finito,
coherente con la cota"* **no era medible con la trayectoria tal cual**: la fase
**RAMP** de `TorqueControlNodeBase` lleva el robot desde `q0` hasta el primer
punto de la tabla con una quíntica, así que al empezar `TRACK` el robot **ya
está sobre la superficie deslizante** (`|s(0)| ≈ 0.003 rad/s`, dentro de la capa
con φ=0.05) y no existe fase de alcance que cronometrar.

### 6.1 Cómo se creó la fase de alcance

Parámetro **`initial_offset[6]`** (default cero: ninguna corrida previa cambia).
Desplaza el **destino de la rampa**, no el estado: la quíntica lleva el brazo
suavemente hasta `tabla[0] + offset` y lo deja **en reposo**, de modo que al
entrar en `TRACK` el error es exactamente `offset` con `dq_e ≈ 0` y por tanto

```
s_i(0) = λ_i · offset_i
```

conocido y controlado. Saltarse la rampa —la otra opción que se barajó— daría un
transitorio sin acotar y muy probablemente el watchdog cortando la corrida.

Con `offset = 0.05 rad` en las seis juntas y `λ = 20`, la predicción es
`s(0) = 1.0 rad/s`. **Medido: 0.961 – 1.035 rad/s**, o sea el mecanismo hace
exactamente lo que dice.

### 6.2 La cota

Con modelo perfecto y `ρ = sgn(s)`:

```
M ṡ = −K ⊙ sgn(s)    ⟹    |ṡ_i| = K_i/M_ii ≥ η_i/M_ii = a_reach
⟹  t_alcance,i ≤ |s_i(0)| / a_reach          (a_reach = 1.0 rad/s²)
```

Es una cota **superior**: `K = η + |cota de incertidumbre| > η`, así que alcanzar
antes es correcto. Lo que invalidaría el modelo sería medir un tiempo **mayor**.

### 6.3 Resultado

`sign` (`smc_540`) y `sat` (`smc_541`), mismo offset:

| junta | s(0) | t alcance `sign` | t alcance `sat` | cota \|s₀\|/a | pico \|s\| |
|---|---|---|---|---|---|
| 1 | 1.000 | 0.414 | 0.414 | 1.000 | 1.000 |
| 2 | 1.002 | 0.400 | 0.402 | 1.002 | 1.002 |
| 3 | 0.991 | 0.144 | 0.146 | 0.991 | 0.991 |
| 4 | 1.049 | 0.434 | 0.440 | 1.049 | **2.973 (×2.83)** |
| 5 | 1.000 | 1.016 | **1.040** | 1.000 | **2.584 (×2.58)** |
| 6 | 0.998 | 0.332 | 0.328 | 0.998 | 0.998 |

*(s(0) y pico de la corrida `sat`; en `sign` difieren en menos del 5 %.)*

**Cinco de las seis juntas alcanzan la capa muy por debajo de la cota** — entre
un 15 % y un 85 % del tiempo disponible. `sign` y `sat` dan prácticamente el
mismo tiempo de alcance, que es lo esperable: fuera de la capa límite
`sat(s/φ) = sgn(s)`, así que las dos leyes son idénticas **durante** el alcance
y solo se separan al entrar.

**La junta 5 queda justo en la cota**: 1.016 s con `sign` (cota 1.035, cumple) y
1.040 s con `sat` (cota 1.000, la excede un 0.4 %). No son resultados
contradictorios sino la misma junta rozando el límite por los dos lados, y
tiene explicación — es precisamente la junta cuyo `|s|` sobrepasa ×2.58.

### 6.4 Por qué: la cota por junta supone `M` diagonal

Las juntas 4 y 5 **se alejan** de la superficie antes de volver: `|s|` sube a
2.8× y 2.6× su valor inicial. No es un fallo, es que la cota por junta sale de
`η_i = M_ii·a_reach`, que implícitamente trata `M` como diagonal. Con la `M`
real acoplada,

```
ṡ = −M⁻¹ K ⊙ sgn(s)
```

y `M⁻¹` mezcla las juntas: lo que garantiza la convergencia es la función de
Lyapunov `V = ½ sᵀM s`, **no** el decrecimiento monótono de cada `|s_i|` por
separado. Que la única junta que roza la cota sea también la que más sobrepasa
no es casualidad: el acoplamiento le mete energía antes de que empiece a
converger, y ese rodeo es tiempo que la cota diagonal no contabiliza.

En el paper la cota por junta hay que presentarla como lo que es —una estimación
de ingeniería, verificada aquí en 5 de 6 juntas y rozada en la sexta— y **no**
como una garantía derivada de la dinámica acoplada. La afirmación sólida es la
de Lyapunov: `V` decrece y el alcance es en tiempo finito.

Las excursiones de `|s|` posteriores a t = 2 s (hasta 0.39 en la junta 4) son los
transitorios de las cinco fases de la incisión (§3), no fallos del alcance.

Analizador: `ur5_identification/scripts/analyze_reaching.py`.

---

## 7. Con la fricción REAL en la planta: el feedforward se bloquea

Tres corridas sobre la trayectoria nueva (`surface_z` = 0.03), **mismas
ganancias**, cambiando solo la condición de fricción. La fricción inyectada es la
medida en el robot físico ([`02_friction_real.md`](02_friction_real.md) §3.1).

| corrida | condición | TCP RMSE | TCP max | max\|s\| | \|s\|/φ |
|---|---|---|---|---|---|
| `smc_401` | sin fricción | **0.130 mm** | 0.52 mm | 0.411 | 8.2 |
| `smc_402` | fricción, sin compensar | 258.99 mm | 324.08 mm | 11.533 | 230.7 |
| `smc_400` | fricción + `tanh` | 170.42 mm | 217.19 mm | 11.475 | 229.5 |

La línea base confirma que **la trayectoria está sana**: 0.130 mm de TCP y
errores articulares de ~1e-5 rad. Todo el deterioro viene de la fricción.

### 7.1 El feedforward funciona donde la junta se mueve, y no hace nada donde se atasca

RMSE articular [rad], y el factor de mejora que aporta compensar:

| junta | sin fricción | sin compensar | con `tanh` | mejora |
|---|---|---|---|---|
| shoulder_lift | 0.00003 | 0.04728 | **0.00030** | **158×** |
| elbow | 0.00006 | 0.25980 | **0.01623** | **16×** |
| shoulder_pan | 0.00001 | 0.16004 | 0.15987 | 1.00× |
| wrist_1 | 0.00049 | 0.40385 | 0.40133 | 1.01× |
| wrist_3 | 0.00003 | 0.16005 | 0.15987 | 1.00× |
| wrist_2 | 0.00002 | 0.00000 | 0.00000 | — |

En `shoulder_lift` la compensación devuelve el seguimiento a **la décima parte
del error de la planta sin fricción** — es decir, funciona casi perfectamente. En
`shoulder_pan`, `wrist_1` y `wrist_3` no cambia nada.

La causa está en el par comandado:

| junta | sin compensar | con `tanh` | aporta | necesita |
|---|---|---|---|---|
| shoulder_lift | 54.04 | 65.65 | **+11.6** | 9.9 |
| shoulder_pan | 3.74 | 3.83 | **+0.09** | 7.99 |

`frictionFeedforward` usa la velocidad **medida**, y el término de Coulomb es
`F_c·tanh(q̇/ε)` con ε = 1e-3 rad/s. Si la junta está clavada, `q̇ ≈ 0`,
`tanh → 0` y la compensación vale cero: **hace falta movimiento para activar lo
que debería producir el movimiento**. `shoulder_lift` y `elbow` arrancan porque
sus términos nominales (gravedad, 18 N·m) bastan para romper la adherencia, y
una vez en marcha la compensación entra y hace su trabajo. `shoulder_pan` no
tiene gravedad ninguna: su `K = η + α·|nominales|` da 3.8 N·m contra 7.99 que
necesita, no rompe, y se queda bloqueada para siempre.

`wrist_2` y `wrist_3` no llegan a comandar ni 0.04 N·m.

> La limitación ya estaba escrita en `torque_command.hpp` («cerca de q̇ = 0 la
> compensación tiende a cero, así que NO cancela la fricción estática») y en
> `02_friction.md`. Lo nuevo es el precio: **un 34 % de mejora global
> (259 → 170 mm) cuando en las juntas que sí arrancan vale 158×.**

### 7.2 Qué haría falta

Con las velocidades reales de esta trayectoria, `K = F_c + F_v·|q̇|max`:

| junta | K necesaria | χ resultante | % de τ_max |
|---|---|---|---|
| shoulder_pan | 7.99 | 0.3 ✅ | 5.3 % |
| shoulder_lift | 9.93 | 0.2 ✅ | 6.6 % |
| elbow | 11.87 | 0.5 ✅ | 7.9 % |
| wrist_1 | 2.42 | 4.2 ❌ | 8.7 % |
| wrist_2 | 2.76 | 20.7 ❌ | 9.9 % |
| wrist_3 | 3.35 | **515** ❌ | 12.0 % |

Las tres juntas grandes se arreglan **subiendo η**, sin acercarse al límite
discreto ni al del actuador: con un 8 % del par disponible. Es sintonía, y le
toca a la FASE 7.

En las muñecas no hay sintonía que valga. El par que necesitan es el 9–12 % de su
actuador —músculo sobra— pero χ se dispara porque su inercia es minúscula. Y el
feedforward por sí solo tampoco basta: compensando al 95 %, a `wrist_3` le
quedarían 0.16 N·m de residual, que siguen dando χ ≈ 25.

**Dos vías, y hay que elegir explícitamente:**

1. **Compensar con `q̇_deseada` en vez de `q̇_medida`.** Rompe el bloqueo: al
   inicio del movimiento `q̇_d ≠ 0`, así que la compensación se activa ANTES de
   que la junta se mueva. Es un cambio aditivo al nodo (un modo más), no rompe la
   interfaz. No resuelve χ en las muñecas, pero quita de en medio el bloqueo, que
   es lo que hoy arruina tres juntas.
2. **φ por junta.** Para χ ≤ 0.8 haría falta φ ≈ 0.26 / 1.29 / 32.2 rad/s en las
   tres muñecas. Con φ = 32 rad/s `wrist_3` deja de tener modo deslizante y pasa
   a ser un lazo lineal de ganancia `K/φ` = 0.10 N·m/(rad/s).

---

### 7.3 Opción A implementada: `friction.dq_source = desired`

Parámetro nuevo en `TorqueControlNodeBase` (`measured` por defecto). Con
`desired`, el feedforward se alimenta de `ref.dq` en vez de la velocidad medida,
así que se activa **antes** de que la junta se mueva.

| corrida | condición | TCP RMSE | max\|s\|/φ | TV(τ) |
|---|---|---|---|---|
| `smc_401` | sin fricción | 0.130 mm | 8.2 | 65257 |
| `smc_402` | fricción, sin compensar | 258.99 mm | 230.7 | 3624 |
| `smc_400` | compensada con q̇ medida | 170.42 mm | 229.5 | 4300 |
| `smc_403` | **compensada con q̇ deseada** | **0.542 mm** | 76.6 | 10087 |

**314× mejor que con la velocidad medida**, y a 4× de la planta sin fricción —
con las MISMAS ganancias. No se tocó ni un η.

| junta | q̇ medida | q̇ deseada | mejora | max\|s\|/φ |
|---|---|---|---|---|
| shoulder_pan | 0.15987 | **0.00066** | **242×** | 0.6 ✅ |
| shoulder_lift | 0.00030 | 0.00014 | 2× | 0.4 ✅ |
| elbow | 0.01623 | 0.00028 | 58× | 0.9 ✅ |
| wrist_1 | 0.40133 | **0.00099** | **405×** | 4.6 |
| wrist_2 | 0.00000 | 0.00000 | — | 0.0 ✅ |
| wrist_3 | 0.15987 | 0.15994 | **1.00×** | **76.6** ❌ |

Cuatro juntas cumplen ya el criterio `|s| = O(φ)`. `shoulder_pan` pasa a comandar
8.05 N·m —los 7.99 que pedía el modelo— y `wrist_1` 4.68.

### 7.4 Por qué `wrist_3` sigue clavada: el feedforward no puede dar margen

No es chattering. Medido sobre la meseta: **4 cambios de signo en 14 717
muestras**, par medio −1.98 N·m, `max|τ|` = 3.30 N·m, y `|q̇|` exactamente 0.

El feedforward vale `F_c·tanh(q̇_d/ε)`, que tiende a `F_c` **por debajo** y nunca
lo supera. Para arrancar hace falta *exceder* la fricción estática, y ese margen
solo puede venir del término conmutado `K·sat(s/φ)`. En `wrist_3`,
η = 0.00026 N·m contra los 3.18 N·m que hay que vencer: cuatro órdenes de
magnitud por debajo del margen necesario.

Es decir: **compensar al 100 % deja la junta exactamente en el punto de
equilibrio, no la arranca.** Un margen del 5 % (0.16 N·m) bastaría, pero da
χ ≈ 25 frente al límite de 0.8 — así que `wrist_3` necesita la opción B (φ por
junta, ≈ 1.5 rad/s), y con ese φ deja de tener modo deslizante.

> **RETIRADO por §7.5.** El razonamiento de arriba es correcto como cálculo,
> pero la evidencia que lo respaldaba NO lo es: `wrist_3` no estaba clavada por
> falta de margen, sino **congelada por el integrador** (`b·dt/I` = 11.1). Con el
> viscoso aislado, arranca y sigue la referencia a 0.14° con 3.21 N·m. Lo que
> quede de este apartado hay que releerlo a la luz de §7.5.

Que `wrist_2` marque 0.00000 no es un éxito: su referencia apenas se mueve en
esta trayectoria, así que no llega a ejercitarse.

---

### 7.5 `phi` por junta, y un ARTEFACTO del simulador que invalida a `wrist_3`

Parámetro nuevo `phi_joint` (6 valores; vacío = usar el escalar `phi`, que es lo
que escriben los ficheros de ganancias de la FASE 7). φ solo significa algo
**relativo a K**, así que subirlo sin subir η debilita la junta en vez de
reforzarla: la opción B es necesariamente un cambio conjunto.

Valores ensayados, de imponer `χ ≤ 0.8` con un margen del 20 % de la fricción:

| junta | η | φ | χ | K/φ efectiva |
|---|---|---|---|---|
| pan · lift · elbow | sin cambio | 0.05 | 0.04 | — |
| wrist_1 | 0.0232 → 0.485 | 0.0521 | 0.80 | 9.30 |
| wrist_2 | 0.0054 → 0.553 | 0.2582 | 0.80 | 2.14 |
| wrist_3 | 0.00026 → 0.670 | 6.5015 | 0.80 | **0.103** |

`wrist_1` mejora 1.8× (RMSE 0.00099 → 0.00056) y el TCP baja de 0.542 a
0.516 mm. La columna `K/φ` es un **techo**, no una elección: vale `0.8·M_ii/dt`,
así que a 500 Hz `wrist_3` no puede tener más autoridad por unidad de error.

#### El artefacto

`wrist_3` marcó `|s|/φ` = 0.59 —cumpliendo `|s| = O(φ)`— con el error
**intacto** (0.15994 → 0.15995 rad). Criterio satisfecho de forma **vacía**: con
φ = 6.5 rad/s y `|s|` ≈ 3.8, `s` está dentro de la capa por construcción. No hay
modo deslizante y el indicador no lo delata.

Al mirar por qué, apareció algo peor. Con η = 8.0 (par de conmutación de
7.83 N·m, **+146 % sobre la fricción estática**, en el 97 % de los ciclos), la
junta se movió **0.0000°** exactos. Eso no es física.

Aislando el término:

| | recorrido de q₆ | RMSE | \|τ₆\| máx |
|---|---|---|---|
| `b₆ = 2.87` (medido) | **0.0000°** | 0.15995 rad | 7.83 |
| `b₆ = 0`, Coulomb intacto en 3.18 | **10.8197°** | **0.00246 rad** | 3.21 |

**Es el viscoso, no el Coulomb.** Quitando solo `b₆`, la junta recorre sus
10.82° (de 10.98 pedidos), sigue a 0.14° y le basta con 3.21 N·m — con los
3.18 N·m de Coulomb inyectados. El modelo de Coulomb de Gazebo aguanta bien la
inercia de `wrist_3`; el viscoso no.

```
número de rigidez   b·dt/I        (dt = 1 ms de física)
   pan    0.014      wrist_1   0.057
  lift    0.005      wrist_2   0.346
 elbow    0.016      wrist_3  11.144   <- unica junta por encima de 1
```

`I/b` = 90 µs frente a un paso de 1 ms. No es un fallo de modelado: la junta es
*genuinamente* así de rígida, y el integrador no puede representarla.

#### Alcance del daño

- **Toda medida de `wrist_3` en Gazebo con la fricción inyectada es inválida**
  (corridas 400, 403, 404, 406). Lo de las otras cinco juntas se sostiene.
- **La FASE 8 produciría resultados de muñeca inválidos en silencio** si inyecta
  la fricción medida.
- **El evaluador de la FASE 7 tiene el mismo problema y peor**: con `dt` = 2 ms,
  `b·dt/I` = 22.3 y su integrador semi-implícito es EXPLÍCITO en `q̇`, factor de
  amplificación −21.3. No se congelaría: **divergiría**, y `simulate()` lo
  marcaría como `diverged` — el NSGA-II descartaría esas ganancias como «no
  viables» cuando lo que falla es el integrador. Antes de enchufar la fricción al
  optimizador hay que tratar el viscoso de forma implícita.

Simular las muñecas con la fricción real pide un paso de ~90 µs: **10× más
lento**. Las alternativas son inyectar solo el Coulomb en `wrist_3` y declarar el
viscoso como no modelado, o bajar el paso solo en las campañas de muñeca.

#### Y un dato que cuestiona el propio criterio χ

En la corrida que sí se movió, `wrist_3` funcionó con **χ = 3.83** —casi cinco
veces el límite de 0.8— y **sin chattering**: cero cambios de signo, `|s|/φ` =
0.05, `TV(τ₆)` = 171 N·m en 30 s. El umbral χ ≤ 0.8 se midió en la FASE 5 sobre
una planta SIN fricción; con fricción real la disipación amortigua el ciclo
límite que ese criterio evita. Conviene re-medirlo antes de imponerlo como
restricción dura en la FASE 7: tal como está, puede estar descartando regiones
válidas del espacio de ganancias.

---

### 7.6 El umbral de chattering MEDIDO con fricción: χ no es universal

El límite `χ ≤ 0.8` se heredó de la FASE 5, medido sobre una planta **sin
fricción** y como `χ_max` agregado. Barrido de φ en Gazebo con la fricción real
inyectada y compensada, sobre las cuatro juntas donde el simulador es fiable
(`wrist_2` no se mueve, `wrist_3` está congelada por §7.5). Energía de `τ` por
encima de 20 Hz en la meseta del corte, misma definición que `analyze_smc.py`:

| φ | pan χ / >20 Hz | lift χ / >20 Hz | elbow χ / >20 Hz | wrist_1 χ / >20 Hz |
|---|---|---|---|---|
| 0.20 | 0.01 · 0.0 % | 0.05 · 0.0 % | 0.08 · 0.0 % | 0.25 · 0.0 % |
| 0.10 | 0.02 · 0.0 % | 0.11 · 0.0 % | 0.16 · 0.0 % | 0.50 · 0.0 % |
| 0.05 | 0.04 · 0.0 % | 0.22 · 0.5 % | 0.32 · 1.1 % | 0.99 · 1.3 % |
| 0.03 | 0.07 · 0.1 % | **0.36 · 93.7 %** | **0.53 · 92.9 %** | **1.67 · 58.8 %** |
| 0.02 | 0.10 · 0.0 % | 0.55 · 67.5 % | 0.79 · 73.0 % | 2.52 · 59.7 % |
| 0.01 | 0.20 · 2.4 % | 1.10 · 98.2 % | 1.59 · 95.9 % | 5.06 · 73.3 % |

**El umbral no es el mismo en todas las juntas:**

| junta | umbral medido | vs. el límite 0.8 |
|---|---|---|
| shoulder_lift | **0.22 – 0.36** | el límite es 2–4× DEMASIADO PERMISIVO |
| elbow | **0.32 – 0.53** | 1.5–2.5× demasiado permisivo |
| wrist_1 | **0.99 – 1.67** | el límite es 1.2–2× demasiado RESTRICTIVO |
| shoulder_pan | sin cruzar hasta χ = 0.20 | no alcanzado en el barrido |

Un factor de 5 entre `shoulder_lift` y `wrist_1`. El argumento de estabilidad
discreta predice un umbral **universal** cercano a 1, y los datos no lo
respaldan.

#### Por qué baja en las juntas grandes

La predicción de partida era que la fricción **subiría** el umbral, porque
disipa y amortigua el ciclo límite. Eso se cumple en `wrist_1` (0.99–1.67 frente
al ~1 teórico) y **falla en las dos juntas grandes**, donde baja a ~0.3.

La explicación que encaja: la fricción de Coulomb no solo disipa, también es una
fuerza **discontinua** en el cruce por cero de la velocidad. El pegado-deslizado
que produce es un mecanismo de ciclo límite por derecho propio, y se acopla con
el término conmutado a ganancias `K/φ` más bajas de las que el análisis lineal
—que trata la fricción como amortiguamiento puro— predice. `shoulder_lift` y
`elbow` son las que más Coulomb tienen (7.20 y 7.89 N·m) y las que antes cruzan.

No es un artefacto de la compensación: `f_c·tanh(q̇_d/ε)` no depende de φ, y la
transición entre φ = 0.05 y φ = 0.03 es de dos órdenes de magnitud en energía de
alta frecuencia.

#### Qué hacer con esto

- **Un `chi_limit` único no sirve.** Con 0.8, la FASE 7 aceptaría ganancias que
  hacen chatear `shoulder_lift` y `elbow`, y rechazaría ganancias válidas en
  `wrist_1`. La restricción debería ser **por junta**, con los umbrales de la
  tabla y margen.
- **Si hay que dejar un escalar**, el conservador es **0.2**, no 0.8: por debajo
  del cruce de `shoulder_lift`.
- **No cierra la brecha de las muñecas.** La cota analítica pide χ = 3.2 en
  `wrist_1` (§ `friction_residual_bound`) y el umbral medido está en 0.99–1.67:
  sigue habiendo un factor ~2. Menos que el 4× que suponía el límite 0.8, pero
  incompatible igual.

---

## 8. Artefactos de paper que habilita esta fase

- **Figura de 3 paneles** (`s(t)`, `τ(t)`, espectro) para `sign` vs `sat`: los
  datos están en `smc_511.csv` (sign) y `smc_510.csv` (sat).
- **Tabla** error / esfuerzo / chattering: §3.
- **Figura del compromiso de φ** con el umbral marcado: §4. Es el resultado más
  interesante de la fase, porque contradice la lectura ingenua de la teoría.
- **Dato para *Methods***: la restricción real de sintonía es `(K/φ)·dt/M`.

Corridas: `smc_510` (sat, φ=0.05), `smc_511` (sign), `smc_520..523` (barrido de
φ), `smc_530..532` (barrido de α).
