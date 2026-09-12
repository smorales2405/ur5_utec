# FASE 9 — Puesta a punto del SMC en el UR5e real

Registro de lo que ha pasado al llevar el SMC de Gazebo al robot físico, qué lo
explica y qué queda por medir. Dos incidentes hasta ahora, **con causas
opuestas**, y los dos con las mismas ganancias.

---

## 1. Los dos incidentes

| | `smc_710` (2026-08-27) | `smc_712` (2026-09-10) |
|---|---|---|
| junta barrida | `wrist_2` | `elbow` |
| síntoma | fuga de 162°, se quedó en −252° | ciclo límite a 35.6 Hz, el brazo entero sacudiéndose |
| par | **nunca** saturó (3.2 de 8.4 N·m) | saturado el **100 %** de los ciclos |
| error de seguimiento | 162° | **0.03–0.2°** durante la vibración |
| desenlace | terminó el barrido y paró solo | paro de emergencia del operador; la caja de control se desplazó |
| `D = M·λ` | **0.9** | **115** |

Mismo `selected_gains.yaml` (`smc_v4_g5`), misma α, misma corrida del
optimizador. Un extremo se quedó sin autoridad y el otro se pasó de ganancia.

---

## 2. Qué gobierna esto: `G = M_ii·λ_i + K_i/φ_i`

La ley del SMC es `τ = b + M·q̈_r − K·sat(s/φ)` con `q̈_r = q̈_des − Λ·ė` y
`s = ė + Λe`. Derivando respecto a la velocidad **medida**, dentro de la capa
límite:

```
G_i = M_ii·λ_i  +  K_i/φ_i        [N·m por rad/s de error de velocidad]
        \____/      \_____/
      control eq.   conmutado
```

Es una ganancia derivativa, y las **dos** mitades cuentan. `sat()` sólo deja de
aportar cuando `|s| > φ`, y en `smc_712` el robot estuvo dentro de la capa el
**100 % de los ciclos en las seis juntas** (|s| mediana 0.007–0.090 contra φ de
0.19–0.88), así que `K/φ` estuvo activa todo el rato.

`λ` sola no dice nada. `M_ii` recorre cuatro órdenes de magnitud en el UR5e, así
que con λ casi uniforme (37–168, salida del optimizador) `M·λ` recorre otros
cuatro:

| junta | M_jj † | λ (FASE 7) | **D** | λ (FASE 5) | D |
|---|---|---|---|---|---|
| shoulder_pan | 1.0582 | 161.2 | **170.6** | 20 | 21.2 |
| shoulder_lift | 2.5914 | 78.0 | **202.2** | 20 | 51.8 |
| elbow | 0.8815 | 130.7 | **115.2** ← falló | 20 | 17.6 |
| wrist_1 | 0.0232 | 139.7 | 3.2 | 20 | 0.46 |
| wrist_2 | 0.0054 | 167.8 | **0.9** ← falló | 20 | 0.11 |
| wrist_3 | 0.00026 | 37.3 | 0.010 | 20 | 0.005 |

† `M_jj` no depende de la propia junta `j`, solo de las distales, así que es
**constante** a lo largo del barrido de `j`. Pinocchio sobre `ur5e.urdf` sin
herramienta.

### 2.0 Bajar λ NO basta: la trampa de las ganancias de la FASE 5

Lo primero que uno hace tras `smc_712` es volver a λ = 20. No sirve, y el motivo
es que `φ` de la FASE 5 es pequeña. `G` completa en la pose del barrido del codo:

| junta | \|\| `M·λ` | `K` | `K/φ` | **`G`** |
|---|---|---|---|---|
| **FASE 5** — λ=20, φ=[0.05,0.07,0.07,0.07,0.05,0.20] | | | | |
| shoulder_pan | 21.2 | 1.06 | 21.2 | 42.4 |
| shoulder_lift | 51.8 | 8.68 | 124.0 | **175.8** |
| elbow | 17.6 | 6.96 | 99.4 | **117.1** |
| **FASE 7** — `smc_v4_g5` | | | | |
| shoulder_pan | 170.6 | 1.53 | 8.2 | 178.8 |
| shoulder_lift | 202.2 | 10.02 | 32.3 | **234.5** |
| elbow | 115.2 | 8.27 | 12.6 | **127.8** ← falló |

El codo con las ganancias de la FASE 5 habría corrido a **117.1**, un **8 % por
debajo** del punto que acaba de entrar en ciclo límite. La λ es seis veces menor
y la ganancia total prácticamente la misma, porque `K/φ` pasa a ser el 85 % del
total. `shoulder_lift` sale incluso peor que con las de la FASE 7 en su mitad
conmutada.

Con `α = 0.3`, `K = η + |α·(M·q̈_r) + α·b + (1−α)·(dM·q̇_r)|` y `b` lleva la
gravedad, así que `K` del codo vale ~7–8 N·m aunque su η valga 0.88: **`K` no se
elige, se calcula**, y con φ pequeña se convierte en una ganancia enorme.

### 2.1 Por arriba: el ruido de `q̇`

Del propio registro de `smc_712`: el encoder cuantiza en **5.722 µrad**
(2²⁰ cuentas/vuelta) y `q̇` es su diferencia finita a 500 Hz, luego

```
Δq̇ = 5.722e-6 / 0.002 = 2.86e-3 rad/s
```

Medido: σ(q̇ − q̇_des) = 2.93e-3 rad/s a 0.05 rad/s en la junta que se mueve, y
**exactamente 0** en las cinco que están quietas. Coincide con la cuantización.

El optimizador de la FASE 7 usó **5e-6 rad/s**, el suelo de Gazebo
(`DQ_NOISE_STD_GAZEBO`). Es **570× menor**. El comentario que hay en
`problem.py` sobre las cotas de λ nombra el mecanismo correcto —«quien la limita
de verdad es el ruido de `q̇`, que λ amplifica»— y lo calibró contra el robot
equivocado.

Rizado de par que sale de ahí, `τ_rizado = G·Δq̇`:

```
elbow, G = 127.8 (FASE 7)  ->  0.366 N·m    inestable  (MEDIDO)
elbow, G = 117.1 (FASE 5)  ->  0.335 N·m    un 8 % menos: NO es un margen
```

### 2.2 Por abajo: la fricción de Coulomb

`wrist_2` falló por lo contrario. Con `D = 0.9` y `η = 0.237 N·m` no tiene
autoridad frente a **1.96 N·m** de Coulomb: no saturó nunca porque nunca pidió
par suficiente. Ver `docs/05_smc.md` §7.

**La fricción no escala con la inercia** —la fija la reductora y es parecida en
todas las juntas grandes— mientras `M_ii` recorre cuatro órdenes. Cualquier
criterio que ignore una de las dos cosas rompe un extremo o el otro.

---

## 3. El modo a 35.6 Hz es ESTRUCTURAL, no del codo

`analyze_vibration.py` sobre `smc_712`, tomando el espectro en el tramo de
rizado máximo:

| junta | rizado/tope | máx. sat. | f dominante | E > 20 Hz |
|---|---|---|---|---|
| shoulder_pan | 46.7 % | 0.66 | 36.3 Hz | 98.8 % |
| shoulder_lift | 69.0 % | 1.00 | 35.0 Hz | 99.9 % |
| elbow | 58.2 % | 1.00 | 35.0 Hz | 100.0 % |
| wrist_1 | 46.2 % | 0.67 | 35.0 Hz | 100.0 % |
| wrist_2 | 5.3 % | 0.00 | 35.0 Hz | 99.1 % |
| wrist_3 | 0.04 % | 0.00 | 35.0 Hz | 98.6 % |

**Las seis juntas a la misma frecuencia.** No es una oscilación local del codo:
es el primer modo estructural del brazo en esa pose, y la ganancia derivativa
del codo le metió energía. Por eso movió la caja de control.

El pico de velocidad de `shoulder_lift` fue **0.91 rad/s** dentro de un recorrido
de 6 mrad, estando quieta.

---

## 4. `tau_scale` no es la solución, es lo que le dio forma

Con el tope al 30 % (±45 N·m en las juntas grandes) el lazo sobre-ganado se
convirtió en un **relé**: `M·λ·ė` valía 46 N·m con `ė = 0.4 rad/s`, por encima
del tope, así que el comando conmutaba entre +45 y −45 en vez de pedir un par
continuo. Subir `tau_scale` quitaría el relé y dejaría la inestabilidad, con más
energía. **Lo que la quita es bajar `D`.**

Umbral de saturación por junta, `ė_crit = τ_tope / D`:

```
ė_crit con tau_scale = 0.30 y las ganancias de la FASE 7:
  pan 0.264   lift 0.223   elbow 0.391   w1 2.59   w2 9.36   w3 875   [rad/s]
```

El barrido pide hasta 0.5 rad/s. `shoulder_lift` saturaría con **0.22 rad/s** de
error de velocidad.

---

## 5. Las guardas

`watchdog.q_err_max` (error de seguimiento) **no puede** ver esto: durante la
vibración el error valía 0.03–0.2°, y el máximo antes del paro de emergencia fue
**0.155 rad**. Ni con umbral 0.3 habría disparado. Los 0.888 rad del log son
posteriores al paro, con el robot ya congelado.

Lo que sí lo ve es el par. Sobre `smc_712`, ventana de 0.2 s:

| indicador | sano (69 s) | en el fallo |
|---|---|---|
| fracción de ciclos saturados | **0.000** en las seis juntas | 0.66 – 1.00 |
| rizado de τ / tope | ≤ 0.97 % | 46 – 69 % |

Se implementa la **fracción saturada** (`watchdog.sat_frac_max`, default 0.25,
ventana 0.2 s) y no el rizado, porque su separación es absoluta —cero contra
0.66— en vez de un factor: con `tau_scale` de puesta a punto no se satura nunca
en operación legítima.

Habría disparado en **t = 75.31 s, 3.49 s antes** de que el operador alcanzase el
pulsador.

**Validación en Gazebo:**

- `smc_730` — barrido del codo completo, 90 s, λ = 20, guarda al 25 %:
  **no dispara**. Fracción saturada 0.000 en las seis juntas, rizado ≤ 1.62 %.
- `smc_731` — el mismo barrido con `tau_scale = 0.23`, que estrangula el par del
  codo por debajo de su pico de 37.6 N·m: **dispara**, nombra la junta y entra
  en `SAFE_HOLD`.

---

## 6. Lo que falta medir: el umbral de `D`

Hay **un** punto medido en el robot real: inestable con `D = 115` en el codo. El
`D = 17.6` de la FASE 5 no se ha ensayado nunca en el robot físico. **No se
inventa el umbral**: se mide, igual que se midió el de chattering de φ
(`05_smc.md` §7.6).

Rampa sobre el codo, una corrida por escalón. Se **mantienen η y φ del
`gains_file` optimizado** y se varía sólo λ: con φ₃ = 0.654, `K/φ` vale 12.3
(entre 11.3 y 12.6 en todo el recorrido, a ±0.5 rad/s) y λ queda como **único
mando**. Con la φ de la FASE 5 no se podría: `K/φ` valdría 99 y dominaría.

| λ₃ | `M·λ` | `K/φ` | **G** | fracción del punto que falló |
|---|---|---|---|---|
| 20 | 17.6 | 12.3 | 30.0 | 0.23 |
| 35 | 30.9 | 12.3 | 43.2 | 0.34 |
| 55 | 48.5 | 12.3 | 60.8 | 0.48 |
| 80 | 70.5 | 12.3 | 82.8 | 0.65 |
| 110 | 97.0 | 12.3 | 109.3 | 0.86 |
| *130.7* | *115.2* | *12.3* | *127.5* | *1.00 — ya medido, no se repite* |

Criterio de parada: el primer escalón en que `analyze_vibration.py` marque
`rizado/tope > 5 %` o `máx. sat > 0.10`.

### 6.1 Resultado (2026-09-10, `smc_713`–`smc_717`): la rampa NO acotó el umbral

| test | λ₃ | G₃ | rizado/tope | máx. sat | \|e\|max | σ(q̇ − q̇_des) |
|---|---|---|---|---|---|---|
| 713 | 20 | 29.9 | 0.30 % | 0.000 | 1.09° | 8.92e-3 |
| 714 | 35 | 43.2 | 0.67 % | 0.000 | 0.72° | 5.57e-3 |
| 715 | 55 | 60.8 | 0.71 % | 0.000 | 0.45° | 4.56e-3 |
| 716 | 80 | 82.8 | 1.60 % | 0.000 | 0.36° | 3.90e-3 |
| 717 | 110 | 109.3 | 1.28 % | 0.000 | 0.26° | 3.14e-3 |

Las cinco limpias, ninguna saturación. **Pero `G₃ = 109.3` está a un 14 % del
`127.8` que entró en ciclo límite, y el rizado es 1.28 % contra 58 %: un factor
45.** Eso no es acercarse a un umbral, es otro régimen.

Lo que sí valida es el **modelo de ruido**. El rizado crece proporcionalmente a
`G` —que es lo que predice `τ_rizado = G·Δq̇`— y en `717` σ(q̇) = 3.14e-3 rad/s
ya casi toca el suelo de cuantización de **2.86e-3**: `G·Δq̇` = 0.343 N·m =
0.76 % del tope, contra el 1.28 % medido como máximo por ventana. Y el
seguimiento **mejora monótonamente** con λ, de 1.09° a 0.26°.

### 6.2 Por qué no acotó: el confundido era el barrido, no el codo

En la rampa se bajaron **las seis** λ a 20. En `smc_712` `shoulder_pan` estaba en
`G = 186` y `shoulder_lift` en `G = 240`. Es decir, la rampa quitó los
amplificadores y midió sólo el arranque.

Reparto de energía **durante** la vibración (t = 75.0–78.5 s, antes del paro):

| junta | máx \|q̇\| | % ciclos sat. | rizado τ | potencia \|τ·q̇\| |
|---|---|---|---|---|
| shoulder_pan | 0.288 | 48.8 % | 18.41 N·m | 5.26 W |
| **shoulder_lift** (QUIETA) | **0.907** | **79.0 %** | **27.14 N·m** | **18.18 W** |
| elbow (la barrida) | 0.591 | 61.1 % | 22.12 N·m | 10.21 W |
| wrist_1 | 0.535 | 45.7 % | 3.34 N·m | 1.93 W |

**La junta que más energía metió en el modo de 35 Hz no se estaba moviendo.**

Secuencia de arranque (primer rizado > 5× su nivel sano):

```
codo           74.827 s   ← empieza
shoulder_lift  74.871 s   +44 ms
wrist_1        74.971 s   +144 ms
shoulder_pan   75.037 s   +210 ms
```

y en 0.4 s `shoulder_lift` **adelanta** al codo en velocidad (0.64 contra 0.53
rad/s) y ya no lo suelta.

**Lectura:** el codo *arranca* el modo —es la junta que se mueve, y por tanto la
única con ruido de cuantización— y `shoulder_lift` lo *amplifica*. Una junta
quieta no puede iniciar nada (σ(q̇) = exactamente 0), pero en cuanto la
estructura vibra su q̇ medida deja de ser cero y su `G` la convierte en par que
refuerza el movimiento.

Por tanto el criterio no es sobre la `G` de la junta que se mueve, sino sobre

```
max_i G_i        sobre las SEIS juntas, se muevan o no
```

y lo medido hasta hoy es: **`max G` = 109 limpio, `max G` = 240 violento.**

### 6.3 `smc_718`: el codo SOLO en su λ del fallo no arranca nada

`λ₃ = 130.67` —exactamente la de `smc_712`— con las otras cinco en 20. Limpio:
cero saturación, rizado 2.34 % en el codo, error 0.21°.

**Queda probado que la `G` del codo no basta.** Hacía falta el hombro.

### 6.4 Dos modos, y sólo uno pierde amortiguamiento con la `G` del codo

Amplitud RMS de τ por banda [N·m], `analyze_vibration.py`:

| G₃ | codo 20–35 Hz | lift 20–35 Hz | codo 40–55 Hz | lift 40–55 Hz |
|---|---|---|---|---|
| 29.9 | 0.012 | 0.008 | 0.008 | 0.005 |
| 43.2 | 0.021 | 0.015 | 0.014 | 0.008 |
| 60.8 | 0.029 | 0.025 | 0.021 | 0.014 |
| 82.8 | 0.044 | 0.041 | 0.029 | 0.019 |
| 109.3 | 0.049 | 0.050 | 0.049 | 0.037 |
| **127.5** | **0.059** | **0.060** | **0.092** | **0.066** |
| *`smc_712`, 14 s antes de irse* | *0.081* | *0.089* | *0.071* | *0.061* |

- **20–35 Hz** (pico en 25.7 Hz): es **el mismo modo** que `smc_712` tenía en su
  tramo sano (25.5 Hz) y que se fue a 35.7 Hz. Crece **lineal** con la `G` del
  codo —A/G casi constante—: el codo lo excita pero **no le quita
  amortiguamiento**. En 718 está al 73 % (codo) y 68 % (hombro) del nivel previo
  al fallo.
- **40–55 Hz** (pico en 47–49 Hz): crece **super-lineal** —casi se dobla en el
  último escalón con un 17 % más de `G`—. Éste sí pierde amortiguamiento.

Ajuste del límite (§ del script: `G/A` lineal en `G` para la junta que se
mueve, cero en `G_c`):

| modo | codo | `shoulder_lift` |
|---|---|---|
| 20–35 Hz | sin pérdida medible (r −0.32) | 250 (r −0.81, débil) |
| 40–55 Hz | **215** (r −0.95) | **184** (r −0.98) |

> **Corrección.** La primera versión de este análisis miraba sólo 40–55 Hz,
> daba `G_c ≈ 180` como si fuese el límite del fallo y comparaba 718 con
> `smc_712` en bandas distintas («81 %»). El `G_c` de 184–215 es real pero es
> del modo **secundario del codo**, no del que se fue.

**Lectura:** que el modo de 25 Hz no pierda amortiguamiento con la `G` del codo
—pero sí se fuese en `smc_712`— encaja con lo que dijo el reparto de energía:
lo desestabilizó la `G` de los hombros. Es la hipótesis que hay que verificar.

### 6.5 Cómo se verifica el hombro

Pregunta: **¿subir la `G` de `shoulder_lift` estando quieta le quita
amortiguamiento al modo de 25 Hz?**

Una junta quieta no inyecta ruido (σ(q̇) = 0 exacto), así que con el codo fijo la
excitación es constante y sólo puede cambiar el amortiguamiento:

```
A ≈ c / (1 − G₂/G_c)     ->     1/A es una recta en G₂ que corta cero en G_c
```

Diseño:

- codo barrido en **λ₃ = 55** (`G₃ = 60.8`): modo de 25 Hz al ~35 % del nivel
  previo al fallo y el de 48 Hz lejos de su `G_c` ≈ 184. Excitación medible
  (3× el suelo de 713) con margen.
- `shoulder_lift` quieta, rampa λ₂ = 20 / 33 / 45 / 60 → `G₂` = 86.1 / 119.8 /
  150.9 / 189.8.
- resto en λ = 20, `tau_scale` 0.30, guardas armadas.
- **calentamiento** previo (F_v cae 9.3 % frío→rodado, §02 8.5, y la fricción
  viscosa amortigua el modo: un robot que se calienta durante la rampa
  parecería perder amortiguamiento por la `G`), y **línea base repetida al
  final** para detectar deriva.

| test | `lambda_joint` | G₂ | |
|---|---|---|---|
| 790 | `20,20,55,20,20,20` | 86.1 | calentamiento, no se analiza |
| 719 | `20,20,55,20,20,20` | 86.1 | línea base (repite 715: comprueba repetibilidad entre días) |
| 720 | `20,33,55,20,20,20` | 119.8 | |
| 721 | `20,45,55,20,20,20` | 150.9 | |
| 722 | `20,60,55,20,20,20` | 189.8 | **sólo si** el ajuste de 719–721 permite `G < 0.8·G_c` |
| 723 | `20,20,55,20,20,20` | 86.1 | línea base repetida |

Resultados posibles:

1. **El modo de 25 Hz crece y `1/A` ajusta** → el hombro es el amplificador y
   `G_c` del hombro es su límite medido.
2. **Plano** → `shoulder_lift` sola no lo desestabiliza; el siguiente sospechoso
   es `shoulder_pan` (`G` = 186 en `smc_712`) o la combinación.
3. **719 difiere de 715 más de ~25 %, o 723 de 719** → la amplitud no es
   repetible entre sesiones / derivó durante la rampa, y la conclusión se
   limita a lo comparable dentro de la sesión.

La cota de diseño queda **pendiente** de este resultado. Lo que sí está medido:
el codo aguanta `G₃ = 127.5` con los hombros bajos, y su modo secundario tiene
`G_c` ≈ 184–215.

---

## 7. Qué limita λ en cada extremo (resumen)

| | juntas grandes | muñecas |
|---|---|---|
| lo que muerde | `G = M·λ + K/φ` contra el ruido de `q̇` y el modo de 35 Hz | `χ = (K/φ)·dt/M` y la autoridad frente a Coulomb |
| ¿medido? | **no** — un solo punto de fallo | sí, `05_smc.md` §7.6 |
| ¿está en el optimizador? | **no** | sí (`g3`, `g5`) |

Que el optimizador se equivocara en los dos extremos a la vez no es casualidad:
su evaluador no tiene ruido de velocidad *ni* fricción en la planta, que son
justo las dos cosas que limitan λ por arriba y por abajo.
