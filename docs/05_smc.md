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

## 6. PENDIENTE: ensayo de tiempo de alcance

El criterio *"con `sign`, `s` alcanza el entorno de cero en tiempo finito,
coherente con la cota"* **no es medible con la trayectoria actual**.

Medido: `|s(0)| ≈ 0.003 rad/s`, ya dentro de la capa límite (φ=0.05). La causa
es estructural: la fase **RAMP** de `TorqueControlNodeBase` lleva el robot desde
`q0` hasta el primer punto de la tabla con una quíntica, así que cuando empieza
`TRACK` el robot **ya está sobre la superficie deslizante**. No existe fase de
alcance que cronometrar.

**Qué hace falta** (≈ media hora, apuntado para antes de la campaña de la FASE 8):

1. Un parámetro que introduzca un **error inicial deliberado** — p. ej.
   `initial_offset[6]` sumado a `q0` al entrar en `TRACK`, o un modo que salte
   la fase RAMP.
2. Con `s(0)` grande y conocido, cronometrar `t` hasta `|s| < φ` por junta.
3. Contrastar contra la cota `t_alcance ≤ |s_i(0)| / a_reach`, que con
   `η_i = I_ii · a_reach` es `|s_i(0)| / 1.0 s`.

Sin ese ensayo, cualquier afirmación sobre tiempo de alcance en el paper
carecería de respaldo experimental.

---

## 7. Artefactos de paper que habilita esta fase

- **Figura de 3 paneles** (`s(t)`, `τ(t)`, espectro) para `sign` vs `sat`: los
  datos están en `smc_511.csv` (sign) y `smc_510.csv` (sat).
- **Tabla** error / esfuerzo / chattering: §3.
- **Figura del compromiso de φ** con el umbral marcado: §4. Es el resultado más
  interesante de la fase, porque contradice la lectura ingenua de la teoría.
- **Dato para *Methods***: la restricción real de sintonía es `(K/φ)·dt/M`.

Corridas: `smc_510` (sat, φ=0.05), `smc_511` (sign), `smc_520..523` (barrido de
φ), `smc_530..532` (barrido de α).
