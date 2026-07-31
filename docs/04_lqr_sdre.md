# FASE 4 — LQR-SDRE

Cierre de la FASE 4 del plan `PLAN_INCISION_UR5e.md`.

- **Fecha:** 2026-07-31
- **Nodo:** `gz_lqr_sdre_control_node` · **Config:** `config/lqr_sdre_params.yaml`
- **Launch:** `launch/lqr_sdre_control.launch.py`
- **Prerrequisitos:** FASES 1 (trayectoria) y 3 (infraestructura)

---

## 1. Formulación

State-Dependent Riccati Equation: un LQR resuelto en cada actualización sobre la
parametrización dependiente del estado (SDC) del **error de seguimiento**, con
la Riccati congelada en el paso.

```
x_e = [q_e ; q̇_e] ∈ R¹²        q_e = q − q_d,  q̇_e = q̇ − q̇_d

A(q,q̇) = [ 0   I ; 0  −M(q)⁻¹ C(q,q̇) ]        (12×12)
B(q)    = [ 0 ; M(q)⁻¹ ]                       (12×6)

J = ∫ (x_eᵀ Q x_e + uᵀ R u) dt,   Q = blkdiag(Qp, Qv),   R ≻ 0
AᵀP + PA − P B R⁻¹ Bᵀ P + Q = 0,   K = R⁻¹ Bᵀ P

tau = M̂(q) q̈_d + Ĉ(q,q̇) q̇_d + g(q) − K x_e
```

### 1.1 Discrepancia del plan, resuelta a favor de `A`

El plan escribe el prealimentado como `Ĉ(q,dq) dq` y a la vez da
`A = [0 I; 0 −M⁻¹C]`. **Las dos cosas no pueden ser ciertas a la vez:**

| prealimentado | dinámica del error | `A` resultante |
|---|---|---|
| `M q̈_d + C q̇ + g` (par calculado clásico) | `M q̈_e = u` | `[0 I; 0 0]` |
| `M q̈_d + C q̇_d + g` | `M q̈_e + C q̇_e = u` | `[0 I; 0 −M⁻¹C]` ✅ |

Con la primera, el término de Coriolis se cancela **entero** contra el de la
planta, la dependencia del estado desaparece de `A` y el esquema deja de ser
SDRE en `A` (solo variaría `B(q)`). Se implementa la **segunda**: hace que el
par `(A, B)` documentado en el plan sea exacto y no una aproximación, y es la
formulación SDRE estándar para manipuladores.

`C` se evalúa en el estado **real** `(q, q̇)` en ambos sitios; es lo que hace que
la diferencia `C q̇ − C q̇_d` sea exactamente `C q̇_e` con la **misma** `C`.

### 1.2 Gravedad (compuerta G3)

`computeTau()` devuelve el par físico completo, `g(q)` incluida. La clase base
decide si se comanda (Gazebo) o se resta (UR5e real). Sin cambios respecto a FL
y SMC.

---

## 2. Síntesis de `Q`: el escalado por inercia, cuarta vez

`Q` y `R` no se dan en bruto. Se dan `wn`, `zeta` y `r`, y el nodo sintetiza

```
Qp = r · wn⁴ · M²        Qv = r · wn² · (4 ζ² − 2) · M²        R = r · I
```

### 2.1 Derivación

En el límite desacoplado (`C = 0`, `R = r I`), con `A = [0 I; 0 0]`,
`B = [0; M⁻¹]` y `P = [P11 P12; P12ᵀ P22]`, la CARE se parte en

```
(1,1):  P12 G22 P12ᵀ = Qp                       G22 = M⁻¹R⁻¹M⁻¹ = M⁻²/r
(2,2):  P12 + P12ᵀ − P22 G22 P22 + Qv = 0
K = R⁻¹BᵀP = (1/r) M⁻¹ [P12ᵀ, P22]   ⟹   Kp = M⁻¹P12ᵀ/r,  Kd = M⁻¹P22/r
```

Imponer el lazo cerrado `M q̈_e + Kd q̇_e + Kp q_e = 0` con **todos** los modos en
`(wn, ζ)` equivale a `Kp = wn² M` y `Kd = 2 ζ wn M`, es decir `P12 = r wn² M²` y
`P22 = 2 r ζ wn M²`; sustituyendo arriba salen las `Q` de la fórmula.

**Verificado numéricamente:** los 12 autovalores caen en `−wn` con residuo
`1.6e-14` y dispersión `1.00×`.

### 2.2 Por qué `M²` y NO `diag(M)²`

`M(q_init)` no es diagonal ni de lejos:

```
M(1,5) = 2.803e-2      frente a      M(5,5) = 5.352e-3
```

un acoplo **cinco veces la propia diagonal**. Con la receta tentadora
`Qp_ii = I_ii² wn⁴` los polos **medidos** se dispersan así:

| receta | \|λ\| medido [rad/s] | dispersión | max\|λ\|·dt (500 Hz) |
|---|---|---|---|
| `Qp = wn⁴ diag(M)²` | 8.42 … 289.89 | **34.4×** | **0.580** ❌ |
| `Qp = wn⁴ M²` | 20.00 … 20.00 | 1.00× | 0.040 ✅ |

Es el mismo escalado por inercia que ya hizo falta en `gravity_comp` y en el
`eta` del SMC — **cuarta vez** en este paquete. Fijado por
`test_care_solver::QConLaDiagonalDeMDispersaLosPolos`.

### 2.3 `ζ ≥ 1/√2` es obligatorio

`Qv = r wn² (4ζ² − 2) M²` solo es semidefinida positiva si `ζ ≥ 1/√2`. **No es
una limitación de la implementación:** es el resultado clásico de que un LQR
sobre un doble integrador no puede amortiguar por debajo de Butterworth, y
`ζ = 1/√2` es exactamente el caso `Qv = 0`. Pedir menos exigiría `Q` indefinida.
El nodo lo rechaza en el constructor con ese mensaje.

### 2.4 Punto de diseño

`wn = 20 rad/s` — mismo polo nominal que el `λ` del SMC, para comparar a
igualdad de ancho de banda; `ζ = 1`; `r = 1`. Da `max|λ|·dt = 0.040` a 500 Hz,
el mismo número de estabilidad discreta que el SMC.

### 2.5 `Q_mode`: `fixed` vs `scheduled`

- **`fixed`** (default): `Q` constante, sintetizada con `M(q_init)`. Es la
  formulación del plan — `Q` y `R` son pesos de **coste**, constantes — y es la
  que hace que la figura de `max Re(λ)` vs `t` tenga algo que enseñar.
- **`scheduled`**: `Q(q)` resintetizada en cada actualización. El marco SDRE
  admite pesos dependientes del estado; así los polos caen en `−wn` en toda la
  trayectoria y no solo en `q_init`.

---

## 3. Solver de la CARE

`include/ur5_dyn_control/care_solver.hpp` — **función signo de la matriz
hamiltoniana** (Roberts, 1971/1980):

```
H = [  A      −B R⁻¹ Bᵀ ]        S = sign(H)  vía  S ← (S + S⁻¹)/2
    [ −Q      −Aᵀ       ]
```

El subespacio invariante estable de `H` es `ker(I + S)`. Con `[I; P]` base de
ese subespacio, `(I + S)[I; P] = 0` da el sistema sobredeterminado

```
[ S12     ] P = − [ S11 + I ]
[ S22 + I ]       [ S21     ]
```

resuelto por mínimos cuadrados (QR con pivoteo de columnas).

### 3.1 Por qué la función signo y no Schur ni autovectores

- **Schur ordenado** es el método de referencia, pero `Eigen::RealSchur` no sabe
  reordenar autovalores y el intercambio de bloques 1×1/2×2 habría que
  escribirlo entero.
- **El método de autovectores** del hamiltoniano (tomar los `n` autovectores con
  `Re < 0` y hacer `P = X₂X₁⁻¹`) **se rompe justo en el punto de diseño de este
  paquete**: con `ζ = 1` el lazo cerrado de cada modo tiene un **polo doble** en
  `−wn`, y `A − BK` en forma compañera con autovalor doble es **defectiva** —
  hay un solo autovector, `X₁` queda singular y `P` sale basura.
  `test_care_solver::PoloDobleDefectivoSeResuelveIgual` cubre ese caso.
- **La función signo** solo necesita que `H` no tenga autovalores en el eje
  imaginario (condición necesaria para que exista solución estabilizante de
  todos modos) y es insensible a autovalores repetidos: calcula el **proyector**
  sobre el subespacio, no una base de autovectores.

### 3.2 Balanceado simpléctico (sin él no converge)

La similaridad `T = diag(D, D⁻¹)` preserva la estructura hamiltoniana:

```
A → D⁻¹AD      G → D⁻¹GD⁻¹      Q → DQD
```

Hace falta porque las inercias articulares van de `2.6e-4` a `2.59 kg·m²`
(**cuatro** órdenes), lo que con `Qp ~ M²` son **ocho** órdenes en `Q` y otros
ocho en sentido contrario en `G = B R⁻¹ Bᵀ ~ M⁻²`. **Medido sin balancear:** la
iteración se estanca con residuo relativo `~1e-4`; los bloques pequeños quedan
sepultados bajo el redondeo de los grandes. **Con balanceado:** `1.6e-14` en 2–8
iteraciones.

El factor por índice se redondea a **potencia de 2** para que el escalado sea
exacto en binario y no meta error propio (misma razón por la que lo hace LAPACK).
Se añade además un balance escalar `c = √(‖Q‖/‖G‖)` entre los dos bloques fuera
de la diagonal.

El **residuo se mide en coordenadas balanceadas**. En las originales sería
engañoso: al normalizar por la norma global solo mediría el bloque del hombro,
ocho órdenes por encima del de `wrist_3`, y daría por buena una `K` con la
muñeca completamente equivocada.

### 3.3 Estabilizabilidad de `(A, B)`

Se certifica por **controlabilidad** (controlable ⟹ estabilizable), con el
margen `σ_min/σ_max` de la matriz de Kalman `[B, AB, …]` construida bloque a
bloque y detenida en cuanto alcanza rango `n`.

Para este par la certificación es barata y basta con dos bloques: ya
`[B, AB] = [0, M⁻¹; M⁻¹, −M⁻¹CM⁻¹]` tiene rango 12 siempre que `M` sea
invertible. **La única vía realista de perderla es que `M(q)` se vuelva
numéricamente singular**, que es justo lo que vigila `cond(M)` al lado.

Si el margen cae por debajo de `lqr.controllability_tol`, o si la CARE falla
`lqr.max_consecutive_failures` veces seguidas, el nodo pide **SAFE_HOLD** y el
par de ese ciclo **no se publica**.

---

## 4. Vigilancia por paso

En **cada** ciclo, no solo en los que se resuelve la CARE, se evalúa
`max Re(eig(A(t) − B(t) K))` con la `K` **realmente vigente**. Esa es la
pregunta que importa: la CARE garantiza estabilidad en el instante en que se
resuelve, pero entre actualizaciones `A(q,q̇)` se mueve y la `K` congelada puede
dejar de estabilizar. Es exactamente el riesgo que introduce la decimación.

El nodo escribe un **CSV de diagnóstico aparte**, `lqr_diag_<test_num>.csv`
(clase `DiagLogger`), con misma cabecera de trazabilidad que el CSV unificado:

| columna | significado |
|---|---|
| `max_re_eig` | `max Re(eig(A − B K))` con la `K` vigente |
| `cond_M` | `λ_max/λ_min` de `M(q)` |
| `ctrl_margin` | margen de controlabilidad de `(A,B)` |
| `care_residual`, `care_iters` | del último solve válido |
| `care_updated` | 1 si este ciclo resolvió la CARE (mide la decimación real) |
| `t_care_us`, `t_law_us` | tiempos de cómputo |
| `k_max`, `care_fails` | mayor `\|K_ij\|` y fallos acumulados |

**Por qué un fichero aparte y no columnas nuevas en el CSV unificado:** el
esquema de `CsvLogger` es deliberadamente el mismo para los cuatro
controladores, para que el análisis de la FASE 10 no se bifurque. Meter aquí
magnitudes que solo existen en esta ley lo llenaría de columnas a cero.

Análisis: `ur5_identification/scripts/analyze_lqr.py`.

---

## 5. Resultados en Gazebo

Mundo `lab_incision_world.sdf`, 500 Hz, `wn = 20`, `ζ = 1`, `r = 1`, sin
fricción compensada. TRACK = 13 802 muestras (27.60 s).

| ensayo | `Q_mode` | CARE | ciclos que resuelven |
|---|---|---|---|
| **10** | `fixed` | cada ciclo | 100.0 % |
| **11** | `scheduled` | cada ciclo | 100.0 % |
| **12** | `fixed` | 50 Hz (ZOH) | 10.0 % |

### 5.1 Criterio 1 — `max Re(eig(A − B K)) < 0` en el 100 % de los pasos

| ensayo | TRACK completo [rad/s] | meseta del corte | % < 0 |
|---|---|---|---|
| 10 | −17.987 … **−6.735** | −6.738 … −6.735 | **100.00** ✅ |
| 11 | −20.000 … **−17.910** | −19.610 … −19.596 | **100.00** ✅ |
| 12 | −17.987 … **−6.734** | −6.739 … −6.734 | **100.00** ✅ |

`cond(M)` máximo 11 717; margen de controlabilidad mínimo `8.53e-5`, tres
órdenes por encima de `lqr.controllability_tol`. Cero fallos de la CARE, cero
`SAFE_HOLD`, residuo máximo `1.6e-13` en 8 iteraciones.

**Hallazgo — el coste de congelar `Q`.** Con `Q_mode: fixed` el margen de
estabilidad se degrada de los `−20 rad/s` de diseño a **`−6.74`** en la meseta
del corte: un factor **3×**. No es un problema de la CARE (la resuelve con
residuo `1e-13`), sino del propio esquema: `Q` se sintetizó con `M(q_init)` y en
la meseta la configuración es otra. Con `Q_mode: scheduled` el polo se queda en
`−19.6`, prácticamente el de diseño.

### 5.2 Criterio 2 — tiempo de cómputo (presupuesto 2000 µs a 500 Hz)

| ensayo | bloque | media | p95 | p99 | máx | > presupuesto |
|---|---|---|---|---|---|---|
| 10 | ley | 524.5 | 825.2 | 968.8 | 1261.2 | **0.00 %** |
| 10 | CARE | 348.2 | 571.2 | 671.3 | 908.6 | 0.00 % |
| 11 | ley | 525.8 | 781.1 | 886.3 | 1679.3 | **0.00 %** |
| 11 | CARE | 307.8 | 479.7 | 565.2 | 1371.1 | 0.00 % |
| 12 | ley | 251.9 | 694.4 | 974.1 | 1797.2 | **0.00 %** |
| 12 | CARE | 488.0 | 748.5 | 830.1 | 942.5 | 0.00 % |

Todo en µs. `CARE` solo sobre los ciclos que efectivamente resolvieron.

**No hace falta decimar:** la CARE cabe en cada ciclo a 500 Hz con margen. Ojo
con el número aislado: en banco, `solveCare` tarda **57 µs**
(`test_care_solver`), pero **dentro del lazo, con Gazebo cargando la máquina
(RTF ≈ 0.07 por el trimesh de la escena), sube a 348 µs de media y 1371 µs de
pico** — un factor 6× en la media y 24× en el pico. Cualquier presupuesto de
cómputo medido en banco es optimista por esa razón.

### 5.3 Criterio 3 — seguimiento sin saturación sostenida

| ensayo | RMSE q [rad] | TCP RMSE [mm] | TCP máx [mm] | θ máx [mrad] | máx\|τ\| [N·m] | sat % | racha sat |
|---|---|---|---|---|---|---|---|
| 10 | 1.84e-4 | 0.0555 | 0.3192 | 1.98 | 27.42 | **0.00** | **0** |
| 11 | 2.5e-5 | **0.0105** | 0.1089 | 0.14 | 27.49 | **0.00** | **0** |
| 12 | 1.93e-4 | 0.0668 | 0.3806 | 1.93 | 27.42 | **0.00** | **0** |

En la meseta del corte (régimen): 0.0149 / 0.0093 / 0.0120 mm respectivamente.
**Saturación nula** en las tres corridas, ni un solo ciclo recortado.

### 5.4 Lo que cuesta cada decisión de diseño

- **`scheduled` vs `fixed`** (11 vs 10): **5.3× mejor** en TCP RMSE sobre TRACK
  (0.0105 vs 0.0555 mm), con el mismo coste de cómputo. Es la consecuencia
  directa de §5.1: el margen de estabilidad cae 3× al congelar `Q`.
- **Decimar la CARE 10:1** (12 vs 10): TCP RMSE **20 % peor** sobre TRACK
  (0.0668 vs 0.0555 mm), a cambio de bajar el tiempo medio de la ley de 525 a
  252 µs. La estabilidad **no** se resiente: `max Re(λ)` sale idéntico, así que
  la `K` congelada sigue estabilizando `A(q,q̇)` entre actualizaciones. Es la
  respuesta cuantificada a la decimación que contempla el plan.

> El **default queda en `Q_mode: fixed`**, que es la formulación del plan (`Q` y
> `R` son pesos de coste constantes). `scheduled` está disponible y medido; si la
> campaña de la FASE 8 debe usarlo, es una decisión a tomar explícitamente, no
> por omisión.

---

## 6. Cambios en la clase base

Dos añadidos a `TorqueControlNodeBase` (se avisa porque el plan lo exige; no se
modifica ninguna firma existente):

1. **`requestSafeHold(reason)`** — parada segura pedida por la **ley**. El
   watchdog de la FASE 3 solo vigila la infraestructura del lazo (ritmo de
   ciclo, llegada de `/joint_states`); hay fallos que solo la ley puede ver, y
   el plan pide explícitamente "si falla, HOLD seguro". Efecto: se entra en
   `SAFE_HOLD` (terminal) y el par de la ley de **ese** ciclo no se publica.
   La FASE 6 (ASTSMC) la necesitará igual.
2. **`traceMetadata()` de `private` a `protected`**, más `csvDir()` y
   `testNum()` — para que una subclase con ficheros de log propios ponga la
   **misma** cabecera de trazabilidad (git SHA, hash de parámetros efectivos)
   que el CSV unificado.

También se refactorizó `csv_logger.cpp` para compartir `makeDirs`,
`isoTimestamp` y `resolveDir` con `diag_logger.cpp` vía
`include/ur5_dyn_control/log_utils.hpp`, en lugar de duplicarlos.

---

## 7. Tests

`test/test_care_solver.cpp` (9 casos):

| test | qué fija |
|---|---|
| `DoubleIntegradorCoincideConLaSolucionCerrada` | `P` y `K` contra la solución analítica, en las 4 inercias extremas del robot |
| `PoloDobleDefectivoSeResuelveIgual` | el caso `ζ = 1` que rompe el método de autovectores |
| `MimoAleatorioDaResiduoPequenoYLazoEstable` | 20 problemas aleatorios: residuo, estabilidad y `P ≻ 0` |
| `RechazaEntradasInvalidas` | `R` no definida positiva, dimensiones, par no estabilizable |
| `MargenDeControlabilidad` | par controlable vs no controlable |
| `ParSdcDelUr5eEsControlableYEstabilizante` | 12 configuraciones del robot real |
| `QProporcionalAMCuadradoPoneLos12PolosEnMenosWn` | 9 combinaciones `(wn, ζ)`, raíces exactas |
| `QConLaDiagonalDeMDispersaLosPolos` | **regresión del escalado por inercia** |
| `CosteDeUnaResolucionCabeEnUnCicloDe500Hz` | presupuesto de cómputo |

---

## 8. Pendiente

- **`ft_data`** — el `wrench` sigue a cero en simulación (FASE 3 pendiente).
- **Fricción** — `friction_compensation: none`; la campaña de identificación en
  el robot real sigue pendiente (FASE 2).
- **FASE 7** — el sintonizador multiobjetivo aún no cubre esta ley. El espacio
  de diseño es de **2 escalares** (`wn`, `ζ`) frente a los 13 del SMC, porque
  `Q ∝ M²` deja el ancho de banda como único grado de libertad efectivo.
- **Robot real** — bloqueado por G1–G4 (`docs/00_prereqs.md`).
