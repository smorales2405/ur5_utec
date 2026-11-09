# Protocolo — B7.1: el SMC en aire en el UR5e real, con las ganancias finales

**Estado:** paso 1 hecho y limpio (§5.1): el modo de 33 Hz depende de la `G`, y la incisión iba con
`shoulder_pan` en 0.50. **El paso 2 (`smc_1610`) vibró a unos 35 Hz y lo paró la guarda, y corrió con
las ganancias equivocadas** (§5.2). El error de configuración está corregido (D30), y la incisión se
repite por escalones, desde r = 0.50 (D31, §3). **El escalón 0.50 (`smc_1620`) pasó, pero con 11 mm de
TCP y un modo de 31.6 Hz casi sin amortiguar mientras `shoulder_lift` desliza (§5.3): no se sube a
0.71.** Casi todo el error era la gravedad que el robot compensa de más (B4.4): el nodo la resta
(D32), y la incisión se repite con ella desde r = 0.50, como `smc_1630`–`1633`. Con la fricción en tabla (D33) y
`wrist_1` a r = 1 (D34), `smc_1650` fue la primera incisión limpia (§5.6); por encima de r = 0.50 la sesión no corre
(D35). **A `shoulder_lift` le faltaba fricción a la velocidad del corte: medida con su carga (`cur_1660`, D36), su fila
de la tabla cambia (D37, §5.7), y la incisión se repite como `smc_1670`.** Plan: B7.1, D28 y D29 de
`Journal_UR5eControl_plan.md`. Código: `ur5_identification/air_bringup.py` y
`scripts/run_air_bringup_real.py`.

**Ganancias:** las de la candidata de la fila 46, con `shoulder_pan` más baja (D28, rampa completa en
`09c_gain_ramp_protocol.md` §8). Las corridas las lanza solo la sesión, como en la rampa: nunca a mano.

## 1. Por qué dos pasos

**La base en la incisión no es la de la rampa.** Al empezar el corte, `M₁₁` vale 2.61, frente a 1.86 en
la postura de la rampa, así que con r = 0.71 la `G` de `shoulder_pan` sube de 44.6 a 54.0. Es la `G`
de r = 0.85 en la rampa, donde su modo de 33 Hz cayó a ζ ≈ 0.014. Los datos de la rampa no separan si
ese modo depende de la `G` sola o de la `G` frente a la inercia:

| | `G` de la base | `G / (M₁₁ + rotor)` | ζ medida |
|---|---|---|---|
| rampa, r = 0.71 (`smc_1510`) | 44.6 | 15.4 | 0.030 |
| rampa, r = 0.85 (`smc_1520`) | 53.4 | 18.5 | 0.014 |
| incisión, r = 0.71 | 54.0 | 14.9 | ? |

Si depende de la `G`, la incisión con 0.71 deja el modo como en `smc_1520`; si depende de la relación,
como en `smc_1510`. **El paso 1 lo mide antes de la incisión.**

En la incisión, `shoulder_pan` no se mueve y su fricción la sujeta, así que el modo apenas se excita: la
incisión sola no lo diría.

## 2. Paso 1: la base, con la inercia de la incisión

- **Postura:** punta en y = 0.69, z = 0.05 (q = 1.5708, −0.9432, 1.4613, −2.0889, −1.5708, 3.1416).
  `M₁₁` = 2.47 y `G` de la base en reposo 52.2 con r = 0.71, frente a 2.61 y 54.0 al empezar el corte.
  En y = 0.70 la `G` de `shoulder_lift` llega a 120.5 en el barrido, y la sesión no lo lanza (g6).
- **Barrido:** `shoulder_pan` ±0.20 rad a 0.05 y 0.20 rad/s, como en la rampa. La punta no baja de
  96 mm sobre el tablero, y recorre unos ±14 cm en horizontal.
- **Corridas:** `smc_1600` con `shoulder_pan` en r = 0.50 y `smc_1601` en 0.71; las demás juntas, a
  r = 1. `G` máxima 118.7 (`shoulder_lift`), margen ×3.55 en las dos.
- **Parada:** los criterios de la rampa (`09c` §4); el crecimiento entre las dos se compara junta a
  junta.
- **Lo que reviso después:** ζ del modo de 33 Hz en la meseta de 0.05 rad/s, con los dos métodos de
  `09c` §8.2. **Propuesta, a decidir al revisar:** si en `smc_1601` sale ζ ≥ 0.025, la incisión va con
  `shoulder_pan` en 0.71; si no, en 0.50. Es una cota declarada.

## 3. Paso 2: la incisión en aire

- **La trayectoria del corte, a la altura real**, de `incision_common.yaml`: 80 mm a 10 mm/s y 5 mm de
  profundidad, con el acople **sin hoja**. Elegido por el usuario (D29).
- **Mesa despejada:** sin fijación, bandeja ni muestras; solo el tablero. La punta baja a 37 mm de él.
- **Guarda de seguimiento:** 0.03 rad, 21 mm a 0.7 m de alcance, por debajo de esos 37 mm. Cota
  declarada: el mayor error de la rampa fue 0.022 rad, en `shoulder_pan` a 0.2 rad/s, y en la incisión
  esa junta no se mueve. La de saturación, la de la rampa.
- **Por escalones (D31, tras `smc_1610`, §5.2):** r = 0.50, 0.71, 0.85 y 1 en todas las juntas menos
  `shoulder_pan`, que se queda en 0.50 (§5.1). Es la primera vez que `shoulder_lift` desliza con par a
  estas ganancias: en la aproximación va a unos 0.17 rad/s, y ahí empezó la vibración de `smc_1610`.

  | `r` | corrida | `K_P` al empezar el corte [N·m/rad] | `G` máx. (base) | margen |
  |---|---|---|---|---|
  | 0.50 | `smc_1650` | 132, 301, 270, 104, 5.6, 0.4 | 59.7 (38.1) | ×5.38 |
  | 0.71 | `smc_1651` | 132, 607, 544, 104, 11.2, 0.7 | 84.8 (38.1) | ×4.95 |
  | 0.85 | `smc_1652` | 132, 869, 779, 104, 16.1, 1.1 | 101.5 (38.1) | ×4.23 |
  | 1.00 | `smc_1653` | 132, 1203, 1078, 104, 22.3, 1.5 | 119.4 (38.1) | ×3.55 |

- **Con la gravedad corregida (D32) y la fricción medida (D33):** la sesión pasa `gravity_correction:=true`
  y `friction_compensation:=table`, y `wrist_1` va a r = 1 en todos los escalones (D34). Corridas
  `smc_1650`–`1653`. Quedan como referencia `smc_1620` (sin corregir), `smc_1630` (gravedad) y `smc_1640`
  (gravedad y fricción, `wrist_1` a 0.50).
- **Una corrida por sesión,** con `--r`, y cada escalón exige el CSV del anterior. **Por encima de r = 0.50, la
  sesión no corre (D35):** el modo de 31.6 Hz se haría inestable (§5.3). Entre escalones,
  revisión: los criterios automáticos no ven bien un modo que crece dentro de una corrida, y en
  `smc_1610` pasó de nada a saturar en 1.6 s.
- **El brazo** empieza en el `q_init` del nodo, y al acabar lo sostiene el JTC donde termine la
  retirada: la sesión no lo recoloca.
- **Parada:** los criterios de la rampa, con el crecimiento comparado junta a junta y una banda más,
  la de 30–40 Hz: el modo de `smc_1610` (34.9 y 36.5 Hz) cae entre las dos de la rampa. Además, un
  error de TCP de más de 5 mm (la profundidad del corte) o la punta medida a menos de 30 mm del
  tablero.
- **Lo que reviso entre escalones:** el espectro de `shoulder_lift` y el codo en 30–40 Hz mientras
  `shoulder_lift` desliza, y si crece con `r` más de lo que crece `G`.

## 4. Ejecución

Preparación: la de la rampa (`09c` §2, puntos 0 a 4). Para el paso 2, además, quitar del tablero la
fijación, la bandeja y las muestras, y comprobar que el acople no lleva la hoja.

```bash
source ~/ur5_ws/install/setup.bash
G=~/ur5_ws/src/ur5_utec/ur5_trajectory_optimization/results/gain_tuning/smc_v7_rotor/candidata_fila46.yaml

# Paso 1, sin mover nada y después la sesión:
ros2 run ur5_identification run_air_bringup_real.py --gains-file $G --paso base --dry-run
ros2 run ur5_identification run_air_bringup_real.py --gains-file $G --paso base --test-base 1600 --payload-kg 0.155

# Paso 2, un escalón por sesión, en orden (0.5, 0.71, 0.85, 1) y revisando cada uno:
ros2 run ur5_identification run_air_bringup_real.py --gains-file $G --paso incision --r 0.5 --dry-run
ros2 run ur5_identification run_air_bringup_real.py --gains-file $G --paso incision --r 0.5 --test-base 1600 --payload-kg 0.155
# Desde D37 crea smc_1670; por encima de r = 0.50 no corre (D35).
```

La sesión pide «ACEPTO» y la masa tecleada al empezar. En el paso 1, el nombre de la junta en cada
corrida y «SUBIR» antes de la segunda; en el paso 2, «INCISION». Pone G4 a cero, lleva el robot a la
postura con el JTC a 0.1 rad/s y nunca sobrescribe un CSV con datos. En el paso 1, `--resume` salta las
corridas hechas.

**Si oye o ve vibración, o la punta se acerca al tablero, paro de emergencia, sin esperar al script.**

## 5. Lo ensayado sin robot

En el gemelo, en otro dominio y con la fricción real en la planta, con las mismas tablas, guardas y
ganancias que la sesión:

| corrida | qué | resultado |
|---|---|---|
| `smc_98961` | base, `shoulder_pan` 0.50 | limpia, sin saturación |
| `smc_98962` | base, `shoulder_pan` 0.71 | limpia, sin saturación |
| `smc_98963` | incisión en aire, `shoulder_pan` 0.71 | limpia; TCP 0.35 mm RMS en el corte, 0.51 mm máx.; punta a 37 mm del tablero; error articular ≤ 2.5 mrad |
| `smc_98964` | incisión en aire, `shoulder_pan` 0.50 | igual que `98963` |

**Corrección (2026-11-05):** `smc_98963` y `98964` corrieron las dos con la candidata a r = 1 también
en `shoulder_pan`, no con 0.71 y 0.50 (§5.2). Salieron iguales por eso, no porque `shoulder_pan` no se
mueva, como escribí. No comprobé sus ganancias efectivas, como sí había hecho en los ensayos de la
rampa.

**Con el arreglo (D30)**, `smc_98965` (incisión, `shoulder_pan` 0.50) y `smc_98966` (base, 0.50)
corrieron con las ganancias pedidas, comprobadas en sus `.params.yaml`, y el nodo lo confirma en su
registro (`gains_check`). La incisión: 0.35 mm RMS de TCP en el corte, punta a 37 mm. Y el launch del
robot, sin robot y en otro dominio (`smc_97401`), arranca con `G` máxima de la base 38.07 sobre la
incisión, frente a 76.1 en `smc_1610`, y con `gravity_in_command`, `tau_max`, el retardo y el rotor
intactos.

**Los escalones de la incisión (D31)**, `smc_98967`–`98970`, con las ganancias pedidas comprobadas en
sus `.params.yaml`: los cuatro limpios, sin crecimiento en ninguna banda (la de 30–40 Hz, ≤ 0.004 N·m)
y con la punta a 37 mm del tablero. El TCP en la meseta del corte baja con `r`: 1.41, 0.70, 0.48 y
0.35 mm de RMS (máximo 1.98 mm a r = 0.50). El gemelo no tiene el modo de 35 Hz: esto no dice dónde
aparecerá en el robot.

Solo prueba que la configuración, las tablas y los indicadores funcionan de principio a fin, y que el
nodo acepta las ganancias. **No prueba el modo de 33 Hz:** el gemelo no lo tiene, y por eso el paso 1
se hace en el robot. Tampoco el error de TCP del robot: el gemelo tiene 1 ciclo de retardo y no tiene
rotor.

## 5.1 Resultado del paso 1 (2026-11-05, `af08746`)

`smc_1600` y `smc_1601`: completas, sin guardas, sin saturación y sin motivos de parada; margen del
nodo ×3.55; el robot siguió en `NORMAL`.

| | `smc_1600` (base 0.50) | `smc_1601` (base 0.71) |
|---|---|---|
| `G` de la base en reposo | 36.8 | 52.2 |
| rizado máx. / banda 20–35 Hz de la base | 0.25 % / 0.024 N·m | 0.27 % / 0.039 N·m |
| ζ del modo, ajuste a la PSD | 0.030 ± 0.003 (POS 0.062, NEG 0.017) | **0.016 ± 0.001** |
| ζ del modo, autocorrelación | 0.015–0.018 | 0.012–0.020 |
| altura del pico sobre el suelo | ×56 | ×116 |
| error máx. de la base (meseta de 0.2 rad/s) | 0.031 rad | 0.022 rad |

- **El modo depende de la `G`, no de la `G` frente a la inercia.** Con la `G` relativa de `smc_1510`
  (14.9 frente a 15.4), `smc_1601` sale como `smc_1520` (ζ 0.014) y con el doble de pico. La pregunta de
  §1 queda contestada: con 0.71, la base del corte tendría el modo como a r = 0.85 en la rampa.
- **Incluso con 0.50 queda poco amortiguado aquí:** 0.015–0.030, frente a 0.025–0.044 en la rampa con
  una `G` algo menor (31.4). La postura extendida no parece ayudar.
- **Por la regla de §2** (ζ ≥ 0.025 en `smc_1601`), la incisión va con `shoulder_pan` en **0.50**. En la
  incisión esa junta no se mueve y su fricción la sujeta, así que el modo apenas se excita; importa en
  cualquier tarea en la que `shoulder_pan` deslice.
- **La fricción de la base depende de la postura:** 7.99 N·m a 0.05 rad/s aquí, frente a 7.62 en la
  postura de la rampa, y 11.2 a 0.2 rad/s. A 0.2 rad/s supera a `K` en 1.6 N·m y el error crece toda la
  meseta (0.031 rad con 0.50). No es inestabilidad, y en la incisión la base no se mueve.

## 5.2 `smc_1610` (2026-11-05): vibración a 35 Hz en la incisión, con las ganancias equivocadas

**Qué pasó.** Con `--pan-r 0.5`, al empezar a bajar se oyó una vibración y el robot se detuvo. A 1.75 s
de la trayectoria, en la aproximación, con `shoulder_lift` y `wrist_1` moviéndose a unos 0.17 rad/s,
empezó a crecer una oscilación de 34.9 y 36.5 Hz: 14.7 N·m en `shoulder_lift`, 6.9 en el codo y 1.1 en
`shoulder_pan`. Se multiplicó por 2–3 cada 0.25 s (ζ ≈ −0.02) hasta saturar `shoulder_lift`, y la guarda
de vibración (12 % de ciclos saturados) dio `SAFE_HOLD` a los 3.35 s. La sesión devolvió el control al
JTC: el robot quedó quieto (deriva 3.5e-5 rad), sostenido por el controlador de trayectoria, en
`NORMAL` y sin paro de protección. El F/T no registró contacto.

**Ganancias equivocadas.** El nodo corrió la candidata a r = 1 en las seis juntas: `shoulder_pan` con
λ = 17.727 y `G` máxima de 76.1 sobre la incisión, no con 8.863 y 38.1. Las sobrescrituras del launch
(`lambda_joint`, `phi_joint`) no llegaron:

- launch_ros escribe los diccionarios de parámetros como `/**` si el nodo no tiene espacio de nombres;
- rclcpp aplica las secciones en el orden en que aparece cada nombre de nodo por primera vez, y dentro
  de una sección gana el fichero posterior;
- en la incisión, el primer fichero (`incision_common.yaml`) abre `/**`, y la sección con nombre del
  gains_file se aplica después: pisa las sobrescrituras. En los barridos, el primer fichero
  (`gain_ramp_params.yaml`) ya tiene nombre y el orden se invierte.

Las doce corridas de la rampa y las dos del paso 1 tuvieron exactamente las ganancias pedidas
(comprobado sobre sus `.params.yaml`); de las del robot, solo `smc_1610` no. En el repositorio, solo la
rampa y B7.1 combinan gains_file con sobrescrituras de ganancias. El test que comprobaba el arranque de
los escalones de la rampa (`test_discrete_check.py`) pasaba las ganancias por `-p`, que siempre gana, y
no lo vio.

**Causa de la vibración: probable, no separada.** El modo de unos 35 Hz de `smc_712`, ahora con
`shoulder_lift` deslizando a `G` cercana a 120 (máximo de la incisión, 119.8) y el codo a 71. La rampa
nunca movió `shoulder_lift`, y este protocolo lo declaraba (§7 de `09c`). La `G` de 76 en
`shoulder_pan`, por el error, puede haber contribuido; esta corrida no lo separa.

**Arreglo (D30).** Los launches escriben las sobrescrituras en un último fichero, con los mismos valores
en `/**`, `<nodo>` y `/<nodo>`: ganan sea cual sea el orden. Y el nodo SMC compara las ganancias que se
le sobrescriben (`gains_check.lambda`, `gains_check.phi_joint`) con las efectivas, y si no coinciden no
arranca, antes de comandar par. Tests: `test_param_precedence.py`, que reproduce la causa con nodos de
verdad.

## 5.3 Escalón r = 0.50 de la incisión (`smc_1620`, 2026-11-09, `2a29e68`)

Completa, sin guardas ni saturación, margen del nodo ×7.62, con las ganancias pedidas (`gains_check` en
el registro, `G` máxima 38, 60 y 36 en las tres juntas grandes). La sesión la marcó por el error de
TCP.

**El TCP: un desfase lento, por encima de lo pedido.** 5.41 mm RMS en la meseta del corte y 11.0 mm de
máximo, al final de la aproximación. Lo pone sobre todo el codo (13 mrad, 8.5 mm por sí solo), y en el
corte `wrist_1` (17 mrad). La punta quedó siempre por encima (z entre +1 y +8 mm; +4.6 de media en el
corte) y no bajó de 39 mm sobre el tablero: con la hoja, el corte saldría casi sin profundidad. En
Gazebo, el mismo escalón da 1.4 mm. Con r = 0.50 la rigidez es la cuarta parte de la final.

**Un modo de 31.6 Hz, casi sin amortiguamiento, mientras `shoulder_lift` desliza.**

- **Frecuencia fija:** en la aproximación, `shoulder_lift` y el codo llevan un pico a 31–33 Hz en cada
  ventana de 0.5 s, mientras el tono de 10 veces el giro de `shoulder_lift` recorre de 4 a 30 Hz. No es
  el tono: es un modo.
- **Amplitud:** crece con el deslizamiento hasta 0.9 N·m en `shoulder_lift` y 0.3 en el codo, y cae
  cuando la junta frena. En Gazebo, 0.004.
- **Amortiguamiento** (2.5–6 s): ζ ≈ 0.004–0.016 por ajuste a la PSD y 0.006–0.010 por
  autocorrelación, en la resolución de la PSD, así que puede ser menor.
- **Con más ganancia:** en `smc_1610` (r = 1) la frecuencia subió a 33–36 Hz y el modo creció sin
  límite (ζ ≈ −0.02). Es lo que hace un modo de lazo cerrado que pierde amortiguamiento con la `G`.

**Extrapolación, no medida:** en línea recta con la `G` de `shoulder_lift` (60 a r = 0.50, 120 a
r = 1), ζ = 0 cae hacia `G` ≈ 77, r ≈ 0.64. El escalón de 0.71 (`G` 85) estaría ya del lado inestable.
`smc_1610` tenía además `shoulder_pan` a r = 1, y eso no se puede separar.

**El sesgo es la gravedad que el robot compensa de más (D32).** Lo que la ley tuvo que añadir, `K·sat(s/φ)`,
coincide con lo que predice B4.4 (ε·ĝ, con g_real = (1 − ε) g_UR):

| fase | `shoulder_lift`: ley | B4.4 | codo: ley | B4.4 |
|---|---|---|---|---|
| aproximación | −1.43 | −1.18 | −1.06 | −1.07 |
| corte | −1.74 | −1.47 | −0.56 | −0.90 |
| retirada | −1.50 | −1.31 | −1.55 | −0.88 |

En N·m. En las fases lentas el codo queda pegado por la fricción y la ley apenas añade. El evaluador, con ese
exceso, da 6.00 mm de TCP a r = 0.50 (medido: 5.41 mm) y 0.11 mm corregido, a cualquier `r`. `wrist_1`
(17 mrad en el corte) no lo explica la gravedad (0.08 N·m).

**Signo, comprobado en Gazebo** (`smc_98971`, r = 0.50, con la corrección): allí la gravedad es exacta, y
corregirla baja la punta −5.2 mm de media en el corte, el mismo tamaño que el robot la subía (+4.6) con el
signo contrario. Peor caso, si el robot no tuviera exceso: la punta a 31 mm del tablero, por encima de los
30 mm del criterio de parada.

**Con la fricción en tabla (D33), en Gazebo** (`smc_98972`, r = 0.50, sin la corrección de gravedad: allí la
gravedad es exacta): limpia, con las ganancias pedidas y `friction_compensation: table`; TCP 1.15 mm RMS en el
corte (1.41 con la fricción lineal, `smc_98967`) y la punta a 36 mm. La planta del gemelo lleva la fricción
lineal, así que esto solo prueba que el nodo carga y usa la tabla.

**Con `wrist_1` a r = 1 (D34), en Gazebo** (`smc_98973`, frente a `98972`): limpia, con las ganancias pedidas. El
error de `wrist_1` en el corte baja de 7.78 a 1.94 mrad RMS, la cuarta parte, y su aporte al TCP de 2.18 a 0.54 mm;
`shoulder_lift` y el codo, idénticos. El TCP total sube de 1.15 a 2.26 mm porque en el gemelo, con su fricción
lineal, la tabla deja el codo con 5.5 mrad de sesgo, y el error de `wrist_1` lo compensaba por casualidad en la
punta. En el robot el codo ya no tiene ese sesgo con la tabla (`smc_1640`, −0.36 mrad).

**Recomendación: no correr r = 0.71.** Con esta ley, la rigidez que pide el corte y el amortiguamiento
de este modo no caben juntos. Las vías posibles son decisión del usuario: un filtro de banda eliminada
en la rama derivativa a 31–36 Hz, mejor compensación de gravedad y fricción para necesitar menos
rigidez, o acción integral.

## 5.4 Con la gravedad corregida (`smc_1630`, 2026-11-09, `7f76807`, D32)

Mismo escalón (r = 0.50, `shoulder_pan` 0.50), con `gravity_correction`: completa, sin guardas, margen ×7.62,
ganancias y ε comprobados en su `.params.yaml` y en el registro del nodo.

| | `smc_1620` (sin corregir) | `smc_1630` (D32) |
|---|---|---|
| TCP RMS en el corte | 5.41 mm | **3.49 mm** |
| TCP máximo | 11.0 mm | **6.83 mm** |
| z − z_des medio en el corte | +4.6 mm | **−1.3 mm** |
| punta sobre el tablero (mín.) | 39 mm | 33 mm |
| banda de 30–40 Hz de `shoulder_lift` en la aproximación (máx. cada 2 s) | 0.52 N·m | 0.19 N·m |

- **La gravedad, corregida:** el sesgo vertical desaparece; z queda entre −3.6 y +2.3 mm en el corte.
- **Lo que queda es fricción a baja velocidad,** en dos formas distintas. En el peor instante (20.8 s, corte),
  `wrist_1` aporta 3.9 mm (14 mrad) y el codo 3.1 mm (4.8 mrad).
  - **Codo: el modelo sobrecompensa.** En el corte va a 0.027 rad/s y la ley resta +0.97 N·m de media. La
    campaña de agosto (`cur_952`, G4 a 0.0, en `q_init`) midió 6.49 N·m a 0.02 rad/s y 7.51 a 0.05, frente a 7.74
    y 8.15 del modelo F_c + F_v·v: a esa velocidad la compensación empuja de más, unos 0.9 N·m.
  - **`wrist_1`: se pega y salta.** Su modelo coincide con la campaña a 0.02–0.2 rad/s (1.81 N·m a 0.02); en el
    corte va a 0.012 rad/s, se queda pegada el 12 % del tiempo y acumula hasta 15.6 mrad antes de soltarse,
    con la rigidez de r = 0.50 (`K_P` 26). Ninguna compensación fija lo quita.
- **El modo de 31.6 Hz** sigue ahí, con menos amplitud: la ley ya no pelea contra 1.5 N·m de sesgo.

## 5.5 Con la fricción en tabla (`smc_1640`, 2026-11-09, `534e2cd`, D33)

Mismo escalón (r = 0.50) con la gravedad corregida y la fricción medida: completa, sin guardas, margen ×7.62, con
`friction_compensation: table`, la velocidad deseada y ε, comprobados en su `.params.yaml` y en el registro del
nodo. El aviso del launch («velocidad MEDIDA (default)») mira solo sus argumentos, no el YAML: es engañoso, no un
fallo.

| | `smc_1620` | `smc_1630` (D32) | `smc_1640` (D32 + D33) |
|---|---|---|---|
| TCP RMS en el corte | 5.41 mm | 3.49 mm | **2.95 mm** |
| TCP máximo | 11.0 mm | 6.83 mm | **5.26 mm** |
| punta sobre el tablero (mín.) | 39 mm | 33 mm | 33 mm |

En el corte, por junta (error medio; lo que añade la ley; tiempo pegada):

- **Codo, resuelto:** +3.70 → −0.36 mrad; +0.99 → −0.10 N·m.
- **`shoulder_lift`, ahora le falta:** −0.64 → +1.48 mrad; −0.19 → +0.43 N·m. Va a 0.015 rad/s, por debajo de la
  menor velocidad medida, y en una postura distinta de la de la campaña: es la salvedad de la tabla.
- **`wrist_1`, sin cambio:** pegada el 15 % del corte, hasta 13.5 mrad. En el peor instante (22.0 s) aporta 3.25 mm,
  y `shoulder_lift` 2.86.

La vía 1 ha dado lo que podía: la gravedad y el sesgo de fricción del codo. Lo que queda es que `wrist_1` se pegue, y
un sesgo de 0.4 N·m en `shoulder_lift` que depende de la postura. Ninguna compensación fija lo quita.

## 5.6 Con `wrist_1` a r = 1 (`smc_1650`, 2026-11-09, `e212f98`, D34): la primera incisión limpia

Completa, sin guardas ni motivos de parada, margen del nodo ×5.37, con lo pedido (λ de `wrist_1` 51.7, tabla de fricción,
velocidad deseada y ε, en su `.params.yaml`).

| | `smc_1620` | `smc_1630` | `smc_1640` | `smc_1650` |
|---|---|---|---|---|
| correcciones | — | gravedad | + fricción en tabla | + `wrist_1` a r = 1 |
| TCP RMS en el corte | 5.41 mm | 3.49 mm | 2.95 mm | **2.34 mm** |
| TCP máximo | 11.0 mm | 6.83 mm | 5.26 mm | **4.24 mm** |
| `wrist_1` pegada en el corte | 12 % | 12 % | 15 % | **2 %** |
| modo de 31.6 Hz: q̇ de `shoulder_lift` (2.5–6 s) | 7.6e-3 | 3.2e-3 | 3.1e-3 | 3.3e-3 |

- **`wrist_1`:** error RMS en el corte de 5.38 a 1.47 mrad, y máximo de 13.45 a 3.97.
- **El modo no empeora:** ζ ≈ 0.031 (ajuste) y 0.016 (autocorrelación), envolvente de ruido de banda estrecha
  (CV 0.57). Con la gravedad corregida su amplitud bajó a menos de la mitad que en `smc_1620`, y así sigue.
- **Lo que queda es `shoulder_lift`:** +1.23 mrad de media en el corte, porque le faltan 0.36 N·m a 0.015 rad/s; en el
  peor instante (17.0 s) aporta 2.39 mm.
- **Frente a los criterios:** el máximo (4.24 mm) queda por debajo de los 5 mm de profundidad de corte, pero el RMS
  (2.34 mm) sigue por encima de la tolerancia declarada de 2 mm (g4).
- **No se sube a r = 0.71:** con estas ganancias, `shoulder_lift` y el codo siguen limitados por el modo (§5.3). La
  sesión lo permitiría, porque solo exige que exista el escalón anterior.

## 5.7 Fricción de `shoulder_lift` a baja velocidad, con la carga del corte (`cur_1660`, 2026-11-09, D36 y D37)

Lo que queda en `smc_1650` es `shoulder_lift`: le faltan 0.36 N·m en el corte, donde va a 0.015 rad/s, por debajo de la
menor velocidad de la campaña de agosto, que además midió en `q_init`, con la mitad de carga gravitatoria (−21 N·m
frente a −45 en el corte).

- **Barrido por control de posición, sin par:** el controlador de trayectoria mueve `shoulder_lift` ±0.06 rad a 0.005,
  0.01, 0.015, 0.02, 0.05 y 0.1 rad/s, unos 160 s; las demás juntas, quietas. G4 a 0.0, como la campaña.
- **Postura:** punta en y = 0.69, z = 0.10: `shoulder_lift` con −42.4 N·m de gravedad, el 93 % del corte; la punta no
  baja de 105 mm sobre el tablero.
- **Resultado:** fricción = ½ (i⁺ − i⁻)·k por velocidad, comparada con la fila de `shoulder_lift` de la tabla actual.
  Con 0.02 y 0.05 rad/s se ve además el efecto de la carga frente a agosto.
- **Comprobado sin robot:** la tabla del barrido la construye el mismo generador de C++ que la campaña: 158 s, las
  doce mesetas a su velocidad y 0.52 rad/s² de aceleración máxima.

```bash
ros2 run ur5_identification run_lift_friction_real.py --dry-run
ros2 run ur5_identification run_lift_friction_real.py --payload-kg 0.155
```

**Primer intento (2026-11-09, 13:36 y 13:38): pareció colgado y no movió el robot.** Antes de barrer, la sesión
construye la tabla con un nodo que la escribe en un par de segundos y se queda en PRE_HOLD; `run_current_sweep.py`
lo mataba a los 120 s fijos, y la sesión mandaba esa salida solo al registro. Se interrumpió las dos veces antes de
que venciera. Ahora se para en cuanto el nodo anuncia la tabla (1.4 s), la salida va también a pantalla, y la
confirmación ya no dice «comandar par».

**Resultado (`cur_1660`, 13:46–13:49, `f46044c`): limpio.** Fricción de `shoulder_lift` [N·m]:

| v [rad/s] | 0.005 | 0.010 | 0.015 | 0.020 | 0.050 | 0.100 |
|---|---|---|---|---|---|---|
| con la carga del corte (`cur_1660`) | 6.398 | 6.235 | 6.358 | 6.464 | 7.065 | 8.109 |
| tabla de agosto (`q_init`; por debajo de 0.02, su valor en 0.02) | 6.182 | 6.182 | 6.182 | 6.182 | 7.136 | 8.096 |

- **A 0.05 y 0.1 rad/s coincide con agosto** (±0.07 N·m). **Por debajo de 0.02, 0.18–0.28 N·m más.** Con un solo
  barrido no se separa si es la carga, la postura o la temperatura; para la incisión da igual, porque se midió en
  sus condiciones.
- **Plana entre 0.005 y 0.02:** sube y baja 0.23 N·m sin tendencia. Se usa tal cual, sin suavizar.
- **Explica el déficit de `smc_1650` con su signo.** En el corte, `shoulder_lift` baja (q̇_des −0.0147 rad/s), va
  1.2 mrad por detrás, y el término conmutado de la ley empuja de media −0.36 N·m en el sentido del movimiento:
  fricción sin compensar. Con la tabla nueva, el feedforward añade −0.22 N·m en ese sentido: el 60 % de lo que faltaba.

**D37: la fila de `shoulder_lift` de la tabla sale de `cur_1660`.** `build_friction_table.py` la toma de ahí
(`INCISION_LOAD_RUNS` en `friction_table.py`), y la tabla pasa a 11 velocidades, de 0.005 a 1 rad/s. Por debajo de
0.02, las demás juntas repiten su primer valor, que es lo que ya hacía el nodo: para ellas no cambia nada. La tabla
solo la usa la incisión (`incision_common.yaml`). La incisión se repite como `smc_1670`–`1673`; 1660 se salta porque
es el barrido.

**Ensayado en Gazebo:** el nodo carga las 11 velocidades (lift a 0.015 rad/s: 6.358 N·m en el `.params.yaml`) y las
ganancias pedidas; la corrida sale limpia. Comparado con el ensayo de D34 (98973, sin la corrección de gravedad, que
en el gemelo es un error a propósito porque no compensa de más), 98975 baja el error medio de lift de +2.23 a
+1.66 mrad y deja igual el codo y `wrist_1`. (98974, con la corrección, no es comparable.)

**Qué esperar en el robot: poco.** El error medio de lift en el corte de `smc_1650` es +1.0 mrad, pero su desviación,
2.3 mrad. Por cinemática sobre `smc_1650`, quitar el 60 % de ese error medio baja el RMS del TCP de 2.34 a unos
2.2 mm; con lift perfecto bajaría a 1.36 mm. Lo que la tabla no toca:

- **Rizado del reductor de lift.** En el corte estable (0.018 rad/s), su error oscila a unos 0.56 Hz, y el par a la misma
  frecuencia con 0.68 N·m de amplitud. En `cur_1660`, por control de posición, la corriente de lift tiene un rizado de
  0.4–0.7 N·m a un orden fijo del giro del motor (2.22, con reducción 101) a 0.01, 0.015 y 0.02 rad/s: depende de la
  posición, cada unos 0.028 rad de la junta. Es la misma componente.
- **El arranque del corte:** el primer segundo y medio, lift va 2.4–3.2 mrad por delante de la referencia.

Por bandas, sin la parte de menos de 0.5 Hz del error de lift (sobre todo, el arranque frente al tramo estable) el TCP
quedaría en 1.79 mm; sin la de 0.5–3 Hz (el rizado), en 2.23.

## 6. Qué devolver

La salida de las sesiones y sus registros (`~/.ros/air_bringup/session_*.yaml`), los CSV `smc_16??`
con sus `.params.yaml`, y lo observado. Para el paso 2, si se oye algo al bajar o durante el corte.

## 7. Lo que esto no prueba

- El corte: no hay hoja, ni fuerza, ni contacto. Antes de cortar, B2 tiene que medir la fuerza: con
  la fila 46 el evaluador se dispara entre 8 y 8.5 N (`07_gain_tuning.md`).
- El modo de 33 Hz con `shoulder_pan` deslizando durante el corte: en la incisión no se mueve.
- El error de TCP del artículo: el del evaluador es sin fricción (`09c` §8.3).
