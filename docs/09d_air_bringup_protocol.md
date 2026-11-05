# Protocolo — B7.1: el SMC en aire en el UR5e real, con las ganancias finales

**Estado:** paso 1 hecho y limpio (§5.1): el modo de 33 Hz depende de la `G`, y la incisión iba con
`shoulder_pan` en 0.50. **El paso 2 (`smc_1610`) vibró a unos 35 Hz y lo paró la guarda, y corrió con
las ganancias equivocadas** (§5.2). El error de configuración está corregido (D30), y la incisión se
repite por escalones, desde r = 0.50 (D31, §3). Ensayado en Gazebo, sin ejecutar en el robot. Plan: B7.1, D28 y D29 de
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
  | 0.50 | `smc_1620` | 132, 301, 270, 26, 5.6, 0.4 | 59.7 (38.1) | ×7.63 |
  | 0.71 | `smc_1621` | 132, 607, 544, 52, 11.2, 0.7 | 84.8 (38.1) | ×5.23 |
  | 0.85 | `smc_1622` | 132, 869, 779, 75, 16.1, 1.1 | 101.5 (38.1) | ×4.28 |
  | 1.00 | `smc_1623` | 132, 1203, 1078, 104, 22.3, 1.5 | 119.4 (38.1) | ×3.55 |

- **Una corrida por sesión,** con `--r`, y cada escalón exige el CSV del anterior. Entre escalones,
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

## 6. Qué devolver

La salida de las sesiones y sus registros (`~/.ros/air_bringup/session_*.yaml`), los CSV `smc_16??`
con sus `.params.yaml`, y lo observado. Para el paso 2, si se oye algo al bajar o durante el corte.

## 7. Lo que esto no prueba

- El corte: no hay hoja, ni fuerza, ni contacto. Antes de cortar, B2 tiene que medir la fuerza: con
  la fila 46 el evaluador se dispara entre 8 y 8.5 N (`07_gain_tuning.md`).
- El modo de 33 Hz con `shoulder_pan` deslizando durante el corte: en la incisión no se mueve.
- El error de TCP del artículo: el del evaluador es sin fricción (`09c` §8.3).
