# Protocolo — B7.1: el SMC en aire en el UR5e real, con las ganancias finales

**Estado:** preparado y ensayado en Gazebo (§5), sin ejecutar en el robot. Plan: B7.1, D28 y D29 de
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
- **Corrida:** `smc_1610`, con `--pan-r` según el paso 1. El brazo empieza en el `q_init` del nodo, y al
  acabar lo sostiene el JTC donde termine la retirada: la sesión no lo recoloca.
- **Parada:** los criterios de la rampa, más un error de TCP de más de 5 mm (la profundidad del corte)
  o la punta medida a menos de 30 mm del tablero.

## 4. Ejecución

Preparación: la de la rampa (`09c` §2, puntos 0 a 4). Para el paso 2, además, quitar del tablero la
fijación, la bandeja y las muestras, y comprobar que el acople no lleva la hoja.

```bash
source ~/ur5_ws/install/setup.bash
G=~/ur5_ws/src/ur5_utec/ur5_trajectory_optimization/results/gain_tuning/smc_v7_rotor/candidata_fila46.yaml

# Paso 1, sin mover nada y después la sesión:
ros2 run ur5_identification run_air_bringup_real.py --gains-file $G --paso base --dry-run
ros2 run ur5_identification run_air_bringup_real.py --gains-file $G --paso base --test-base 1600 --payload-kg 0.155

# Paso 2, solo tras revisar el paso 1 y con --pan-r decidido (0.5 o 0.71):
ros2 run ur5_identification run_air_bringup_real.py --gains-file $G --paso incision --pan-r <r> --dry-run
ros2 run ur5_identification run_air_bringup_real.py --gains-file $G --paso incision --pan-r <r> --test-base 1600 --payload-kg 0.155
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
| `smc_98964` | incisión en aire, `shoulder_pan` 0.50 | igual que `98963`: `shoulder_pan` no se mueve |

Solo prueba que la configuración, las tablas y los indicadores funcionan de principio a fin, y que el
nodo acepta las ganancias. **No prueba el modo de 33 Hz:** el gemelo no lo tiene, y por eso el paso 1
se hace en el robot. Tampoco el error de TCP del robot: el gemelo tiene 1 ciclo de retardo y no tiene
rotor.

## 6. Qué devolver

La salida de las sesiones y sus registros (`~/.ros/air_bringup/session_*.yaml`), los CSV `smc_16??`
con sus `.params.yaml`, y lo observado. Para el paso 2, si se oye algo al bajar o durante el corte.

## 7. Lo que esto no prueba

- El corte: no hay hoja, ni fuerza, ni contacto. Antes de cortar, B2 tiene que medir la fuerza: con
  la fila 46 el evaluador se dispara entre 8 y 8.5 N (`07_gain_tuning.md`).
- El modo de 33 Hz con `shoulder_pan` deslizando durante el corte: en la incisión no se mueve.
- El error de TCP del artículo: el del evaluador es sin fricción (`09c` §8.3).
