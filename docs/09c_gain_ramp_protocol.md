# Protocolo — rampa de ganancias del SMC en el UR5e real

**Estado:** en curso, sin ningún escalón completo. El comando de la primera corrida, lanzado a mano
el 2026-11-05 (`smc_1405`), acabó en paro de emergencia antes de ejecutar la ley
(`09_real_bringup.md` §5.1): **las corridas se lanzan solo con la sesión.** Con la sesión,
`smc_1505` (`wrist_3`, r = 0.50) paró por seguimiento, sin rastro de vibración, y `wrist_2` y
`wrist_3` salieron de los barridos (§1.1). El chequeo de
arranque del nodo suma el rotor medido y exige ×2 (plan D25); si rechaza un escalón, ese escalón no
se corre: la sesión nunca pasa `discrete_check.allow_unstable`. Plan: V16, D25 y D26 de
`Journal_UR5eControl_plan.md`. Código: `ur5_identification/gain_ramp.py` y
`scripts/run_gain_ramp_real.py`.

**Por qué.** La candidata (`results/gain_tuning/smc_v7_rotor/candidata_fila46.yaml`, fila 46 del
frente) supera lo probado limpio en cuatro juntas, y el evaluador con el que se eligió no tiene
modo flexible ni fricción. Se llega a ella por escalones, moviendo la junta: en `713`–`722`
`wrist_1` estaba quieta, y la fricción estática puede tapar una inestabilidad.

## 1. Qué hace

Cada escalón multiplica `λ` por `r` y divide `φ` por `r`, con `η` fija: `G` escala con `r` y `K_P`
con `r²`. Son 12 corridas: 4 escalones × 3 juntas barridas, una cada vez, con las demás sostenidas.

| `r` | `K_P` en la postura [N·m/rad] | `G` máx. en los barridos | margen (5 ciclos, con rotor) | por encima de lo probado |
|---|---|---|---|---|
| probado limpio | 163, 1987, 1704, 50, 19, 0.9 | | | |
| 0.50 | 132, 273, 282, 26, 5.6, 0.4 | 59.7 | ×7.64 | ninguna |
| 0.71 | 266, 549, 569, 52, 11.2, 0.7 | 84.7 | ×5.20 | `shoulder_pan`, `wrist_1` |
| 0.85 | 382, 788, 815, 75, 16.1, 1.1 | 101.4 | ×4.24 | `shoulder_pan`, `wrist_1`, `wrist_3` |
| 1.00 | 528, 1090, 1129, 104, 22.3, 1.5 | 119.3 | ×3.51 | `shoulder_pan`, `wrist_1`, `wrist_2`, `wrist_3` |

- **Juntas barridas, en este orden:** `wrist_1`, codo, `shoulder_pan`. El codo no supera lo probado,
  pero arrancó el ciclo de `smc_712` y domina el modo que limita el margen en el modelo rígido (unos
  19 Hz). `shoulder_lift` no se barre: está por debajo de lo probado, y es el barrido que más acerca
  la punta al tablero. `wrist_2` y `wrist_3` tampoco (§1.1).
- **Barrido:** ±0.20 rad a 0.05 y 0.20 rad/s, en los dos sentidos. Unos 37 s de tabla por corrida.
- **Postura:** q = (1.5708, −1.1947, 1.5748, −1.9509, −1.5708, 3.1416), punta en y = 0.62, z = 0.15.
  Tiene el alcance del trazo, donde la `G` de la base es alta (72, frente a 76 en la incisión), a
  150 mm sobre la cota del corte. En las posturas del trazo, barrer el codo baja la punta a menos
  de 50 mm del tablero. La punta no baja de 112 mm sobre él.
- **Guardas del nodo:** `watchdog_sat_frac_max` 0.10 y `watchdog_q_err_max` 0.30 rad.
  `tau_scale` 0.30. Vigilante de comandos activo.

### 1.1 Por qué `wrist_2` y `wrist_3` no se barren

En `smc_1505` (r = 0.50, barriendo `wrist_3`) saltó la guarda de seguimiento a los 28.4 s de `TRACK`,
con 0.30 rad de error en el barrido a +0.2 rad/s. No había rastro de vibración: ningún ciclo saturado,
rizado de par ≤ 0.02 % del tope en las seis juntas, y las otras cinco a ≤ 0.00013 rad. El control
volvió al JTC, con una deriva posterior de 0.00004 rad.

La ley no podía con esa junta. `K` es casi lo máximo que puede pedir, y no cambia entre escalones:

| junta | `K` máx. en su barrido (r = 1) [N·m] | `F_c` [N·m] | `K / F_c` | invertir la compensación a 0.2 rad/s [s] |
|---|---|---|---|---|
| `shoulder_pan` | 1.52 | 6.85 | 22 % | 0.004 |
| codo | 2.91 | 7.46 | 39 % | 0.009 |
| `wrist_1` | 0.92 | 1.78 | 52 % | 0.057 |
| `wrist_2` | 0.061 | 1.96 | 3 % | 0.21 |
| `wrist_3` | 0.0031 | 2.35 | 0.13 % | **5.9** |

En `wrist_3`, el par de la ley medido en el CSV coincide con el calculado (mediana 0.0030 N·m), y solo
la mueve la compensación de fricción, que sube como mucho 0.95 N·m/s: el límite de tasa usa la `M` del
URDF, sin rotor (3.8e-4 kg·m²). El barrido a 0.2 rad/s dura 2.6 s, menos de lo que tarda en invertirse.

Barrerlas probaría la compensación, no la ley, y con 60 y 3 mN·m la ley apenas puede excitarlas.
Siguen vigiladas, quietas, en todas las corridas, y la incisión no las mueve. **Lo que se pierde:** a
r = 1 superan lo probado (`K_P` de 22.3 frente a 19.2, y de 1.5 frente a 0.9) y eso solo se ve con la
junta quieta. **Para la incisión:** en el robot, a esas dos juntas las sujeta sobre todo la fricción,
no la ley, y el evaluador no tiene fricción: en esto no se parece al robot.

## 2. Preparación

Como `09a_session_protocol.md` §0 y §1:

0. **Ningún otro proyecto ROS 2 abierto en el dominio del robot** (`09_real_bringup.md` §8).
1. **Acople montado sin la hoja.** Mesa despejada: sin muestras ni fijación. Con `shoulder_pan`, la
   punta recorre unos ±13 cm en horizontal.
2. **Pendant:** payload **0.155 kg**, CoG **(−2.2, 2.1, 29.4) mm**, TCP en (0, 0, 0).
3. Velocidad reducida activa, paro de emergencia a mano, nadie dentro.
4. **Driver** con External Control en *play*, y **G4 a cero** (`09a` §1): el sostén del punto 5 no
   usa compensación, así que el nodo no lo hace por ti. Después, la sesión y el nodo la vuelven a
   poner a cero y esperan a que el driver lo acepte; ninguno vuelve a leer los valores.
5. **Seguridad de B7.0 en la misma sesión, y sostén con payload** (`09a` §2), con `test_num` nuevos:
   `1200`–`1204` ya existen.

## 3. Ejecución

```bash
source ~/ur5_ws/install/setup.bash
G=~/ur5_ws/src/ur5_utec/ur5_trajectory_optimization/results/gain_tuning/smc_v7_rotor/candidata_fila46.yaml

# Escalones y holguras, sin mover nada:
ros2 run ur5_identification run_gain_ramp_real.py --gains-file $G --test-base 1500 --dry-run

# La sesión:
ros2 run ur5_identification run_gain_ramp_real.py --gains-file $G --test-base 1500 --payload-kg 0.155
```

Pide «ACEPTO» y la masa tecleada al empezar, «SUBIR» en cada escalón y el nombre de la junta en cada
corrida. Lleva el robot a la postura con el JTC a 0.1 rad/s, y recoloca entre corridas. Números:
`smc_1500 + 10·escalón + junta` (`smc_1405`, el paro, y `smc_1505` no se borran). Calculo unos 15 min
en total, a partir de una sola corrida incompleta.

Se puede parar al acabar cualquier escalón. `--resume` continúa: salta los CSV con datos y aparta
los vacíos. Nunca sobrescribe uno con datos.

## 4. Cuándo parar

**Si oye o ve vibración, paro de emergencia, sin esperar al script.**

La sesión se para sola, y no sube, en cualquiera de estos casos:

| criterio | límite | referencia |
|---|---|---|
| guarda del nodo o corrida incompleta | `SAFE_HOLD`, o brazo sin detenerse | |
| rizado de τ sobre el tope, en cualquier junta | > 5 % | `718`, el limpio más alto: 2.34 %. Fallo de `712`: 69 % |
| ciclos saturados, en cualquier junta | > 10 % | limpias: 0. Fallo de `712`: 91 % |
| banda de 20–35 Hz en `shoulder_lift` y codo | ≥ 85 % de lo de `712` antes de irse (0.089 y 0.081 N·m) | limpias: hasta el 73 % |
| crecimiento de una banda entre escalones | > 2 veces lo esperado por ruido (∝ `r`), si supera 0.02 N·m | `713`–`717` no lo disparan. **Es una cota declarada, no medida** |

Si se para, no se repite el escalón ni se sube: revisar la corrida antes. Los criterios se han
pasado por `smc_713`–`717` (no paran) y por `smc_712` (para).

## 5. Lo ensayado sin robot

En el gemelo, a r = 0.50 y r = 1, con la fricción real en la planta (`smc_98931`, `98932`, `98941`–
`98944`: `wrist_1`, codo, `shoulder_pan`): las seis acaban sin disparar ninguna guarda, sin saturación
y con un error ≤ 0.0136 rad.

**El gemelo reproduce `smc_1505`:** barriendo `wrist_3` a r = 0.50 (`smc_98945`), la guarda salta con
−0.30 rad en la misma meseta, a los 28.7 s de `TRACK` (en el robot, a los 28.4 s). `wrist_3` no se
había ensayado en el gemelo antes de llevarla al robot: el ensayo tiene que cubrir **todas** las
juntas que se vayan a barrer.

Sin robot, en otro dominio y con la configuración del real, el nodo acepta las 12 corridas (margen
de ×3.52 a ×7.91, `G` hasta 118.9) y `ur5e_real.launch.py` le entrega el rotor y el ×2: con esos
parámetros, la ley de `smc_712` no arranca (×1.19).

Solo prueba que la configuración y las tablas funcionan de principio a fin. **No prueba la
estabilidad:** el gemelo tiene 1 ciclo de retardo, no tiene rotor ni modo flexible, y su `q̇` apenas
tiene ruido.

Con la compensación de fricción activa y **sin** fricción en la planta (`smc_98901`), `wrist_1` se
escapó y la guarda de error la cortó a los 0.66 s. Fue un error de configuración del ensayo, pero
muestra qué pasa si la compensación es mayor que la fricción real: `K` ≈ 0.9 N·m no basta para
frenarla. Con una fuerza externa ocurre lo mismo (`07_gain_tuning.md` §5.9).

Tras el paro de `smc_1405`, el mismo caso en el gemelo, sin fricción en la planta: en 1 s, las seis
juntas se desvían de 25° a 54° con el binario anterior, y 1.32° como mucho con el corregido
(`09_real_bringup.md` §5.1).

## 6. Qué devolver

La salida de `--dry-run` y de la sesión, el registro (`~/.ros/gain_ramp/session_*.yaml`), los CSV
`smc_15??` con sus `.params.yaml`, y lo observado en cada escalón.

## 7. Lo que esto no prueba

- La incisión: esa postura no es la del corte, las velocidades y aceleraciones no son las suyas, y
  no hay fuerza. Después viene B7.1, en aire, y antes de cortar, B2.
- El error de TCP: ninguna de estas corridas lo mide en el corte.
- Que `G ≤ 120` baste en la combinación de las tres juntas grandes: solo se prueba con una de ellas
  en movimiento cada vez.
- `wrist_2` y `wrist_3` en movimiento con estas ganancias (§1.1).
