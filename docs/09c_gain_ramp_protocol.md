# Protocolo — rampa de ganancias del SMC en el UR5e real

**Estado:** preparado, sin ejecutar. **Hoy no se puede ejecutar:** el chequeo de arranque del
nodo usa la M del URDF, con la que todos los escalones dan margen 0, así que el nodo se niega a
arrancar. Cambiarlo está pendiente de una decisión aparte (plan D25). La sesión nunca pasa
`discrete_check.allow_unstable`. Plan: V16 y D25 de `Journal_UR5eControl_plan.md`. Código:
`ur5_identification/gain_ramp.py` y `scripts/run_gain_ramp_real.py`.

**Por qué.** La candidata (`results/gain_tuning/smc_v7_rotor/candidata_fila46.yaml`, fila 46 del
frente) supera lo probado limpio en cuatro juntas, y el evaluador con el que se eligió no tiene
modo flexible ni fricción. Se llega a ella por escalones, moviendo la junta: en `713`–`722`
`wrist_1` estaba quieta, y la fricción estática puede tapar una inestabilidad.

## 1. Qué hace

Cada escalón multiplica `λ` por `r` y divide `φ` por `r`, con `η` fija: `G` escala con `r` y `K_P`
con `r²`. Son 20 corridas: 4 escalones × 5 juntas barridas, una cada vez, con las demás sostenidas.

| `r` | `K_P` en la postura [N·m/rad] | `G` máx. en los barridos | margen (5 ciclos, con rotor) | por encima de lo probado |
|---|---|---|---|---|
| probado limpio | 163, 1987, 1704, 50, 19, 0.9 | | | |
| 0.50 | 132, 273, 282, 26, 5.6, 0.4 | 59.7 | ×7.64 | ninguna |
| 0.71 | 266, 549, 569, 52, 11.2, 0.7 | 84.7 | ×5.20 | `shoulder_pan`, `wrist_1` |
| 0.85 | 382, 788, 815, 75, 16.1, 1.1 | 101.4 | ×4.24 | `shoulder_pan`, `wrist_1`, `wrist_3` |
| 1.00 | 528, 1090, 1129, 104, 22.3, 1.5 | 119.3 | ×3.51 | `shoulder_pan`, `wrist_1`, `wrist_2`, `wrist_3` |

- **Juntas barridas, en este orden:** `wrist_3`, `wrist_2`, `wrist_1`, codo, `shoulder_pan`. El codo
  no supera lo probado, pero arrancó el ciclo de `smc_712` y domina el modo que limita el margen en
  el modelo rígido (unos 19 Hz). `shoulder_lift` no se barre: está por debajo de lo probado, y es
  el barrido que más acerca la punta al tablero.
- **Barrido:** ±0.20 rad a 0.05 y 0.20 rad/s, en los dos sentidos. Unos 37 s de tabla por corrida.
- **Postura:** q = (1.5708, −1.1947, 1.5748, −1.9509, −1.5708, 3.1416), punta en y = 0.62, z = 0.15.
  Tiene el alcance del trazo, donde la `G` de la base es alta (72, frente a 76 en la incisión), a
  150 mm sobre la cota del corte. En las posturas del trazo, barrer el codo baja la punta a menos
  de 50 mm del tablero. La punta no baja de 112 mm sobre él.
- **Guardas del nodo:** `watchdog_sat_frac_max` 0.10 y `watchdog_q_err_max` 0.30 rad.
  `tau_scale` 0.30. Vigilante de comandos activo.

## 2. Preparación

Como `09a_session_protocol.md` §0 y §1:

1. **Acople montado sin la hoja.** Mesa despejada: sin muestras ni fijación. Con `shoulder_pan`, la
   punta recorre unos ±13 cm en horizontal.
2. **Pendant:** payload **0.155 kg**, CoG **(−2.2, 2.1, 29.4) mm**, TCP en (0, 0, 0).
3. Velocidad reducida activa, paro de emergencia a mano, nadie dentro.
4. **Driver** con External Control en *play*. La sesión fija G4 a cero y lo verifica: con G4 activa
   el robot suma su fricción a la del nodo.
5. **Seguridad de B7.0 en la misma sesión, y sostén con payload** (`09a` §2.1).

## 3. Ejecución

```bash
source ~/ur5_ws/install/setup.bash
G=~/ur5_ws/src/ur5_utec/ur5_trajectory_optimization/results/gain_tuning/smc_v7_rotor/candidata_fila46.yaml

# Escalones, holguras y comandos, sin mover nada:
ros2 run ur5_identification run_gain_ramp_real.py --gains-file $G --test-base 1400 --dry-run

# La sesión:
ros2 run ur5_identification run_gain_ramp_real.py --gains-file $G --test-base 1400 --payload-kg 0.155
```

Pide «ACEPTO» y la masa tecleada al empezar, «SUBIR» en cada escalón y el nombre de la junta en cada
corrida. Lleva el robot a la postura con el JTC a 0.1 rad/s, y recoloca entre corridas. Números:
`smc_1400 + 10·escalón + junta`. Calculo unos 30 min en total, sin haberlo medido en el robot.

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

En el gemelo, a r = 1, con la fricción real en la planta (`smc_98911`–`98913`: `wrist_1`, codo,
`shoulder_pan`): las tres acaban sin disparar ninguna guarda, sin saturación, con `G` de hasta 118.9
y un rizado de hasta el 0.9 % del tope al 30 %.

Solo prueba que la configuración y las tablas funcionan de principio a fin. **No prueba la
estabilidad:** el gemelo tiene 1 ciclo de retardo, no tiene rotor ni modo flexible, y su `q̇` apenas
tiene ruido.

Con la compensación de fricción activa y **sin** fricción en la planta (`smc_98901`), `wrist_1` se
escapó y la guarda de error la cortó a los 0.66 s. Fue un error de configuración del ensayo, pero
muestra qué pasa si la compensación es mayor que la fricción real: `K` ≈ 0.9 N·m no basta para
frenarla. Con una fuerza externa ocurre lo mismo (`07_gain_tuning.md` §5.9).

## 6. Qué devolver

La salida de `--dry-run` y de la sesión, el registro (`~/.ros/gain_ramp/session_*.yaml`), los CSV
`smc_14??` con sus `.params.yaml`, y lo observado en cada escalón.

## 7. Lo que esto no prueba

- La incisión: esa postura no es la del corte, las velocidades y aceleraciones no son las suyas, y
  no hay fuerza. Después viene B7.1, en aire, y antes de cortar, B2.
- El error de TCP: ninguna de estas corridas lo mide en el corte.
- Que `G ≤ 120` baste en la combinación de las tres juntas grandes: solo se prueba con una de ellas
  en movimiento cada vez.
