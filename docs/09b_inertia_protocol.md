# Protocolo — inercia efectiva por junta en el UR5e real (B4.6)

**Estado:** preparado, sin ejecutar. Plan: V16, D25 y B4.6 de
`Journal_UR5eControl_plan.md`. Método: `ur5_identification/inertia_id.py`.

**Por qué.** El URDF no incluye la inercia de los rotores y `g7` se calcula con él. Con
esa planta la re-sintonía del SMC no tiene solución (V16). Los barridos de fricción
apuntan a unos 0.19 kg·m² añadidos en cada muñeca, pero no se hicieron para medir esto.

**Qué es.** Control de **posición**, como B1: una junta cada vez sigue un seno pequeño y
se graba la corriente. No hay ley de par en el lazo. Es movimiento lento y de pocos
grados, pero **es la primera vez que se pide esta aceleración a las muñecas**.

| juntas | amplitud | frecuencias | aceleración máxima |
|---|---|---|---|
| hombro y codo | 0.6–1.8° | 1 a 3 Hz | 3.8 rad/s² |
| muñecas | 0.9–3.6° | 1 a 4 Hz | 10 rad/s² |

## 1. Preparación

1. **Acople montado sin la hoja**, como en B4.4. Masa **0.155 kg**.
2. **Mesa despejada**, sin muestras ni fijación. La punta baja hasta 60 mm sobre el
   corte; el script comprueba que queda lejos del tablero.
3. **Pendant:** TCP en (0, 0, 0), slider al 100 %. El payload no influye (por posición, la
   corriente es el par total); se puede dejar en 0.155 kg.
4. Paro de emergencia a mano, nadie dentro.
5. **Driver** como en `09a_session_protocol.md` §1, con External Control en *play*. Las
   escalas de fricción (G4) no hacen falta: no se manda par. **No lanzar `ur5e_real.launch.py`.**
6. Robot en `q_init` o cerca: el script lo lleva a cada postura a 0.1 rad/s.

## 2. Ejecución

**2.1 Comprobar el plan, sin mover nada:**

```bash
source ~/ur5_ws/install/setup.bash
ros2 run ur5_identification run_inertia_id.py --solo-plan --masa-kg 0.155
```

**2.2 Primera prueba, solo `wrist_1`, unos 30 s:**

```bash
ros2 run ur5_identification run_inertia_id.py --test-num 1300 --masa-kg 0.155 \
    --juntas 4 --posturas inicio --frecuencias 1 2 3
```

Pide Enter antes de moverse. Al acabar imprime una tabla. **Enviarme esa salida y esperar**:
quiero ver la señal real antes de mover las seis juntas.

**2.3 Corrida completa, unos 8 min** (40 segmentos, dos posturas):

```bash
ros2 run ur5_identification run_inertia_id.py --test-num 1301 --masa-kg 0.155
```

**2.4 Repetición, unos 4.5 min** (solo la postura `inicio`, para ver la dispersión):

```bash
ros2 run ur5_identification run_inertia_id.py --test-num 1302 --masa-kg 0.155 --posturas inicio
```

**2.5 Análisis conjunto, sin robot:**

```bash
cd ~/.ros/ur5_dyn_control
ros2 run ur5_identification run_inertia_id.py --analizar iner_1301.csv iner_1302.csv \
    --yaml iner_B46.yaml
```

## 3. Cuándo parar

**Paro de emergencia o pausa** ante cualquier ruido o vibración que no sea el vaivén
suave previsto, o si algo se acerca a la mesa o al cableado.

El script se detiene solo y conserva lo grabado si External Control deja de estar en *play*,
si hay un paro de protección o si la junta no sigue el seno (menos del 70 % o más del 130 %
de la amplitud). En ese caso, **no relanzar con el mismo número** (se niega a sobrescribir):
enviarme lo que salió por consola y lo que mostró el pendant.

## 4. Cómo se lee la tabla

| columna | significado |
|---|---|
| `M med` y `+-` | inercia total medida [kg·m²] y su error de ajuste |
| `M modelo` | la del URDF con el acople |
| `rotor` | la diferencia: lo que el URDF no trae |
| `+-2 ms` | cuánto cambiaría `M` con un desfase de 2 ms entre corriente y posición |
| `s` | rigidez que no depende de la frecuencia [N·m/rad]: gravedad y fricción antes de deslizar |
| `resid` | residuo del ajuste [N·m] |

## 5. Qué se decide con el resultado

Fijado **antes** de ver los datos, para no acomodarlo después. La referencia es `wrist_1`,
la junta que limita:

| rotor de `wrist_1` | lectura | siguiente paso |
|---|---|---|
| ≥ 0.10 kg·m² | confirma los barridos | meter la inercia medida en la planta y en `g7`, y re-sintonizar |
| 0.05–0.10 | efecto real pero menor | re-sintonizar igual; puede que no baste |
| < 0.05 | el URDF no estaba tan lejos | volver a las opciones de V16: el conflicto sería real |

**No se da por buena una junta** si la tabla la marca (`POCA SENAL`, `NO USABLE`, `LAS
POSTURAS NO COINCIDEN`). Comprobaciones que espero:

- Las tres muñecas, con motores iguales, deberían dar valores parecidos.
- El rotor de `shoulder_pan` no debe cambiar entre posturas, aunque su `M` del URDF se
  duplique. Es la prueba del método en una junta donde el modelo sí varía.
- Si las muñecas salen muy distintas o `shoulder_pan` no coincide, lo revisamos antes de
  usar ningún número.

## 6. Qué no prueba esta medida

Es una medida de **1 a 4 Hz, en régimen de brazo rígido**. No dice nada del modo flexible
de 25–35 Hz, que fue el del ciclo límite de `smc_712`. `G ≤ 120` sigue vigente, y unas
ganancias nuevas de `wrist_1` irían por encima de lo probado limpio (unos 50 N·m/rad): antes
de cualquier incisión hará falta una rampa por escalones en esa junta, con su propio
protocolo.

## 7. Qué devolver

La salida de la consola de 2.2 a 2.5 y lo observado en el pendant. Los datos quedan en
`~/.ros/ur5_dyn_control/iner_*.csv` e `iner_*.yaml`.
