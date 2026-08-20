# FASE 2 (real) — Campaña de fricción sobre el UR5e físico

Contraparte en hardware de [`02_friction.md`](02_friction.md), que cubre la
campaña en Gazebo. Los dos documentos NO son intercambiables: allí la fricción
se inyectaba conocida para validar el estimador, aquí se mide la del robot.

- **Fecha:** 2026-08-03
- **Prerrequisitos:** [FASE 0](00_prereqs.md), G4 y G5 en particular
- **Robot:** UR5e, driver 2.13.2, **sin herramienta montada** (A1 vigente)
- **Datos:** `~/.ros/ur5_dyn_control/{cur,fl}_9??.csv` · sesiones en `~/.ros/friction_campaign/`

---

## 1. Por qué esta campaña no pudo hacerse por control de par

La FASE 2 original comanda par y despeja la fricción del residuo. En el robot
real eso **no funciona en las muñecas**, y el fallo no es de sintonía:

- `wrist_1` se queda clavada: bajo linealización por realimentación la autoridad
  contra una perturbación de par es `M_jj · kp_j`, y con `M_44 = 0.023 kg m²` no
  hay `kp` razonable que produzca los ~1.85 N·m de Coulomb que hemos acabado
  midiendo.
- `wrist_2` satura `tau_max` (8.4 N·m con `tau_scale` 0.3) antes de romper a
  moverse.
- `wrist_3` necesitaría `kp ≈ 155 000`, que da `ω_n·dt = 0.79`: inestable.

Corridas descartadas por esto: `fl_903`, `fl_913`, `fl_914`.

### 1.1 El método que sí funciona

Barrido por **control de posición** (el `scaled_joint_trajectory_controller`
mueve la junta, las demás sostenidas) leyendo **corriente de motor**, y
conversión posterior a par.

Es imprescindible recordar G5: `/joint_states.effort` en el UR5e **no es par**,
es el campo RTDE `actual_current` (verificado en `hardware_interface.cpp:800`).
UR no publica las constantes `k_t·N` que lo convertirían. Por eso los CSV de
barrido usan columnas `cur1..cur6` y nunca `tau*`.

La constante sale de los propios datos. En la meseta, misma postura y misma
rapidez en los dos sentidos:

```
i(+v)·k = g(q) + C(q,+v)(+v) + f_v·v + f_c
i(−v)·k = g(q) + C(q,−v)(−v) − f_v·v − f_c
```

Gravedad e inercia son **pares** en la velocidad (Coriolis es cuadrático), así
que:

```
SUMA  ->  k·[i(+v) + i(−v)]/2 = g(q) + C·v     <- lo conoce el modelo  ->  da k
DIF   ->  k·[i(+v) − i(−v)]/2 = f_v·v + f_c    <- es la fricción
```

Además de resolver las muñecas, salió **más preciso** que la vía de par:
`shoulder_lift` dio F_v = 12.30 ± 0.61 frente a 12.47 ± 1.75, con R² 0.9965
frente a 0.9693.

### 1.2 Las juntas donde `k` no es identificable

La suma entre sentidos vale `g(q) + C·v`. Si la gravedad no carga esa junta, la
suma es cero y el ajuste degenera en 0/0. Medido como **media con signo** del par
gravitatorio sobre el rango barrido, que es lo que el ajuste ve —compara medias
de meseta, y las mesetas a `+v` y `−v` recorren el mismo tramo:

| junta | \|g\| medio [N·m] | ¿`k` identificable? |
|---|---|---|
| shoulder_pan | 0.00000 | **NO** — eje vertical |
| shoulder_lift | 18.20797 | sí |
| elbow | 18.20810 | sí |
| wrist_1 | 1.63993 | sí |
| wrist_2 | 0.00000 | **NO** — cruce por cero |
| wrist_3 | 0.00000 | **NO** — eje de la herramienta |

Tres casos distintos, y conviene no confundirlos:

- **`shoulder_pan`** no tiene arreglo por postura: su eje es vertical, luego el
  par gravitatorio respecto a él es nulo en *toda* configuración.
- **`wrist_2`** arranca en −90°, justo el cruce por cero de su par gravitatorio,
  y barre entre valores opuestos. Su RMS sobre el rango es 0.240 N·m —aparenta
  carga de sobra— pero la **media se cancela**. Distinguir media de RMS decide
  este caso; la multipostura sí lo resuelve.
- **`wrist_3`** gira sobre el eje de la herramienta con la masa centrada en él:
  nulo siempre, y la multipostura tampoco puede rescatarlo.

El runner lo comprueba **antes** de pedir la confirmación §7, para no descubrirlo
tras cinco minutos de barrido.

---

## 2. Protocolo ejecutado

18 corridas = 6 juntas × 3 niveles de compensación interna (G4), 282 s cada una.

| | |
|---|---|
| Niveles de \|q̇\| | 0.02, 0.05, 0.10, 0.20, 0.35, 0.50, 0.75, 1.00 rad/s |
| Amplitud | ±45° desde `q_init` |
| `q_init` | `[1.5708, −1.5708, 1.5708, −1.5708, −1.5708, 3.1416]` |
| Niveles G4 | `0.0` · `default` (0.9/0.8) · `1.0` |
| Numeración | `950 + 10·índice_de_nivel + junta` |

```bash
ros2 run ur5_identification run_friction_campaign_real.py --test-base 950 \
    --home-between-runs --k 3:9.7582 4:11.6792
```

Salvaguardas, todas motivadas por incidentes reales de esta fase:
confirmación tecleando el nombre de la junta, preflight de `q_init` que aborta
si alguna junta se desvía, `--home-between-runs`, guarda de rango en el
generador de barrido, y publicación de par cero en el destructor del nodo.

---

## 3. Resultados

### 3.1 Fricción física (lado motor)

Media de los tres niveles. Modelo `viscous_coulomb`, R² entre 0.9960 y 0.9994.

| junta | F_v [N·m·s/rad] | F_c [N·m] | `k` [N·m/A] | origen de `k` |
|---|---|---|---|---|
| shoulder_pan | 14.55 | 7.31 | 11.88 | cruce de métodos |
| shoulder_lift | 12.30 | 7.20 | 11.85 | ajuste propio |
| elbow | 14.53 | 7.89 | 11.63 | ajuste propio |
| wrist_1 | 1.33 | 1.85 | 9.00 | ajuste propio |
| wrist_2 | 1.85 | 2.68 | 11.68 | multipostura |
| wrist_3 | 0.245 A/(rad/s) | 0.272 A | — | **no identificable** |

### 3.2 Repetibilidad — el resultado más sólido

Dispersión entre las tres campañas independientes:

| junta | disp. F_v | disp. F_c |
|---|---|---|
| shoulder_pan | 8.1 % | 2.3 % |
| shoulder_lift | 5.0 % | **0.6 %** |
| elbow | 4.2 % | 4.4 % |
| wrist_1 | 5.2 % | **0.7 %** |
| wrist_2 | 5.1 % | **0.5 %** |
| wrist_3 | 4.4 % | **0.9 %** |

El término de Coulomb se reproduce por debajo del 1 % en cuatro de las seis
juntas. El viscoso ronda el 5 %, consistente con su mayor sensibilidad al ruido
de las mesetas lentas.

### 3.3 `k` de `shoulder_pan` por cruce de métodos

`shoulder_pan` no puede ajustar `k` de sus datos, pero sí se mueve bajo control
de par, así que admite las dos vías. Al **nivel 0.0**, donde ambas miden lo mismo
(§4), el cociente entre ellas es `k`:

| término | vía de par (`fl_900`) | vía de corriente (`cur_950`) | `k` |
|---|---|---|---|
| viscoso | 14.5465 N·m·s/rad | 1.18725 A/(rad/s) | 12.252 |
| Coulomb | 7.1462 N·m | 0.62085 A | 11.510 |

**Discrepancia 6.4 %.** Que dos términos independientes converjan al mismo
escalar es la validación no circular del método completo, y sitúa a
`shoulder_pan` en la misma familia que `shoulder_lift`, `elbow` y `wrist_2`
(11.6–11.9 N·m/A).

> Una vez adoptada la `k` media (11.881), el acuerdo entre F_v y F_c medidos por
> las dos vías es **por construcción** y no prueba nada. Lo que valida es la
> convergencia de los dos cocientes ANTES de promediar.

---

## 4. La compensación interna no altera lo que mide la corriente

Los tres niveles de G4 dan el mismo número por la vía de corriente. No es que el
servicio fallara: la vía de par distingue los niveles con toda claridad usando el
mismo servicio en la misma sesión. **Miden magnitudes distintas:**

- La **corriente de motor** es el par total que la máquina entrega. Lo fija la
  física, no cómo el controlador reparta entre feedforward y realimentación.
  Es invariante, y por eso mide la fricción *real*.
- El **control por par** mide lo que *nuestro* mando debe aportar: la fricción
  real **menos** lo que el robot ya pone por su cuenta.

Medido al nivel `default`:

| junta | F_c física | F_c residual | compensado | F_v física | F_v residual | compensado |
|---|---|---|---|---|---|---|
| shoulder_pan | 7.31 | 1.08 | 85 % | 14.55 | 9.00 | **38 %** |
| shoulder_lift | 7.20 | 0.40 | 94 % | 12.30 | 7.47 | **39 %** |
| elbow | 7.89 | 0.31 | 96 % | 14.53 | 0.37 | 97 % |

Dos consecuencias:

1. **El modelo interno del driver compensa bien el Coulomb (85–96 %) y mal el
   viscoso en hombro y elevación (38–39 %).** El codo lo hace al 97 % pese a
   tener las escalas más BAJAS (0.8/0.7 frente a 0.9/0.8): las escalas no
   predicen la calidad de la compensación.
2. **Cada número tiene su uso.** La fricción física va al modelo de Gazebo, para
   que la simulación sea predictiva. El **residual** es lo que el controlador debe
   rechazar, y es lo que dimensiona `eta`. Mezclarlos sobrecompensaría justo en
   la cantidad que el robot ya aporta.

No hay dato de residual para las muñecas: el control por par nunca consiguió
moverlas (§1).

---

## 5. El modelo de Stribeck se descarta

Ajusta mejor en R² (0.9999 frente a 0.9965) y aun así hay que rechazarlo.

**Los parámetros no son físicos.** `F_s − F_c` sale **negativo en 11 de las 12
corridas** (−9.85, −10.34, −10.75 en `shoulder_lift`): dice que la fricción
estática es menor que la de Coulomb, lo cual es imposible. El término está
absorbiendo curvatura de signo contrario al de Stribeck.

**Y no son reproducibles.** En el codo, sobre los mismos datos:

| modelo | nivel 0.0 | nivel default | nivel 1.0 |
|---|---|---|---|
| `viscous_coulomb` F_v | 14.83 | 14.22 | 14.53 |
| `stribeck` F_v | 4.81 | 10.82 | 7.66 |

El modelo de dos parámetros no se mueve; el de tres oscila el doble.

**La causa es de diseño experimental:** el barrido arranca en 0.02 rad/s y el
régimen de Stribeck vive muy por debajo. Los datos no lo contienen, así que el
tercer parámetro sólo puede ajustar ruido.

> **Limitación declarada para el paper.** Esta campaña no cubre stiction ni
> Stribeck. Es la región que domina en los cruces por cero de velocidad y en el
> arranque y el final de la incisión, precisamente donde cabe esperar los picos
> de error de seguimiento.

---

## 6. Anomalía abierta: la `k` de `wrist_1`

`wrist_1` ajusta `k = 9.00` de forma muy estable (9.029 / 8.980 / 9.005 en los
tres niveles, residuo relativo 1.5 %), un **23 % por debajo de `wrist_2`
(11.68)**. Comparten motor y reductora, así que esa diferencia no debería
existir.

Como `k = par_modelo / corriente`, una `k` baja significa que el robot gasta más
corriente de la que el URDF predice: **el modelo estaría subestimando en torno a
un 25 % la masa que cuelga de `wrist_1`**. La hipótesis más simple es que haya
algo atornillado en la brida que el URDF no incluye —la placa del acople del
bisturí, aunque el bisturí no esté montado.

Pendiente: **comprobación física de la brida.** Si aparece masa no modelada,
afecta también a la compensación de gravedad de los cuatro controladores, no
sólo a esta campaña.

> **Ya no hace falta esperar a esa comprobación para zanjarlo.** El método de
> gravedad no puede distinguir una `k` distinta de un error de masas del URDF,
> porque un error de masas se va ENTERO a `k`. La vía `k = tau_phys/cur` (§8.1)
> no usa el URDF, así que separa las dos hipótesis: si da 11.7, el modelo
> subestima la masa distal; si da 9.0, la `k` de `wrist_1` es realmente distinta.

Mientras tanto se adopta la `k` propia de `wrist_1`: con la de multipostura
(9.7582) el residuo relativo era del 8.5 %, con la suya baja a 1.5 %.

---

## 7. Implicaciones para el SMC (FASE 5)

Contra `eta: [1.058, 2.591, 0.881, 0.0232, 0.00535, 0.00026]`, que está escalada
por inercia (`η_j = I_jj · a_reach`, `a_reach = 1 rad/s²`):

| junta | η [N·m] | residual F_c | ¿cubre? |
|---|---|---|---|
| shoulder_pan | 1.058 | 1.08 | ⚠️ al límite |
| shoulder_lift | 2.591 | 0.40 | ✅ |
| elbow | 0.881 | 0.31 | ✅ |
| wrist_1 | 0.0232 | sin dato (física 1.85) | ❌ |
| wrist_2 | 0.00535 | sin dato (física 2.68) | ❌ |
| wrist_3 | 0.00026 | sin dato (física ~3.18 †) | ❌ |

† `wrist_3` se convierte con `k = 11.7`, el valor de familia, porque el suyo no
es identificable (§1.2). Es una **hipótesis declarada, no una medida**: con
`k = 9.0` —la de `wrist_1`— saldría 2.44 N·m. Todos sus números escalan
linealmente con `k`.

**El escalado por inercia es correcto para la estabilidad discreta y equivocado
para el rechazo de fricción.** La fricción la pone la reductora y es del mismo
orden en las tres juntas grandes (7.2–7.9 N·m de Coulomb) mientras la inercia
entre ellas ya varía 3×, y entre hombro y `wrist_3` varía 4000×. En Gazebo no se
notaba porque allí la fricción es ~0.

Con `χ = (K/φ)·dt/M ≤ 0.8`, `φ = 0.05` y `dt = 2 ms`, subir `K` hasta cubrir la
fricción física da:

| junta | χ resultante |
|---|---|
| shoulder_pan | 0.28 ✅ |
| shoulder_lift | 0.11 ✅ |
| elbow | 0.36 ✅ |
| wrist_1 | 3.2 ❌ |
| wrist_2 | 20 ❌ |
| wrist_3 | **489** ❌ |

Las tres juntas grandes pueden absorber su fricción en `K` sin romper nada. Las
muñecas no. Aun suponiéndoles la mejor compensación observada (96 %, la del
codo), a `wrist_3` le quedaría ~0.13 N·m de residual contra una η de 0.00026
—500× corta— y χ seguiría en 20 frente al límite de 0.8.

**Conclusión: en las muñecas el feedforward de fricción no es una mejora de
precisión, es la condición para que el SMC sea implementable.** Con la fricción
en el modelo, `K` sólo tiene que cubrir el error de identificación (≈5 %) en vez
de la fricción entera.

---

## 8. Validación de la inyección en Gazebo

Antes de optimizar ganancias contra una planta con fricción hay que comprobar
que la planta tiene la fricción que uno cree. Se inyectaron los seis pares de
valores del §3.1 y se repitió el barrido por control de par sobre
`shoulder_lift`, en `empty_test_world.sdf`:

```bash
ros2 launch ur5_dyn_control fl_control.launch.py gazebo_gui:=false \
    params_file:=<share>/config/sweep_params.yaml \
    world:=<share>/worlds/empty_test_world.sdf \
    test_num:=300 sweep_joint:=1 \
    joint_damping:="14.55 12.30 14.53 1.33 1.85 2.87" \
    joint_friction:="7.31 7.20 7.89 1.85 2.68 3.18"
```

| | inyectado | recuperado | error |
|---|---|---|---|
| F_v | 12.30 | 12.2020 ± 0.0032 | **−0.80 %** |
| F_c | 7.20 | 7.2316 ± 0.0016 | **+0.44 %** |

R² = 1.0000, RMSE 0.0026 N·m por diferenciación; el ajuste RNEA coincide con
ella en 0.003 N·m·s/rad, lo que dice que el URDF describe bien esta postura.

El déficit de F_v es en su mayor parte el **artefacto ya documentado** en
`02_friction.md` §«control negativo»: en una planta SIN fricción, `shoulder_lift`
(dg/dq = −36.7) arroja un F_v aparente de −0.0401 por el desfase de ~1 ms entre
el par publicado y el estado leído. Corrigiéndolo, el error baja a **−0.47 %**.

**Lo que esto NO valida: las muñecas.** Con 1.85–3.18 N·m de Coulomb inyectados y
`kp` de 100 en `wrist_3`, la autoridad `M_jj·kp_j` vale 0.026 N·m/rad — el
barrido por par no puede moverla, exactamente igual que en el robot físico
(§1). Que Gazebo reproduzca ahora ese fallo es en sí un resultado: la
simulación pasa a predecir la limitación real en vez de ocultarla. Pero implica
que la inyección en las muñecas queda sin comprobar por esta vía.

**Trampa operativa.** El primer intento falló con `The plugin failed to load...
ur_robot_driver/URPositionHardwareInterface ... does not exist`. La causa no
estaba en el parcheo del URDF: el **driver del robot real seguía vivo** y su
`robot_state_publisher` publicaba la descripción del robot físico, que
`gz_ros2_control` tomó en lugar de la de Gazebo. Hay que parar
`ur_control.launch.py` antes de lanzar Gazebo. Y ojo con comprobarlo mediante
`pgrep -f "ruby.*ign gazebo"` dentro del propio comando: el patrón se encuentra a
sí mismo en la línea de órdenes y da un falso positivo.

---

### 8.1 `k` sin gravedad y sin URDF: `k = tau_phys / cur`

El método de §1.1 saca `k` de la SUMA entre sentidos, que vale `g(q) + C·v`, así
que **exige que la gravedad cargue la junta**. En `wrist_3` eso no pasa en
ninguna postura (§1.2) y no hay multipostura que lo arregle.

Bajo control de par hay otra vía, y no depende de la gravedad. Con G4 = 0.0 y
`gravity_in_command:=false`, el par físico de la junta es el comandado más la
gravedad que pone el robot — que es lo que ya registra la columna `tau_phys`—, y
`cur` trae la corriente de la misma fila:

```
k_i = media(tau_phys_i) / media(cur_i)      [N·m/A]
```

Sin modelo, sin URDF y sin gravedad. Vale para las seis juntas.

**Lo que lo desbloqueó** fue el feedforward alimentado por la velocidad deseada
(`docs/05_smc.md` §7.3): el control de par no podía mover las muñecas, y ahora
sí. Y un detalle que lo hace utilizable: **la `k` supuesta para calcular ese
feedforward NO sesga el resultado.** Solo sirve para que la junta arranque; la
medida usa el par realmente comandado y la corriente realmente leída.

**Se RESTA entre sentidos**, y no es cosmético:

```
tau_phys = tau_cmd + g_NUESTRO(q)     <- lo que registra la columna
par real = tau_cmd + g_UR(q)          <- lo que el robot entrega
```

El nodo calcula `tau_phys` con su modelo (G3) mientras el robot compensa la
gravedad con el suyo, que no publican. El cociente crudo saldría sesgado por el
cociente de las dos gravedades. La gravedad es PAR en la velocidad, así que
restando los dos sentidos se cancela arriba y abajo a la vez, y con ella toda
dependencia del modelo: el numerador pasa a ser par que **nosotros comandamos**.

> **NO se puede ensayar en Gazebo.** Lo intenté y la validación que primero di
> por buena era falsa. En la corrida de incisión `k` salió 0.9886 ≈ 1.0, pero eso
> solo demostraba que ambas columnas ven la misma GRAVEDAD (−44.45 y −44.96):
> las dos estaban dominadas por ella. Al mirar la diferencia entre sentidos, que
> es la parte que importa, aparece que **el `effort` de Gazebo no incluye la
> fuerza de fricción**, que el motor de física resuelve como restricción aparte:
>
> ```
> SWEEP_1.000_POS   tau_phys  -0.6121   cur  -20.0747
> SWEEP_1.000_NEG   tau_phys -39.5396   cur  -20.0719
> ```
>
> `tau_phys` cambia 19.5 N·m —exactamente la fricción inyectada— y `cur` no se
> mueve. En el robot real `cur` es corriente de motor y el motor SÍ tiene que
> producir ese par, así que allí funciona; pero **la primera medida real es
> también su primera prueba**. Por eso se valida sobre una junta cuya `k` ya se
> conoce por otra vía antes de usarlo en `wrist_3`.

### 8.2 Validado en el robot real, y discrepa un 6.1 % con la gravedad

Medido sobre `shoulder_lift`, cuya `k` por gravedad es 11.850:

| nivel | dif τ [N·m] | dif i [A] | k |
|---|---|---|---|
| 0.020 | 6.4222 | 0.5768 | 11.1346 |
| 0.100 | 8.4915 | 0.7638 | 11.1172 |
| 0.350 | 13.1951 | 1.1875 | 11.1120 |
| 1.000 | 19.9004 | 1.7864 | 11.1399 |

**k = 11.1240, dispersión 0.10 % sobre 50× de rango de velocidad.** Que `k` no
derive con la velocidad era el criterio de aceptación: confirma que
`direct_torque` entrega el par comandado, que es la suposición de fondo.

Y hay una validación no circular mejor que la de §3.3: ajustando `dif τ` y
`dif i` por separado contra la velocidad, los términos **viscoso y de Coulomb**
—independientes entre sí— dan `k` = 11.141 y 11.112, que **coinciden al 0.26 %**
(en §3.3 la misma comprobación daba 6.4 %). Aquí `τ` y `cur` salen de la misma
fila del mismo instante, no de dos corridas separadas en el tiempo.

**Pero discrepa un 6.1 % con la `k` por gravedad**, y la nueva tiene 0.10 % de
dispersión, así que no es ruido. La asimetría decide la lectura: `Δτ/Δi` es libre
de modelo, `g_modelo/i_suma` no. Como `k_gravedad/k_nueva` = 1.065, lo que dice
el dato es que **el URDF sobreestima la gravedad de `shoulder_lift` en un 6.5 %**.

Si se confirma en otra junta, **todas las `k` de §3.1 llevan ese sesgo**, y con
ellas todos los valores de fricción en N·m que hoy están en `smc_params.yaml` y
en el optimizador. No cambia ninguna conclusión cualitativa, pero sí los números
que van al paper. Y da una explicación alternativa para la anomalía de `wrist_1`
(§6): si cada junta tiene su propio error de gravedad en el URDF, su `k` = 9.00
puede ser sesgo del modelo y no una `k` realmente distinta.

### 8.3 Cuatro juntas medidas: dos familias, y la anomalía era otra

| junta | `k` gravedad | `k` diferencia | dispersión | familia |
|---|---|---|---|---|
| shoulder_lift | 11.850 | **11.1240** | 0.10 % | grande |
| elbow | 11.630 | **11.0012** | 0.18 % | grande |
| wrist_1 | 9.000 | **8.6660** | 0.15 % | muñeca |
| wrist_3 | *no identificable* | **8.6437** | 0.05 % † | muñeca |
| wrist_2 | 11.679 | — | | **sospechosa** |

† `wrist_3` excluyendo su nivel más lento, que es un atípico (9.30 frente a
8.64). Mirando `|v|`: pidió 0.020 rad/s y la junta fue a 0.1695 — 8.5× más. Con
`kp` = 100 la realimentación es nula y manda el feedforward solo, así que la
velocidad no se controla y el nivel lento no se asienta. No invalida la medida
—`k = Δτ/Δi` no depende de la velocidad real— pero el nivel se descarta.

**El sesgo de la gravedad es sistemático** (+6.5 % y +5.7 %), y hay una
explicación más económica que un error del URDF: **un offset en la corriente**.
Un `i0` único de ≈ −0.095 A explica las dos juntas a la vez, mientras que la
hipótesis del URDF necesita dos errores independientes que casualmente salen
parecidos. Ya había evidencia: la multipostura de `wrist_1` midió `i0` = −0.0133 A.

Lo que decide es que el método de gravedad usa la **suma** entre sentidos, donde
un offset **no se cancela**, y el de diferencias lo cancela igual que cancela la
gravedad. **El método nuevo es inmune a las dos explicaciones**, así que es el
fiable con independencia de cuál sea la correcta.

#### La anomalía de `wrist_1` queda cerrada — y apuntaba al sitio equivocado

`wrist_1` (8.6660) y `wrist_3` (8.6437) coinciden al **0.26 %**: dos juntas de
muñeca con la misma `k`, medidas por separado y sin modelo. Es lo que se espera
si comparten actuador.

Así que la lectura de §6 estaba invertida. `wrist_1` valía ~8.67 y **estaba
bien**; la que se sale es `wrist_2` con 11.68, que vino de la multipostura — un
método basado en gravedad, con el mismo sesgo, y con un condicionamiento que ya
se vio flojo. **La comprobación física de la brida deja de estar indicada**: no
hay masa no modelada que explicar. Lo que hay que rehacer es la `k` de `wrist_2`.

#### `wrist_3` deja de ser el caso duro

Con su `k` por fin medida, y no supuesta:

| | antes (`k` = 11.7 hipotética, ±8.8 %) | ahora (`k` = 8.644 medida) |
|---|---|---|
| F_v | 2.87 N·m·s/rad | **2.12** |
| F_c | 3.18 N·m | **2.35** |
| cota residual `d` | 0.293 N·m | **0.0281** |
| φ mínima | 1.710 rad/s | **0.164** |
| error de filo | 4.90° | **0.47°** |

**Un factor 10**, mejor que el 6× estimado en `05_smc.md` §7.7, porque no solo
cae la incertidumbre de `k` sino la fricción misma (−26 %). Con φ = 0.164
`wrist_3` recupera modo deslizante de verdad, en vez del lazo lineal degenerado
que se temía.

**Consecuencia para los números publicados:** las `k` de §3.1 llevan ~6 % de
sesgo y con ellas todos los valores de fricción en N·m que hoy están en
`smc_params.yaml` y en el optimizador. No cambia ninguna conclusión cualitativa
—ni el umbral de chattering, ni la incompatibilidad de las muñecas— pero sí las
cifras del paper.

Script: `ur5_identification/scripts/calibrate_k_from_torque.py`.

---

### 8.4 Las seis `k` medidas, y la fricción recalculada

| junta | `k` gravedad | `k` diferencia | dispersión | familia |
|---|---|---|---|---|
| shoulder_pan | 11.881 † | **11.139** | 0.08 % | grande |
| shoulder_lift | 11.850 | **11.124** | 0.10 % | grande |
| elbow | 11.630 | **11.001** | 0.18 % | grande |
| wrist_1 | 9.000 | **8.666** | 0.15 % | muñeca |
| wrist_2 | 11.679 | **8.542** | 0.24 % | muñeca |
| wrist_3 | — | **8.644** | 0.05 % | muñeca |

† por cruce de métodos (§3.3), nunca por gravedad: su eje es vertical.

Dispersión **dentro** de cada familia: 1.3 % en las grandes, 1.4 % en las
muñecas. `wrist_2` es el caso extremo: su multipostura daba 11.679 frente a
8.542 reales, un **37 % de error** — no el sesgo del 6 % del resto, sino una
medida sencillamente mala. Era la que hacía parecer contradictoria a `wrist_1`.

Fricción recalculada (corriente de la campaña × `k` nueva):

| junta | F_v antes | F_v ahora | F_c antes | F_c ahora | |
|---|---|---|---|---|---|
| shoulder_pan | 14.55 | **13.64** | 7.31 | **6.85** | −6.2 % |
| shoulder_lift | 12.30 | **11.55** | 7.20 | **6.76** | −6.1 % |
| elbow | 14.53 | **13.74** | 7.89 | **7.46** | −5.4 % |
| wrist_1 | 1.33 | **1.28** | 1.85 | **1.78** | −3.8 % |
| wrist_2 | 1.85 | **1.35** | 2.68 | **1.96** | **−26.9 %** |
| wrist_3 | 2.87 | **2.12** | 3.18 | **2.35** | **−26.1 %** |

### 8.5 `F_v` depende del estado TÉRMICO — declararlo

Repitiendo `shoulder_lift` al final de la sesión, tras seis barridos:

| | 1ª corrida (frío) | 7ª corrida (rodado) | variación |
|---|---|---|---|
| `k` | 11.1240 | 11.1189 | **0.05 %** |
| F_c | 7.294 | 7.239 | 0.76 % |
| **F_v** | 13.711 | **12.437** | **9.3 %** |

`k` repite a la quinta cifra y `F_c` apenas se mueve; **el viscoso baja un
9.3 % con el robot caliente**, que es lo que se espera al afinarse el
lubricante. El Coulomb es contacto seco y no se entera.

Eso explica un desacuerdo que parecía preocupante: la primera corrida daba F_v =
13.71 frente a los 12.30 de la campaña de agosto, un +11.5 %. La séptima da
12.44, a **1.1 %** de la campaña. **No era deriva entre sesiones ni una campaña
equivocada: era el robot frío.**

> **Para el paper.** Un `F_v` sin declarar el estado térmico arrastra un ±10 %
> que no aparece en ninguna barra de error. Los valores de §8.4 son los de
> RÉGIMEN, que es la condición en la que ocurre la incisión. El protocolo es
> descartar los primeros barridos, que es lo que la campaña de agosto hizo sin
> pretenderlo. `k` y `F_c` no necesitan esa salvedad.

### 8.6 Efecto en las ganancias

Con la fricción nueva y la incertidumbre de `k` bajando de ±8.8 % a ±0.5 %:

| junta | φ mínima | φ configurada | e_ss |
|---|---|---|---|
| shoulder_pan | 0.0004 | 0.05 | 0.14° |
| shoulder_lift | 0.0008 | 0.07 | 0.20° |
| elbow | 0.0030 | 0.07 | 0.20° |
| wrist_1 | 0.0021 | 0.07 | 0.20° |
| wrist_2 | 0.0052 | 0.05 | 0.14° |
| **wrist_3** | **0.1972** | **0.20** | **0.56°** |

Cinco juntas tienen margen de sobra. **`wrist_3` es la única que cambia**: pasa
de 0.05 a 0.20. Antes pedía 1.71 con 4.9° de error de filo, porque su `k` era una
hipótesis con ±8.8 %; medida de verdad, la fricción cae un 26 % y la
incertidumbre casi desaparece.

> El umbral de chattering de `wrist_3` **no está medido** — Gazebo la congela
> (`05_smc.md` §7.5) — así que se le supone el de `wrist_1` (1.33). Es el único
> φ de la lista apoyado en una suposición y no en una medida.

---

## 9. Uso

```bash
# Campaña completa (18 corridas, ~90 min). Pide confirmación por junta.
ros2 run ur5_identification run_friction_campaign_real.py --test-base 950 \
    --home-between-runs --k 3:9.0044 4:11.6792

# Un nivel suelto
ros2 run ur5_identification run_friction_campaign_real.py --test-base 950 \
    --home-between-runs --levels 0.0 --k 3:9.0044 4:11.6792

# Calibración multipostura de k (juntas sin gravedad en q_init)
ros2 run ur5_identification calibrate_multipose.py --joint 4

# Identificación sobre un CSV ya convertido
ros2 run ur5_identification run_identification \
    --csv ~/.ros/ur5_dyn_control/fl_951.csv --models viscous_coulomb
```

`--skip NIVEL:JUNTA` admite nombre de junta y `*` como comodín;
`--resume` salta las corridas cuyo CSV convertido ya existe.

---

## 10. Trazabilidad

| nivel | test | juntas | `fl_*` generado |
|---|---|---|---|
| 0.0 | 950–955 | 0–5 | 951, 952, 953, 954 |
| default | 960–965 | 0–5 | 961, 962, 963, 964 |
| 1.0 | 970–975 | 0–5 | 971, 972, 973, 974 |

Los `cur_*` existen para las 18. No hay `fl_*` para las juntas 0 y 5 porque su
`k` no es identificable (§1.2); su fricción está medida, en amperios, en
`~/.ros/friction_campaign/run_9?0.log` y `run_9?5.log`.

Corridas por control de par usadas en §3.3 y §4: `fl_900`–`fl_902` (nivel 0.0) y
`fl_910`–`fl_912` (nivel default).

---

## 11. Pendiente

- [x] Medir `k` por `tau_phys/cur` (§8.1). Hechas 4 de 6 (§8.3): `wrist_3`
      cerrada y la anomalía de `wrist_1` resuelta.
- [ ] Medir `k` de `wrist_2` y `shoulder_pan` por la misma vía. `wrist_2` es
      ahora la sospechosa: 11.679 la sitúa con las grandes cuando debería estar
      con las muñecas (~8.65).
- [ ] Recalcular la fricción en N·m de las seis juntas con las `k` nuevas, y
      propagarlo a `smc_params.yaml` y a `FRICTION_REAL_G4_0`.
- [ ] ~~Comprobación física de la brida~~ — ya NO indicada (§8.3): `wrist_1` no
      tenía masa sin modelar, su `k` es genuinamente la de una muñeca.
- [ ] Residual tras compensación en las muñecas — requiere una vía distinta al
      control de par, que no puede moverlas.
- [ ] Campaña a baja velocidad (< 0.02 rad/s) si se quiere modelar stiction.
- [ ] Meter la fricción física medida en el modelo de Gazebo, para que la
      comparación de los cuatro controladores sea predictiva.
- [ ] Deriva térmica: estos valores son de un robot a la temperatura de la
      campaña; no se ha caracterizado.

---

## 12. Archivos

```
ur5_identification/scripts/run_friction_campaign_real.py   runner semiautomático
ur5_identification/scripts/run_current_sweep.py            barrido por posición
ur5_identification/scripts/calibrate_current.py            corriente -> par
ur5_identification/scripts/calibrate_multipose.py          k por multipostura
ur5_identification/ur5_identification/campaign_levels.py   niveles G4 compartidos
ur5_dyn_control/launch/ur5e_real.launch.py                 launch del robot real
ur5_dyn_control/config/sweep_params.yaml                   ganancias y barrido
```
