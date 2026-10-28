"""
Mundo por defecto de los launches de control por torque — ÚNICO sitio donde vive.

Los cuatro launches (`ur5e_effort_gz`, `gravity_comp`, `fl_control`,
`smc_control`) importan de aquí en vez de repetir la ruta, para que no puedan
divergir: que el barrido de un controlador corriese en una escena y el de otro
en otra invalidaría la comparación del paper, y no daría ningún síntoma en los
CSV.

Por qué la escena de incisión y no la de par genérico
-----------------------------------------------------
`lab_torque_world.sdf` heredó, al copiarse de `lab_base_world.sdf`, el
**obstáculo del caso de uso de pick & place**: una caja AABB de 0.40 × 0.60 ×
0.20 m en (0.85, 0.00, 0.73). No pinta nada en el experimento de corte y ningún
código de control por torque la referencia.

En la trayectoria nominal la holgura mínima medida (orígenes de junta y TCP
frente a la caja, sobre la tabla de referencias completa) es de **0.150 m**, así
que las campañas ya registradas NO están afectadas. Se retira mirando a la
FASE 8: con desajuste de modelo de ±50 % y carga de 4 kg en el TCP el brazo se
desvía mucho más que eso, y una colisión contra un objeto ajeno al experimento
contaminaría la corrida sin dejar rastro evidente en el CSV.

`lab_torque_world.sdf` se conserva intacto. OJO: la caja cae sobre el trazo
de la incisión ACTUAL (corte en `base_y` 0.70–0.80, junto a la mesa): con la
tabla de hoy la punta se apoya en su cara superior y el watchdog salta en
`wrist_3` (medido 2026-10-27). Solo sirve con la trayectoria de su campaña.
Para reproducir una campaña anterior basta con pasarlo explícitamente:

    ros2 launch ur5_dyn_control smc_control.launch.py \\
        world:=$(ros2 pkg prefix --share ur5_dyn_control)/worlds/lab_torque_world.sdf
"""

import os

from ament_index_python.packages import get_package_share_directory

#: Escena de la incisión (sin el obstáculo del pick & place).
DEFAULT_WORLD_FILE = "lab_incision_world.sdf"

#: Escena histórica, con obstáculo. Solo para reproducir campañas anteriores.
LEGACY_WORLD_FILE = "lab_torque_world.sdf"

#: Altura [m] de la base del UR5e sobre el suelo de Gazebo = alto del pedestal.
#: 0.816 es el pedestal real (meshes/ur5_mounting_base, medido 2026-10-27).
MOUNT_Z = 0.816

#: El pedestal anterior (model://ur5_base de ur5_pick_place) medía 0.63. La
#: escena histórica lo conserva, y con él su altura: su obstáculo y su mesa
#: están colocados respecto a una base a 0.63.
LEGACY_MOUNT_Z = 0.63


def default_world() -> str:
    """Ruta absoluta del mundo por defecto, ya instalado."""
    return os.path.join(get_package_share_directory("ur5_dyn_control"),
                        "worlds", DEFAULT_WORLD_FILE)


def mount_z(world: str) -> float:
    """Altura a la que spawnear el UR5e en `world` (ruta del SDF).

    Va con el mundo y no como argumento de launch: un -z que no corresponda al
    pedestal de la escena deja el robot flotando o empotrado en él, y eso no da
    ningún síntoma en los CSV (la base está soldada al mundo y la gravedad es
    uniforme), pero sí mueve el robot respecto a la mesa.
    """
    if os.path.basename(world) == LEGACY_WORLD_FILE:
        return LEGACY_MOUNT_Z
    return MOUNT_Z
