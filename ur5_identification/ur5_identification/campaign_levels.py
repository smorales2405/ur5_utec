"""
Definiciones compartidas de la campaña de fricción (FASE 2).

Viven aquí, y no duplicadas en cada script, porque una divergencia entre el
runner de campaña y el barrido suelto no daría ningún síntoma: las corridas
saldrían con niveles distintos de los que dice el registro de sesión, y el
desacuerdo solo aparecería al comparar resultados meses después.
"""

JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

#: Niveles de compensación interna de fricción del robot (docs/00_prereqs.md
#: §G4), como (viscous_scale, coulomb_scale).
#:
#: `default` son los valores de fábrica del driver: se incluye porque es lo que
#: usaría cualquiera que NO llamase al servicio, y hay que poder compararlo.
LEVELS = {
    "0.0":     ([0.0] * 6, [0.0] * 6),
    "default": ([0.9, 0.9, 0.8, 0.9, 0.9, 0.9], [0.8, 0.8, 0.7, 0.8, 0.8, 0.8]),
    "1.0":     ([1.0] * 6, [1.0] * 6),
}

FRICTION_SRV = "/friction_model_controller/set_friction_model_parameters"
