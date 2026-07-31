"""
LQR-SDRE del UR5e en Gazebo Fortress (FASE 4).

Incluye el bringup effort PAUSADO con auto_start:=false; el nodo
gz_lqr_sdre_control_node construye la tabla de referencias (IK offline de la
trayectoria de incision), publica g(q_init) a ciegas, activa
[joint_state_broadcaster, forward_effort_controller] y despausa el mundo ->
el robot nunca cae. Luego: HOLD -> rampa quintica -> TRACK -> HOLD_END.

Ademas del CSV unificado escribe `lqr_diag_<test_num>.csv` con, por paso,
max Re(eig(A - B K)), cond(M), residuo de la CARE y tiempos de computo.

Usage:
  ros2 launch ur5_dyn_control lqr_sdre_control.launch.py gazebo_gui:=false test_num:=1
  ros2 launch ur5_dyn_control lqr_sdre_control.launch.py wn:=35 zeta:=1.0
  ros2 launch ur5_dyn_control lqr_sdre_control.launch.py care_update_rate:=50
  ros2 launch ur5_dyn_control lqr_sdre_control.launch.py \
      world:=<...>/empty_test_world.sdf     # desarrollo rapido (RTF ~1)
"""

import os
import sys as _sys
_sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from world_defaults import default_world  # noqa: E402
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    dyn_pkg = get_package_share_directory("ur5_dyn_control")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dyn_pkg, "launch", "ur5e_effort_gz.launch.py")
        ),
        launch_arguments={
            "paused": "true",
            "auto_start": "false",          # el nodo activa y despausa
            "world": LaunchConfiguration("world"),
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
            "joint_damping": LaunchConfiguration("joint_damping"),
            "joint_friction": LaunchConfiguration("joint_friction"),
        }.items(),
    )

    # Solo se anaden los overrides que el usuario dio explicitamente, de modo
    # que sin ellos manda el params_file (comportamiento historico).
    overrides = {}
    mode = LaunchConfiguration("friction_compensation").perform(context).strip()
    if mode:
        overrides["friction_compensation"] = mode
    q_mode = LaunchConfiguration("Q_mode").perform(context).strip()
    if q_mode:
        overrides["lqr.Q_mode"] = q_mode
    for arg, key in (("wn", "lqr.wn"),
                     ("zeta", "lqr.zeta"),
                     ("r", "lqr.r"),
                     ("care_update_rate", "lqr.care_update_rate")):
        raw = LaunchConfiguration(arg).perform(context).strip()
        if raw:
            overrides[key] = float(raw)
    for arg, key in (("friction_f_v", "friction.f_v"),
                     ("friction_f_c", "friction.f_c"),
                     ("initial_offset", "initial_offset")):
        raw = LaunchConfiguration(arg).perform(context).strip()
        if raw:
            vals = [float(v) for v in raw.replace(",", " ").split() if v]
            if len(vals) != 6:
                raise RuntimeError(f"{arg} debe tener 6 valores, se dio: {raw!r}")
            overrides[key] = vals

    # FASE 7: fichero de pesos generado por el sintonizador. Se superpone
    # DESPUES del params_file, asi que solo pisa lo que traiga y deja intacto el
    # resto de la configuracion.
    params = [LaunchConfiguration("params_file")]
    gains_file = LaunchConfiguration("gains_file").perform(context).strip()
    if gains_file:
        if not os.path.exists(gains_file):
            raise RuntimeError(f"gains_file no existe: {gains_file}")
        params.append(gains_file)

    node = Node(
        package="ur5_dyn_control",
        executable="gz_lqr_sdre_control_node",
        output="screen",
        parameters=params + [
            {
                "test_num": LaunchConfiguration("test_num"),
                "t_sim": LaunchConfiguration("t_sim"),
            },
            overrides,
        ],
    )

    return [sim, node]


def generate_launch_description():
    dyn_pkg = get_package_share_directory("ur5_dyn_control")

    declared_arguments = [
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(dyn_pkg, "config", "lqr_sdre_params.yaml"),
            description="YAML de parametros del nodo LQR-SDRE.",
        ),
        DeclareLaunchArgument("test_num", default_value="1"),
        DeclareLaunchArgument("t_sim", default_value="0.0"),
        DeclareLaunchArgument(
            "world",
            default_value=default_world(),   # ver launch/world_defaults.py
        ),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        # FASE 2: friccion articular inyectada en la planta de Gazebo.
        DeclareLaunchArgument("joint_damping", default_value="0"),
        DeclareLaunchArgument("joint_friction", default_value="0"),
        DeclareLaunchArgument(
            "friction_compensation", default_value="",
            description="'' = usar el params_file; si no: none|viscous|viscous_coulomb"),
        DeclareLaunchArgument(
            "friction_f_v", default_value="",
            description="6 coeficientes viscosos coma-separados [N.m.s/rad]"),
        DeclareLaunchArgument(
            "friction_f_c", default_value="",
            description="6 coeficientes de Coulomb coma-separados [N.m]"),
        # FASE 4 — barridos de diseno sin duplicar YAMLs.
        DeclareLaunchArgument(
            "wn", default_value="",
            description="ancho de banda objetivo del lazo cerrado [rad/s]"),
        DeclareLaunchArgument(
            "zeta", default_value="",
            description="amortiguamiento objetivo; DEBE ser >= 1/sqrt(2)"),
        DeclareLaunchArgument(
            "r", default_value="", description="peso del esfuerzo (R = r I)"),
        DeclareLaunchArgument(
            "Q_mode", default_value="",
            description="'' = params_file; si no: fixed | scheduled"),
        DeclareLaunchArgument(
            "care_update_rate", default_value="",
            description="Hz de resolucion de la CARE; 0 = cada ciclo (ZOH sobre K)"),
        # FASE 5 — error inicial deliberado (ensayo de tiempo de alcance).
        DeclareLaunchArgument(
            "initial_offset", default_value="",
            description="6 offsets [rad] sumados al destino de la rampa"),
        # FASE 7 — pesos optimizados.
        DeclareLaunchArgument(
            "gains_file", default_value="",
            description="YAML de pesos que se superpone al params_file"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])
