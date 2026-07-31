"""
Feedback Linearization (computed torque) del UR5e en Gazebo Fortress.

Incluye el bringup effort PAUSADO con auto_start:=false; el nodo
gz_fl_control_node construye la tabla de referencias (IK offline de la
trayectoria cartesiana spline), publica g(q_init) a ciegas, activa
[joint_state_broadcaster, forward_effort_controller] y despausa el mundo ->
el robot nunca cae. Luego: HOLD -> rampa quintica -> TRACK -> HOLD_END.

Usage:
  ros2 launch ur5_dyn_control fl_control.launch.py gazebo_gui:=false test_num:=1
  ros2 launch ur5_dyn_control fl_control.launch.py \
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

    # Overrides de FASE 2: solo se anaden los que el usuario dio explicitamente,
    # de modo que sin ellos el params_file manda (comportamiento historico).
    overrides = {}
    mode = LaunchConfiguration("friction_compensation").perform(context).strip()
    if mode:
        overrides["friction_compensation"] = mode
    sweep_joint = LaunchConfiguration("sweep_joint").perform(context).strip()
    if sweep_joint:
        overrides["sweep.joint"] = int(sweep_joint)
    for arg, key in (("friction_f_v", "friction.f_v"),
                     ("friction_f_c", "friction.f_c")):
        raw = LaunchConfiguration(arg).perform(context).strip()
        if raw:
            vals = [float(v) for v in raw.replace(",", " ").split() if v]
            if len(vals) != 6:
                raise RuntimeError(f"{arg} debe tener 6 valores, se dio: {raw!r}")
            overrides[key] = vals

    node = Node(
        package="ur5_dyn_control",
        executable="gz_fl_control_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
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
            default_value=os.path.join(dyn_pkg, "config", "fl_control_params.yaml"),
            description="YAML de parametros del nodo FL.",
        ),
        DeclareLaunchArgument("test_num", default_value="1"),
        DeclareLaunchArgument("t_sim", default_value="0.0"),
        DeclareLaunchArgument(
            "world",
            default_value=default_world(),
        ),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        # FASE 2: friccion articular inyectada en la planta de Gazebo, para
        # poder validar el identificador contra la verdad. "0" = URDF sin tocar.
        DeclareLaunchArgument("joint_damping", default_value="0"),
        DeclareLaunchArgument("joint_friction", default_value="0"),
        # FASE 2: compensacion de friccion identificada. Sobrescriben lo que
        # diga el params_file, para poder hacer barridos A/B sin duplicar YAMLs.
        DeclareLaunchArgument(
            "friction_compensation", default_value="",
            description="'' = usar el params_file; si no: none|viscous|viscous_coulomb"),
        DeclareLaunchArgument(
            "friction_f_v", default_value="",
            description="6 coeficientes viscosos coma-separados [N.m.s/rad]"),
        DeclareLaunchArgument(
            "friction_f_c", default_value="",
            description="6 coeficientes de Coulomb coma-separados [N.m]"),
        # FASE 2: junta a barrer, para poder recorrer la campana sin duplicar YAMLs.
        DeclareLaunchArgument(
            "sweep_joint", default_value="",
            description="'' = usar el params_file; si no, junta 0..5 a barrer"),
    ]


    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])
