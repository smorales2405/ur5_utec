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
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
            default_value=os.path.join(dyn_pkg, "worlds", "lab_torque_world.sdf"),
        ),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dyn_pkg, "launch", "ur5e_effort_gz.launch.py")
        ),
        launch_arguments={
            "paused": "true",
            "auto_start": "false",          # el nodo activa y despausa
            "world": LaunchConfiguration("world"),
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
        }.items(),
    )

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
        ],
    )

    return LaunchDescription(declared_arguments + [sim, node])
