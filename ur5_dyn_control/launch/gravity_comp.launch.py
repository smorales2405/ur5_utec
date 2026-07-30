"""
Compensacion de gravedad del UR5e por torque en Gazebo Fortress (smoke test).

Incluye el bringup effort PAUSADO con auto_start:=false; el nodo
gz_gravity_comp_node publica g(q_init) a ciegas, activa
[joint_state_broadcaster, forward_effort_controller] y despausa el mundo
cuando esta listo -> el robot nunca cae y queda regulado en q_init.

Usage:
  ros2 launch ur5_dyn_control gravity_comp.launch.py gazebo_gui:=false
  ros2 launch ur5_dyn_control gravity_comp.launch.py \
      world:=<...>/empty_test_world.sdf test_num:=2
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
            default_value=os.path.join(dyn_pkg, "config", "gravity_comp_params.yaml"),
            description="YAML de parametros del nodo.",
        ),
        DeclareLaunchArgument("test_num", default_value="1"),
        DeclareLaunchArgument("t_sim", default_value="0.0"),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(dyn_pkg, "worlds", "lab_torque_world.sdf"),
        ),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        # FASE 2: friccion articular inyectada en la planta de Gazebo, para
        # poder validar el identificador contra la verdad. "0" = URDF sin tocar.
        DeclareLaunchArgument("joint_damping", default_value="0"),
        DeclareLaunchArgument("joint_friction", default_value="0"),
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
            "joint_damping": LaunchConfiguration("joint_damping"),
            "joint_friction": LaunchConfiguration("joint_friction"),
        }.items(),
    )

    node = Node(
        package="ur5_dyn_control",
        executable="gz_gravity_comp_node",
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
