import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_pick_place = get_package_share_directory("ur5_pick_place")

    declared = [
        DeclareLaunchArgument(
            "method",
            default_value="clamped_spline",
            description="Interpolation method: clamped_spline or piecewise_linear",
        ),
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="UR robot type passed to the simulation launch",
        ),
        DeclareLaunchArgument(
            "node_start_delay",
            default_value="20.0",
            description=(
                "Real-time seconds to wait after launching Gazebo before "
                "starting pick_place_node. Increase on slower machines."
            ),
        ),
    ]

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_pick_place, "launch", "ur5_robotiq_gz.launch.py")
        ),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
        }.items(),
    )

    pick_place_node = Node(
        package="ur5_pick_place",
        executable="pick_place_node",
        name="pick_place_node",
        output="screen",
        parameters=[
            os.path.join(pkg_pick_place, "config", "pick_place_params.yaml"),
            {"method": LaunchConfiguration("method")},
        ],
    )

    return LaunchDescription(
        declared + [
            gz_launch,
            TimerAction(
                period=LaunchConfiguration("node_start_delay"),
                actions=[pick_place_node],
            ),
        ]
    )
