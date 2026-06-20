from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("ur5_pick_place")

    return LaunchDescription([
        DeclareLaunchArgument(
            "method",
            default_value="clamped_spline",
            description="Trajectory method: clamped_spline or piecewise_linear",
        ),
        Node(
            package="ur5_pick_place",
            executable="pick_place_node",
            name="pick_place_node",
            output="screen",
            parameters=[
                os.path.join(pkg, "config", "pick_place_params.yaml"),
                {"method": LaunchConfiguration("method")},
            ],
        ),
    ])
