from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory("ur5_pick_place")

    params = [
        os.path.join(pkg, "config", "pick_place_params.yaml"),
        {"method": LaunchConfiguration("method")},
    ]

    extra = LaunchConfiguration("extra_params_file").perform(context).strip()
    if extra:
        params.append(extra)

    return [Node(
        package="ur5_pick_place",
        executable="pick_place_node",
        name="pick_place_node",
        output="screen",
        parameters=params,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "method",
            default_value="clamped_spline",
            description="Trajectory method: clamped_spline or piecewise_linear",
        ),
        DeclareLaunchArgument(
            "extra_params_file",
            default_value="",
            description=(
                "Optional extra params file loaded after pick_place_params.yaml "
                "(later keys win). Use for CU3 selected_solution.yaml overrides."
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
