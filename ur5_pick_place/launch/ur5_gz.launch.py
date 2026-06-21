"""
Launches Gazebo Fortress with the UR5e arm only (no Robotiq gripper).

The robot description uses ur5e.urdf.xacro, which keeps the 'gripper_tcp'
fixed frame at 0.141 m from tool0 so that pick_place_node and IKWrapper
work unchanged.

Usage:
  ros2 launch ur5_pick_place ur5_gz.launch.py
  ros2 launch ur5_pick_place ur5_gz.launch.py ur_type:=ur5e gazebo_gui:=true
"""

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    ur_type              = LaunchConfiguration("ur_type")
    safety_limits        = LaunchConfiguration("safety_limits")
    safety_pos_margin    = LaunchConfiguration("safety_pos_margin")
    safety_k_position    = LaunchConfiguration("safety_k_position")
    tf_prefix            = LaunchConfiguration("tf_prefix")
    start_joint_controller   = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")

    # Reuse the same controllers YAML (gripper controller is commented out there)
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("ur5_pick_place"), "config", "ur5_robotiq_controllers.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur5_pick_place"), "urdf", "ur5e.urdf.xacro"]
            ),
            " ", "name:=ur",
            " ", "ur_type:=",             ur_type,
            " ", "tf_prefix:=",           tf_prefix,
            " ", "safety_limits:=",       safety_limits,
            " ", "safety_pos_margin:=",   safety_pos_margin,
            " ", "safety_k_position:=",   safety_k_position,
            " ", "simulation_controllers:=", controllers_yaml,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
    )

    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller,
            "-c", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        condition=IfCondition(start_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller,
            "-c", "/controller_manager",
            "--stopped",
            "--controller-manager-timeout", "30",
        ],
        condition=UnlessCondition(start_joint_controller),
    )

    delay_joint_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                initial_joint_controller_spawner_started,
                initial_joint_controller_spawner_stopped,
            ],
        )
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name",  "ur5e",
            "-allow_renaming", "true",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.63",
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "0.0",
        ],
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    return [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_joint_controller_after_jsb,
        gz_spawn_entity,
        gz_sim_bridge,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3", "ur5", "ur10", "ur3e", "ur5e", "ur7e", "ur10e",
                "ur12e", "ur16e", "ur8long", "ur15", "ur18", "ur20", "ur30",
            ],
            default_value="ur5e",
        ),
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        ),
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        ),
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        ),
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix for joint/link names (multi-robot setups).",
        ),
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Start joint_trajectory_controller on launch.",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="UR controller to activate on launch.",
        ),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Start Gazebo with GUI.",
        ),
    ]

    world_pkg     = get_package_share_directory("ur5_pick_place")
    ur_desc_share = get_package_share_directory("ur_description")

    resource_path_value = ":".join(filter(None, [
        os.path.dirname(ur_desc_share),
        os.path.join(world_pkg, "meshes"),
        os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
    ]))

    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = resource_path_value

    set_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=resource_path_value,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": os.path.join(world_pkg, "worlds", "lab_base_world.sdf") + " -r"
        }.items(),
    )

    return LaunchDescription(
        declared_arguments + [
            set_resource_path,
            gz_sim,
            OpaqueFunction(function=launch_setup),
        ]
    )
