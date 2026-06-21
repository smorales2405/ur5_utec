"""
Launch: RViz inspection del UR5e con markers en los 5 waypoints CU3.

Uso:
  ros2 launch ur5_pick_place rviz_inspection.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    ur_type   = LaunchConfiguration('ur_type').perform(context)
    tf_prefix = LaunchConfiguration('tf_prefix').perform(context)

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare('ur5_pick_place'), 'config', 'ur5_robotiq_controllers.yaml']
    )

    # Mismo xacro que usa Gazebo (ur5_robotiq_gz.launch.py)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution(
            [FindPackageShare('ur5_pick_place'), 'urdf', 'ur5_robotiq_2f85.urdf.xacro']
        ),
        ' ', 'name:=ur',
        ' ', 'ur_type:=',   ur_type,
        ' ', 'tf_prefix:=', tf_prefix,
        ' ', 'safety_limits:=true',
        ' ', 'simulation_controllers:=', controllers_yaml,
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # robot_state_publisher — publica TF desde /robot_description + /joint_states
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}],
    )

    # joint_state_publisher_gui — sliders para mover articulaciones
    # Posicion inicial: pose de spawn de Gazebo (shoulder_lift=-90 deg, elbow=+90 deg)
    jsp_params_path = PathJoinSubstitution(
        [FindPackageShare('ur5_pick_place'), 'config', 'joint_state_initial.yaml']
    ).perform(context)   # resolver a string antes de pasar al nodo
    jsp_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[jsp_params_path],
    )

    # Markers en /waypoint_markers (frame base_link)
    markers_node = Node(
        package='ur5_pick_place',
        executable='waypoint_markers_node.py',
        output='screen',
    )

    # RViz2
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('ur5_pick_place'), 'config', 'inspection.rviz']
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return [rsp_node, jsp_node, markers_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ur_type',   default_value='ur5e'),
        DeclareLaunchArgument('tf_prefix', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
