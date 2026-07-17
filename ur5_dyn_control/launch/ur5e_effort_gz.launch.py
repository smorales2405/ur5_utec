"""
Bringup del UR5e con interfaz de comando EFFORT en Gazebo Fortress.

Secuencia anti-caida (arranca PAUSADO por defecto):
  1. Gazebo pausado (mundo 1 kHz por defecto) + spawn del UR5e en la pose
     inicial (initial_value de las state interfaces) a z=0.63 sobre la mesa.
  2. Spawners con --inactive: joint_state_broadcaster y
     forward_effort_controller (cargar+configurar SI funciona con la sim
     pausada; ACTIVAR no: gz_ros2_control solo corre el update() del
     controller_manager en cada paso de fisica).
  3a. auto_start:=true (default, bringup standalone): activate_and_unpause.py
      envia el switch de activacion (asincrono, queda pendiente) y despausa
      -> la activacion se consuma en el primer paso de fisica. OJO: sin un
      nodo publicando torque el brazo cae (la unica interfaz es effort).
  3b. auto_start:=false (usado por gravity_comp/fl_control.launch.py): el
      NODO de control publica g(q_init) a ciegas, pide la activacion y
      despausa cuando esta listo -> el robot nunca cae.

NOTA (evaluado empiricamente): en gz_ros2_control 0.7.19 declarar la
interfaz effort activa el modo effort del joint desde initSim y el modo
posicion deja de aplicarse a la fisica; por eso NO hay controlador de
posicion ni switch posicion->esfuerzo en simulacion. En el robot real el
driver de UR si soporta ese switch (JTC <-> forward_effort_controller).

Usage:
  ros2 launch ur5_dyn_control ur5e_effort_gz.launch.py gazebo_gui:=false
  ros2 launch ur5_dyn_control ur5e_effort_gz.launch.py \
      world:=<...>/empty_test_world.sdf   # mundo sin mesas (RTF ~1, dev)
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
    ur_type            = LaunchConfiguration("ur_type")
    safety_limits      = LaunchConfiguration("safety_limits")
    safety_pos_margin  = LaunchConfiguration("safety_pos_margin")
    safety_k_position  = LaunchConfiguration("safety_k_position")
    tf_prefix          = LaunchConfiguration("tf_prefix")
    controllers_file   = LaunchConfiguration("controllers_file")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    world              = LaunchConfiguration("world").perform(context)
    world_name         = LaunchConfiguration("world_name").perform(context)
    gazebo_gui         = LaunchConfiguration("gazebo_gui").perform(context).strip().lower()
    paused             = LaunchConfiguration("paused").perform(context).strip().lower()
    auto_start         = LaunchConfiguration("auto_start").perform(context).strip().lower()
    activate_list      = [
        c.strip() for c in
        LaunchConfiguration("activate_controllers").perform(context).split(",")
        if c.strip()
    ]

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur5_dyn_control"), "urdf", "ur5e_effort.urdf.xacro"]
            ),
            " ", "name:=ur",
            " ", "ur_type:=",                ur_type,
            " ", "tf_prefix:=",              tf_prefix,
            " ", "safety_limits:=",          safety_limits,
            " ", "safety_pos_margin:=",      safety_pos_margin,
            " ", "safety_k_position:=",      safety_k_position,
            " ", "initial_positions_file:=", initial_positions_file,
            " ", "simulation_controllers:=", controllers_file,
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

    # Spawners con --inactive: la ACTIVACION la hace el orquestador
    # (auto_start:=true) o el nodo de control (auto_start:=false).
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--inactive",
            "--controller-manager-timeout", "120",
        ],
    )

    forward_effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_effort_controller",
            "-c", "/controller_manager",
            "--inactive",
            "--controller-manager-timeout", "120",
        ],
    )

    activate_and_unpause = Node(
        package="ur5_dyn_control",
        executable="activate_and_unpause.py",
        output="screen",
        arguments=[
            "--controller-manager", "/controller_manager",
            "--world", world_name,
            "--activate", *activate_list,
        ],
    )

    delay_effort_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[forward_effort_controller_spawner],
        )
    )

    actions_after_effort = []
    if auto_start in ("true", "1"):
        actions_after_effort.append(activate_and_unpause)

    delay_start_after_effort = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=forward_effort_controller_spawner,
            on_exit=actions_after_effort,
        )
    ) if actions_after_effort else None

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

    gz_args = world
    if paused not in ("true", "1"):
        gz_args += " -r"   # correr desde t=0 (el brazo caera sin nodo de torque)
    if gazebo_gui not in ("true", "1"):
        gz_args += " -s"   # server-only, sin ventana

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    actions = [
        gz_sim,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_effort_after_jsb,
        gz_spawn_entity,
        gz_sim_bridge,
    ]
    if delay_start_after_effort is not None:
        actions.append(delay_start_after_effort)
    return actions


def generate_launch_description():
    dyn_pkg = get_package_share_directory("ur5_dyn_control")

    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur5", "ur10", "ur3e", "ur5e", "ur10e", "ur16e", "ur20", "ur30"],
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
            "controllers_file",
            default_value=os.path.join(dyn_pkg, "config", "ur5e_effort_controllers.yaml"),
            description="Controllers YAML (update_rate debe casar con el paso del mundo).",
        ),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=os.path.join(dyn_pkg, "config", "initial_positions.yaml"),
            description="YAML con la pose articular inicial (initial_value del ros2_control).",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(dyn_pkg, "worlds", "lab_torque_world.sdf"),
            description=(
                "Mundo SDF. Default: lab_torque_world.sdf (contenido del "
                "laboratorio, 1 kHz; RTF bajo por colisiones trimesh). Para "
                "desarrollo rapido: empty_test_world.sdf (RTF ~1)."
            ),
        ),
        DeclareLaunchArgument(
            "world_name",
            default_value="default",
            description="Nombre del <world> del SDF (para /world/<name>/control).",
        ),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Start Gazebo with GUI. false = headless (mejor RTF).",
        ),
        DeclareLaunchArgument(
            "paused",
            default_value="true",
            description=(
                "Arrancar Gazebo pausado (sin -r). Con el arranque pausado el "
                "robot queda congelado en q_init hasta que el orquestador o el "
                "nodo de control despausa -> nunca cae."
            ),
        ),
        DeclareLaunchArgument(
            "auto_start",
            default_value="true",
            description=(
                "true: activate_and_unpause.py activa los controladores y "
                "despausa (bringup standalone). false: lo hara el nodo de "
                "control dinamico cuando este listo (usado por "
                "gravity_comp/fl_control.launch.py)."
            ),
        ),
        DeclareLaunchArgument(
            "activate_controllers",
            default_value="joint_state_broadcaster,forward_effort_controller",
            description="Controladores (coma-separados) que activa el orquestador.",
        ),
    ]

    # Recursos: meshes de ur_description (dirname del share) y de ur5_pick_place
    # (model://ur5_base y model://surgery_table del mundo).
    pick_place_pkg = get_package_share_directory("ur5_pick_place")
    ur_desc_share  = get_package_share_directory("ur_description")

    resource_path_value = ":".join(filter(None, [
        os.path.dirname(ur_desc_share),
        os.path.join(pick_place_pkg, "meshes"),
        os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
    ]))

    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = resource_path_value

    set_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=resource_path_value,
    )

    return LaunchDescription(
        declared_arguments + [
            set_resource_path,
            OpaqueFunction(function=launch_setup),
        ]
    )
