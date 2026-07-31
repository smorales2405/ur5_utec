"""
Sliding Mode Control del UR5e en Gazebo Fortress (FASE 5).

Incluye el bringup effort PAUSADO con auto_start:=false; el nodo
gz_smc_control_node construye la tabla de referencias (IK offline de la
trayectoria cartesiana spline), publica g(q_init) a ciegas, activa
[joint_state_broadcaster, forward_effort_controller] y despausa el mundo ->
el robot nunca cae. Luego: HOLD -> rampa quintica -> TRACK -> HOLD_END.

Usage:
  ros2 launch ur5_dyn_control smc_control.launch.py gazebo_gui:=false test_num:=1
  ros2 launch ur5_dyn_control smc_control.launch.py \
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
    sw = LaunchConfiguration("switching_function").perform(context).strip()
    if sw:
        overrides["switching_function"] = sw
    for arg in ("phi", "alpha"):
        raw = LaunchConfiguration(arg).perform(context).strip()
        if raw:
            overrides[arg] = float(raw)
    for arg, key in (("friction_f_v", "friction.f_v"),
                     ("friction_f_c", "friction.f_c"),
                     ("initial_offset", "initial_offset")):
        raw = LaunchConfiguration(arg).perform(context).strip()
        if raw:
            vals = [float(v) for v in raw.replace(",", " ").split() if v]
            if len(vals) != 6:
                raise RuntimeError(f"{arg} debe tener 6 valores, se dio: {raw!r}")
            overrides[key] = vals

    # FASE 7: fichero de ganancias generado por `run_gain_tuning`. Se superpone
    # DESPUES del params_file, asi que solo pisa lambda/eta/phi/alpha y deja
    # intacto el resto de la configuracion. Es el criterio de aceptacion del
    # plan: las ganancias seleccionadas se cargan sin edicion manual.
    params = [LaunchConfiguration("params_file")]
    gains_file = LaunchConfiguration("gains_file").perform(context).strip()
    if gains_file:
        if not os.path.exists(gains_file):
            raise RuntimeError(f"gains_file no existe: {gains_file}")
        params.append(gains_file)

    node = Node(
        package="ur5_dyn_control",
        executable="gz_smc_control_node",
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
            default_value=os.path.join(dyn_pkg, "config", "smc_params.yaml"),
            description="YAML de parametros del nodo SMC.",
        ),
        DeclareLaunchArgument("test_num", default_value="1"),
        DeclareLaunchArgument("t_sim", default_value="0.0"),
        DeclareLaunchArgument(
            "world",
            default_value=default_world(),   # ver launch/world_defaults.py
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
        # FASE 5: barridos A/B sin duplicar YAMLs.
        DeclareLaunchArgument(
            "switching_function", default_value="",
            description="'' = params_file; si no: sign | sat"),
        DeclareLaunchArgument("phi", default_value="",
                              description="ancho de la capa limite de sat(s/phi)"),
        DeclareLaunchArgument("alpha", default_value="",
                              description="fraccion de incertidumbre en (0,1]"),
        # FASE 5 — ensayo de tiempo de alcance.
        DeclareLaunchArgument(
            "initial_offset", default_value="",
            description="6 offsets [rad] sumados al destino de la rampa; crean "
                        "un error inicial deliberado con s(0) = Lambda*offset"),
        # FASE 7 — ganancias optimizadas.
        DeclareLaunchArgument(
            "gains_file", default_value="",
            description="YAML de ganancias de run_gain_tuning; se superpone al "
                        "params_file (solo lambda/eta/phi/alpha)"),
    ]


    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])
