"""
Control por torque en el UR5e REAL (FASES 2 y 9).

Este launch arranca SOLO el nodo de control. **No** levanta el driver a
propósito: el bring-up del robot, la carga del programa `External Control` en el
teach pendant y la verificación de los controladores son pasos que el operador
debe hacer y comprobar *antes* de que nada empiece a comandar par. Encadenarlos
en un único launch significaría que el primer torque sale en cuanto el driver
esté listo, sin ventana para abortar.

Secuencia de sesión
-------------------
1. Driver (terminal aparte), con el robot encendido y sin protective stop::

     ros2 launch ur_robot_driver ur_control.launch.py \\
         ur_type:=ur5e robot_ip:=192.168.0.102 launch_rviz:=false \\
         initial_joint_controller:=scaled_joint_trajectory_controller

2. En el pendant: cargar el programa con `External Control` y darle a play.
3. Comprobar::

     ros2 control list_controllers        # los cuatro cargados
     ros2 topic hz /joint_states          # ~500 Hz

4. Fijar las escalas de fricción interna EXPLÍCITAMENTE (compuerta G4 —
   sin la llamada el valor efectivo no queda registrado y la campaña no es
   reproducible). Ver `docs/00_prereqs.md` §G4.
5. `zero_ftsensor` antes de aproximar (checklist §7).
6. Este launch.

Lo que fija por ti (y por qué)
------------------------------
- ``gravity_in_command:=false`` — **compuerta G3**. `direct_torque()` compensa
  la gravedad dentro del robot; comandarla otra vez la duplica y el brazo se
  acelera hacia arriba. No es un default: se fuerza y no se expone como
  argumento, para que no pueda quedarse a `true` por descuido.
- ``use_sim_time:=false`` y ``perform_unpause:=false`` — no hay `/clock` ni
  mundo que despausar.
- Listas de switch del driver: `scaled_joint_trajectory_controller` y
  `forward_effort_controller` son **mutuamente excluyentes** (tabla de
  compatibilidad del hardware, ver G2), así que hay que desactivar el primero.

`tau_scale` (seguridad)
-----------------------
El checklist §7 pide «`tau_max` conservador para el primer ensayo (p. ej. 30 %
del nominal) y subida gradual». `tau_scale` multiplica los límites nominales del
UR5e; **el default es 0.30**, no 1.0. Subirlo es una decisión consciente del
operador, que es justo lo que se quiere.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: Límites de par nominales del UR5e (ur_description/config/ur5e/joint_limits.yaml).
TAU_MAX_NOMINAL = [150.0, 150.0, 150.0, 28.0, 28.0, 28.0]

#: Ejecutable por controlador.
EXECUTABLES = {
    "gravity_comp": "gz_gravity_comp_node",
    "fl": "gz_fl_control_node",
    "smc": "gz_smc_control_node",
}

#: YAML de parámetros por defecto de cada controlador.
PARAMS = {
    "gravity_comp": "gravity_comp_params.yaml",
    "fl": "fl_control_params.yaml",
    "smc": "smc_params.yaml",
}


def launch_setup(context, *args, **kwargs):
    dyn_pkg = get_package_share_directory("ur5_dyn_control")

    controller = LaunchConfiguration("controller").perform(context).strip()
    if controller not in EXECUTABLES:
        raise RuntimeError(
            f"controller desconocido: {controller!r} "
            f"(validos: {', '.join(sorted(EXECUTABLES))})")

    scale = float(LaunchConfiguration("tau_scale").perform(context))
    if not (0.0 < scale <= 1.0):
        raise RuntimeError(f"tau_scale debe estar en (0, 1], se dio {scale}")
    tau_max = [scale * t for t in TAU_MAX_NOMINAL]

    params_file = LaunchConfiguration("params_file").perform(context).strip()
    if not params_file:
        params_file = os.path.join(dyn_pkg, "config", PARAMS[controller])
    if not os.path.exists(params_file):
        raise RuntimeError(f"params_file no existe: {params_file}")

    params = [params_file]
    gains_file = LaunchConfiguration("gains_file").perform(context).strip()
    if gains_file:
        if not os.path.exists(gains_file):
            raise RuntimeError(f"gains_file no existe: {gains_file}")
        params.append(gains_file)

    # Overrides que NO son negociables en el robot real.
    real = {
        "use_sim_time": False,
        "gravity_in_command": False,     # G3
        "perform_unpause": False,
        "perform_switch": True,
        "controller_manager": "/controller_manager",
        "activate_controllers": ["forward_effort_controller"],
        "deactivate_controllers": ["scaled_joint_trajectory_controller"],
        "tau_max": tau_max,
    }

    # Overrides opcionales del operador.
    optional = {}
    for arg, key, cast in (("test_num", "test_num", int),
                           ("t_sim", "t_sim", float),
                           ("switching_function", "switching_function", str),
                           ("sweep_joint", "sweep.joint", int),
                           ("trajectory_type", "trajectory_type", str),
                           ("skip_trajectory", "skip_trajectory",
                            lambda v: v.lower() in ("1", "true", "yes"))):
        raw = LaunchConfiguration(arg).perform(context).strip()
        if raw:
            optional[key] = cast(raw)
    for arg, key in (("initial_offset", "initial_offset"),
                     ("q_init", "q_init")):
        raw = LaunchConfiguration(arg).perform(context).strip()
        if raw:
            vals = [float(v) for v in raw.replace(",", " ").split() if v]
            if len(vals) != 6:
                raise RuntimeError(f"{arg} debe tener 6 valores, se dio: {raw!r}")
            optional[key] = vals

    # Banner de seguridad: lo que se va a comandar, ANTES de comandarlo.
    print("\n" + "=" * 72)
    print(f"  UR5e REAL — {controller}")
    print(f"  params : {params_file}" + (f"\n  ganancias: {gains_file}" if gains_file else ""))
    print(f"  tau_max: {['%.1f' % t for t in tau_max]}  "
          f"({100 * scale:.0f} % del nominal)")
    print("  gravity_in_command = FALSE  (G3: el robot compensa g(q) dentro)")
    print("  switch: +forward_effort_controller  "
          "-scaled_joint_trajectory_controller")
    print("  Paro de emergencia a mano. Nadie dentro del espacio de trabajo.")
    print("=" * 72 + "\n")

    return [Node(
        package="ur5_dyn_control",
        executable=EXECUTABLES[controller],
        output="screen",
        emulate_tty=True,
        parameters=params + [real, optional],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "controller", default_value="smc",
            description="gravity_comp | fl | smc"),
        DeclareLaunchArgument(
            "params_file", default_value="",
            description="'' = el YAML por defecto del controlador elegido"),
        DeclareLaunchArgument(
            "tau_scale", default_value="0.30",
            description="fraccion del par nominal del UR5e (checklist §7: "
                        "empezar en 0.30 y subir gradualmente)"),
        DeclareLaunchArgument("test_num", default_value=""),
        # Sin esto, el ensayo de sostén (skip_trajectory:=true) no termina
        # nunca: `t_sim` es lo que acota su duración. Solo tiene efecto con
        # skip_trajectory; una corrida con trayectoria acaba con la tabla.
        DeclareLaunchArgument(
            "t_sim", default_value="",
            description="duracion [s] del ensayo de sosten (con skip_trajectory)"),
        DeclareLaunchArgument(
            "trajectory_type", default_value="",
            description="'' = params_file; joint_sweep para la campana de FASE 2"),
        DeclareLaunchArgument(
            "sweep_joint", default_value="",
            description="junta 0..5 a barrer (con trajectory_type:=joint_sweep)"),
        DeclareLaunchArgument(
            "skip_trajectory", default_value="",
            description="true = quedarse en HOLD (primer ensayo: el brazo solo "
                        "se sostiene, sin moverse)"),
        DeclareLaunchArgument("q_init", default_value=""),
        DeclareLaunchArgument("initial_offset", default_value=""),
        DeclareLaunchArgument(
            "switching_function", default_value="",
            description="SMC: sign | sat"),
        DeclareLaunchArgument(
            "gains_file", default_value="",
            description="YAML de ganancias de run_gain_tuning (FASE 7)"),
        OpaqueFunction(function=launch_setup),
    ])
