"""
Capas de parámetros de la incisión — ÚNICO sitio donde vive la regla.

Las tres leyes (CT, LQR-SDRE, SMC) tienen que ejecutar la MISMA tabla de
referencia. Por eso la trayectoria, la herramienta, la fricción y los límites
viven en `config/incision_common.yaml`, y el YAML de cada ley solo lleva sus
ganancias más `trajectory_type: "incision"`. Los launches componen:

    [incision_common.yaml, <ley>_params.yaml, gains_file?, overrides]

El común se antepone SOLO si el YAML de la ley declara `trajectory_type:
incision`. No se carga siempre porque `fl_control_params.yaml` (waypoints),
`gravity_comp_params.yaml` y `sweep_params.yaml` no declaran ese tipo y lo
heredarían: cambiarían de modo en silencio. La decisión se toma leyendo el
fichero, no por su nombre.

Si alguien arranca el nodo sin el común (p. ej. `ros2 run` con un solo
`--params-file`), el nodo se niega: `incision_common` vacío con
`trajectory_type = incision` es un error (torque_control_node_base.cpp).
"""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

#: Bloque común de la incisión, en `config/` del paquete instalado.
COMMON_FILE = "incision_common.yaml"


def common_file() -> str:
    """Ruta absoluta de `incision_common.yaml`, ya instalado."""
    return os.path.join(get_package_share_directory("ur5_dyn_control"),
                        "config", COMMON_FILE)


def declared_trajectory_type(params_file: str):
    """`trajectory_type` que declara un YAML de parámetros de ROS 2, o None.

    Mira todas las secciones de nodo (`<nodo>:` y `/**:`); si dos declaran
    tipos distintos, es un fichero ambiguo y se rechaza.
    """
    with open(params_file, encoding="utf-8") as fh:
        doc = yaml.safe_load(fh) or {}
    found = set()
    for section in doc.values():
        if isinstance(section, dict):
            p = section.get("ros__parameters", {}) or {}
            if "trajectory_type" in p:
                found.add(p["trajectory_type"])
    if len(found) > 1:
        raise RuntimeError(
            f"{params_file}: trajectory_type ambiguo {sorted(found)}")
    return found.pop() if found else None


def incision_layers(params_file: str, common: str | None = None) -> list:
    """Lista de ficheros a pasar a `Node(parameters=...)`, en orden.

    `[common, params_file]` si `params_file` declara una incisión; si no,
    `[params_file]` tal cual (comportamiento histórico).
    """
    if declared_trajectory_type(params_file) != "incision":
        return [params_file]
    common = common or common_file()
    if not os.path.exists(common):
        raise RuntimeError(f"no existe el bloque comun de la incision: {common}")
    return [common, params_file]


#: B0.5: datos de la SESION de corte -> parametro `session.*` del nodo. Van a la
#: cabecera del CSV y al fichero lateral de parametros. Vacio = no aplica.
#: (argumento de launch, parametro, tipo, descripcion)
SESSION_ARGS = [
    ("material", "session.material", str,
     "material de la muestra: pork, eva, ecoflex, pumpkin (plan 3.5)"),
    ("lane", "session.lane", int, "carril de corte en la fijacion indexada"),
    ("surface_z_measured", "session.surface_z_measured", float,
     "superficie medida en el carril con la punta en 3 puntos [m, base_link]"),
    ("minutes_open", "session.minutes_open", float,
     "minutos desde que se abrio la muestra (perecibles)"),
    ("blade_uses", "session.blade_uses", int, "cortes hechos con esta hoja"),
    ("table_clearance", "session.table_clearance", float,
     "holgura mounting base - mesa medida en la sesion [m] (D16: 0.050)"),
]


def session_arguments() -> list:
    """DeclareLaunchArgument de la sesion, los MISMOS en sim y en real (se
    generan aqui para que no puedan divergir entre launches)."""
    return [DeclareLaunchArgument(arg, default_value="", description=desc)
            for arg, _, _, desc in SESSION_ARGS]


def session_overrides(context) -> dict:
    """Parametros `session.*` con los argumentos que se hayan dado."""
    out = {}
    for arg, param, cast, _ in SESSION_ARGS:
        raw = LaunchConfiguration(arg).perform(context).strip()
        if raw:
            value = cast(raw)
            if isinstance(value, float) and not math.isfinite(value):
                raise RuntimeError(f"{arg} no es un numero finito: {raw!r}")
            out[param] = value
    return out

