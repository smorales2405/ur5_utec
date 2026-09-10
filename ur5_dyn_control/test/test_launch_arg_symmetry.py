"""
Los dos launches del SMC tienen que declarar los MISMOS argumentos de control.

Por que existe este test. Gazebo solo sirve para validar el robot real si puede
reproducir EXACTAMENTE su configuracion. Cada vez que un argumento ha existido
en un launch y no en el otro, la corrida de simulacion lo ha ignorado EN
SILENCIO —sin error, sin aviso— y ha parecido validar algo que no era. Ha
pasado cinco veces:

  - `friction_dq_eps` y `friction_ff_dv_max`: solo en el real.
  - `watchdog_q_err_max`: en ninguno de los dos, siendo un umbral de seguridad.
  - `phi`, `alpha`, `phi_joint`: solo en el de simulacion, asi que la capa
    limite no se podia barrer en el robot.
  - `lambda_joint`: en ninguno, y es la ganancia que entro en ciclo limite en
    smc_712.

La asimetria no se ve leyendo: hay que compararla. Eso es este fichero.
"""

import os
import sys

import pytest

_LAUNCH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "launch")

#: Argumentos que solo tienen sentido en un lado, con el motivo.
SOLO_SIM = {
    "world", "gazebo_gui", "joint_damping", "joint_friction",  # planta de Gazebo
    "params_file", "t_sim", "reference_table_out",
}
SOLO_REAL = {
    "controller", "tool_mounted",   # no existen en simulacion
    "trajectory_type", "skip_trajectory", "q_init", "params_file", "t_sim",
}


def _declared(nombre):
    """Nombres de los DeclareLaunchArgument de un launch, sin ejecutarlo."""
    import ast
    ruta = os.path.join(_LAUNCH, nombre)
    arbol = ast.parse(open(ruta, encoding="utf-8").read())
    args = set()
    for nodo in ast.walk(arbol):
        if (isinstance(nodo, ast.Call) and
                getattr(nodo.func, "id", None) == "DeclareLaunchArgument" and
                nodo.args and isinstance(nodo.args[0], ast.Constant)):
            args.add(nodo.args[0].value)
    assert args, f"{nombre}: no se encontro ningun DeclareLaunchArgument"
    return args


def test_argumentos_de_control_declarados_en_ambos():
    sim = _declared("smc_control.launch.py")
    real = _declared("ur5e_real.launch.py")

    falta_en_real = sim - real - SOLO_SIM
    falta_en_sim = real - sim - SOLO_REAL
    assert not falta_en_real, (
        "declarados en smc_control pero NO en ur5e_real (el robot real no "
        f"podra fijarlos): {sorted(falta_en_real)}")
    assert not falta_en_sim, (
        "declarados en ur5e_real pero NO en smc_control (Gazebo no podra "
        f"reproducir la corrida real): {sorted(falta_en_sim)}")


@pytest.mark.parametrize("arg", [
    "lambda_joint", "phi_joint", "phi", "alpha",
    "watchdog_q_err_max", "watchdog_sat_frac_max", "watchdog_sat_window",
    "friction_dq_eps", "friction_ff_dv_max", "friction_dq_source",
    "tau_scale",
])
def test_argumentos_criticos_presentes(arg):
    """Los que han costado una corrida, fijados por nombre."""
    for nombre in ("smc_control.launch.py", "ur5e_real.launch.py"):
        assert arg in _declared(nombre), f"{arg} no esta en {nombre}"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
