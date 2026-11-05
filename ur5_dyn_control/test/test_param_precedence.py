"""
`smc_1610` (2026-11-05): las ganancias que pide la sesión son las que corre el nodo.

La incisión en aire corrió con las de la candidata en vez de las pedidas: launch_ros escribía las
sobrescrituras como `/**`, y con `incision_common.yaml` (`/**`) delante, la sección con nombre del
gains_file se aplicaba después y las pisaba (`launch/param_layers.py`). En los barridos el primer
fichero ya tiene nombre y no pasaba.

Se arrancan nodos de verdad, sin robot, en un ROS_DOMAIN_ID propio y con los CSV en un directorio
temporal. Las ganancias efectivas se leen del fichero lateral que el nodo escribe al arrancar, antes
de su lazo. Las sobrescrituras NO van por `-p`: por línea de órdenes ganan siempre, y así es como el
test del chequeo de la rampa (`test_discrete_check.py`) no vio este fallo.
"""

import importlib.util
import os
import random
import subprocess
import sys
import time

import numpy as np
import pytest
import yaml

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_CONFIG = os.path.join(_PKG, "config")
_LAUNCH = os.path.join(_PKG, "launch")
sys.path.insert(0, _LAUNCH)

from param_layers import override_file, with_gain_checks  # noqa: E402

_CANDIDATA = os.path.join(os.path.dirname(_PKG), "ur5_trajectory_optimization", "results",
                          "gain_tuning", "smc_v7_rotor", "candidata_fila46.yaml")
_NODE = "gz_smc_control_node"
_INCISION = ["incision_common.yaml", "smc_params.yaml"]

pytestmark = pytest.mark.skipif(not os.path.exists(_CANDIDATA), reason="sin smc_v7_rotor")


def _candidate():
    return next(iter(yaml.safe_load(open(_CANDIDATA)).values()))["ros__parameters"]


def _requested():
    """La base a la mitad, como pedía `smc_1610` (`--pan-r 0.5`)."""
    g = _candidate()
    lam, phi = list(g["lambda"]), list(g["phi_joint"])
    lam[0], phi[0] = lam[0] * 0.5, phi[0] / 0.5
    return {"lambda": lam, "phi_joint": phi}


def _wildcard_file(tmp_path, params):
    """Lo que launch_ros escribía con un diccionario: todo en `/**`."""
    path = tmp_path / "launch_params_dict.yaml"
    path.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
    return str(path)


def _executable():
    from ament_index_python.packages import get_package_prefix
    exe = os.path.join(get_package_prefix("ur5_dyn_control"), "lib", "ur5_dyn_control", _NODE)
    if not os.path.exists(exe):
        pytest.fail(f"no esta instalado {exe}: compilar antes de testear")
    return exe


def _start(tmp_path, files, test_num):
    """Arranca el nodo y espera a que salga o entre en su lazo. (código o None, log, efectivos)."""
    cmd = [_executable(), "--ros-args", "-r", f"__node:={_NODE}"]
    for f in files:
        cmd += ["--params-file", os.path.join(_CONFIG, f) if "/" not in f else f]
    for k, v in (("use_sim_time", "false"), ("perform_switch", "false"),
                 ("perform_unpause", "false"), ("gravity_in_command", "false"),
                 ("csv_output_dir", str(tmp_path)), ("test_num", str(test_num))):
        cmd += ["-p", f"{k}:={v}"]
    env = dict(os.environ, ROS_DOMAIN_ID=str(random.randint(150, 200)), ROS_LOCALHOST_ONLY="1")
    log_path = tmp_path / f"node_{test_num}.log"
    rc = None
    with open(log_path, "w") as log:
        proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            t0 = time.time()
            while time.time() - t0 < 120:
                if proc.poll() is not None:
                    rc = proc.returncode
                    break
                if "Lazo de control" in log_path.read_text():
                    break
                time.sleep(0.5)
            else:
                pytest.fail("el nodo ni arranco ni salio en 120 s")
        finally:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    proc.kill()
    sidecar = tmp_path / f"smc_{test_num}.params.yaml"
    effective = (next(iter(yaml.safe_load(sidecar.read_text()).values()))["ros__parameters"]
                 if sidecar.exists() else None)
    return rc, log_path.read_text(), effective


def _assert_gains(effective, want):
    for key, values in want.items():
        np.testing.assert_allclose(effective[key], values, rtol=1e-6, err_msg=key)


# ── la causa ────────────────────────────────────────────────────────────────


def test_como_diccionario_la_sobrescritura_pierde_en_la_incision(tmp_path):
    """Lo que pasó en `smc_1610`: corre la candidata, no lo pedido."""
    rc, log, eff = _start(tmp_path, [*_INCISION, _CANDIDATA,
                                     _wildcard_file(tmp_path, _requested())], 61)
    assert rc is None, log[-2000:]
    _assert_gains(eff, {k: _candidate()[k] for k in ("lambda", "phi_joint")})


def test_como_diccionario_si_ganaba_en_el_barrido(tmp_path):
    """Por qué la rampa y el paso 1 de B7.1 sí corrieron con sus ganancias."""
    rc, log, eff = _start(tmp_path, ["gain_ramp_params.yaml", _CANDIDATA,
                                     _wildcard_file(tmp_path, _requested())], 62)
    assert rc is None, log[-2000:]
    _assert_gains(eff, _requested())


# ── el arreglo ──────────────────────────────────────────────────────────────


@pytest.mark.parametrize("layers", [_INCISION, ["gain_ramp_params.yaml"]],
                         ids=["incision", "barrido"])
def test_el_fichero_de_sobrescrituras_gana_siempre(tmp_path, layers):
    rc, log, eff = _start(tmp_path, [*layers, _CANDIDATA,
                                     override_file(_NODE, _requested())], 63)
    assert rc is None, log[-2000:]
    _assert_gains(eff, _requested())


def test_si_no_le_llegan_las_pedidas_no_arranca(tmp_path):
    """La comprobación del nodo, con el fallo de antes reproducido a mano."""
    rc, log, _ = _start(tmp_path, [*_INCISION, _CANDIDATA,
                                   _wildcard_file(tmp_path, with_gain_checks(_requested()))], 64)
    assert rc == 1, log[-2000:]
    assert "NO ARRANCA: `lambda` efectiva" in log
    assert "Lazo de control" not in log


def test_con_las_pedidas_arranca_y_lo_dice(tmp_path):
    rc, log, eff = _start(tmp_path, [*_INCISION, _CANDIDATA,
                                     override_file(_NODE, with_gain_checks(_requested()))], 65)
    assert rc is None, log[-2000:]
    assert "gains_check: `lambda` efectiva = pedida" in log
    assert "gains_check: `phi_joint` efectiva = pedida" in log
    _assert_gains(eff, _requested())


# ── los launches ────────────────────────────────────────────────────────────


class _FakeNode:
    made = []

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        _FakeNode.made.append(self)


def _run_launch_setup(name, args):
    """Ejecuta el `launch_setup` de un launch con sus valores por defecto más `args`, sin lanzar
    nada, y devuelve el último fichero de parámetros del nodo de control."""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"),
                                                  os.path.join(_LAUNCH, name))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ctx = LaunchContext()
    for entity in mod.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.default_value is not None:
            ctx.launch_configurations[entity.name] = perform_substitutions(
                ctx, entity.default_value)
    ctx.launch_configurations.update(args)
    mod.Node = _FakeNode
    _FakeNode.made = []
    mod.launch_setup(ctx)
    node = next(n for n in _FakeNode.made if n.kwargs.get("executable") == _NODE)
    last = node.kwargs["parameters"][-1]
    assert isinstance(last, str), "las sobrescrituras tienen que ir en un fichero"
    return yaml.safe_load(open(last))


@pytest.mark.parametrize("launch", ["ur5e_real.launch.py", "smc_control.launch.py"])
def test_los_launches_del_smc_escriben_y_comprueban_las_ganancias(launch):
    def fmt(values):
        return ",".join(f"{v:.6f}" for v in values)
    want = {k: [round(v, 6) for v in vals] for k, vals in _requested().items()}
    args = {"gains_file": _CANDIDATA, "lambda_joint": fmt(want["lambda"]),
            "phi_joint": fmt(want["phi_joint"])}
    if launch == "ur5e_real.launch.py":
        args.update(controller="smc", test_num="")
    doc = _run_launch_setup(launch, args)
    for section in ("/**", _NODE, "/" + _NODE):
        params = doc[section]["ros__parameters"]
        for key in ("lambda", "phi_joint"):
            np.testing.assert_allclose(params[key], want[key])
            np.testing.assert_allclose(params[f"gains_check.{key}"], want[key])


def test_ningun_launch_pasa_sobrescrituras_como_diccionario():
    for name in ("ur5e_real.launch.py", "smc_control.launch.py", "lqr_sdre_control.launch.py",
                 "fl_control.launch.py"):
        src = open(os.path.join(_LAUNCH, name), encoding="utf-8").read()
        assert "override_file(" in src, name
        assert "session_overrides(context)," not in src and "optional, session" not in src, name
