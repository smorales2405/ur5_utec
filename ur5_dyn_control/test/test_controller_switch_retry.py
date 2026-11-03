"""
Una respuesta de servicio perdida en el arranque no puede colgar el nodo.

En el robot (2026-11-02) `fl_1111` se quedó para siempre en PRE_HOLD: la respuesta a
`list_controllers` se perdió. Y si se perdiera la del switch, la corrida entera iría con
el JTC mandando y nadie aplicando el par, sin ningún aviso. Se arranca el nodo de verdad
con la configuración del real contra `fake_controller_manager.py`, en un ROS_DOMAIN_ID
propio y con los CSV en un directorio temporal.
"""

import json
import os
import random
import subprocess
import sys
import time

import pytest

_TEST = os.path.dirname(os.path.abspath(__file__))
_CONFIG = os.path.join(os.path.dirname(_TEST), "config")


def _executable(name):
    from ament_index_python.packages import get_package_prefix
    exe = os.path.join(get_package_prefix("ur5_dyn_control"), "lib", "ur5_dyn_control", name)
    if not os.path.exists(exe):
        pytest.fail(f"no esta instalado {exe}: compilar antes de testear")
    return exe


def _run(tmp_path, until, timeout=30.0, **fake):
    """Arranca fake + nodo; espera `until` en el log. (lo vio, log, eventos)."""
    domain = random.choice([d for d in range(150, 200)
                            if str(d) != os.environ.get("ROS_DOMAIN_ID")])
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
    events = tmp_path / "fake.jsonl"
    args = [sys.executable, os.path.join(_TEST, "fake_controller_manager.py"),
            "--domain", str(domain), "--log", str(events)]
    for k, v in fake.items():
        args += ["--" + k.replace("_", "-"), str(v)]
    cm = subprocess.Popen(args, env=env)
    cmd = [_executable("gz_gravity_comp_node"), "--ros-args", "-r",
           "__node:=gz_gravity_comp_node",
           "--params-file", os.path.join(_CONFIG, "gravity_comp_params.yaml")]
    for k, v in [("use_sim_time", "false"), ("perform_switch", "true"),
                 ("perform_unpause", "false"), ("gravity_in_command", "false"),
                 ("pipeline_delay_steps", "5"), ("csv_output_dir", str(tmp_path)),
                 ("test_num", "1"), ("controller_manager", "/controller_manager"),
                 ("activate_controllers", "[forward_effort_controller]"),
                 ("deactivate_controllers", "[scaled_joint_trajectory_controller]")]:
        cmd += ["-p", f"{k}:={v}"]
    log_path = tmp_path / "node.log"
    seen = False
    with open(log_path, "w") as log:
        node = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            t0 = time.time()
            while time.time() - t0 < timeout and node.poll() is None:
                if until in log_path.read_text():
                    seen = True
                    break
                time.sleep(0.2)
        finally:
            for p in (node, cm):
                p.terminate()
                try:
                    p.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    p.kill()
    evs = [json.loads(x) for x in events.read_text().splitlines()] if events.exists() else []
    return seen, log_path.read_text(), evs


def _applied(events):
    return [e for e in events if e["kind"] == "switch" and e.get("applied")]


def test_sin_perdidas_cambia_una_vez_y_arranca(tmp_path):
    seen, log, evs = _run(tmp_path, "Estado -> HOLD_START")
    assert seen, log[-1500:]
    assert len(_applied(evs)) == 1


def test_lista_perdida_se_reintenta(tmp_path):
    """El caso de fl_1111."""
    seen, log, evs = _run(tmp_path, "Estado -> HOLD_START", drop_list=1)
    assert seen, log[-1500:]
    assert len(_applied(evs)) == 1
    assert "sin respuesta" in log


def test_switch_aplicado_con_respuesta_perdida_no_se_repite(tmp_path):
    """Repetirlo como STRICT fallaría: ya está activo. Se vuelve a mirar la lista."""
    seen, log, evs = _run(tmp_path, "Estado -> HOLD_START", drop_switch=1)
    assert seen, log[-1500:]
    assert len(_applied(evs)) == 1
    assert "ya en el estado pedido" in log


def test_switch_perdido_entero_se_reenvia_antes_de_seguir(tmp_path):
    """Sin esto, la corrida iría entera con el JTC mandando."""
    seen, log, evs = _run(tmp_path, "Estado -> HOLD_START", lose_switch=1)
    assert seen, log[-1500:]
    assert len(_applied(evs)) == 1
    assert _applied(evs)[-1]["state"]["forward_effort_controller"] == "active"


def test_sin_respuesta_nunca_se_rinde_sin_comandar(tmp_path):
    seen, log, evs = _run(tmp_path, "Estado -> DONE", timeout=40.0, drop_list=1000)
    assert seen, log[-1500:]
    assert "HOLD_START" not in log
    assert not _applied(evs)


def test_el_launch_no_pisa_una_corrida_con_datos(tmp_path):
    import importlib.util
    path = os.path.join(os.path.dirname(_TEST), "launch", "ur5e_real.launch.py")
    spec = importlib.util.spec_from_file_location("ur5e_real_launch", path)
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    csv = tmp_path / "gravity_comp_1.csv"
    assert not launch.csv_has_data(str(csv))
    csv.write_text("# test_num=1\nt_sim,q1\n")
    assert not launch.csv_has_data(str(csv)), "solo cabecera: nada que perder"
    csv.write_text("# test_num=1\nt_sim,q1\n0.0,1.0\n")
    assert launch.csv_has_data(str(csv))
