"""
R10: en el robot real el nodo NO arranca si su ley es inestable con el retardo
de tubería medido (5 ciclos, plan V15).

Se arrancan los nodos de verdad con la configuración del real
(`gravity_in_command:=false`, `pipeline_delay_steps:=5`) pero sin robot: el
chequeo corre en el constructor, antes de comandar nada. Fuera de `dry_run`,
en un ROS_DOMAIN_ID propio y con los CSV en un directorio temporal.
"""

import os
import random
import subprocess
import time

import pytest

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_CONFIG = os.path.join(_PKG, "config")
#: Las ganancias de la FASE 7 vigentes, sintonizadas con 1 ciclo de retardo.
_V5_G6 = os.path.join(os.path.dirname(_PKG), "ur5_trajectory_optimization", "results",
                      "gain_tuning", "smc_v5_g6", "selected_gains.yaml")


def _executable(name):
    from ament_index_python.packages import get_package_prefix
    exe = os.path.join(get_package_prefix("ur5_dyn_control"), "lib",
                       "ur5_dyn_control", name)
    if not os.path.exists(exe):
        pytest.fail(f"no esta instalado {exe}: compilar antes de testear")
    return exe


def _start(tmp_path, exe, files, test_num, extra=()):
    """Arranca el nodo como en el real y espera a que salga o a que entre en
    su lazo. Devuelve (codigo de salida o None si sigue vivo, log)."""
    cmd = [_executable(exe), "--ros-args", "-r", f"__node:={exe}"]
    for f in files:
        cmd += ["--params-file", os.path.join(_CONFIG, f) if "/" not in f else f]
    for k, v in [("use_sim_time", "false"), ("perform_switch", "false"),
                 ("perform_unpause", "false"), ("gravity_in_command", "false"),
                 ("pipeline_delay_steps", "5"), ("csv_output_dir", str(tmp_path)),
                 ("test_num", str(test_num)), *extra]:
        cmd += ["-p", f"{k}:={v}"]
    env = dict(os.environ, ROS_DOMAIN_ID=str(random.randint(150, 200)),
               ROS_LOCALHOST_ONLY="1")
    log_path = tmp_path / f"node_{test_num}.log"
    with open(log_path, "w") as log:
        proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            t0 = time.time()
            while time.time() - t0 < 120:
                if proc.poll() is not None:
                    return proc.returncode, log_path.read_text()
                if "Lazo de control" in log_path.read_text():
                    return None, log_path.read_text()
                time.sleep(0.5)
            pytest.fail("el nodo ni arranco ni salio en 120 s")
        finally:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    proc.kill()


@pytest.mark.skipif(not os.path.exists(_V5_G6), reason="sin smc_v5_g6")
def test_smc_v5_g6_no_arranca_en_el_real(tmp_path):
    rc, log = _start(tmp_path, "gz_smc_control_node",
                     ["incision_common.yaml", "smc_params.yaml", _V5_G6], 41)
    assert rc == 1, log[-2000:]
    assert "INESTABLE con 5 ciclos de retardo" in log
    assert "wrist_1_joint" in log


@pytest.mark.skipif(not os.path.exists(_V5_G6), reason="sin smc_v5_g6")
def test_se_puede_forzar_con_el_override(tmp_path):
    rc, log = _start(tmp_path, "gz_smc_control_node",
                     ["incision_common.yaml", "smc_params.yaml", _V5_G6], 42,
                     extra=[("discrete_check.allow_unstable", "true")])
    assert rc is None, log[-2000:]
    assert "INESTABLE con 5 ciclos de retardo" in log


def test_el_ct_nominal_arranca_con_margen(tmp_path):
    rc, log = _start(tmp_path, "gz_fl_control_node",
                     ["incision_common.yaml", "incision_params.yaml"], 43)
    assert rc is None, log[-2000:]
    assert "margen de ganancia x6.95" in log


def test_el_real_y_el_evaluador_usan_el_mismo_retardo():
    """El launch del real fija el retardo del banner; el evaluador de B5 el de la
    simulacion y de g7. Si divergen, se sintoniza para un lazo y se comprueba
    otro."""
    import ast
    from ur5_trajectory_optimization.gain_tuning.closed_loop import (
        PIPELINE_DELAY_STEPS)
    src = open(os.path.join(_PKG, "launch", "ur5e_real.launch.py"), encoding="utf-8").read()
    valores = [n.values[n.keys.index(k)].value
               for n in ast.walk(ast.parse(src)) if isinstance(n, ast.Dict)
               for k in n.keys if isinstance(k, ast.Constant)
               and k.value == "pipeline_delay_steps"]
    assert valores == [PIPELINE_DELAY_STEPS] == [5]
