"""
Lo que se comprueba antes de pasar a par en el real, y lo que hace SAFE_HOLD.

Paro de `smc_1405` (2026-11-05): con el robot a 0.38 rad de la postura y G4 sin poner a
cero, el nodo desactivó el JTC, entró en SAFE_HOLD y la compensación de fricción, alimentada
con la velocidad medida, se sumó a la del robot. El robot se movió y hubo que pulsar el paro
de emergencia. Y de ese estado no quedó ni una muestra en el CSV.

Se arranca el nodo de verdad con la configuración del real contra
`fake_controller_manager.py`, en un ROS_DOMAIN_ID propio y con los CSV en un directorio temporal.
"""

import ast
import json
import os
import random
import subprocess
import sys
import time

import numpy as np
import pytest

_TEST = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_TEST)
_CONFIG = os.path.join(_PKG, "config")
_CANDIDATA = os.path.join(os.path.dirname(_PKG), "ur5_trajectory_optimization", "results",
                          "gain_tuning", "smc_v7_rotor", "candidata_fila46.yaml")
_SERVICE = "/friction_model_controller/set_friction_model_parameters"
_ROTOR = (1.0324, 1.2466, 1.0545, 0.1247, 0.1386, 0.0719)

Q_INIT = [1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 3.1416]
#: La postura de la rampa, a 0.38 rad de Q_INIT: la de `smc_1405`.
Q_RAMP = [1.5708, -1.194717, 1.574816, -1.950896, -1.5708, 3.1416]
#: `kSafeHoldKd` del nodo.
KD = np.array([10.0, 10.0, 10.0, 1.0, 0.5, 0.2])
_FF = [("friction_compensation", "viscous_coulomb"), ("friction.dq_source", "desired"),
       ("friction.robot_model_service", _SERVICE)]


def _list(values):
    return "[" + ",".join(f"{float(v):.6f}" for v in values) + "]"


def _executable(name):
    from ament_index_python.packages import get_package_prefix
    exe = os.path.join(get_package_prefix("ur5_dyn_control"), "lib", "ur5_dyn_control", name)
    if not os.path.exists(exe):
        pytest.fail(f"no esta instalado {exe}: compilar antes de testear")
    return exe


def _run(tmp_path, until, q_init=Q_INIT, extra=(), exe="gz_gravity_comp_node",
         files=("gravity_comp_params.yaml",), timeout=30.0, after=0.0, **fake):
    """Arranca fake + nodo y espera `until` en el log, y `after` s más. (lo vio, log, eventos)."""
    domain = random.choice([d for d in range(150, 200)
                            if str(d) != os.environ.get("ROS_DOMAIN_ID")])
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
    events = tmp_path / "fake.jsonl"
    args = [sys.executable, os.path.join(_TEST, "fake_controller_manager.py"),
            "--domain", str(domain), "--log", str(events)]
    for k, v in fake.items():
        args += ["--" + k.replace("_", "-"), str(v)]
    cm = subprocess.Popen(args, env=env)
    cmd = [_executable(exe), "--ros-args", "-r", f"__node:={exe}"]
    for f in files:
        cmd += ["--params-file", os.path.join(_CONFIG, f) if "/" not in f else f]
    for k, v in [("use_sim_time", "false"), ("perform_switch", "true"),
                 ("perform_unpause", "false"), ("gravity_in_command", "false"),
                 ("pipeline_delay_steps", "5"), ("csv_output_dir", str(tmp_path)),
                 ("test_num", "1"), ("controller_manager", "/controller_manager"),
                 ("activate_controllers", "[forward_effort_controller]"),
                 ("deactivate_controllers", "[scaled_joint_trajectory_controller]"),
                 ("q_init", _list(q_init)), *extra]:
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
                time.sleep(0.1)
            if seen:
                time.sleep(after)
        finally:
            for p in (node, cm):
                p.terminate()
                try:
                    p.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    p.kill()
    evs = [json.loads(x) for x in events.read_text().splitlines()] if events.exists() else []
    return seen, log_path.read_text(), evs


def _kinds(events, kind):
    return [e for e in events if e["kind"] == kind]


def _order(events):
    """Tipos de evento de los servicios, en el orden en que llegaron."""
    return [e["kind"] for e in events if e["kind"] in ("friction", "switch")]


# ── La postura ────────────────────────────────────────────────────────────────


def test_fuera_de_postura_no_pasa_a_par(tmp_path):
    seen, log, evs = _run(tmp_path, "Estado -> DONE", q_init=Q_RAMP)
    assert seen, log[-1500:]
    assert "NO ARRANCA" in log and "difiere de q_init en 0.38" in log
    assert not _kinds(evs, "switch"), "el JTC tiene que seguir sosteniendo el robot"
    assert "SAFE_HOLD" not in log
    assert not _kinds(evs, "tau_nonzero")


def test_el_caso_de_smc_1405(tmp_path):
    """SMC, candidata, compensación activa, robot en q_init y postura de la rampa."""
    seen, log, evs = _run(tmp_path, "Estado -> DONE", q_init=Q_RAMP, exe="gz_smc_control_node",
                          files=("gain_ramp_params.yaml", _CANDIDATA),
                          extra=[*_FF, ("sweep.q_fixed", _list(Q_RAMP)), ("sweep.joint", "5"),
                                 ("trajectory_type", "joint_sweep"),
                                 ("discrete_check.rotor_inertia", _list(_ROTOR)),
                                 ("discrete_check.margin_min", "2.0")],
                          timeout=60.0, friction="ok")
    assert seen, log[-1500:]
    assert "NO ARRANCA" in log
    assert not _kinds(evs, "switch")
    assert not _kinds(evs, "friction"), "la postura va primero: no se toca nada del robot"
    assert not _kinds(evs, "tau_nonzero")


def test_dentro_de_la_tolerancia_arranca(tmp_path):
    q = list(Q_INIT)
    q[3] += 0.10
    seen, log, evs = _run(tmp_path, "Estado -> HOLD_START", q_init=q)
    assert seen, log[-1500:]
    assert len(_kinds(evs, "switch")) == 1


def test_sin_joint_states_no_pasa_a_par(tmp_path):
    seen, log, evs = _run(tmp_path, "Esperando /joint_states", silent=1)
    assert seen, log[-1500:]
    assert not _kinds(evs, "switch")


# ── G4 ────────────────────────────────────────────────────────────────────────


def test_con_compensacion_pone_g4_a_cero_antes_del_switch(tmp_path):
    seen, log, evs = _run(tmp_path, "Estado -> HOLD_START", extra=_FF, friction="ok")
    assert seen, log[-1500:]
    assert _order(evs) == ["friction", "switch"]
    req = _kinds(evs, "friction")[0]
    assert req["viscous"] == [0.0] * 6 and req["coulomb"] == [0.0] * 6
    assert "G4 a cero, confirmado" in log


def test_g4_rechazada_no_pasa_a_par(tmp_path):
    seen, log, evs = _run(tmp_path, "Estado -> DONE", extra=_FF, friction="reject")
    assert seen, log[-1500:]
    assert "NO ARRANCA" in log and "rechazo" in log
    assert not _kinds(evs, "switch")


def test_g4_sin_servicio_no_pasa_a_par(tmp_path):
    seen, log, evs = _run(tmp_path, "Estado -> DONE", extra=_FF, timeout=30.0)
    assert seen, log[-1500:]
    assert "NO ARRANCA" in log and _SERVICE in log
    assert not _kinds(evs, "switch")


def test_g4_con_respuesta_perdida_se_repite(tmp_path):
    seen, log, evs = _run(tmp_path, "Estado -> HOLD_START", extra=_FF, friction="ok",
                          drop_friction=1)
    assert seen, log[-1500:]
    assert _order(evs) == ["friction", "friction", "switch"]


def test_sin_compensacion_no_se_toca_g4(tmp_path):
    """Las campañas de fricción fijan a propósito otros niveles de G4."""
    seen, log, evs = _run(tmp_path, "Estado -> HOLD_START",
                          extra=[("friction.robot_model_service", _SERVICE)], friction="ok")
    assert seen, log[-1500:]
    assert _order(evs) == ["switch"]


def test_en_seco_no_se_toca_g4(tmp_path):
    seen, log, evs = _run(tmp_path, "Estado -> HOLD_START", extra=[*_FF, ("dry_run", "true")],
                          friction="ok")
    assert seen, log[-1500:]
    assert not _kinds(evs, "friction")


def test_el_launch_real_lo_activa():
    tree = ast.parse(open(os.path.join(_PKG, "launch", "ur5e_real.launch.py")).read())
    real = next(n.value for n in ast.walk(tree) if isinstance(n, ast.Assign)
                and getattr(n.targets[0], "id", "") == "real")
    keys = [ast.literal_eval(k) for k in real.keys]
    values = dict(zip(keys, real.values))
    assert ast.literal_eval(values["friction.robot_model_service"]) == _SERVICE


# ── SAFE_HOLD ─────────────────────────────────────────────────────────────────


def _safe_hold(tmp_path, source):
    """Junta moviéndose a 0.05 rad/s y SAFE_HOLD por /joint_states mudo (estado congelado)."""
    extra = [("friction_compensation", "viscous_coulomb"), ("friction.dq_source", source),
             ("friction.f_v", "[13.64,11.55,13.74,1.28,1.35,2.12]"),
             ("friction.f_c", "[6.85,6.76,7.46,1.78,1.96,2.35]"),
             ("friction.ff_dv_max", "0.01"), ("friction.dq_eps", "0.01")]
    return _run(tmp_path, "Estado -> SAFE_HOLD", extra=extra, after=1.0, dq=0.05,
                stop_state_after=1.0)


@pytest.mark.parametrize("source", ["desired", "measured"])
def test_safe_hold_frena_en_vez_de_empujar(tmp_path, source):
    seen, log, evs = _safe_hold(tmp_path, source)
    assert seen, log[-1500:]
    # El último distinto de cero: al terminar, el nodo publica par cero.
    tau = np.array([e["tau"] for e in _kinds(evs, "tau") if any(e["tau"])][-1])
    # Solo el PD: con la compensación, wrist_3 pedía +2 N·m a favor del movimiento.
    np.testing.assert_allclose(tau, -KD * 0.05, atol=1e-6)


def test_safe_hold_queda_en_el_csv(tmp_path):
    seen, log, _ = _safe_hold(tmp_path, "desired")
    assert seen, log[-1500:]
    csv = tmp_path / "gravity_comp_1.csv"
    rows = [x for x in csv.read_text().splitlines() if x.endswith(",SAFE_HOLD")]
    assert len(rows) > 100, "a 500 Hz durante 1 s"
