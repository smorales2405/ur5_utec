"""Una corrida que no arranca aborta la sesión en vez de colgarla (fl_1111, 2026-11-02)."""

import io
import os
import subprocess
import sys
import time

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
pytest.importorskip("rclpy")
from check_session_config import (COG_ACOPLE_MM, K, Q_INIT,  # noqa: E402
                                  check_payload, expected_gravity)
from run_friction_campaign_real import follow_run  # noqa: E402
from run_gravity_sweeps_real import is_stub, set_aside  # noqa: E402


def _proc(script):
    return subprocess.Popen(["sh", "-c", script], stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)


def _follow(script, **kw):
    proc = _proc(script)
    try:
        return follow_run(proc, io.StringIO(), settle_s=0.0, **kw)
    finally:
        proc.kill()
        proc.wait()


def test_nodo_callado_no_cuelga():
    t0 = time.time()
    state, completed = _follow("echo arrancando; sleep 60", startup_s=1.0)
    assert (state, completed) == ("?", False)
    assert time.time() - t0 < 5.0


def test_corrida_completa():
    script = 'echo "Estado -> HOLD_START (t)"; echo "Estado -> HOLD_END (t)"; sleep 60'
    assert _follow(script, startup_s=1.0) == ("HOLD_END", True)


def test_done_sin_completar_aborta():
    script = 'echo "Estado -> DONE (t)"; sleep 60'
    assert _follow(script, startup_s=5.0) == ("DONE", False)


def test_safe_hold_aborta():
    script = 'echo "Estado -> HOLD_START (t)"; echo "Estado -> SAFE_HOLD (t)"; sleep 60'
    assert _follow(script, startup_s=5.0) == ("SAFE_HOLD", False)


def test_el_proceso_muere_sin_completar():
    assert _follow('echo "Estado -> TRACK (t)"', startup_s=5.0) == ("TRACK", False)


def test_arrancado_no_le_aplica_el_limite_de_arranque():
    script = 'echo "Estado -> TRACK (t)"; sleep 2; echo "Estado -> HOLD_END (t)"; sleep 60'
    assert _follow(script, startup_s=0.5) == ("HOLD_END", True)


def test_csv_sin_datos_se_aparta_sin_borrarse(tmp_path):
    csv = tmp_path / "fl_1111.csv"
    side = tmp_path / "fl_1111.params.yaml"
    csv.write_text("# test_num=1111\nt_sim,q1\n")
    side.write_text("tool_mass: 0.182\n")
    assert is_stub(str(csv))
    out = set_aside(str(csv))
    assert out.endswith("fl_1111.stub.csv") and os.path.exists(out)
    assert not csv.exists() and not side.exists()
    assert (tmp_path / "fl_1111.stub.params.yaml").exists()
    # El glob del análisis no lo recoge.
    assert not list(tmp_path.glob("fl_11??.csv"))
    csv.write_text("# test_num=1111\nt_sim,q1\n0.0,1.0\n")
    assert not is_stub(str(csv))


# --- Comprobación del payload ------------------------------------------------------

def _hold(q, g_robot):
    """Un sostén: brazo quieto en q, `k*i = g_robot`, tau_cmd = 0."""
    cols = [f"q{j}" for j in range(1, 7)] + [f"dq{j}" for j in range(1, 7)] \
        + [f"cur{j}" for j in range(1, 7)] + [f"tau{j}" for j in range(1, 7)]
    i = {n: k for k, n in enumerate(cols)}
    row = np.r_[q, np.zeros(6), np.asarray(g_robot) / K, np.zeros(6)]
    return np.tile(row, (200, 1)), i


@pytest.mark.parametrize("q", [Q_INIT, Q_INIT + np.array([0, 0.3, -0.2, 0.1, 0, 0])])
def test_payload_bien_puesto_pasa_en_cualquier_postura(q):
    d, i = _hold(q, expected_gravity(q, 0.155, COG_ACOPLE_MM))
    assert check_payload(d, i, payload_kg=0.155)


def test_la_lectura_real_con_el_acople_pasa():
    """gravity_comp_1200: dio una falsa CARGA FANTASMA de 1.29 N.m con la constante."""
    g = np.array([0.0, -21.06, -21.007, -1.977, 0.0, 0.0])
    d, i = _hold(Q_INIT, g)
    assert check_payload(d, i, payload_kg=0.155)


def test_payload_olvidado_en_cero_salta():
    d, i = _hold(Q_INIT, expected_gravity(Q_INIT, 0.0, COG_ACOPLE_MM))
    assert not check_payload(d, i, payload_kg=0.155)


def test_fantasma_de_un_kilo_salta():
    d, i = _hold(Q_INIT, expected_gravity(Q_INIT, 1.068, COG_ACOPLE_MM))
    assert not check_payload(d, i, payload_kg=0.0)


def test_brazo_en_movimiento_no_se_evalua():
    d, i = _hold(Q_INIT, np.zeros(6))
    d[:, i["dq2"]] = 0.05
    assert check_payload(d, i, payload_kg=0.155)
