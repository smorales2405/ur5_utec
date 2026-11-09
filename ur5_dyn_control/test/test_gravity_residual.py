"""
D32: el nodo resta el exceso de compensación de gravedad del robot, medido en B4.4.

En `smc_1620` la ley tuvo que restar 1–2 N·m en `shoulder_lift` y el codo durante toda la incisión,
y la punta quedó siempre por encima de lo pedido: es lo que B4.4 midió (g_real = (1 − ε) g_UR). El
signo y la cancelación los fija el gtest `GravityResidual`; aquí, el valor, los launches y el nodo.
"""

import os
import sys

import pytest
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "launch"))

from param_layers import GRAVITY_RESIDUAL_B44, override_file  # noqa: E402
from test_param_precedence import (_CANDIDATA, _INCISION, _NODE,  # noqa: E402
                                   _run_launch_setup, _start)


def test_el_valor_es_el_de_b4_4_y_el_del_evaluador():
    problem = pytest.importorskip("ur5_trajectory_optimization.gain_tuning.problem")
    assert list(GRAVITY_RESIDUAL_B44) == list(problem.GRAVITY_ERROR_REAL)


@pytest.mark.skipif(not os.path.exists(_CANDIDATA), reason="sin smc_v7_rotor")
@pytest.mark.parametrize("launch", ["ur5e_real.launch.py", "smc_control.launch.py"])
def test_los_launches_solo_corrigen_si_se_pide(launch):
    base = {"gains_file": _CANDIDATA}
    if launch == "ur5e_real.launch.py":
        base.update(controller="smc", test_num="")
    off = _run_launch_setup(launch, base)
    assert "gravity_residual.eps" not in off[_NODE]["ros__parameters"]
    on = _run_launch_setup(launch, {**base, "gravity_correction": "true"})
    for section in ("/**", _NODE, "/" + _NODE):
        assert on[section]["ros__parameters"]["gravity_residual.eps"] == GRAVITY_RESIDUAL_B44


@pytest.mark.skipif(not os.path.exists(_CANDIDATA), reason="sin smc_v7_rotor")
def test_el_nodo_lo_anuncia(tmp_path):
    rc, log, eff = _start(tmp_path, [*_INCISION, _CANDIDATA, override_file(
        _NODE, {"gravity_residual.eps": GRAVITY_RESIDUAL_B44})], 71)
    assert rc is None, log[-2000:]
    assert "D32: se resta el exceso de compensacion de gravedad" in log
    assert eff["gravity_residual.eps"] == GRAVITY_RESIDUAL_B44


@pytest.mark.skipif(not os.path.exists(_CANDIDATA), reason="sin smc_v7_rotor")
@pytest.mark.parametrize("eps", [[0.0, 0.03, 0.05, 0.04, 0.0], [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]],
                         ids=["cinco_valores", "fuera_de_rango"])
def test_valores_raros_no_arrancan(tmp_path, eps):
    rc, log, _ = _start(tmp_path, [*_INCISION, _CANDIDATA,
                                   override_file(_NODE, {"gravity_residual.eps": eps})], 72)
    assert rc == 1, log[-2000:]
    assert "gravity_residual.eps" in log
