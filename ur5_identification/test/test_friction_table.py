"""D33: fricción medida por velocidad (campaña de agosto, G4 a 0.0) para la compensación del nodo.
D37: la fila de shoulder_lift, con la carga del corte y hasta 0.005 rad/s (cur_1660)."""

import os

import numpy as np
import pytest
import yaml

from ur5_identification.friction_table import (CAMPAIGN_G4_0, INCISION_LOAD_RUNS,
                                               LIFT_INCISION_LOAD, build_table, interp)

_SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_COMMON = os.path.join(_SRC, "ur5_dyn_control", "config", "incision_common.yaml")
_CSV = os.path.expanduser("~/.ros/ur5_dyn_control")
_F_V = np.array([13.64, 11.55, 13.74, 1.28, 1.35, 2.12])
_F_C = np.array([6.85, 6.76, 7.46, 1.78, 1.96, 2.35])


def _friction():
    return next(iter(yaml.safe_load(open(_COMMON)).values()))["ros__parameters"]["friction"]


def _campaign():
    runs = list(CAMPAIGN_G4_0) + [n for _, n in INCISION_LOAD_RUNS]
    if not all(os.path.exists(os.path.join(_CSV, f"cur_{n}.csv")) for n in runs):
        pytest.skip("sin la campaña cur_950-955 o sin cur_1660")
    return build_table(_CSV)


def _yaml_table():
    fr = _friction()
    return np.array(fr["table_speeds"]), np.reshape(fr["table"], (6, -1))


def test_interpolacion_como_el_nodo():
    s, f = [0.02, 0.05, 0.10], [6.0, 7.0, 9.0]
    assert interp(s, f, 0.01) == 6.0 and interp(s, f, -0.035) == pytest.approx(6.5)
    assert interp(s, f, 0.2) == pytest.approx(13.0)


def test_la_tabla_del_yaml_es_la_medida():
    speeds, table = _campaign()
    fr = _friction()
    np.testing.assert_allclose(fr["table_speeds"], speeds)
    np.testing.assert_allclose(np.reshape(fr["table"], (6, -1)), table, atol=5e-4)
    assert all(isinstance(v, float) for v in fr["table_speeds"]), "ROS 2: sin mezclar enteros"


def test_el_modelo_lineal_sobrecompensa_las_grandes_a_baja_velocidad():
    """Por qué existe (smc_1630): a 0.02 rad/s, ~1 N·m de más en el codo."""
    speeds, table = _yaml_table()
    at = list(speeds).index(0.02)
    model = _F_C[:, None] + _F_V[:, None] * speeds[None, :]
    assert (model[:3, at] - table[:3, at]).min() > 0.5, "lift, 0.53 con la carga del corte (D37)"
    assert model[2, at] - table[2, at] == pytest.approx(1.24, abs=0.05)
    assert np.abs(model[3:, :] - table[3:, :]).max() < 0.35, "las muñecas, como el modelo"


def test_la_tabla_es_fisica():
    speeds, table = _yaml_table()
    assert np.all(np.diff(speeds) > 0) and speeds[0] > 0 and np.all(table > 0)
    fast = speeds >= 0.02
    assert np.all(np.diff(table[:, fast], axis=1) > 0), "desde 0.02 rad/s, crece con la velocidad"
    # Por debajo, casi plana: en lift, lo medido sube y baja 0.23 N·m (cur_1660); se usa tal cual.
    assert np.ptp(table[:, ~fast | (speeds == 0.02)], axis=1).max() < 0.3


def test_lift_con_la_carga_del_corte():
    """D36: cur_1660 frente a agosto. Más fricción a la velocidad del corte; igual a 0.05–0.1."""
    if not os.path.exists(os.path.join(_CSV, "cur_1660.csv")):
        pytest.skip("sin cur_1660")
    speeds, table = build_table(_CSV, extra=LIFT_INCISION_LOAD)
    lift, august = table[1], dict(zip([0.02, 0.05, 0.10], [6.182, 7.136, 8.096]))
    at = lambda v: lift[list(speeds).index(v)]  # noqa: E731
    assert 0.15 < at(0.015) - august[0.02] < 0.3, "a 0.015 rad/s (el corte de smc_1650)"
    assert abs(at(0.05) - august[0.05]) < 0.1 and abs(at(0.10) - august[0.10]) < 0.1


def test_la_incision_usa_la_tabla_de_agosto():
    """D39: con la fila de lift de D37, lift no se despegó al empezar el corte (smc_1670, 1674)."""
    assert INCISION_LOAD_RUNS == ()
    speeds, table = _yaml_table()
    assert list(speeds) == [0.02, 0.05, 0.10, 0.20, 0.35, 0.50, 0.75, 1.00]
    assert table[1, 0] == pytest.approx(6.182)
