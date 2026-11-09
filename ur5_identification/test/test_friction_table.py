"""D33: fricción medida por velocidad (campaña de agosto, G4 a 0.0) para la compensación del nodo."""

import os

import numpy as np
import pytest
import yaml

from ur5_identification.friction_table import CAMPAIGN_G4_0, build_table, interp

_SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_COMMON = os.path.join(_SRC, "ur5_dyn_control", "config", "incision_common.yaml")
_CSV = os.path.expanduser("~/.ros/ur5_dyn_control")
_F_V = np.array([13.64, 11.55, 13.74, 1.28, 1.35, 2.12])
_F_C = np.array([6.85, 6.76, 7.46, 1.78, 1.96, 2.35])


def _friction():
    return next(iter(yaml.safe_load(open(_COMMON)).values()))["ros__parameters"]["friction"]


def _campaign():
    if not all(os.path.exists(os.path.join(_CSV, f"cur_{n}.csv")) for n in CAMPAIGN_G4_0):
        pytest.skip("sin la campaña cur_950-955")
    return build_table(_CSV)


def test_interpolacion_como_el_nodo():
    s, f = [0.02, 0.05, 0.10], [6.0, 7.0, 9.0]
    assert interp(s, f, 0.01) == 6.0 and interp(s, f, -0.035) == pytest.approx(6.5)
    assert interp(s, f, 0.2) == pytest.approx(13.0)


def test_la_tabla_del_yaml_es_la_de_la_campana():
    speeds, table = _campaign()
    fr = _friction()
    np.testing.assert_allclose(fr["table_speeds"], speeds)
    np.testing.assert_allclose(np.reshape(fr["table"], (6, -1)), table, atol=5e-4)
    assert all(isinstance(v, float) for v in fr["table_speeds"]), "ROS 2: sin mezclar enteros"


def test_el_modelo_lineal_sobrecompensa_las_grandes_a_baja_velocidad():
    """Por qué existe (smc_1630): a 0.02 rad/s, ~1 N·m de más en el codo."""
    fr = _friction()
    speeds, table = np.array(fr["table_speeds"]), np.reshape(fr["table"], (6, -1))
    model = _F_C[:, None] + _F_V[:, None] * speeds[None, :]
    assert (model[:3, 0] - table[:3, 0]).min() > 0.7
    assert model[2, 0] - table[2, 0] == pytest.approx(1.24, abs=0.05)
    assert np.abs(model[3:, :] - table[3:, :]).max() < 0.35, "las muñecas, como el modelo"


def test_la_tabla_es_fisica():
    fr = _friction()
    speeds, table = np.array(fr["table_speeds"]), np.reshape(fr["table"], (6, -1))
    assert np.all(np.diff(speeds) > 0) and speeds[0] > 0
    assert np.all(table > 0) and np.all(np.diff(table, axis=1) > 0), "crece con la velocidad"
