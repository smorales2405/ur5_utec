"""
D33: el nodo compensa la fricción con la tabla medida (`friction_compensation: table`).

La tabla vive en `config/incision_common.yaml`; su procedencia la comprueba
`ur5_identification/test/test_friction_table.py`, y la interpolación, el gtest `FrictionTable`.
Aquí, que el nodo la carga y rechaza una mal formada antes de arrancar.
"""

import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "launch"))

from param_layers import override_file  # noqa: E402
from test_param_precedence import _CANDIDATA, _INCISION, _NODE, _start  # noqa: E402

pytestmark = pytest.mark.skipif(not os.path.exists(_CANDIDATA), reason="sin smc_v7_rotor")


def test_la_incision_compensa_con_la_tabla(tmp_path):
    rc, log, eff = _start(tmp_path, [*_INCISION, _CANDIDATA, override_file(
        _NODE, {"friction_compensation": "table", "friction.dq_source": "desired"})], 81)
    assert rc is None, log[-2000:]
    assert "Compensacion de friccion 'table' (D33): medida a 8 velocidades" in log
    assert eff["friction_compensation"] == "table" and len(eff["friction.table"]) == 48


@pytest.mark.parametrize("bad", [{"friction.table": [1.0] * 47},
                                 {"friction.table_speeds": [0.05, 0.02, 0.1, 0.2, 0.35, 0.5,
                                                            0.75, 1.0]}],
                         ids=["filas_incompletas", "velocidades_desordenadas"])
def test_una_tabla_mal_formada_no_arranca(tmp_path, bad):
    rc, log, _ = _start(tmp_path, [*_INCISION, _CANDIDATA, override_file(
        _NODE, {"friction_compensation": "table", **bad})], 82)
    assert rc == 1, log[-2000:]
    assert "friction.table" in log
