"""B1/B2: el CSV de la incision por posicion lo lee el estimador como uno por par."""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from run_position_incision import write_csv, zero_time  # noqa: E402
from ur5_identification.force_estimator import estimate_from_csv  # noqa: E402

try:
    from ur5_trajectory_optimization.gain_tuning.closed_loop import (
        Q_INIT, Plant, default_urdf)
    URDF = default_urdf()
except Exception:  # pragma: no cover
    URDF = ""


def test_el_cero_cae_en_el_reposo_antes_del_contacto():
    phase = np.array(["approach"] * 100 + ["contact"] * 10)
    assert zero_time(0.01, phase) == pytest.approx(1.0 - 0.15)


@pytest.mark.skipif(not os.path.exists(URDF), reason="URDF/evaluador no instalado")
def test_csv_de_posicion_compatible_con_el_estimador(tmp_path):
    plant = Plant(URDF)
    n, dt = 50, 0.002
    q = np.tile(Q_INIT, (n, 1))
    z = np.zeros((n, 6))
    phase = np.array(["approach"] * 20 + ["contact"] * 30)
    plateau = np.zeros(n, int)
    ref = (dt, q, z, z, phase, plateau)
    samples = [(100.0 + k * dt, q[k], z[k], np.ones(6)) for k in range(n)]
    # F/T en tool0_controller: 1 N a lo largo de x de la herramienta.
    ft = [(100.0 + k * dt, np.array([1.0, 0, 0, 0, 0, 0])) for k in range(n)]
    out = tmp_path / "pos_1.csv"
    assert write_csv(str(out), samples, ft, 100.03, ref, {"test_num": 1}, plant)
    r = estimate_from_csv(str(out), plant=plant, gazebo=True)
    assert r["F"].shape == (n, 3)
    # Con la hoja hacia abajo (A4), x de la herramienta = -x de base_link.
    np.testing.assert_allclose(r["F_ft"][:, 0], -1.0, atol=1e-9)
    st = np.genfromtxt(str(out), delimiter=",", names=True, dtype=None,
                       encoding="utf-8", skip_header=1)
    assert list(st["ft_zero"][:15]) == [0] * 15 and st["ft_zero"][-1] == 1
