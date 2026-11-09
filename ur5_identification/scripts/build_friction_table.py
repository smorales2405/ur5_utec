#!/usr/bin/env python3
"""
D33: tabla de fricción medida por velocidad, desde la campaña de agosto (G4 a 0.0, cur_950–955).
D37: la fila de shoulder_lift, desde `cur_1660` (con la carga del corte, hasta 0.005 rad/s).

Imprime el bloque YAML para `friction:` de `ur5_dyn_control/config/incision_common.yaml`, y con
`--check` compara el que hay en ese fichero con el recalculado (código 1 si no coinciden).
Método y salvedades: `ur5_identification/friction_table.py`.

    build_friction_table.py
    build_friction_table.py --check <ruta>/incision_common.yaml
"""

import argparse
import os
import sys

import numpy as np
import yaml

from ur5_identification.friction_table import build_table


def yaml_block(speeds, table) -> str:
    rows = ",\n".join("        " + ", ".join(f"{v:.3f}" for v in row) for row in table)
    # Siempre con decimales: ROS 2 rechaza una lista que mezcle enteros y reales («1» y «0.75»).
    return ("      table_speeds: [" + ", ".join(f"{v:.3f}" for v in speeds) + "]\n"
            "      table: [\n" + rows + "]\n")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--csv-dir", default=os.path.expanduser("~/.ros/ur5_dyn_control"))
    ap.add_argument("--check", default=None, help="incision_common.yaml a comparar")
    args = ap.parse_args(argv)
    speeds, table = build_table(args.csv_dir)
    if args.check is None:
        print(yaml_block(speeds, table), end="")
        return 0
    fr = next(iter(yaml.safe_load(open(args.check)).values()))["ros__parameters"]["friction"]
    ok = (np.allclose(fr["table_speeds"], speeds) and
          np.allclose(np.reshape(fr["table"], (6, -1)), table, atol=5e-4))
    print("coincide con lo medido" if ok else "NO coincide con lo medido")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
