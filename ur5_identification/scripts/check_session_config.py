#!/usr/bin/env python3
"""
Comprueba, a partir de un CSV de par del UR5e real, dos ajustes del robot que no
se ven en ningun log y que ya han pasado desapercibidos:

1. PAYLOAD DEL PENDANT (docs/02_friction_real.md 8.7). El robot compensa por
   dentro la gravedad del payload configurado (G3). Volvio solo a 1.068 kg al
   recargar el programa y asi estuvo un mes. Se mira en las juntas QUIETAS en
   q_init lo que el robot anade al comando,

       g_robot = cur*k - tau_cmd

   contra lo medido con la brida desnuda y payload 0 (smc_712..718). 1.068 kg en
   la brida mueven shoulder_lift ~5-6 N.m y wrist_1 ~1 N.m.

2. G4, COMPENSACION INTERNA DE FRICCION (docs/00_prereqs.md G4: se opera a 0.0,
   fijado por servicio en cada sesion). Si no se llama al servicio, el robot
   usa sus escalas por defecto y SUMA su propia compensacion a nuestro
   feedforward. Paso el 2026-09-12 (smc_790, 719-723). Se mira en la junta
   BARRIDA la parte IMPAR en la velocidad de g_robot, por diferencia entre
   sentidos a q emparejada: la gravedad (y un payload fantasma) es par en v y
   se cancela; lo que queda es friccion que anade el robot. Medido en el codo:

       G4 = 0        |impar| <= 0.03 N.m   (fl_502, smc_713..718)
       G4 default    6.2 / 7.1 / 8.8 N.m a 0.05 / 0.2 / 0.5 rad/s  (smc_719..723)

Uso:
    check_session_config.py ~/.ros/ur5_dyn_control/smc_<n>.csv
    check_session_config.py --payload-kg 0.19 ~/.ros/ur5_dyn_control/fl_<n>.csv
Sale con codigo 1 si algun ajuste no es el esperado.

PAYLOAD CON EL ACOPLE MONTADO. En el real la gravedad la compensa el ROBOT
(G3): el nodo manda la ley menos su g(q) y el robot suma la suya, con el
payload del pendant. Con el acople montado y payload 0, su peso queda sin
compensar (hasta 1.05 N.m en shoulder_lift). El pendant tiene que llevar el
acople (masa PESADA, CoG del CAD desde tool0); `--payload-kg` declara lo que
se configuro y la referencia pasa a ser la de sin carga MAS la firma de ese
payload en q_init.
"""

import argparse
import os
import sys

import numpy as np

#: k medida por diferencia entre sentidos (docs/02_friction_real.md 8.4) [N.m/A].
#: Una sola fuente: la usa tambien el estimador de fuerza por corriente (B1).
from ur5_identification.force_estimator import K_CURRENT as K  # noqa: E402
Q_INIT = np.array([1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 3.1416])
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]

#: g_robot sin carga en q_init, MEDIDO (2026-09-10, brida desnuda, payload 0).
REF_SIN_CARGA = {1: -19.02, 3: -1.70}
#: Aviso de payload [N.m]. En shoulder_lift la dispersion entre dias es 0.18
#: (-19.02 el 09-10, -18.84 el 09-12) y la firma del acople en q_init -0.88,
#: asi que 0.5 separa "acople configurado" de "payload 0" (la de 1.068 kg, el
#: payload fantasma, es ~5-6). En wrist_1 la del acople es -0.18: ahi solo se
#: ve un payload grande.
UMBRAL_PAYLOAD = {1: 0.5, 3: 0.5}
#: CoG del acople desde tool0 [mm] (CAD, docs/00_assumptions.md A1).
COG_ACOPLE_MM = (-2.227, 2.134, 29.398)
#: F_c fisica medida (02 8.4). El aviso de G4 salta con una parte impar mayor
#: que el 15 % de ella: con G4 = 0 se mide <= 0.03 N.m en el codo, con las
#: escalas por defecto 6.2 N.m ya a 0.05 rad/s.
F_C = np.array([6.85, 6.76, 7.46, 1.78, 1.96, 2.35])
FRAC_G4 = 0.15


def load(path):
    hdr, rows = None, []
    with open(path, errors="replace") as f:
        for line in f:
            if line.startswith("#"):
                continue
            if hdr is None:
                hdr = line.strip().split(",")
                continue
            rows.append(line)
    d = np.genfromtxt(rows, delimiter=",", usecols=range(len(hdr) - 1),
                      invalid_raise=False)
    return d, {n: i for i, n in enumerate(hdr)}


def odd_part(q, v, y, level, tol=0.005, bin_rad=np.radians(1.0)):
    """Media de [y(+v) - y(-v)]/2 a q emparejada en bins de 1 grado."""
    ok = np.isfinite(q) & np.isfinite(v) & np.isfinite(y)
    mp = ok & (np.abs(v - level) < tol)
    mn = ok & (np.abs(v + level) < tol)
    if mp.sum() < 200 or mn.sum() < 200:
        return None
    lo, hi = max(q[mp].min(), q[mn].min()), min(q[mp].max(), q[mn].max())
    a, b = [], []
    for b0 in np.arange(lo, hi - bin_rad, bin_rad):
        s = (q >= b0) & (q < b0 + bin_rad)
        if (s & mp).sum() > 5 and (s & mn).sum() > 5:
            a.append(np.median(y[s & mp]))
            b.append(np.median(y[s & mn]))
    return 0.5 * (np.mean(a) - np.mean(b)) if len(a) > 10 else None


def payload_signature(mass_kg, cog_mm, q=Q_INIT, gravity=9.8):
    """Par de gravedad [N.m] que suma un payload puntual en `cog_mm` (tool0)."""
    if mass_kg == 0.0:
        return np.zeros(6)
    import pinocchio as pin
    from ur5_trajectory_optimization.gain_tuning.closed_loop import default_urdf
    model = pin.buildModelFromUrdf(default_urdf())
    data = model.createData()
    fid = model.getFrameId("tool0")
    pin.framesForwardKinematics(model, data, np.asarray(q, float))
    pin.computeJointJacobians(model, data, np.asarray(q, float))
    J = pin.getFrameJacobian(model, data, fid, pin.LOCAL_WORLD_ALIGNED)
    r = data.oMf[fid].rotation @ (np.asarray(cog_mm, float) * 1e-3)
    # velocidad del punto: v_tool0 + w x r  ->  J_p = J_v - [r]x J_w
    rx = np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]])
    Jp = J[:3] - rx @ J[3:]
    # g(q) del convenio M q'' + C q' + g = tau: el peso es -m g en z.
    return Jp.T @ np.array([0.0, 0.0, mass_kg * gravity])


def check_payload(d, i, payload_kg=0.0, cog_mm=COG_ACOPLE_MM):
    q = d[:, [i[f"q{j}"] for j in range(1, 7)]]
    dq = d[:, [i[f"dq{j}"] for j in range(1, 7)]]
    ok = True
    sig = payload_signature(payload_kg, cog_mm)
    ref = {j: REF_SIN_CARGA[j] + sig[j] for j in REF_SIN_CARGA}
    if payload_kg:
        print(f"  PAYLOAD  declarado {payload_kg:.3f} kg en {cog_mm} mm: "
              f"referencia = sin carga + firma")
    print("  PAYLOAD  junta          g_robot   esperado   diferencia")
    for j in (1, 3):
        quieta = (np.abs(q[:, j] - Q_INIT[j]) < 0.05) & (np.abs(dq[:, j]) < 1e-3)
        if quieta.sum() < 100:
            print(f"           {JOINTS[j]:<14} (se movia: no se evalua)")
            continue
        g = float(np.nanmedian(d[quieta, i[f"cur{j + 1}"]] * K[j]
                               - d[quieta, i[f"tau{j + 1}"]]))
        delta = g - ref[j]
        mal = abs(delta) > UMBRAL_PAYLOAD[j]
        ok &= not mal
        print(f"           {JOINTS[j]:<14} {g:+8.2f}   {ref[j]:+8.2f}   "
              f"{delta:+7.2f}{'  <-- CARGA FANTASMA?' if mal else ''}")
    if not ok:
        print("           -> pendant: Instalacion > Payload. Vuelve solo al "
              "valor por defecto al recargar el programa.")
    return ok


def check_g4(d, i):
    q = d[:, [i[f"q{j}"] for j in range(1, 7)]]
    sw = int(np.argmax(np.nanmax(q, 0) - np.nanmin(q, 0)))
    if f"dq{sw + 1}_des" not in i:
        print("  G4       sin dq_des: no se puede evaluar")
        return True
    v = d[:, i[f"dq{sw + 1}_des"]]
    r = d[:, i[f"cur{sw + 1}"]] * K[sw] - d[:, i[f"tau{sw + 1}"]]
    levels = sorted({round(abs(x), 2) for x in v[np.isfinite(v)] if abs(x) > 0.02})
    umbral = FRAC_G4 * F_C[sw]
    vals = [(lv, odd_part(q[:, sw], v, r, lv)) for lv in levels]
    vals = [(lv, x) for lv, x in vals if x is not None]
    if not vals:
        print(f"  G4       {JOINTS[sw]}: sin mesetas en los dos sentidos, no se evalua")
        return True
    peor = max(abs(x) for _, x in vals)
    mal = peor > umbral
    txt = "  ".join(f"{x:+.2f}@{lv:.2f}" for lv, x in vals)
    print(f"  G4       {JOINTS[sw]} (barrida): friccion que anade el robot "
          f"[N.m @ rad/s]  {txt}")
    print(f"           umbral {umbral:.2f}{'  <-- COMPENSACION INTERNA ACTIVA' if mal else '  -> G4 = 0'}")
    if mal:
        print("           -> no se llamo a set_friction_model_parameters con "
              "escalas 0: nuestro feedforward y el del robot se SUMAN.")
    return not mal


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--payload-kg", type=float, default=0.0,
                    help="payload configurado en el pendant (0 = brida desnuda)")
    ap.add_argument("--payload-cog", type=float, nargs=3, default=COG_ACOPLE_MM,
                    metavar=("X", "Y", "Z"), help="CoG desde tool0 [mm] "
                    "(por defecto, el del acople)")
    a = ap.parse_args(argv)
    rc = 0
    for p in a.csv:
        d, i = load(p)
        print(f"\n=== {os.path.basename(p)} ===")
        if "cur1" not in i:
            print("  sin columnas `cur`: CSV anterior a G5, no se puede comprobar")
            continue
        ok = check_payload(d, i, a.payload_kg, tuple(a.payload_cog))
        ok &= check_g4(d, i)
        rc |= 0 if ok else 1
    return rc


if __name__ == "__main__":
    sys.exit(main())
