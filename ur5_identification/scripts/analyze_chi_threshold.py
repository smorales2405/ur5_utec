"""
Umbral de chattering CON friccion, medido en Gazebo.

Solo se analizan las juntas donde el simulador es fiable: wrist_2 no se mueve en
esta trayectoria y wrist_3 esta congelada por b*dt/I = 11.1 (docs/05_smc.md 7.5).

chi_i se calcula con la MISMA ley que corre en el nodo, evaluada sobre la
trayectoria REAL registrada: el espejo SmcLaw es fiel para K y chi — lo que
quedo sin validar fue el modelo de friccion de la PLANTA, que aqui no se usa.
"""
import sys
import os

import numpy as np

sys.path.insert(0, "/home/utec/ur5_ws/src/ur5_utec/ur5_trajectory_optimization")
from ur5_trajectory_optimization.gain_tuning.closed_loop import (  # noqa: E402
    Plant, SmcLaw)

URDF = "/home/utec/ur5_ws/install/ur5_kinematics/share/ur5_kinematics/ur5e.urdf"
INER = np.array([1.05823, 2.59146, 0.881455, 0.0232406, 0.00535152, 0.00025756])
N = ["pan", "lift", "elbow", "w1", "w2", "w3"]
FIABLES = [0, 1, 2, 3]          # wrist_2 no se mueve, wrist_3 esta congelada
DT = 0.002


def carga(test):
    """
    Devuelve (TRACK completo, MESETA del corte).

    La meseta se deduce de la geometria —donde la punta baja al plano de corte—
    y no de la ventana (15.90, 23.90) que trae `analyze_smc.py`: esa es de la
    trayectoria anterior, y con `surface_z` = 0.03 ya no cae donde debe.
    """
    p = os.path.expanduser(f"~/.ros/ur5_dyn_control/smc_{test}.csv")
    if not os.path.exists(p) or os.path.getsize(p) < 5_000_000:
        return None, None
    nm = sum(1 for l in open(p) if l.startswith("#"))
    d = np.genfromtxt(p, delimiter=",", names=True, dtype=None, encoding="utf-8",
                      skip_header=nm, invalid_raise=False)
    tr = d[np.asarray(d["state"], dtype=str) == "TRACK"]
    z = tr["z_des"]
    return tr, tr[np.isfinite(z) & (z < 0.0255)]


def energia_alta(tau, dt=DT, f_corte=20.0):
    """
    Fraccion de la energia de TAU por encima de f_corte.

    Misma definicion que `analyze_smc.py` de la FASE 5 —FFT de tau con ventana
    de Hanning, no de su derivada— para que los numeros sean comparables con los
    ya publicados. Derivar es un filtro paso-alto: sobre `dtau/dt` una señal
    perfectamente suave con algo de ruido ya da 98 % por encima de 20 Hz, que es
    lo que me salio al medirlo mal.
    """
    x = tau[np.isfinite(tau)]
    if len(x) < 64:
        return np.nan
    x = x - x.mean()
    P = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
    f = np.fft.rfftfreq(len(x), dt)
    tot = P.sum()
    return float(P[f > f_corte].sum() / tot) if tot > 0 else np.nan


def chi_por_junta(d, phi, plant):
    """chi_i = (K_i/phi)*dt/M_ii, con K del propio nodo sobre la traza real."""
    law = SmcLaw(lam=np.full(6, 20.0), eta=INER.copy(), phi=phi, alpha=0.3)
    q = np.column_stack([d[f"q{j+1}"] for j in range(6)])
    dq = np.column_stack([d[f"dq{j+1}"] for j in range(6)])
    qd = np.column_stack([d[f"q{j+1}_des"] for j in range(6)])
    dqd = np.column_stack([d[f"dq{j+1}_des"] for j in range(6)])
    ddqd = np.column_stack([d[f"ddq{j+1}_des"] for j in range(6)])
    ok = np.all(np.isfinite(np.hstack([q, dq, qd, dqd, ddqd])), axis=1)
    idx = np.where(ok)[0][::25]              # 1 de cada 25 basta para el maximo
    chi = np.zeros(6)
    for k in idx:
        _, _, info = law(plant.model, plant.data, q[k], dq[k], qd[k], dqd[k], ddqd[k])
        chi = np.maximum(chi, info["chi"] * DT)
    return chi


def main():
    plant = Plant(URDF)
    phis = [0.20, 0.10, 0.05, 0.03, 0.02, 0.01]
    print(f"{'phi':>6}", end="")
    for j in FIABLES:
        print(f"{N[j]:>26}", end="")
    print()
    print(f"{'':>6}" + "".join(f"{'chi':>8}{'TV(tau)':>10}{'>20Hz':>8}" for _ in FIABLES))
    print("-" * (6 + 26 * len(FIABLES)))
    filas = []
    for i, phi in enumerate(phis):
        d, mes = carga(421 + i)
        if d is None:
            print(f"{phi:6.3f}   (sin datos)")
            continue
        chi = chi_por_junta(d, phi, plant)
        fila = {"phi": phi}
        print(f"{phi:6.3f}", end="")
        for j in FIABLES:
            tau = d[f"tau{j+1}"]
            tv = float(np.nansum(np.abs(np.diff(tau[np.isfinite(tau)]))))
            hf = energia_alta(mes[f"tau{j+1}"])      # espectro SOLO en regimen
            fila[j] = (chi[j], tv, hf)
            print(f"{chi[j]:8.2f}{tv:10.0f}{100*hf:7.1f}%", end="")
        print()
        filas.append(fila)

    print("\n\nTRANSICION por junta (primer phi donde >20Hz supera el 50 %):")
    print("-" * 62)
    for j in FIABLES:
        prev = None
        for f in filas:
            chi, tv, hf = f[j]
            if hf is not None and not np.isnan(hf) and hf > 0.5:
                lo = f"chi entre {prev[0]:.2f} y {chi:.2f}" if prev else f"chi <= {chi:.2f}"
                print(f"  {N[j]:>6}: cruza en phi = {f['phi']:.3f}  ->  {lo}")
                break
            prev = (chi, tv, hf)
        else:
            print(f"  {N[j]:>6}: NO cruza en el rango barrido "
                  f"(chi max = {max(f[j][0] for f in filas):.2f})")


if __name__ == "__main__":
    main()
