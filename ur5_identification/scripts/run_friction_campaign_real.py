#!/usr/bin/env python3
"""
Campaña de fricción en el UR5e REAL — SEMIAUTOMÁTICA (FASE 2).

18 corridas: 6 juntas x 3 niveles de compensación interna de fricción. El script
las prepara, las comprueba y las encadena, pero **no comanda par sin que el
operador lo confirme escribiendo**, corrida por corrida.

Por qué semiautomática y no automática
--------------------------------------
Cada corrida mueve una junta ±45° a hasta 1 rad/s con control por par. Encadenar
18 de esas sin supervisión es exactamente lo que no hay que hacer: entre corrida
y corrida hay que mirar dónde quedó el brazo, y una condición de arranque mala
(robot lejos de q_init, controlador equivocado activo) se convierte en un
movimiento largo no planificado. Este script hace las comprobaciones que un
humano olvidaría a la corrida 12, y deja la decisión de comandar en el humano.

Confirmación
------------
Hay que teclear el NOMBRE DE LA JUNTA, no pulsar Enter. Es deliberado: Enter se
pulsa por inercia, un nombre hay que leerlo.

Antes de CADA corrida se comprueba
----------------------------------
  - driver vivo y `/joint_states` llegando
  - `forward_effort_controller` cargado
  - robot dentro de la tolerancia de q_init  (si no, el nodo pararía en
    SAFE_HOLD nada más arrancar: mejor detectarlo aquí y decir cómo arreglarlo)

Y se aborta la campaña —no solo la corrida— si el nodo entra en SAFE_HOLD.

Trazabilidad (compuerta G4)
---------------------------
Las escalas de fricción interna se fijan por servicio ANTES de cada nivel y se
verifica `success`. Sin esa llamada el controlador no impone nada y el valor
efectivo no queda registrado, así que la campaña no sería reproducible. Todo lo
que se fija y lo que se ejecuta va a un YAML de sesión.

Uso
---
    # 1) Driver en otra terminal, programa External Control en marcha
    # 2) Firmar el checklist §7 del plan
    ros2 run ur5_identification run_friction_campaign_real.py --test-base 900

    # Reanudar tras una interrupción (salta las corridas cuyo CSV ya existe)
    ros2 run ur5_identification run_friction_campaign_real.py --test-base 900 --resume
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

import rclpy
import yaml
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from ur_msgs.srv import SetFrictionModelParameters

from ur5_identification.campaign_levels import (FRICTION_SRV, JOINT_NAMES,
                                                 LEVELS)



# ─────────────────────────────────────────────────────────────────────────────
# Comprobaciones contra el sistema vivo
# ─────────────────────────────────────────────────────────────────────────────

class Checker(Node):
    """Lee el estado real por rclpy, no parseando la salida de la CLI.

    `ros2 control list_controllers` emite códigos ANSI y un grep de " active"
    NO casa (hay un ESC entre el espacio y la palabra); ya costó un diagnóstico
    equivocado en este proyecto. Aquí se consulta el servicio directamente.
    """

    def __init__(self):
        super().__init__("friction_campaign_checker")
        self._q = None
        self._t = 0.0
        self.create_subscription(JointState, "/joint_states", self._cb, 10)
        self._fric = self.create_client(SetFrictionModelParameters, FRICTION_SRV)
        self._switch = self.create_client(SwitchController,
                                          "/controller_manager/switch_controller")
        self._jtc = self.create_publisher(
            JointTrajectory,
            "/scaled_joint_trajectory_controller/joint_trajectory", 10)

    def _cb(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        if all(n in idx for n in JOINT_NAMES):
            self._q = [msg.position[idx[n]] for n in JOINT_NAMES]
            self._t = time.time()

    def joint_state(self, timeout: float = 3.0):
        """q actual, o None si /joint_states no llega."""
        self._q = None
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._q is not None:
                return list(self._q)
        return None

    def set_friction(self, viscous, coulomb, timeout: float = 10.0):
        """Fija las escalas y devuelve (ok, mensaje). G4: se VERIFICA."""
        if not self._fric.wait_for_service(timeout_sec=timeout):
            return False, f"servicio {FRICTION_SRV} no disponible"
        req = SetFrictionModelParameters.Request()
        req.parameters.viscous_scale = [float(v) for v in viscous]
        req.parameters.coulomb_scale = [float(c) for c in coulomb]
        fut = self._fric.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            return False, "sin respuesta del servicio"
        res = fut.result()
        ok = bool(getattr(res, "success", True))
        return ok, str(getattr(res, "message", "")) or ("ok" if ok else "rechazado")


    def release_effort(self, timeout: float = 10.0):
        """
        Devuelve el mando al controlador de trayectoria entre corridas.

        Dejar `forward_effort_controller` activo sin nadie publicando es lo que
        hizo que shoulder_pan girase sola 4.7 rad entre dos corridas: el
        controlador conserva el ultimo comando y el driver lo sigue aplicando.
        El nodo ya publica par cero al morir; esto es el cinturon, por si el
        nodo muere de forma que no ejecute su destructor.
        """
        if not self._switch.wait_for_service(timeout_sec=timeout):
            return False, "servicio switch_controller no disponible"
        req = SwitchController.Request()
        req.deactivate_controllers = ["forward_effort_controller"]
        req.activate_controllers = ["scaled_joint_trajectory_controller"]
        req.strictness = SwitchController.Request.BEST_EFFORT
        fut = self._switch.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            return False, "sin respuesta"
        return bool(fut.result().ok), "ok" if fut.result().ok else "rechazado"

    def wait_until_still(self, tol: float = 0.01, samples: int = 5,
                         timeout: float = 15.0):
        """
        Espera a que el brazo deje de moverse. Devuelve (quieto, desplazamiento).

        Sin esto, el preflight puede leer una posicion que ya no es valida un
        segundo despues: fue exactamente el modo de fallo (el preflight leyo
        1.498 y el nodo, 1.8 s despues, 6.204).
        """
        t0 = time.time()
        prev, still = None, 0
        worst = 0.0
        while time.time() - t0 < timeout:
            q = self.joint_state(timeout=2.0)
            if q is None:
                return False, float("inf")
            if prev is not None:
                d = max(abs(a - b) for a, b in zip(q, prev))
                worst = max(worst, d)
                still = still + 1 if d < tol else 0
                if still >= samples:
                    return True, d
            prev = q
            time.sleep(0.2)
        return False, worst

    def go_home(self, q_init, speed: float = 0.25, settle: float = 2.0):
        """
        Lleva el robot a q_init con el JTC (control de POSICION, no de par).

        Hace falta porque las juntas NO barridas se mueven durante el barrido:
        el par de acoplamiento las arrastra y el FL no puede retenerlas — su
        autoridad es `M_jj·kp_j` y en las muñecas eso vale 2.32 N·m/rad. Medido
        barriendo el elbow a 1.0 rad/s, wrist_1 recorrió 15° con la compensación
        interna a 0.0 y **114°** con la compensación en `default`: el robot le
        quita a la muñeca la fricción que la sujetaba.

        El resultado es que la corrida siguiente arranca fuera de tolerancia y
        hay que recolocar a mano. Esto lo automatiza, pero con el JTC y despacio,
        que es un movimiento acotado y de posición — no control por par.
        """
        q = self.joint_state()
        if q is None:
            return False, "sin /joint_states"
        err = max(abs(a - b) for a, b in zip(q, q_init))
        if err < 1e-3:
            return True, "ya en q_init"
        dur = max(3.0, err / max(speed, 1e-6))

        msg = JointTrajectory()
        msg.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q_init]
        pt.velocities = [0.0] * 6
        pt.time_from_start = Duration(sec=int(dur), nanosec=int((dur % 1) * 1e9))
        msg.points = [pt]
        self._jtc.publish(msg)

        t0 = time.time()
        while time.time() - t0 < dur + settle:
            rclpy.spin_once(self, timeout_sec=0.1)
        q = self.joint_state()
        if q is None:
            return False, "sin /joint_states tras el movimiento"
        err = max(abs(a - b) for a, b in zip(q, q_init))
        return err < 0.05, f"error final {err:.4f} rad"




def preflight(chk: Checker, q_init, tol: float):
    """Devuelve (ok, lista_de_problemas). No lanza nada; solo informa."""
    problems = []
    q = chk.joint_state()
    if q is None:
        problems.append("no llega /joint_states — ¿está el driver arriba y el "
                        "programa External Control en marcha?")
        return False, problems, None

    err = max(abs(a - b) for a, b in zip(q, q_init))
    if err > tol:
        worst = max(range(6), key=lambda i: abs(q[i] - q_init[i]))
        problems.append(
            f"el robot está a {err:.3f} rad de q_init (tolerancia {tol:.3f}); "
            f"la peor es {JOINT_NAMES[worst]} "
            f"({q[worst]:+.3f} vs {q_init[worst]:+.3f}).\n"
            f"      El nodo pararía en SAFE_HOLD nada más arrancar. Lleve el "
            f"robot a q_init con el pendant o con el JTC antes de continuar.")
    return (not problems), problems, q


# ─────────────────────────────────────────────────────────────────────────────
# Confirmación y ejecución
# ─────────────────────────────────────────────────────────────────────────────


def parse_skip(specs):
    """
    `--skip nivel:junta` -> conjunto de pares (nivel, indice_de_junta).

    La junta admite índice (0-5) o nombre (`wrist_1_joint`, o `wrist_1`), y `*`
    vale como comodín en cualquiera de los dos lados. Ejemplos:

        --skip 0.0:wrist_1 0.0:wrist_2 0.0:wrist_3   las muñecas sin compensar
        --skip 0.0:*                                 el nivel 0.0 entero
        --skip *:5                                   wrist_3 en todos los niveles

    Existe porque hay combinaciones que NO son alcanzables, no que no interesen:
    con la compensación interna a 0.0 la ley FL entrega `M_jj·kp_j·e` de par por
    error, y en las muñecas eso no llega a vencer la fricción estática — wrist_3
    da 0.02 N·m con 44° de error. Saltarlas a mano y dejar constancia es más
    honesto que bajar la tolerancia hasta que "pase".
    """
    out = set()
    for spec in specs or []:
        if ":" not in spec:
            raise SystemExit(f"--skip: formato esperado nivel:junta, se dio {spec!r}")
        lv, jt = spec.split(":", 1)
        levels = list(LEVELS) if lv.strip() == "*" else [lv.strip()]
        for level in levels:
            if level not in LEVELS:
                raise SystemExit(f"--skip: nivel desconocido {level!r} "
                                 f"(válidos: {', '.join(LEVELS)})")
        if jt.strip() == "*":
            joints = list(range(6))
        else:
            key = jt.strip()
            if key.isdigit():
                joints = [int(key)]
            else:
                cand = [i for i, n in enumerate(JOINT_NAMES)
                        if n == key or n == key + "_joint"]
                if not cand:
                    raise SystemExit(f"--skip: junta desconocida {key!r}")
                joints = cand
        for level in levels:
            for j in joints:
                if not 0 <= j <= 5:
                    raise SystemExit(f"--skip: junta fuera de rango: {j}")
                out.add((level, j))
    return out


def confirm(word: str) -> bool:
    """Confirmación por PALABRA, no por Enter: Enter se pulsa por inercia."""
    try:
        got = input(f"    Escriba «{word}» para comandar par (o 'saltar'): ").strip()
    except (EOFError, KeyboardInterrupt):
        return False
    return got == word



def run_one_position(joint: int, test_num: int, params_file: str, level: str,
                     log_dir: str, out_dir: str, k_known=None,
                     timeout_s: float = 900.0):
    """
    Corrida por CONTROL DE POSICIÓN: barrido + calibración corriente→par.

    Se invoca a los scripts en vez de duplicar su lógica, y con `--yes` porque
    el operador ya confirmó en el runner: pedir dos veces la misma confirmación
    entrena a confirmar sin leer, que es lo contrario de lo que se busca.
    """
    log_path = os.path.join(log_dir, f"run_{test_num}.log")
    cur_csv = os.path.join(out_dir, f"cur_{test_num}.csv")
    tau_csv = os.path.join(out_dir, f"fl_{test_num}.csv")
    res = {"test_num": test_num, "joint": joint, "method": "position",
           "joint_name": JOINT_NAMES[joint], "level": level,
           "csv": tau_csv, "cur_csv": cur_csv, "node_log": log_path}

    sweep = ["ros2", "run", "ur5_identification", "run_current_sweep.py",
             "--joint", str(joint), "--test-num", str(test_num),
             "--params-file", params_file, "--friction-level", level,
             "--out-dir", out_dir, "--yes"]
    print(f"    $ {' '.join(sweep)}")
    with open(log_path, "w") as log:
        rc = subprocess.run(sweep, stdout=log, stderr=subprocess.STDOUT,
                            timeout=timeout_s).returncode
    res["sweep_rc"] = rc
    if rc != 0 or not os.path.exists(cur_csv):
        print(f"    barrido FALLÓ (rc={rc}); ver {log_path}")
        res["csv_exists"] = False
        return res

    cal = ["ros2", "run", "ur5_identification", "calibrate_current.py",
           "--csv", cur_csv, "--joint", str(joint), "--out", tau_csv]
    # `k` conocida de la calibración multipostura. Sin ella, las juntas SIN par
    # gravitatorio en q_init —wrist_2 a -90 grados y wrist_3 siempre— no pueden
    # convertir la corriente a par y la corrida se queda a medias.
    if k_known is not None:
        cal += ["--k", f"{k_known:.6f}"]
        res["k_supplied"] = float(k_known)
    with open(log_path, "a") as log:
        log.write("\n=== calibracion ===\n")
        rc = subprocess.run(cal, stdout=log, stderr=subprocess.STDOUT,
                            timeout=300).returncode
    res["calib_rc"] = rc
    res["csv_exists"] = os.path.exists(tau_csv)
    # `k` y su residuo salen de la cabecera del CSV convertido: son el
    # indicador de si la conversion corriente->par es fiable en esa junta.
    if res["csv_exists"]:
        with open(tau_csv) as fh:
            for line in fh:
                if not line.startswith("#"):
                    break
                if "k_current_to_torque=" in line:
                    res["k"] = float(line.split("=")[1])
                if "k_residuo_relativo=" in line:
                    res["k_residual"] = float(line.split("=")[1])
        print(f"    k = {res.get('k', float('nan')):.4f} N·m/A   "
              f"residuo {100 * res.get('k_residual', float('nan')):.3f} %")
    return res


def run_one(joint: int, test_num: int, params_file: str, tau_scale: float,
            log_dir: str, tool_mounted: bool = False, settle_s: float = 8.0,
            timeout_s: float = 900.0):
    """
    Lanza UNA corrida y espera a que la trayectoria acabe.

    Se detecta el final leyendo el log del nodo (`Estado -> HOLD_END`) en vez de
    esperar un tiempo fijo: la duración de la tabla depende de las velocidades
    del barrido, y un timeout a ojo o corta la corrida o alarga la sesión.
    Si aparece SAFE_HOLD se corta de inmediato y se aborta la campaña.
    """
    cmd = ["ros2", "launch", "ur5_dyn_control", "ur5e_real.launch.py",
           "controller:=fl", f"params_file:={params_file}",
           "trajectory_type:=joint_sweep", f"sweep_joint:={joint}",
           # La campaña se corre con el efector DESCARGADO (§7). Declararlo
           # aquí evita la herramienta fantasma en el modelo del nodo.
           f"tool_mounted:={'true' if tool_mounted else 'false'}",
           f"tau_scale:={tau_scale}", f"test_num:={test_num}"]
    log_path = os.path.join(log_dir, f"run_{test_num}.log")
    print(f"    $ {' '.join(cmd)}")

    with open(log_path, "w") as log:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True,
                                preexec_fn=os.setsid)
        state, t0 = "?", time.time()
        try:
            for line in proc.stdout:
                log.write(line)
                log.flush()
                if "Estado ->" in line:
                    state = line.split("Estado ->")[1].split("(")[0].strip()
                    print(f"      [{time.time() - t0:6.1f}s] {state}")
                if "SAFE_HOLD" in line:
                    print("    *** SAFE_HOLD: el nodo pidió parada segura ***")
                    break
                if "HOLD_END" in line:
                    # NO se mata el nodo aqui. En HOLD_END el controlador regula
                    # en el ultimo punto de la tabla (= q_center), y matarlo en
                    # ese instante deja el brazo con el error de seguimiento que
                    # tuviera: medido, el elbow quedo 0.215 rad fuera y la
                    # corrida siguiente no pudo arrancar. Se le da tiempo a
                    # converger, acotado, porque una junta que no puede vencer
                    # su friccion no convergeria nunca.
                    print(f"      trayectoria completada; asentando {settle_s:.0f} s")
                    time.sleep(settle_s)
                    break
                if time.time() - t0 > timeout_s:
                    print(f"    *** timeout de {timeout_s:.0f} s ***")
                    break
        finally:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=15)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass

    csv = os.path.expanduser(f"~/.ros/ur5_dyn_control/fl_{test_num}.csv")
    return {"test_num": test_num, "joint": joint,
            "joint_name": JOINT_NAMES[joint], "state_reached": state,
            "csv": csv, "csv_exists": os.path.exists(csv),
            "node_log": log_path, "safe_hold": state == "SAFE_HOLD"}


# ─────────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--joints", type=int, nargs="+", default=list(range(6)))
    ap.add_argument("--levels", nargs="+", default=list(LEVELS),
                    choices=list(LEVELS))
    ap.add_argument("--test-base", type=int, default=900)
    ap.add_argument("--tau-scale", type=float, default=0.30,
                    help="fracción del par nominal (checklist §7: empezar en 0.30)")
    ap.add_argument("--params-file", default=None)
    ap.add_argument("--tool-mounted", action="store_true",
                    help="el acople del bisturi ESTA montado (por defecto no, §7)")
    ap.add_argument("--k", nargs="+", default=None, metavar="JUNTA:VALOR",
                    help="constantes corriente->par conocidas, de "
                         "calibrate_multipose.py. Imprescindible en las juntas "
                         "sin par gravitatorio en q_init (wrist_2, wrist_3). "
                         "Ej: --k 3:9.7582 4:11.6792 5:11.6792")
    ap.add_argument("--method", choices=["position", "torque"], default="position",
                    help="position (default): barrido por el JTC leyendo "
                         "corriente; funciona en las SEIS juntas y salió mas "
                         "preciso. torque: la via original por control de par, "
                         "que no puede mover las muñecas")
    ap.add_argument("--home-between-runs", action="store_true",
                    help="tras cada corrida, volver a q_init con el JTC "
                         "(movimiento de POSICION, lento y acotado)")
    ap.add_argument("--skip", nargs="+", default=None, metavar="NIVEL:JUNTA",
                    help="combinaciones a saltar; admite nombre de junta y '*' como comodin. Ej: --skip 0.0:wrist_1 0.0:wrist_2 0.0:wrist_3")
    ap.add_argument("--resume", action="store_true",
                    help="salta las corridas cuyo CSV ya existe")
    ap.add_argument("--log-dir", default=os.path.expanduser("~/.ros/friction_campaign"))
    args = ap.parse_args()

    skip = parse_skip(args.skip)
    k_map = {}
    for spec in args.k or []:
        if ":" not in spec:
            raise SystemExit(f"--k: formato esperado junta:valor, se dio {spec!r}")
        a, b = spec.split(":", 1)
        k_map[int(a)] = float(b)
    os.makedirs(args.log_dir, exist_ok=True)
    if args.params_file is None:
        from ament_index_python.packages import get_package_share_directory
        args.params_file = os.path.join(
            get_package_share_directory("ur5_dyn_control"),
            "config", "sweep_params.yaml")
    cfg = yaml.safe_load(open(args.params_file))
    root = list(cfg)[0]
    rp = cfg[root]["ros__parameters"]
    q_init = rp["q_init"]
    tol = rp.get("q_init_check_tol", 0.15)

    print("=" * 74)
    print("  CAMPAÑA DE FRICCIÓN — UR5e REAL")
    n_total = len(args.joints) * len(args.levels)
    n_skip = sum(1 for lv in args.levels for j in args.joints if (lv, j) in skip)
    print(f"  {len(args.joints)} juntas x {len(args.levels)} niveles = "
          f"{n_total} corridas" + (f"  ({n_skip} saltadas por --skip)" if n_skip else ""))
    print(f"  tau_scale = {args.tau_scale}  ({100 * args.tau_scale:.0f} % del nominal)")
    print(f"  q_init    = {[round(v, 4) for v in q_init]}  (tol {tol})")
    print("=" * 74)
    print("\n  Checklist §7 del plan — confirme ANTES de continuar:")
    for item in ("velocidad reducida activa en el teach pendant",
                 "paro de emergencia al alcance de la mano",
                 "nadie dentro del espacio de trabajo",
                 "primer ensayo SIN bisturí montado",
                 "planos de seguridad configurados alrededor de la mesa"):
        print(f"    [ ] {item}")
    try:
        if input("\n  Escriba «ACEPTO» para iniciar la sesión: ").strip() != "ACEPTO":
            print("  Cancelado."); return 1
    except (EOFError, KeyboardInterrupt):
        print("\n  Cancelado."); return 1

    rclpy.init()
    chk = Checker()
    session = {"started": datetime.now().isoformat(timespec="seconds"),
               "method": args.method,
               "k_supplied": {str(a): b for a, b in k_map.items()},
               "tool_mounted": bool(args.tool_mounted),
               "tau_scale": args.tau_scale, "params_file": args.params_file,
               "q_init": q_init, "levels": {}, "runs": [],
               # Qué se saltó y por orden de quién: sin esto, una campaña
               # incompleta es indistinguible de una que falló a medias.
               "skipped": [], "skip_spec": list(args.skip or [])}
    rc = 0
    try:
        for level in args.levels:
            visc, coul = LEVELS[level]
            print(f"\n{'=' * 74}\n  NIVEL «{level}»  viscous={visc[0]} coulomb={coul[0]}"
                  f"\n{'=' * 74}")
            ok, msg = chk.set_friction(visc, coul)
            print(f"  escalas de fricción interna: {'OK' if ok else 'FALLÓ'} — {msg}")
            session["levels"][level] = {"viscous_scale": visc,
                                        "coulomb_scale": coul,
                                        "service_ok": ok, "message": msg}
            if not ok:
                print("  Sin poder fijar las escalas la campaña NO es reproducible "
                      "(G4). Se aborta este nivel.")
                rc = 1
                continue

            for j in args.joints:
                test_num = args.test_base + list(LEVELS).index(level) * 10 + j
                csv = os.path.expanduser(f"~/.ros/ur5_dyn_control/fl_{test_num}.csv")
                head = f"[nivel {level} · junta {j} · {JOINT_NAMES[j]} · test {test_num}]"
                print(f"\n  {head}")
                if (level, j) in skip:
                    print("    SALTADA por --skip")
                    session["skipped"].append(
                        {"level": level, "joint": j,
                         "joint_name": JOINT_NAMES[j], "test_num": test_num})
                    continue
                if args.resume and os.path.exists(csv):
                    print("    ya existe el CSV — se salta (--resume)")
                    continue

                good, problems, q = preflight(chk, q_init, tol)
                if q is not None:
                    print(f"    q actual = {[round(v, 3) for v in q]}")
                for p in problems:
                    print(f"    !! {p}")
                if not good:
                    print("    Corrida NO lanzada. Corrija y vuelva a ejecutar "
                          "con --resume.")
                    rc = 1
                    continue

                amp = rp["sweep"]["amplitude"]
                print(f"    Va a mover {JOINT_NAMES[j]} ±{amp:.3f} rad "
                      f"(±{amp * 57.2958:.0f}°) a {len(rp['sweep']['velocities'])} "
                      f"velocidades, hasta {max(rp['sweep']['velocities'])} rad/s.")
                if not confirm(JOINT_NAMES[j]):
                    print("    Saltada por el operador.")
                    continue

                if args.method == "position":
                    res = run_one_position(
                        j, test_num, args.params_file, level, args.log_dir,
                        os.path.expanduser("~/.ros/ur5_dyn_control"),
                        k_known=k_map.get(j))
                else:
                    res = run_one(j, test_num, args.params_file, args.tau_scale,
                                  args.log_dir, args.tool_mounted)
                res["level"] = level
                # Soltar el mando ANTES de leer nada: si el efector sigue
                # comandado, la lectura de posicion no vale para nada.
                ok_rel, msg_rel = chk.release_effort()
                res["effort_released"] = ok_rel
                print(f"    control devuelto al JTC: {'OK' if ok_rel else 'FALLO'} — {msg_rel}")
                if args.home_between_runs:
                    ok_home, msg_home = chk.go_home(q_init)
                    res["homed"] = ok_home
                    print(f"    vuelta a q_init: {'OK' if ok_home else 'FALLO'} — {msg_home}")
                still, drift = chk.wait_until_still()
                res["settled"] = still
                res["drift_after"] = drift
                if not still:
                    print(f"    !! el brazo NO se detiene (desplazamiento {drift:.4f} "
                          f"rad entre lecturas). Revise el robot antes de seguir.")
                    rc = 1
                    raise SystemExit
                session["runs"].append(res)
                print(f"    CSV: {'OK' if res['csv_exists'] else 'NO SE CREÓ'} "
                      f"{res['csv']}")
                if res["safe_hold"]:
                    print("\n  *** El nodo entró en SAFE_HOLD. Se aborta la campaña. ***")
                    print("  Revise el robot antes de continuar.")
                    rc = 1
                    raise SystemExit
    except SystemExit:
        pass
    except KeyboardInterrupt:
        print("\n  Interrumpido por el operador.")
        rc = 1
    finally:
        session["finished"] = datetime.now().isoformat(timespec="seconds")
        path = os.path.join(args.log_dir,
                            f"session_{datetime.now():%Y%m%d_%H%M%S}.yaml")
        with open(path, "w") as fh:
            yaml.safe_dump(session, fh, default_flow_style=False, sort_keys=False,
                           allow_unicode=True)
        print(f"\n  Registro de sesión: {path}")
        print(f"  {len(session['runs'])} corridas ejecutadas.")
        chk.destroy_node()
        rclpy.shutdown()

    if session["runs"]:
        nums = " ".join(str(r["test_num"]) for r in session["runs"]
                        if r["csv_exists"])
        print("\n  Identificación:")
        print(f"    ros2 run ur5_identification run_identification \\")
        print(f"        --csv " + " ".join(
            f"~/.ros/ur5_dyn_control/fl_{n}.csv" for n in nums.split()[:3]) + " ... \\")
        print(f"        --models viscous_coulomb stribeck --out ~/friction_real.yaml")
    return rc


if __name__ == "__main__":
    sys.exit(main())
