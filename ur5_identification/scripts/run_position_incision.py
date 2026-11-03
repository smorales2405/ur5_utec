#!/usr/bin/env python3
"""
Incisión por CONTROL DE POSICIÓN registrando corriente y F/T (B1 y B2 del plan).

Manda la MISMA tabla de la incisión que ejecutan las tres leyes por par
(config/incision_common.yaml, construida por el nodo C++ en `dry_run`) al
`scaled_joint_trajectory_controller` del driver, y graba `/joint_states`
(corriente en `effort`, G5) y el wrench del F/T integrado. Llama a
`zero_ftsensor` en el reposo del final de la aproximación, con la hoja quieta en
el aire 30 mm sobre la muestra.

Para qué
--------
- **Cierre de B1**: en aire, con y sin un peso conocido colgado cerca de la
  punta. Las dos vías (F/T y corriente, `force_estimator.py`) tienen que dar
  `ΔF_z = m·g` y, de paso, fijan el SIGNO del wrench del driver. Tiene que ser
  con el brazo MOVIÉNDOSE: con una junta parada su fricción es indeterminada y
  el estimador por corriente no la puede usar (force_estimator.py).
- **B2**: los cortes piloto por material, sin ninguna ley propia en el lazo.

El CSV usa los nombres de columna del CSV unificado (q, dq, cur, *_des,
wrench, ft_stamp, ft_zero, state), así que `force_estimator.estimate_from_csv`
lo lee igual que el de una corrida por par. `state` lleva la fase de la
incisión (approach ... withdraw) y `plateau` la meseta del corte.

La tabla NO arranca en q_init sino 21.7 mm más arriba (`start_pose` de
incision_common.yaml). Las leyes por par lo absorben con su transición; aquí el
primer punto de la tabla va en t = 0 y el JTC SALTARÍA a él, así que antes de
empezar el script lleva el robot al inicio, despacio y pidiendo confirmación
(MAX_MOVE: desde q_init o desde el final de la corrida anterior, sí; de más lejos,
a mano).

Uso (driver arriba, External Control en play, slider al 100 %, escalas G4 a 0.0):
    run_position_incision.py --test-num 800 --material eva --lane 1
    run_position_incision.py --test-num 801 --peso-kg 0.5     # cierre de B1
    run_position_incision.py --solo-tabla                      # sin robot
"""

import argparse
import os
import subprocess
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

#: Salto máximo admitido al primer punto de la tabla, que va en t = 0.
Q_START_TOL = 0.005         # [rad]
#: Traslado al inicio de la tabla, a la velocidad de run_gravity_sweeps_real.py.
MOVE_SPEED = 0.10           # [rad/s]
#: Hasta aquí se recoloca solo: la tabla se aleja 0.83 rad de su inicio (wrist_1),
#: así que cubre q_init y el final de una corrida. Más lejos, a mano.
MAX_MOVE = 1.0              # [rad]
#: Lo que el traslado puede bajar la punta respecto al más bajo de sus extremos.
MAX_DIP = 0.005             # [m]
#: El JTC escala SU tiempo con el slider; el cero del F/T y las fases del CSV van
#: en tiempo de pared. Por debajo, el cero cae con el brazo en movimiento.
MIN_SPEED_SCALING = 0.99
#: Se vacía la cola de /joint_states antes de grabar: se llenó durante las esperas
#: y sus ~100 muestras viejas adelantarían t0 0.2 s, desfasando q_des y las fases
#: (2 mm de error aparente a 10 mm/s).
DRAIN_TIME = 0.3            # [s]
#: Antes del primer contacto: el reposo de 0.3 s ya empezó, la hoja está quieta.
ZERO_LEAD = 0.15            # [s]
FT_TOPIC = "/force_torque_sensor_broadcaster/wrench"
ZERO_SRV = "/io_and_status_controller/zero_ftsensor"
SCALING_TOPIC = "/speed_scaling_state_broadcaster/speed_scaling"
TCP_TOPIC = "/tcp_pose_broadcaster/pose"
#: TCP del pendant admitido. La cinemática calibrada del robot difiere ~1 mm de la
#: nominal, y 0.2° de orientación (medido, 2026-11-02).
MAX_TCP_OFFSET = 0.005      # [m]
MAX_TCP_ROTATION = 0.02     # [rad]


def build_incision_reference(out_csv: str, timeout: float = 120.0) -> bool:
    """Vuelca la tabla de la incisión con el generador de C++ (sin robot).

    `dry_run`, sin switch ni despausa, y el CSV del nodo a un directorio
    temporal: NO toca ~/.ros/ur5_dyn_control (el `test_num` por defecto
    sobrescribiría <prefijo>_1.csv).
    """
    from ament_index_python.packages import (get_package_prefix,
                                              get_package_share_directory)
    cfg = os.path.join(get_package_share_directory("ur5_dyn_control"), "config")
    tmp = tempfile.mkdtemp(prefix="ref_incision_")
    sidecar = os.path.join(tmp, "smc_1.params.yaml")
    # El EJECUTABLE, no `ros2 run`: terminate() sobre `ros2 run` mata solo el
    # envoltorio y el nodo queda huerfano, colgado de systemd (paso, 2026-10-27).
    exe = os.path.join(get_package_prefix("ur5_dyn_control"), "lib",
                       "ur5_dyn_control", "gz_smc_control_node")
    cmd = [exe, "--ros-args",
           "--params-file", os.path.join(cfg, "incision_common.yaml"),
           "--params-file", os.path.join(cfg, "smc_params.yaml"),
           "-p", "dry_run:=true", "-p", "use_sim_time:=false",
           "-p", "perform_switch:=false", "-p", "perform_unpause:=false",
           "-p", f"csv_output_dir:={tmp}", "-p", "test_num:=1",
           "-p", f"reference_table_out:={out_csv}"]
    if os.path.exists(out_csv):
        os.remove(out_csv)
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    t0 = time.time()
    while time.time() - t0 < timeout and not os.path.exists(sidecar):
        time.sleep(0.2)
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    return os.path.exists(out_csv) and os.path.getsize(out_csv) > 0


def load_incision_reference(path: str):
    """dt, q, dq, ddq, phase, plateau de la tabla exportada (B0.3)."""
    n_meta, dt = 0, 0.002
    with open(path) as fh:
        for line in fh:
            if not line.startswith("#"):
                break
            n_meta += 1
            if line.startswith("# dt="):
                dt = float(line.split("=")[1])
    d = np.genfromtxt(path, delimiter=",", names=True, dtype=None,
                      encoding="utf-8", skip_header=n_meta)
    col = lambda p: np.column_stack([d[p % i] for i in range(1, 7)])  # noqa: E731
    return (dt, col("q%d"), col("dq%d"), col("ddq%d"),
            np.asarray(d["phase"], dtype=str), np.asarray(d["plateau"], dtype=int))


def zero_time(dt: float, phase: np.ndarray) -> float:
    """Instante [s] desde el inicio de la tabla en que se pide el cero del F/T."""
    k = int(np.flatnonzero(phase == "contact")[0])
    return k * dt - ZERO_LEAD


def scaling_problem(value: float):
    """Motivo para no correr con esta escala de velocidad, o None.

    El tópico da 0..1 o 0..100 según la versión del driver; se admiten las dos.
    Con External Control parado vale 0 (medido, 2026-11-02): el JTC acepta la
    trayectoria y no la avanza, y sin esto se grabarían 31 s de brazo parado.
    """
    frac = value / 100.0 if value > 1.5 else value
    if frac <= 0.0:
        return ("speed_scaling = 0: External Control no esta en play "
                "(o el robot esta en paro). Darle play en el pendant.")
    if frac < MIN_SPEED_SCALING:
        return (f"slider de velocidad al {100 * frac:.0f} %: ponerlo al 100 %. El JTC "
                "escala su tiempo, pero el cero del F/T y las fases del CSV no.")
    return None


def tcp_offset(plant, q, position, quat_xyzw):
    """(traslación [m], rotación [rad]) del TCP del pendant respecto a la brida.

    Compara la pose que publica el controlador con la FK del modelo, en ejes de
    tool0. La base del controlador es base_link girada 180° sobre z.
    """
    import pinocchio as pin
    from ur5_trajectory_optimization.gain_tuning.closed_loop import TCP_OFFSET
    pin.framesForwardKinematics(plant.model, plant.data, np.asarray(q, float))
    tip = plant.data.oMf[plant.tcp]
    flange = tip.translation - tip.rotation @ np.array([0.0, 0.0, TCP_OFFSET])
    rz = np.diag([-1.0, -1.0, 1.0])
    expected = rz @ tip.rotation
    x, y, z, w = quat_xyzw
    actual = pin.Quaternion(w, x, y, z).toRotationMatrix()
    return (expected.T @ (np.asarray(position, float) - rz @ flange),
            pin.log3(expected.T @ actual))


def tcp_problem(offset, rotation):
    """Motivo para no correr con este TCP del pendant, o None.

    El driver refiere el momento del wrench a ese punto, y el CSV lo traslada a la
    punta suponiendo que es la brida. Con rotación, también giran las fuerzas.
    """
    if (np.linalg.norm(offset) <= MAX_TCP_OFFSET
            and np.linalg.norm(rotation) <= MAX_TCP_ROTATION):
        return None
    mm = np.round(1e3 * np.asarray(offset), 1).tolist()
    return (f"TCP del pendant en {mm} mm, rotacion {np.linalg.norm(rotation):.3f} rad: "
            "ponerlo en (0, 0, 0) en Instalacion > TCP.")


def check_pendant_tcp(node, plant, q, timeout: float = 3.0):
    """Motivo para no correr según el TCP del pendant, o None."""
    import rclpy
    from geometry_msgs.msg import PoseStamped
    seen = []
    sub = node.create_subscription(PoseStamped, TCP_TOPIC, seen.append, 10)
    t0 = time.time()
    while not seen and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    if not seen:
        return f"No llega {TCP_TOPIC}: ¿driver arriba?"
    p, o = seen[-1].pose.position, seen[-1].pose.orientation
    return tcp_problem(*tcp_offset(plant, q, (p.x, p.y, p.z), (o.x, o.y, o.z, o.w)))


def start_move(plant, q, q0):
    """(err, problema) del traslado articular lineal q -> q0.

    `err` es |q - q0| máximo [rad]; `problema`, el motivo para no hacerlo solo, o
    None. No se mira la holgura absoluta sino que la punta no baje por el camino:
    tras un aborto puede estar legítimamente cerca de la muestra.
    """
    from run_gravity_sweeps_real import path_min_tip_z
    from ur5_identification.gravity_sweeps import tip_position
    q, q0 = np.asarray(q, float), np.asarray(q0, float)
    err = float(np.abs(q - q0).max())
    if err > MAX_MOVE:
        return err, (f"El robot esta a {err:.3f} rad del inicio de la tabla (maximo "
                     f"{MAX_MOVE} rad para recolocarlo solo): llevarlo con el pendant "
                     f"a {np.round(q0, 4).tolist()}")
    z_low = min(tip_position(plant, q)[2], tip_position(plant, q0)[2])
    z_min = path_min_tip_z(plant, q, q0)
    if z_min < z_low - MAX_DIP:
        return err, (f"El traslado al inicio de la tabla bajaria la punta a z = "
                     f"{z_min:.3f} m (extremos: {z_low:.3f} m): llevarlo con el pendant "
                     f"a {np.round(q0, 4).tolist()}")
    return err, None


def wait_listener(node, pub, timeout: float = 5.0) -> bool:
    """Espera a que el JTC escuche: lo publicado antes se pierde sin aviso."""
    import rclpy
    t0 = time.time()
    while pub.get_subscription_count() == 0:
        if time.time() - t0 > timeout:
            return False
        rclpy.spin_once(node, timeout_sec=0.1)
    return True


def check_speed_scaling(node, timeout: float = 3.0):
    """Motivo para no correr según la escala de velocidad, o None."""
    import rclpy
    from std_msgs.msg import Float64
    seen = []
    sub = node.create_subscription(Float64, SCALING_TOPIC,
                                   lambda m: seen.append(m.data), 10)
    t0 = time.time()
    while not seen and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    if not seen:
        return f"No llega {SCALING_TOPIC}: ¿driver arriba?"
    return scaling_problem(seen[-1])


def move_to_start(q, q0, err, ask: bool = True):
    """Lleva el robot a q0 con el JTC (`go_home` de las campañas) y lo comprueba."""
    from run_friction_campaign_real import Checker
    from ur5_identification.campaign_levels import JOINT_NAMES
    q, q0 = np.asarray(q, float), np.asarray(q0, float)
    worst = JOINT_NAMES[int(np.abs(q - q0).argmax())]
    dur = max(3.0, err / MOVE_SPEED)
    print(f"  El robot esta a {err:.3f} rad del inicio de la tabla (peor: {worst}).")
    if ask:
        input(f"  Traslado con el JTC a {MOVE_SPEED} rad/s, {dur:.1f} s. Paro de "
              f"emergencia a mano. Enter para moverlo... ")
    chk = Checker()
    if not wait_listener(chk, chk._jtc):
        sys.exit("El JTC no escucha en su topico: no se ha movido nada.")
    chk.go_home(q0.tolist(), speed=MOVE_SPEED)
    still, _ = chk.wait_until_still()
    now = chk.joint_state()
    chk.destroy_node()
    left = np.abs(np.asarray(now) - q0).max() if now is not None else float("inf")
    if not still or left > Q_START_TOL:
        sys.exit(f"El traslado no llego al inicio de la tabla (error {left:.4f} rad, "
                 f"quieto: {still}). ¿External Control en play, slider al 100 %?")
    print(f"  En el inicio de la tabla (error {left:.4f} rad).")


def write_csv(path, samples, ft, zero_t, ref, meta, plant):
    """CSV con los nombres del CSV unificado (ver el docstring del módulo).

    `samples`: (t, q, dq, cur) de /joint_states; `ft`: (t, w[6]) del driver en
    tool0_controller; `zero_t`: instante del cero confirmado (None si no hubo).
    El wrench se lleva a la punta y a base_link con la FK del modelo, como
    `wrenchAtTipInBase` en el nodo (ft.ref_offset_z = 0: TCP del pendant en la
    brida).
    """
    import pinocchio as pin
    from ur5_trajectory_optimization.gain_tuning.closed_loop import TCP_OFFSET
    dt, q_ref, dq_ref, ddq_ref, phase, plateau = ref
    if not samples:
        return False
    t0 = samples[0][0]
    ft_t = np.array([f[0] for f in ft]) if ft else np.zeros(0)
    tip = TCP_OFFSET            # brida -> punta de la hoja
    n = len(phase)
    with open(path, "w") as fh:
        for k, v in meta.items():
            fh.write(f"# {k}={v}\n")
        cols = ["t_sim", "t_wall"]
        for g in ("q", "dq", "cur", "q%d_des", "dq%d_des", "ddq%d_des", "wrench"):
            cols += [(g % i) if "%" in g else f"{g}{i}" for i in range(1, 7)]
        cols += ["ft_stamp", "ft_zero", "plateau", "state"]
        fh.write(",".join(cols) + "\n")
        for t, q, dq, cur in samples:
            rel = t - t0
            k = min(max(int(round(rel / dt)), 0), n - 1)
            w = np.full(6, np.nan)
            stamp = float("nan")
            if len(ft_t):
                j = int(np.searchsorted(ft_t, t))
                j = min(max(j, 0), len(ft_t) - 1)
                stamp = ft_t[j]
                pin.framesForwardKinematics(plant.model, plant.data, np.asarray(q))
                R = plant.data.oMf[plant.tcp].rotation
                f, m = R @ ft[j][1][:3], R @ ft[j][1][3:]
                r = R @ np.array([0.0, 0.0, tip])
                w = np.r_[f, m - np.cross(r, f)]
            zeroed = int(zero_t is not None and t >= zero_t)
            row = [rel, t, *q, *dq, *cur, *q_ref[k], *dq_ref[k], *ddq_ref[k], *w,
                   stamp, zeroed, int(plateau[k])]
            fh.write(",".join(f"{v:.9f}" for v in row) + f",{phase[k]}\n")
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--test-num", type=int, required=False)
    ap.add_argument("--material", default="")
    ap.add_argument("--lane", type=int, default=-1)
    ap.add_argument("--peso-kg", type=float, default=0.0,
                    help="peso conocido colgado cerca de la punta (cierre de B1)")
    ap.add_argument("--stride", type=int, default=10,
                    help="submuestreo de la tabla para el JTC (500 Hz / stride)")
    ap.add_argument("--solo-tabla", action="store_true",
                    help="construir la tabla y salir, sin conectar con el robot")
    ap.add_argument("--yes", action="store_true", help="no pedir confirmacion")
    ap.add_argument("--out-dir", default=os.path.expanduser("~/.ros/ur5_dyn_control"))
    args = ap.parse_args()

    ref_csv = os.path.join(tempfile.gettempdir(), "incision_position_ref.csv")
    print("Construyendo la tabla de la incision (incision_common.yaml) ...")
    if not build_incision_reference(ref_csv):
        sys.exit("No se pudo construir la tabla.")
    ref = load_incision_reference(ref_csv)
    dt, q_ref, dq_ref = ref[0], ref[1], ref[2]
    t_zero = zero_time(dt, ref[4])
    print(f"  {len(q_ref)} muestras, {len(q_ref) * dt:.2f} s; cero del F/T a los "
          f"{t_zero:.2f} s; q0 = {np.round(q_ref[0], 4)}")
    if args.solo_tabla:
        print(f"  tabla: {ref_csv}")
        return
    if args.test_num is None:
        sys.exit("--test-num es obligatorio para ejecutar")
    out = os.path.join(args.out_dir, f"pos_{args.test_num}.csv")
    if os.path.exists(out):
        sys.exit(f"{out} ya existe: no se sobrescriben corridas")

    import rclpy
    from geometry_msgs.msg import WrenchStamped
    from rclpy.qos import qos_profile_sensor_data
    from std_srvs.srv import Trigger
    from run_current_sweep import SweepRunner
    from ur5_trajectory_optimization.gain_tuning.closed_loop import (
        Plant, default_urdf)

    rclpy.init()
    node = SweepRunner()
    ft = []
    node.create_subscription(
        WrenchStamped, FT_TOPIC,
        lambda m: ft.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                             np.array([m.wrench.force.x, m.wrench.force.y,
                                       m.wrench.force.z, m.wrench.torque.x,
                                       m.wrench.torque.y, m.wrench.torque.z]))),
        qos_profile_sensor_data)
    zero_cli = node.create_client(Trigger, ZERO_SRV)

    st = node.wait_state()
    if st is None:
        sys.exit("Sin /joint_states: ¿driver arriba?")
    problem = check_speed_scaling(node)
    if problem:
        sys.exit(problem)
    plant = Plant(default_urdf())
    problem = check_pendant_tcp(node, plant, st[0])
    if problem:
        sys.exit(problem)
    err, problem = start_move(plant, st[0], q_ref[0])
    if problem:
        sys.exit(problem)
    ok, why = node.ensure_jtc()
    if not ok:
        sys.exit(f"No se pudo activar el JTC: {why}")
    if err > Q_START_TOL:
        move_to_start(st[0], q_ref[0], err, ask=not args.yes)
    if not wait_listener(node, node._pub):
        sys.exit("El JTC no escucha en su topico: no se ha movido nada.")
    if not args.yes:
        input(f"Incision en POSICION ({len(q_ref) * dt:.1f} s), peso {args.peso_kg} kg. "
              f"Paro de emergencia a mano. Enter para empezar... ")

    t_drain = time.time()
    while (time.time() - t_drain < DRAIN_TIME
           or (not ft and time.time() - t_drain < 3.0)):
        rclpy.spin_once(node, timeout_sec=0.01)
    if not ft:
        sys.exit(f"Sin F/T en {FT_TOPIC}: la corrida no serviria para el cierre. "
                 "No se ha movido nada.")

    # El cero se pide desde el bucle de grabacion, a su hora.
    state = {"zero_t": None, "asked": False}
    t_start = time.time()

    def on_spin():
        if not state["asked"] and time.time() - t_start >= t_zero:
            state["asked"] = True
            if zero_cli.service_is_ready():
                fut = zero_cli.call_async(Trigger.Request())
                fut.add_done_callback(lambda f: state.update(
                    zero_t=(time.time() if f.result() and f.result().success else None)))
            else:
                print("  AVISO: zero_ftsensor no disponible; la corrida queda sin cero")

    node.create_timer(0.01, on_spin)
    node.run(dt, q_ref, dq_ref, args.stride)
    meta = {"controller_id": "position_jtc", "test_num": args.test_num,
            "material": args.material, "lane": args.lane, "peso_kg": args.peso_kg,
            "ft_topic": FT_TOPIC, "ft_zero_ok": state["zero_t"] is not None,
            "reference": "incision_common.yaml (dry_run del nodo C++)"}
    # zero_t va en reloj de pared; las muestras de /joint_states, en el sello del
    # driver, que en el real es reloj de pared.
    if write_csv(out, node.samples, ft, state["zero_t"], ref, meta, plant):
        print(f"CSV: {out}  ({len(node.samples)} muestras de estado, {len(ft)} de F/T)")
    else:
        print("Sin muestras: no se escribio el CSV")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
