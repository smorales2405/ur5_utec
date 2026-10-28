"""
B7.0: el vigilante de comandos corta el par latcheado cuando el nodo calla.

Se lanza `command_deadman.py` de verdad (proceso aparte, como en el real) contra
un controller_manager falso que solo registra la peticion de switch.
"""

import os
import random
import subprocess
import sys
import time

import pytest

SCRIPT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      "scripts", "command_deadman.py")
TOPIC = "/dm_test/commands"
CM = "/dm_test_cm"


@pytest.fixture
def ros():
    # Dominio propio: no ver ni molestar a nada que este corriendo.
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(100, 200))
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    import rclpy
    from controller_manager_msgs.srv import SwitchController
    from std_msgs.msg import Float64MultiArray
    rclpy.init()
    node = rclpy.create_node("dm_test")
    state = {"switch": [], "zeros": 0, "msgs": 0}

    def on_switch(req, res):
        state["switch"].append((list(req.activate_controllers),
                                list(req.deactivate_controllers), req.strictness))
        res.ok = True
        return res

    def on_msg(msg):
        state["msgs"] += 1
        if all(v == 0.0 for v in msg.data):
            state["zeros"] += 1

    node.create_service(SwitchController, f"{CM}/switch_controller", on_switch)
    node.create_subscription(Float64MultiArray, TOPIC, on_msg, 50)
    pub = node.create_publisher(Float64MultiArray, TOPIC, 50)
    proc = subprocess.Popen(
        [sys.executable, SCRIPT, "--ros-args", "-p", f"command_topic:={TOPIC}",
         "-p", f"controller_manager:={CM}", "-p", "timeout:=0.1"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def spin(seconds, publish=None):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            if publish is not None:
                pub.publish(Float64MultiArray(data=list(publish)))
            rclpy.spin_once(node, timeout_sec=0.002)

    # Que el vigilante descubra el servicio y el topico antes de empezar.
    spin(3.0)
    yield spin, state
    proc.terminate()
    proc.wait(timeout=10)
    node.destroy_node()
    rclpy.shutdown()


def test_corta_el_par_y_devuelve_el_mando_al_jtc(ros):
    spin, state = ros
    spin(1.0, publish=[0.0, 1.5, -0.3, 0.1, 0.0, 0.0])     # nodo comandando
    assert state["switch"] == [] and state["zeros"] == 0
    spin(0.6)                                               # el nodo calla
    assert len(state["switch"]) == 1
    act, deact, strict = state["switch"][0]
    assert act == ["scaled_joint_trajectory_controller"]
    assert deact == ["forward_effort_controller"]
    assert strict == 1                                      # BEST_EFFORT
    assert state["zeros"] >= 1
    spin(0.5)                                               # sus ceros no lo rearman
    assert len(state["switch"]) == 1


def test_los_ceros_del_arranque_no_lo_arman(ros):
    spin, state = ros
    spin(1.0, publish=[0.0] * 6)                            # PRE_HOLD / WAIT_STATE
    spin(0.6)
    assert state["switch"] == []


def test_se_rearma_con_la_corrida_siguiente(ros):
    spin, state = ros
    spin(0.5, publish=[0.2] * 6)
    spin(0.5)
    spin(0.5, publish=[0.2] * 6)
    spin(0.5)
    assert len(state["switch"]) == 2
