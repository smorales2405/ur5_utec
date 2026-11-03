"""B4.6: la inercia efectiva sale de la corriente sin suponer la fricción ni la gravedad.

Los `e2e` ejecutan el runner contra `fake_ur_driver.py`, en un ROS_DOMAIN_ID propio: el
robot real vive en otro y no se entera.
"""

import math
import os
import random
import subprocess
import sys

import numpy as np
import pytest

_TEST = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.join(_TEST, "..", "scripts")
sys.path.insert(0, _SCRIPTS)

from ur5_identification.force_estimator import K_CURRENT  # noqa: E402
from ur5_identification.inertia_id import (  # noqa: E402
    DT, FREQS, RAMP, V_PEAK, append_segment, combine_joints, dynamic_stiffness,
    excitation, fit_groups, fit_slope, harmonic, model_inertia, read_csv, rotor_range,
    steady_window, velocity_cell_fit, write_header)

try:
    from ur5_trajectory_optimization.gain_tuning.closed_loop import (
        Q_INIT, Plant, default_urdf)
    URDF = default_urdf()
except Exception:  # pragma: no cover
    URDF = ""

needs_plant = pytest.mark.skipif(not os.path.exists(URDF),
                                 reason="URDF/evaluador no instalado")
Q0 = np.array([1.5708, -1.5694, 1.5141, -1.5155, -1.5708, 3.1416])


def synth_samples(seg, inertia, stiffness=0.0, coulomb=0.0, viscous=0.0, noise=0.0,
                  skew=0.0, sign=1.0, seed=0, dt=DT, friction_lag=0.0):
    """/joint_states de un brazo que sigue la consigna, con la física de la junta.

    `skew`: la corriente va `skew` segundos por detrás de la posición.
    `friction_lag`: solo la fricción de Coulomb va retrasada, como al invertir.
    """
    rng = np.random.default_rng(seed)
    j = seg.joint
    t = np.arange(len(seg.q)) * dt
    dq = seg.dq[:, j]
    ddq = np.gradient(dq, dt)
    x = seg.q[:, j] - seg.q0[j]
    tau = (inertia * ddq + stiffness * x
           + coulomb * np.tanh(np.interp(t - friction_lag, t, dq) / 0.002)
           + viscous * dq + 3.0)
    if skew:
        tau = np.interp(t - skew, t, tau)
    tau = sign * tau + rng.normal(0.0, noise, len(t)) if noise else sign * tau
    out = []
    for k in range(len(t)):
        cur = np.zeros(6)
        cur[j] = tau[k] / K_CURRENT[j]
        out.append((1000.0 + t[k], seg.q[k], seg.dq[k], cur))
    return out


def segments_from(tmp_path, joint, fake, freqs=None, posture="inicio", index0=0, **kw):
    """Pasa por el CSV de verdad: lo que se ajusta es lo que se habría grabado."""
    path = tmp_path / f"s{joint}_{posture}.csv"
    with open(path, "w") as fh:
        write_header(fh, {"test_num": 1, "masa_kg": 0.0, "cog_mm": "0 0 0"})
        for i, f in enumerate(freqs or FREQS[joint]):
            seg = excitation(posture, Q0, joint, f)
            append_segment(fh, index0 + i, seg, synth_samples(seg, fake, **kw))
    return read_csv(str(path))[1]


# --- Excitación ------------------------------------------------------------------

@pytest.mark.parametrize("joint", range(6))
def test_excitacion_empieza_y_acaba_quieta(joint):
    for f in FREQS[joint]:
        s = excitation("inicio", Q0, joint, f)
        np.testing.assert_allclose(s.q[0], Q0, atol=1e-12)
        np.testing.assert_allclose(s.q[-1], Q0, atol=1e-9)
        np.testing.assert_allclose(s.dq[[0, -1]], 0.0, atol=1e-9)
        others = [i for i in range(6) if i != joint]
        assert np.ptp(s.q[:, others], axis=0).max() == 0.0
        assert s.dq[:, joint].max() == pytest.approx(V_PEAK[joint], rel=0.02)


def test_excitacion_velocidad_coherente_con_posicion():
    s = excitation("inicio", Q0, 3, 2.0)
    np.testing.assert_allclose(np.gradient(s.q[:, 3], DT), s.dq[:, 3], atol=2e-3)


def test_ventana_es_un_numero_entero_de_ciclos_en_amplitud_constante():
    s = excitation("inicio", Q0, 3, 3.0)
    t = np.arange(len(s.q)) * DT
    mask = steady_window(t, s.freq, s.cycles)
    assert t[mask][0] >= RAMP
    assert t[mask][-1] <= s.duration - RAMP
    span = mask.sum() * DT * s.freq
    assert abs(span - round(span)) < 0.02 and span >= 5


def test_fasor():
    t = np.arange(0.0, 4.0, DT)
    w = 2 * np.pi * 3.0
    x = 2.0 * np.cos(w * t + 0.4) + 0.7 + 0.1 * t + 0.5 * np.sin(3 * w * t)
    assert abs(harmonic(t, x, 3.0) - 2.0 * np.exp(0.4j)) < 1e-6


# --- La física ---------------------------------------------------------------------

def _fit(tmp_path, joint, **kw):
    fake = kw.pop("fake")
    rows = fit_groups(segments_from(tmp_path, joint, fake, **kw))
    assert len(rows) == 1
    return rows[0]


@pytest.mark.parametrize("joint, fake", [(3, 0.214), (4, 0.199), (5, 0.2), (1, 3.0), (2, 1.8)])
def test_recupera_la_inercia_sin_nada_mas(tmp_path, joint, fake):
    r = _fit(tmp_path, joint, fake=fake)
    assert r["M"] == pytest.approx(fake, rel=0.01)
    assert r["err"] < 0.02 * fake and r["note"] == ""


def test_la_friccion_y_la_gravedad_no_la_sesgan(tmp_path):
    """Las magnitudes medidas en wrist_1: F_c 1.8 N.m frente a un pico de 2 N.m."""
    r = _fit(tmp_path, 3, fake=0.214, coulomb=1.8, viscous=1.5, stiffness=8.0, noise=0.2)
    assert r["M"] == pytest.approx(0.214, rel=0.03)
    assert r["s"] == pytest.approx(8.0, rel=0.25)


def test_la_rigidez_constante_no_se_confunde_con_inercia(tmp_path):
    """Por eso hacen falta varias frecuencias: con una sola, `s` entraría en M."""
    r = _fit(tmp_path, 3, fake=0.214, stiffness=60.0)
    assert r["M"] == pytest.approx(0.214, rel=0.01)
    assert r["s"] == pytest.approx(60.0, rel=0.02)
    # La lectura ingenua a 1 Hz, sin la pendiente, se equivocaría por mucho.
    f, z, _ = r["points"][0]
    naive = -z.real / (2.0 * math.pi * f) ** 2
    assert abs(naive - 0.214) > 0.5 * 0.214


def test_el_desfase_imita_inercia_y_se_informa(tmp_path):
    """La fórmula del docstring, con la fricción medida de wrist_1 y 1 ms."""
    base = _fit(tmp_path, 3, fake=0.214, coulomb=1.8, viscous=1.5)
    late = _fit(tmp_path, 3, fake=0.214, coulomb=1.8, viscous=1.5, skew=0.001)
    bias = abs(late["M"] - base["M"])
    assert 0.002 < bias < 0.02
    # ±2 ms de sensibilidad cubren ese milisegundo.
    assert base["sens"] > bias


def test_corriente_invertida_no_se_da_por_buena(tmp_path):
    r = _fit(tmp_path, 3, fake=0.214, sign=-1.0)
    assert "NO USABLE" in r["note"]
    assert combine_joints([dict(r, dI=r["M"], M_model=0.0)]) == {}


def test_con_mucho_ruido_avisa(tmp_path):
    r = _fit(tmp_path, 3, fake=0.01, noise=3.0)
    assert r["note"] != ""


def test_dos_frecuencias_no_bastan(tmp_path):
    r = _fit(tmp_path, 3, fake=0.2, freqs=(1.0, 2.0))
    assert r["M"] is None and "frecuencias" in r["note"]


def test_pendiente_exacta_con_datos_perfectos():
    pts = []
    for f, amp in ((1.0, 0.06), (2.0, 0.03), (4.0, 0.015)):
        w2 = (2 * np.pi * f) ** 2
        pts.append((f, complex(10.0 - 0.2 * w2, 5.0), complex(amp, 0.0)))
    m, s, err, resid = fit_slope(pts)
    assert m == pytest.approx(0.2) and s == pytest.approx(10.0)
    assert err < 1e-9 and resid < 1e-9


def test_desfase_rota_z():
    t = np.arange(0.0, 3.0, DT)
    w = 2 * np.pi * 2.0
    q = np.sin(w * t)
    cur = np.cos(w * t) / K_CURRENT[3]      # solo viscoso: en fase con la velocidad
    z, _ = dynamic_stiffness(t, q, cur, 2.0, K_CURRENT[3])
    assert abs(z.real) < 1e-6
    assert z.imag == pytest.approx(1.0, rel=1e-6)
    z2, _ = dynamic_stiffness(t, q, cur, 2.0, K_CURRENT[3], skew=0.002)
    assert z2.real == pytest.approx(-math.sin(w * 0.002), abs=1e-3)


# --- Por junta ---------------------------------------------------------------------

def _row(joint, posture, d_i, err=0.01, sens=0.002):
    return {"joint": joint, "posture": posture, "dI": d_i, "err": err, "sens": sens,
            "note": ""}


def test_dos_posturas_que_coinciden():
    c = combine_joints([_row(0, "a", 1.00), _row(0, "b", 1.02)])[0]
    assert c["consistent"] and c["dI"] == pytest.approx(1.01) and c["postures"] == 2


def test_dos_posturas_que_no_coinciden_se_dicen():
    c = combine_joints([_row(0, "a", 1.00), _row(0, "b", 1.50)])[0]
    assert not c["consistent"] and c["err"] == pytest.approx(0.25)


def test_una_junta_con_aviso_no_entra():
    bad = dict(_row(1, "a", 2.0), note="POCA SENAL")
    assert combine_joints([bad, _row(0, "a", 1.0)]).keys() == {0}


# --- Celdas de velocidad -----------------------------------------------------------

def test_celdas_recuperan_inercia_y_rigidez(tmp_path):
    segs = segments_from(tmp_path, 3, 0.19, stiffness=1.0, coulomb=1.8, viscous=1.3,
                         noise=0.02)
    M, s = velocity_cell_fit(segs)
    assert M == pytest.approx(0.19, abs=0.002)
    assert s == pytest.approx(1.0, abs=0.3)


def test_celdas_no_ven_el_retraso_de_la_friccion_y_los_fasores_si(tmp_path):
    # El motivo de tener los dos: el sesgo de fasores es (4 F_c/pi)(lag/v) = 0.017.
    segs = segments_from(tmp_path, 3, 0.19, coulomb=1.8, friction_lag=0.003)
    rows = fit_groups(segs)
    assert rows[0]["M"] == pytest.approx(0.19 - 0.017, abs=0.002)
    assert velocity_cell_fit(segs)[0] == pytest.approx(0.19, abs=0.002)


def test_celdas_toleran_otra_friccion_en_cada_segmento(tmp_path):
    # La fricción deriva con la temperatura entre un segmento y el siguiente.
    a = segments_from(tmp_path, 3, 0.19, freqs=(1.0, 2.0), coulomb=1.8)
    b = segments_from(tmp_path, 3, 0.19, freqs=(3.0, 4.0), index0=2, coulomb=2.3,
                      posture="otra")
    M, _ = velocity_cell_fit(a + b)
    assert M == pytest.approx(0.19, abs=0.002)


def test_celdas_con_dos_frecuencias_no_ajustan(tmp_path):
    assert velocity_cell_fit(segments_from(tmp_path, 3, 0.19, freqs=(1.0, 3.0))) is None


def test_el_rango_cubre_los_dos_estimadores_y_todas_las_posturas():
    rows = [dict(_row(3, "inicio", 0.132), dI_cell=0.147),
            dict(_row(3, "corte", 0.125), dI_cell=0.145),
            dict(_row(4, "inicio", 0.139), dI_cell=None),
            dict(_row(5, "inicio", 0.5), dI_cell=0.6, note="POCA SENAL")]
    assert rotor_range(rows) == {3: (0.125, 0.147), 4: (0.139, 0.139)}


# --- Modelo rígido ---------------------------------------------------------------

@needs_plant
@pytest.mark.parametrize("joint", range(6))
def test_inercia_de_la_herramienta_coincide_con_pinocchio(joint):
    """La masa puntual, contra un cuerpo añadido de verdad al modelo."""
    import pinocchio as pin
    plant = Plant(URDF)
    q = np.array(Q_INIT)
    mass, cog = 0.179, (-2.227, 2.134, 29.398)
    got = model_inertia(plant.model, plant.data, q, joint, mass, cog)

    model = plant.model.copy()
    frame = model.frames[model.getFrameId("tool0")]
    place = frame.placement * pin.SE3(np.eye(3), np.array(cog) * 1e-3)
    model.appendBodyToJoint(frame.parentJoint,
                            pin.Inertia(mass, np.zeros(3), np.zeros((3, 3))), place)
    expected = float(pin.crba(model, model.createData(), q)[joint, joint])
    assert got == pytest.approx(expected, rel=1e-9, abs=1e-12)


@needs_plant
def test_rotor_es_lo_que_el_modelo_no_explica(tmp_path):
    plant = Plant(URDF)
    base = model_inertia(plant.model, plant.data, Q0, 3)
    rows = fit_groups(segments_from(tmp_path, 3, base + 0.19, coulomb=1.8),
                      plant.model, plant.data)
    assert rows[0]["dI"] == pytest.approx(0.19, abs=0.01)


# --- Plan --------------------------------------------------------------------------

@needs_plant
def test_el_plan_por_defecto_se_puede_ejecutar():
    from run_inertia_id import build_plan, plan_problems
    plant = Plant(URDF)
    segments, q_of = build_plan(plant)
    assert set(q_of) == {"inicio", "corte_medio"}
    assert plan_problems(plant, segments, 0.155) == []
    total = sum(s.duration + 3.5 for s in segments)
    assert total < 10.0 * 60.0
    assert {s.joint for s in segments if s.posture == "inicio"} == set(range(6))


@needs_plant
def test_el_plan_rechaza_bajar_a_la_mesa_y_pedir_demasiado_par():
    from run_inertia_id import plan_problems
    from ur5_identification.gravity_sweeps import incision_posture
    plant = Plant(URDF)
    low = incision_posture(plant, 0.65, 0.0)
    near = excitation("baja", low, 1, 1.0, v_peak=1.5)
    assert "tablero" in plan_problems(plant, [near])[0]
    hard = excitation("inicio", Q0, 3, 4.0, v_peak=3.0)
    assert "pico previsto" in plan_problems(plant, [hard])[0]


# --- Aborto ------------------------------------------------------------------------

def test_csv_de_corrida_abortada_se_lee_hasta_donde_llego(tmp_path):
    seg = excitation("inicio", Q0, 3, 2.0)
    path = tmp_path / "a.csv"
    with open(path, "w") as fh:
        write_header(fh, {"test_num": 7, "masa_kg": 0.155, "cog_mm": "1 2 3"})
        append_segment(fh, 0, seg, synth_samples(seg, 0.2))
        fh.write("1,inicio,4,3,0.02")        # línea cortada al morir
    meta, segs = read_csv(str(path))
    assert meta["test_num"] == "7" and meta["masa_kg"] == "0.155"
    assert len(segs) == 1 and segs[0]["joint"] == 3


def test_segmento_cortado_no_da_punto(tmp_path):
    seg = excitation("inicio", Q0, 3, 2.0)
    samples = synth_samples(seg, 0.2)
    path = tmp_path / "b.csv"
    with open(path, "w") as fh:
        write_header(fh, {"test_num": 7})
        append_segment(fh, 0, seg, samples[: len(samples) // 3])
    rows = fit_groups(read_csv(str(path))[1])
    assert rows[0]["n"] == 0 and rows[0]["M"] is None


def test_segmento_sin_seguir_la_consigna_para_la_corrida():
    from run_inertia_id import segment_problem
    seg = excitation("inicio", Q0, 3, 2.0)
    assert segment_problem(seg, synth_samples(seg, 0.2)) is None
    still = [(t, seg.q0, np.zeros(6), np.zeros(6)) for t, _, _, _ in synth_samples(seg, 0.2)]
    assert "pendant" in segment_problem(seg, still)
    assert "muestras" in segment_problem(seg, still[:50])
    nan = [(t, q, dq, cur * np.nan) for t, q, dq, cur in synth_samples(seg, 0.2)]
    assert "corriente" in segment_problem(seg, nan)


# --- De punta a punta, contra el driver falso --------------------------------------

e2e = pytest.mark.skipif(not os.path.exists(URDF), reason="evaluador no instalado")
ROTOR = [0.0, 0.0, 0.0, 0.19, 0.0, 0.0]


def _run(tmp_path, *extra, scaling=1.0, q0=None, rotor=ROTOR, timeout=200):
    domain = random.choice([d for d in range(150, 200)
                            if str(d) != os.environ.get("ROS_DOMAIN_ID")])
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1",
               TMPDIR=str(tmp_path))
    log = tmp_path / "fake.jsonl"
    fake = subprocess.Popen(
        [sys.executable, os.path.join(_TEST, "fake_ur_driver.py"), "--domain", str(domain),
         "--scaling", str(scaling), "--log", str(log),
         "--rotor", *[str(v) for v in rotor],
         "--q0", *[str(v) for v in (Q_INIT if q0 is None else q0)]], env=env)
    try:
        res = subprocess.run(
            [sys.executable, os.path.join(_SCRIPTS, "run_inertia_id.py"), "--yes",
             "--out-dir", str(tmp_path), *extra],
            env=env, capture_output=True, text=True, timeout=timeout)
    finally:
        fake.terminate()
        fake.wait(timeout=10)
    return res, (log.read_text() if log.exists() else "")


@e2e
def test_e2e_mide_el_rotor_desde_q_init(tmp_path):
    res, events = _run(tmp_path, "--test-num", "5", "--masa-kg", "0", "--juntas", "4",
                       "--posturas", "inicio", "--frecuencias", "2", "3", "4")
    assert res.returncode == 0, res.stdout[-1500:] + res.stderr[-1500:]
    out = tmp_path / "iner_5.csv"
    assert out.exists() and (tmp_path / "iner_5.yaml").exists()
    # Un traslado al inicio, tres senos y la vuelta (no hace falta: ya está en q0).
    assert events.count('"trajectory"') == 4
    import yaml
    data = yaml.safe_load(open(tmp_path / "iner_5.yaml"))
    assert data["rotor_kg_m2"][3] == pytest.approx(0.19, abs=0.02)
    _, segs = read_csv(str(out))
    assert len(segs) == 3 and {s["joint"] for s in segs} == {3}


@e2e
def test_e2e_sin_external_control_no_mueve_ni_escribe(tmp_path):
    res, events = _run(tmp_path, "--test-num", "6", "--masa-kg", "0", "--juntas", "4",
                       "--posturas", "inicio", "--frecuencias", "2", "3", "4", scaling=0.0)
    assert res.returncode != 0
    assert "External Control" in res.stderr + res.stdout
    assert '"trajectory"' not in events
    assert not (tmp_path / "iner_6.csv").exists()


@e2e
def test_e2e_no_pisa_una_corrida(tmp_path):
    out = tmp_path / "iner_8.csv"
    out.write_text("datos del usuario\n")
    res, events = _run(tmp_path, "--test-num", "8", "--masa-kg", "0", "--juntas", "4",
                       "--posturas", "inicio", "--frecuencias", "2", "3", "4")
    assert res.returncode != 0 and "no se sobrescriben" in res.stderr + res.stdout
    assert out.read_text() == "datos del usuario\n"
    assert '"trajectory"' not in events


@e2e
def test_e2e_solo_plan_no_necesita_robot(tmp_path):
    env = dict(os.environ, ROS_DOMAIN_ID="199", ROS_LOCALHOST_ONLY="1")
    res = subprocess.run(
        [sys.executable, os.path.join(_SCRIPTS, "run_inertia_id.py"), "--solo-plan"],
        env=env, capture_output=True, text=True, timeout=60)
    assert res.returncode == 0, res.stderr[-800:]
    assert "Plan:" in res.stdout
