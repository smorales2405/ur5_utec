"""
Tests del módulo de sintonía de ganancias (FASE 7).

Se centran en lo que puede romperse en silencio: la traducción entre el espacio
de búsqueda (log10) y las ganancias físicas, la lectura de la tabla de
referencias, la caché del evaluador y la consistencia de χ con su definición.

No se testea la CALIDAD del óptimo —eso depende de la semilla y del
presupuesto—, sino que la maquinaria no mienta.
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from ur5_trajectory_optimization.gain_tuning.closed_loop import (  # noqa: E402
    Q_INIT, TAU_MAX, CuttingForce, JointFriction, Plant, Reference, SmcLaw,
    load_reference, simulate)
from ur5_trajectory_optimization.gain_tuning.optimize import (  # noqa: E402
    _cubic_features, alpha_sensitivity, seed_points)
from ur5_trajectory_optimization.gain_tuning.problem import (  # noqa: E402
    CHI_THRESHOLD, GainEvaluator, SmcParameterization, disturbance_bound,
    friction_residual_bound)

URDF = "/home/utec/ur5_ws/install/ur5_kinematics/share/ur5_kinematics/ur5e.urdf"
INERTIA = np.array([1.05823, 2.59146, 0.881455, 0.0232406, 0.00535152, 0.00025756])


# ─────────────────────────────────────────────────────────────────────────────
# Parametrización
# ─────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("mode,n", [("scalar", 3), ("full", 13)])
def test_n_var_y_nombres(mode, n):
    p = SmcParameterization(inertia=INERTIA, mode=mode)
    assert p.n_var == n
    assert len(p.names) == n


@pytest.mark.parametrize("mode", ["scalar", "full"])
def test_bounds_bien_ordenadas(mode):
    xl, xu = SmcParameterization(inertia=INERTIA, mode=mode).bounds()
    assert xl.shape == xu.shape
    assert np.all(xu > xl)


def test_encode_gains_ida_y_vuelta_full():
    """En modo `full` la codificación es exacta: 6 λ + 6 η + φ."""
    p = SmcParameterization(inertia=INERTIA, mode="full")
    lam = np.array([10.0, 20.0, 30.0, 40.0, 50.0, 60.0])
    eta = np.array([1.0, 2.0, 0.5, 0.05, 0.01, 0.001])
    lam2, eta2, phi2 = p.gains(p.encode(lam, eta, 0.3))
    np.testing.assert_allclose(lam2, lam, rtol=1e-12)
    np.testing.assert_allclose(eta2, eta, rtol=1e-12)
    assert phi2 == pytest.approx(0.3)


def test_encode_gains_ida_y_vuelta_scalar():
    """En modo `scalar` η queda atado a la inercia: η_i = M_ii·a_reach."""
    p = SmcParameterization(inertia=INERTIA, mode="scalar")
    lam2, eta2, phi2 = p.gains(p.encode(np.full(6, 20.0), INERTIA * 3.0, 0.1))
    np.testing.assert_allclose(lam2, np.full(6, 20.0), rtol=1e-9)
    np.testing.assert_allclose(eta2, INERTIA * 3.0, rtol=1e-9)
    assert phi2 == pytest.approx(0.1)


def test_eta_bounds_ancladas_al_actuador():
    """Las cotas de η salen de τ_max, no de un número elegido a mano."""
    p = SmcParameterization(inertia=INERTIA, mode="full")
    xl, xu = p.bounds()
    np.testing.assert_allclose(10.0 ** xl[6:12], 1e-4 * TAU_MAX, rtol=1e-9)
    np.testing.assert_allclose(10.0 ** xu[6:12], 0.2 * TAU_MAX, rtol=1e-9)


def test_modo_desconocido_falla():
    with pytest.raises(ValueError):
        SmcParameterization(inertia=INERTIA, mode="cuadratico")


# ─────────────────────────────────────────────────────────────────────────────
# Lectura de la tabla de referencias
# ─────────────────────────────────────────────────────────────────────────────

def test_load_reference_salta_cabeceras_de_comentario(tmp_path):
    """
    REGRESIÓN: `numpy.genfromtxt(names=True)` NO salta las líneas `#` — toma la
    PRIMERA línea del fichero como cabecera de columnas, comentario incluido.
    El fallo es silencioso (columnas con nombres absurdos) y ya se coló dos
    veces en este proyecto.
    """
    path = tmp_path / "ref.csv"
    cols = (["q%d" % i for i in range(1, 7)] + ["dq%d" % i for i in range(1, 7)]
            + ["ddq%d" % i for i in range(1, 7)])
    with open(path, "w") as fh:
        fh.write("# dt=0.004\n# n=3\n# comentario extra\n")
        fh.write("k," + ",".join(cols) + ",phase\n")
        for k in range(3):
            fh.write(f"{k}," + ",".join(str(float(k)) for _ in cols) + ",TRACK\n")

    ref = load_reference(str(path))
    assert ref.dt == pytest.approx(0.004)
    assert ref.n == 3
    assert ref.q.shape == (3, 6)
    np.testing.assert_allclose(ref.ddq[2], np.full(6, 2.0))
    assert list(ref.phase) == ["TRACK"] * 3


# ─────────────────────────────────────────────────────────────────────────────
# Planta y ley de control
# ─────────────────────────────────────────────────────────────────────────────

@pytest.mark.skipif(not os.path.exists(URDF), reason="URDF no instalado")
def test_inercia_reproduce_el_eta_de_la_fase5():
    """`eta` de smc_params.yaml es diag M(q_init)·1.0: si esto cambia, el YAML
    y el optimizador dejarían de hablar de lo mismo."""
    diag = Plant(URDF).inertia_diag(Q_INIT)
    np.testing.assert_allclose(diag, INERTIA, rtol=1e-4)


@pytest.mark.skipif(not os.path.exists(URDF), reason="URDF no instalado")
def test_smclaw_devuelve_chi_consistente_con_su_definicion():
    """χ_i = (K_i/φ)/M_ii, con K = η + |cota|, así que χ ≥ (η/φ)/M."""
    plant = Plant(URDF)
    eta, phi = INERTIA * 2.0, 0.25
    law = SmcLaw(lam=np.full(6, 20.0), eta=eta, phi=phi, alpha=0.3)
    q = Q_INIT.copy()
    _, _, info = law(plant.model, plant.data, q, np.zeros(6), q,
                     np.zeros(6), np.zeros(6))
    # Tolerancia relativa: INERTIA está redondeada a 6 cifras, así que el
    # cociente contra la diagonal exacta de M difiere en ~1e-5.
    assert np.all(info["chi"] >= 0.9999 * (eta / phi) / INERTIA)


@pytest.mark.skipif(not os.path.exists(URDF), reason="URDF no instalado")
def test_disturbance_bound_es_positiva_y_nula_en_wrist3():
    """`wrist_3` gira en torno al eje de la herramienta: una fuerza pura en el
    TCP no le hace par. Si esto deja de valer, el frame del TCP cambió."""
    plant = Plant(URDF)
    ref = _ref_sintetica(plant)
    d = disturbance_bound(plant, ref, CuttingForce(), stride=1)
    assert np.all(d >= 0.0)
    assert d[5] < 1e-9
    assert d[:3].max() > 0.1


def _ref_sintetica(plant, n=12, dt=0.002):
    """Referencia corta e inmóvil: basta para ejercitar la maquinaria."""
    q = np.tile(Q_INIT, (n, 1))
    z = np.zeros((n, 6))
    return Reference(dt=dt, q=q, dq=z.copy(), ddq=z.copy(),
                     phase=np.array(["TRACK"] * n))


# ─────────────────────────────────────────────────────────────────────────────
# Evaluador
# ─────────────────────────────────────────────────────────────────────────────

@pytest.mark.skipif(not os.path.exists(URDF), reason="URDF no instalado")
def _evaluador(mode="full"):
    plant = Plant(URDF)
    ref = _ref_sintetica(plant)
    param = SmcParameterization(inertia=INERTIA, mode=mode)
    return GainEvaluator(param, ref, plant, force=None, alpha=0.3)


@pytest.mark.skipif(not os.path.exists(URDF), reason="URDF no instalado")
def test_cache_no_reevalua_el_mismo_punto():
    ev = _evaluador()
    x = ev.param.encode(np.full(6, 20.0), INERTIA, 0.3)
    ev.objectives(x)
    n1 = ev.n_eval
    ev.objectives(x)          # mismo punto
    ev.constraints(x)         # y las restricciones del mismo punto
    assert ev.n_eval == n1
    assert ev.n_hit >= 2


@pytest.mark.skipif(not os.path.exists(URDF), reason="URDF no instalado")
def test_constraints_devuelve_las_cuatro():
    ev = _evaluador()
    g = ev.constraints(ev.param.encode(np.full(6, 20.0), INERTIA, 0.3))
    assert g.shape == (GainEvaluator.N_CON,)


@pytest.mark.skipif(not os.path.exists(URDF), reason="URDF no instalado")
def test_alpha_sensitivity_restaura_alpha():
    """El barrido muta `evaluator.alpha`; si no lo restaurase, todo lo que
    viniera después se evaluaría con la ley equivocada."""
    ev = _evaluador()
    x = ev.param.encode(np.full(6, 20.0), INERTIA, 0.3)
    rows = alpha_sensitivity(ev, x, alphas=(0.1, 0.5))
    assert [r["alpha"] for r in rows] == [0.1, 0.5]
    assert ev.alpha == pytest.approx(0.3)


@pytest.mark.skipif(not os.path.exists(URDF), reason="URDF no instalado")
def test_seed_points_dentro_de_la_caja():
    ev = _evaluador()
    xl, xu = ev.param.bounds()
    pts = seed_points(ev, np.array([2.3, 2.5, 2.6, 0.86, 1.11, 0.0]))
    assert len(pts) == 10
    assert np.all(pts >= xl - 1e-9) and np.all(pts <= xu + 1e-9)


@pytest.mark.skipif(not os.path.exists(URDF), reason="URDF no instalado")
def test_simulate_marca_divergencia_sin_reventar():
    """Ganancias absurdas deben devolver `diverged`, no propagar NaN."""
    plant = Plant(URDF)
    ref = _ref_sintetica(plant, n=50)
    law = SmcLaw(lam=np.full(6, 1e6), eta=np.full(6, 1e6), phi=1e-6, alpha=1.0)
    r = simulate(law, ref, plant)
    assert r.diverged or np.isfinite(r.f1_iae)


# ─────────────────────────────────────────────────────────────────────────────
# Sustituto polinómico
# ─────────────────────────────────────────────────────────────────────────────

def test_base_cubica_tiene_el_numero_de_terminos_correcto():
    """d=3 → 1 + 3 + 6 + 10 = 20 términos (constante, lineal, cuadrático con
    cruzados, cúbico con cruzados)."""
    assert _cubic_features(np.zeros((5, 3))).shape == (5, 20)
    assert _cubic_features(np.zeros((2, 2))).shape == (2, 10)


@pytest.mark.skipif(not os.path.exists(URDF), reason="URDF no instalado")
def test_la_friccion_no_cuenta_como_saturacion_del_actuador():
    """
    `n_sat` mide cuando el CONTROLADOR pide mas par del que el actuador da.

    La friccion es una fuerza de la planta: no la manda nadie y ningun limite de
    actuador la recorta. Se aplica a nivel de velocidad (ver `JointFriction`),
    asi que no puede tocar `n_sat` en absoluto.
    """
    plant = Plant(URDF)
    ref = _ref_sintetica(plant, n=20)
    law = SmcLaw(lam=np.full(6, 1.0), eta=np.full(6, 1e-3), phi=0.1, alpha=0.1)
    fric = JointFriction(f_v=np.zeros(6), f_c=np.array([0, 0, 0, 40.0, 0, 0]))

    r_sin = simulate(law, ref, plant)
    r_con = simulate(law, ref, plant, friction=fric)
    assert r_con.n_sat == r_sin.n_sat


@pytest.mark.skipif(not os.path.exists(URDF), reason="URDF no instalado")
def test_friccion_rigida_en_wrist3_no_diverge():
    """
    REGRESION del artefacto de 2026-08-04 (docs/05_smc.md §7.5).

    Con la friccion real del UR5e sumada al PAR e integrada explicitamente,
    `wrist_3` diverge: el factor de amplificacion del modo viscoso es
    `1 - f_v·dt/M` = 1 - 2.87·0.002/2.6e-4 = -21.3. Y el Coulomb suavizado es
    aun peor, con pendiente `f_c/eps` = 3180 en el origen.

    Aplicada a nivel de velocidad —viscoso implicito, Coulomb por proyeccion—
    tiene que quedarse acotada pase lo que pase.
    """
    plant = Plant(URDF)
    ref = _ref_sintetica(plant, n=400)
    law = SmcLaw(lam=np.full(6, 20.0), eta=INERTIA.copy(), phi=0.05, alpha=0.3)
    fric = JointFriction(f_v=np.array([14.55, 12.30, 14.53, 1.33, 1.85, 2.87]),
                         f_c=np.array([7.31, 7.20, 7.89, 1.85, 2.68, 3.18]))

    r = simulate(law, ref, plant, friction=fric)
    assert not r.diverged, "la friccion rigida hizo divergir el evaluador"
    assert np.isfinite(r.rmse_q)


@pytest.mark.skipif(not os.path.exists(URDF), reason="URDF no instalado")
def test_coulomb_pega_la_junta_en_vez_de_cruzar_el_cero():
    """
    La proyeccion no debe empujar la junta al otro lado: si el impulso de
    friccion supera la cantidad de movimiento, la velocidad queda en CERO
    exacto, no negativa. Es lo que distingue este esquema de restar un par.
    """
    fric = JointFriction(f_v=np.zeros(6), f_c=np.full(6, 1.0))
    M = np.diag(np.full(6, 0.01))          # `apply` trabaja en la metrica de M
    dq = fric.apply(np.full(6, 0.05), M, dt=0.002)   # impulso = 0.2 > 0.05
    np.testing.assert_array_equal(dq, np.zeros(6))
    # Y con holgura de sobra, solo frena:
    dq = fric.apply(np.full(6, 5.0), M, dt=0.002)
    np.testing.assert_allclose(dq, np.full(6, 4.8))


@pytest.mark.skipif(not os.path.exists(URDF), reason="URDF no instalado")
def test_phi_por_junta_escala_chi_junta_a_junta():
    """
    `phi` por junta: `chi_i = (K_i/phi_i)/M_ii` tiene que responder junta a
    junta, no en bloque.

    Es lo que permite darle a wrist_3 la autoridad que necesita sin arrastrar
    al resto: con un phi comun, el unico valor que mantiene chi bajo alli deja
    a las juntas grandes con una capa limite absurdamente ancha.
    """
    plant = Plant(URDF)
    ref = _ref_sintetica(plant, n=5)
    kw = dict(lam=np.full(6, 20.0), eta=np.full(6, 1.0), alpha=0.3)

    ley_esc = SmcLaw(phi=0.05, **kw)
    ley_vec = SmcLaw(phi=np.full(6, 0.05), **kw)
    args = (plant.model, plant.data, Q_INIT, np.zeros(6),
            Q_INIT, np.zeros(6), np.zeros(6))
    # Un escalar y su difusion a seis tienen que dar exactamente lo mismo:
    # es lo que garantiza que los ficheros de ganancias ya generados por la
    # FASE 7, que escriben `phi` escalar, sigan significando lo mismo.
    assert np.allclose(ley_esc(*args)[2]["chi"], ley_vec(*args)[2]["chi"])

    phi = np.array([0.05, 0.05, 0.05, 0.05, 0.05, 5.0])
    chi_ref = ley_esc(*args)[2]["chi"]
    chi_pj = SmcLaw(phi=phi, **kw)(*args)[2]["chi"]
    assert np.allclose(chi_pj[:5], chi_ref[:5]), "un phi por junta afecto a otras"
    assert np.isclose(chi_pj[5], chi_ref[5] * 0.05 / 5.0), "chi_6 no escalo con 1/phi_6"

    with pytest.raises(ValueError):
        SmcLaw(phi=np.array([0.05] * 5 + [0.0]), **kw)


def test_cota_de_friccion_residual_distingue_arrancar_de_estar_quieta():
    """
    La friccion estatica ENTERA solo es perturbacion donde la junta tiene que
    arrancar. Una junta que no se mueve en toda la trayectoria no necesita
    romper nada —la friccion la ayuda a sostenerse— y cargarle `f_c` le pediria
    al optimizador una `eta` para un problema que no tiene.
    """
    n = 100
    dq = np.zeros((n, 6))
    dq[:, 0] = np.linspace(-0.5, 0.5, n)      # invierte: SI arranca
    dq[:, 1] = 0.5                            # nunca se para: no arranca
    # la junta 2 se queda en cero todo el rato: tampoco arranca
    ref = Reference(dt=0.002, q=np.zeros((n, 6)), dq=dq, ddq=np.zeros((n, 6)),
                    phase=np.array(["TRACK"] * n))
    fr = JointFriction(f_v=np.full(6, 1.0), f_c=np.full(6, 5.0))
    d = friction_residual_bound(ref, fr)

    assert d[0] == pytest.approx(5.0), "la junta que invierte necesita todo f_c"
    assert d[1] < 1.0, "la junta que nunca se para solo arrastra error de parametros"
    assert d[2] < 1.0, "la junta inmovil no necesita margen de arranque"


def test_chi_se_normaliza_por_junta_y_no_en_bloque():
    """
    El umbral de ciclo limite NO es el mismo en todas las juntas: medido con
    friccion va de 0.22 en shoulder_lift a 0.99 en wrist_1, un factor 5. La
    restriccion tiene que comparar cada junta con el SUYO.

    Con un escalar unico pasaba lo peor de los dos mundos: se aceptaban
    ganancias que hacen chatear al hombro y se rechazaban ganancias validas en
    la muñeca.
    """
    assert CHI_THRESHOLD.shape == (6,)
    assert np.all(CHI_THRESHOLD > 0)
    # La muñeca aguanta bastante mas que el hombro; si esto se invierte, alguien
    # copio los numeros en el orden equivocado.
    assert CHI_THRESHOLD[3] > 3.0 * CHI_THRESHOLD[1]

    # Una junta al 90 % de SU umbral viola un factor 0.75 aunque su chi absoluto
    # sea pequeño; y otra con chi mayor pero al 50 % del suyo, no.
    chi = 0.90 * CHI_THRESHOLD.copy()
    assert np.max(chi / CHI_THRESHOLD) > 0.75
    chi = CHI_THRESHOLD.copy() * 0.5
    assert np.max(chi / CHI_THRESHOLD) < 0.75
    # chi absoluto de la muñeca (0.495) > el del hombro (0.11) y aun asi ambos
    # son factibles: es justo lo que un limite escalar no sabe expresar.
    assert chi[3] > CHI_THRESHOLD[1]


def test_full_phi_busca_seis_phi_y_los_conserva():
    """
    `full_phi` amplia la busqueda a phi POR JUNTA (18 variables).

    Hace falta porque el umbral de ciclo limite es por junta: con un phi unico,
    el optimo tiene que subirlo hasta satisfacer el MAS BAJO y arrastra a las
    demas. Medido: la corrida en modo `full` dio phi = 0.69 en las seis cuando
    las muñecas toleran chi = 0.99, y como `e_ss ~ phi` eso paga error
    permanente en cinco juntas para proteger a una.
    """
    p = SmcParameterization(inertia=INERTIA, mode="full_phi")
    assert p.n_var == 18
    assert p.names[-6:] == [f"phi{i}" for i in range(1, 7)]

    lam = np.full(6, 20.0)
    phi = np.array([0.05, 0.07, 0.07, 0.07, 0.05, 0.20])
    lam2, eta2, phi2 = p.gains(p.encode(lam, INERTIA, phi))
    np.testing.assert_allclose(phi2, phi, rtol=1e-12)
    np.testing.assert_allclose(eta2, INERTIA, rtol=1e-9)

    # Un escalar se difunde a las seis, para poder sembrar con la config manual.
    _, _, phi3 = p.gains(p.encode(lam, INERTIA, 0.05))
    np.testing.assert_allclose(phi3, np.full(6, 0.05), rtol=1e-12)

    # Y el modo `full` RECHAZA un phi vectorial en vez de quedarse con el primero
    # en silencio, que es como se acaba optimizando otra cosa de la que se cree.
    with pytest.raises(ValueError):
        SmcParameterization(inertia=INERTIA, mode="full").encode(lam, INERTIA, phi)
