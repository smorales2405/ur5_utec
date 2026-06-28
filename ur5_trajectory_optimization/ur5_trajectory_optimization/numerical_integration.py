"""
Cuadraturas numéricas de la Unidad 5 — implementadas a mano (Trabajo Integrador).

Reglas sobre una función callable ``f(t)`` en ``[a, b]``:
  - ``trapezoid``      — Newton-Cotes cerrada, regla compuesta del trapecio.
  - ``simpson``        — Simpson 1/3 compuesta (n par).
  - ``romberg``        — trapecio con bisección sucesiva + Richardson.
  - ``gauss_legendre`` — nodos/pesos de Gauss-Legendre mapeados de [-1,1] a [a,b].

Variantes tabulares (datos ya muestreados ``y`` sobre ``x``):
  - ``trapezoid_samples`` — equivalente a ``np.trapz`` (referencia).
  - ``simpson_samples``   — Simpson 1/3 sobre la malla tabular.

Para la columna de eficiencia del estudio de convergencia se ofrece
``CountingFunction``, un envoltorio que cuenta cuántas veces se evaluó el
integrando.  Las reglas cerradas tienen un número de evaluaciones determinista
(trapecio/Simpson: ``n+1``; Gauss: ``n``; Romberg: ``2**max_k + 1``), pero el
contador funciona de forma uniforme para todas.

Sólo NumPy.  ``scipy`` / ``np.trapz`` se usan únicamente como referencia externa
en los scripts, no aquí.
"""

from __future__ import annotations
import numpy as np
from typing import Callable, Tuple


# ─────────────────────────────────────────────────────────────────────────────
# Contador de evaluaciones del integrando
# ─────────────────────────────────────────────────────────────────────────────

class CountingFunction:
    """
    Envuelve un callable ``f(t)`` y cuenta cuántas veces se le invoca.

    Permite reportar el número de evaluaciones del integrando que consumió
    cada regla de cuadratura (columna ``n_evals`` del estudio de convergencia).

    Examples
    --------
    >>> cf = CountingFunction(np.sin)
    >>> _ = trapezoid(cf, 0.0, np.pi, 8)
    >>> cf.n_calls
    9
    """

    def __init__(self, f: Callable[[float], float]) -> None:
        self._f = f
        self.n_calls = 0

    def __call__(self, t: float) -> float:
        self.n_calls += 1
        return self._f(t)

    def reset(self) -> None:
        self.n_calls = 0


# ─────────────────────────────────────────────────────────────────────────────
# Newton-Cotes cerradas
# ─────────────────────────────────────────────────────────────────────────────

def trapezoid(f: Callable[[float], float], a: float, b: float, n: int) -> float:
    """
    Regla compuesta del trapecio con ``n`` subintervalos.

    ``∫_a^b f(t) dt ≈ h·[½f(a) + Σ_{i=1}^{n-1} f(a+ih) + ½f(b)]``,  ``h=(b-a)/n``.

    Usa ``n+1`` evaluaciones del integrando.
    """
    if n < 1:
        raise ValueError("n debe ser >= 1")
    h = (b - a) / n
    x = a + h * np.arange(n + 1)
    y = np.array([f(xi) for xi in x], dtype=float)
    return float(h * (0.5 * y[0] + 0.5 * y[-1] + y[1:-1].sum()))


def simpson(f: Callable[[float], float], a: float, b: float, n: int) -> float:
    """
    Regla compuesta de Simpson 1/3 con ``n`` subintervalos (``n`` par).

    Si ``n`` es impar se incrementa en 1 para cumplir la condición de Simpson.
    Usa ``n+1`` evaluaciones del integrando.
    """
    if n < 2:
        raise ValueError("n debe ser >= 2 para Simpson")
    if n % 2 == 1:
        n += 1
    h = (b - a) / n
    x = a + h * np.arange(n + 1)
    y = np.array([f(xi) for xi in x], dtype=float)
    s = y[0] + y[-1] + 4.0 * y[1:-1:2].sum() + 2.0 * y[2:-1:2].sum()
    return float(h / 3.0 * s)


# ─────────────────────────────────────────────────────────────────────────────
# Romberg (trapecio + extrapolación de Richardson)
# ─────────────────────────────────────────────────────────────────────────────

def romberg(
    f:     Callable[[float], float],
    a:     float,
    b:     float,
    max_k: int   = 6,
    tol:   float = 1e-8,
) -> Tuple[float, np.ndarray]:
    """
    Integración de Romberg: trapecio con bisección sucesiva del paso y
    extrapolación de Richardson en la tabla triangular ``R``.

    ``R[k,0]``  = trapecio compuesto con ``2**k`` subintervalos (reutiliza
                  los puntos del nivel anterior).
    ``R[k,j]``  = ``R[k,j-1] + (R[k,j-1] - R[k-1,j-1]) / (4**j - 1)``.

    Parameters
    ----------
    max_k : niveles de bisección (``2**max_k`` subintervalos como máximo).
    tol   : si ``tol > 0`` se detiene cuando ``|R[k,k]-R[k-1,k-1]| < tol``;
            con ``tol <= 0`` se ejecutan siempre los ``max_k`` niveles
            (útil para el estudio de convergencia a ``n`` fijo).

    Returns
    -------
    value : mejor estimación ``R[k,k]``.
    table : tabla triangular usada (sub-bloque ``(k+1, k+1)`` de ``R``).

    Usa ``2**k + 1`` evaluaciones del integrando hasta el nivel ``k`` alcanzado.
    """
    if max_k < 1:
        raise ValueError("max_k debe ser >= 1")
    R = np.zeros((max_k + 1, max_k + 1))
    h = b - a
    R[0, 0] = 0.5 * h * (f(a) + f(b))

    for k in range(1, max_k + 1):
        h *= 0.5
        # Puntos nuevos de esta bisección: los de índice impar (2^(k-1) puntos).
        s = 0.0
        for i in range(1, 2 ** k, 2):
            s += f(a + i * h)
        R[k, 0] = 0.5 * R[k - 1, 0] + h * s
        for j in range(1, k + 1):
            R[k, j] = R[k, j - 1] + (R[k, j - 1] - R[k - 1, j - 1]) / (4.0 ** j - 1.0)
        if tol > 0.0 and abs(R[k, k] - R[k - 1, k - 1]) < tol:
            return float(R[k, k]), R[:k + 1, :k + 1]

    return float(R[max_k, max_k]), R


# ─────────────────────────────────────────────────────────────────────────────
# Gauss-Legendre
# ─────────────────────────────────────────────────────────────────────────────

def gauss_legendre(f: Callable[[float], float], a: float, b: float, n: int) -> float:
    """
    Cuadratura de Gauss-Legendre de ``n`` nodos.

    Nodos/pesos en [-1,1] vía ``np.polynomial.legendre.leggauss`` (raíces de
    P_n implementadas en NumPy), mapeados a ``[a,b]``:

      ``t = ½(b-a)·ξ + ½(a+b)``,   ``∫ ≈ ½(b-a)·Σ wᵢ f(tᵢ)``.

    Usa exactamente ``n`` evaluaciones del integrando.
    """
    if n < 1:
        raise ValueError("n debe ser >= 1")
    xi, wi = np.polynomial.legendre.leggauss(n)
    half = 0.5 * (b - a)
    mid  = 0.5 * (a + b)
    t = half * xi + mid
    y = np.array([f(ti) for ti in t], dtype=float)
    return float(half * np.dot(wi, y))


# ─────────────────────────────────────────────────────────────────────────────
# Variantes tabulares (datos ya muestreados)
# ─────────────────────────────────────────────────────────────────────────────

def trapezoid_samples(y: np.ndarray, x: np.ndarray) -> float:
    """
    Regla del trapecio sobre datos tabulados ``(x, y)``, paso posiblemente
    no uniforme.  Equivalente a ``np.trapz(y, x)`` (referencia del CU3).
    """
    y = np.asarray(y, dtype=float)
    x = np.asarray(x, dtype=float)
    if len(y) < 2:
        return 0.0
    return float(np.sum(0.5 * (y[1:] + y[:-1]) * (x[1:] - x[:-1])))


def simpson_samples(y: np.ndarray, x: np.ndarray) -> float:
    """
    Regla de Simpson 1/3 sobre datos tabulados ``(x, y)`` con paso ~uniforme.

    Requiere número impar de puntos (``n`` par de intervalos).  Si el número de
    puntos es par, aplica Simpson a los primeros ``n-1`` intervalos y cierra el
    último con un trapecio, evitando romper el muestreo del CU3.
    """
    y = np.asarray(y, dtype=float)
    x = np.asarray(x, dtype=float)
    n = len(y) - 1
    if n < 2:
        return trapezoid_samples(y, x)

    h = (x[-1] - x[0]) / n
    m = n if n % 2 == 0 else n - 1   # número par de intervalos para Simpson
    s = y[0] + y[m] + 4.0 * y[1:m:2].sum() + 2.0 * y[2:m:2].sum()
    total = h / 3.0 * s
    if m != n:   # intervalo sobrante → trapecio
        total += 0.5 * (y[m] + y[m + 1]) * (x[m + 1] - x[m])
    return float(total)
