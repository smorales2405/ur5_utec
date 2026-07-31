#include "ur5_dyn_control/care_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace ur5_dyn_control
{

namespace
{

using Eigen::MatrixXd;

/**
 * Balanceado SIMPLECTICO diagonal del problema (A, G, Q).
 *
 * La similaridad T = diag(D, D^-1) con D = diag(d) preserva la estructura
 * hamiltoniana y transforma
 *
 *     A -> D^-1 A D,     G -> D^-1 G D^-1,     Q -> D Q D
 *
 * POR QUE HACE FALTA. La planta de la FASE 4 tiene inercias articulares que van
 * de 2.6e-4 a 2.59 kg.m^2 (CUATRO ordenes de magnitud). Con la Q escalada por
 * inercia (Qp ~ I^2 wn^4) eso son OCHO ordenes en Q, y G = B R^-1 B^T ~ M^-2
 * pone otros ocho en sentido contrario. Sin balancear, la iteracion de Newton
 * de la funcion signo se estanca con residuo relativo ~1e-4: los bloques
 * pequenos quedan sepultados bajo el error de redondeo de los grandes. MEDIDO
 * (test_care_solver, antes de anadir esto).
 *
 * Escalado por indice, a la Parlett-Reinsch: para el indice i, subir d_i hace
 * crecer la fila/columna i de Q y la columna i de A, y decrecer la fila/columna
 * i de G y la fila i de A. Igualando ambas sumas sale f = sqrt(q_i / p_i). Se
 * redondea a POTENCIA DE 2 para que el escalado sea exacto en binario y no
 * meta error de redondeo propio (misma razon por la que lo hace LAPACK).
 */
void balanceHamiltonian(MatrixXd & A, MatrixXd & G, MatrixXd & Q,
                        Eigen::VectorXd & d)
{
  const int n = static_cast<int>(A.rows());
  d = Eigen::VectorXd::Ones(n);

  constexpr int kMaxSweeps = 20;
  for (int sweep = 0; sweep < kMaxSweeps; ++sweep) {
    bool changed = false;
    for (int i = 0; i < n; ++i) {
      // Lo que CRECE con d_i: columna i de A (sin la diagonal) y fila i de Q.
      const double p = A.col(i).cwiseAbs().sum() - std::abs(A(i, i)) +
        Q.row(i).cwiseAbs().sum();
      // Lo que DECRECE con d_i: fila i de A (sin la diagonal) y fila i de G.
      const double q = A.row(i).cwiseAbs().sum() - std::abs(A(i, i)) +
        G.row(i).cwiseAbs().sum();
      if (!(p > 0.0) || !(q > 0.0)) {continue;}

      const double f = std::exp2(std::round(0.5 * std::log2(q / p)));
      if (!(f > 0.0) || !std::isfinite(f) || f == 1.0) {continue;}
      changed = true;

      d(i) *= f;
      A.row(i) /= f;
      A.col(i) *= f;
      G.row(i) /= f;
      G.col(i) /= f;
      Q.row(i) *= f;
      Q.col(i) *= f;
    }
    if (!changed) {break;}
  }
}

/// log|det(S)| a partir de la diagonal de U de la LU. Se evita det() directo
/// porque el determinante de una 24x24 con autovalores de orden 1e2 desborda
/// el rango de un double mucho antes de que el escalado deje de hacer falta.
bool logAbsDet(const Eigen::PartialPivLU<MatrixXd> & lu, int n, double & out)
{
  const MatrixXd & LU = lu.matrixLU();
  double acc = 0.0;
  for (int i = 0; i < n; ++i) {
    const double d = std::abs(LU(i, i));
    if (!(d > 0.0)) {return false;}     // singular: H tiene un autovalor en 0
    acc += std::log(d);
  }
  out = acc;
  return std::isfinite(acc);
}

}  // namespace

CareResult solveCare(const MatrixXd & A, const MatrixXd & B,
                     const MatrixXd & Q, const MatrixXd & R,
                     const CareOptions & opt)
{
  const int n = static_cast<int>(A.rows());
  const int m = static_cast<int>(B.cols());

  CareResult res;
  res.P = MatrixXd::Zero(n, n);
  res.K = MatrixXd::Zero(m, n);

  if (A.cols() != n || B.rows() != n || Q.rows() != n || Q.cols() != n ||
      R.rows() != m || R.cols() != m)
  {
    res.reason = "dimensiones incoherentes";
    return res;
  }

  // R debe ser definida positiva: si no, R^-1 no existe y el problema LQR no
  // esta bien planteado (coste de control no penalizado en alguna direccion).
  const Eigen::LLT<MatrixXd> Rllt(R);
  if (Rllt.info() != Eigen::Success) {
    res.reason = "R no es definida positiva";
    return res;
  }
  // ── Cambio de coordenadas: z = D^-1 x ────────────────────────────────────
  // Se resuelve TODO (iteracion, residuo, autovalores del lazo cerrado) en las
  // coordenadas balanceadas, que es donde los numeros son comparables entre si;
  // al final se deshace con  P = D^-1 P_z D^-1,  K = K_z D^-1.
  MatrixXd Az = A;
  MatrixXd Gz = B * Rllt.solve(B.transpose());
  MatrixXd Qz = Q;
  Eigen::VectorXd d;
  balanceHamiltonian(Az, Gz, Qz, d);
  const Eigen::VectorXd dinv = d.cwiseInverse();

  // Balance escalar entre los bloques G y Q: con P = c P~, la CARE en P~ tiene
  // G~ = c G y Q~ = Q / c. Con c = sqrt(||Q||/||G||) los dos bloques fuera de
  // la diagonal del hamiltoniano quedan con la misma norma.
  const double gz_norm = Gz.norm();
  const double qz_norm = Qz.norm();
  const double c = (gz_norm > 0.0 && qz_norm > 0.0)
    ? std::sqrt(qz_norm / gz_norm)
    : 1.0;

  const int n2 = 2 * n;
  MatrixXd S(n2, n2);
  S.topLeftCorner(n, n) = Az;
  S.topRightCorner(n, n) = -c * Gz;
  S.bottomLeftCorner(n, n) = -Qz / c;
  S.bottomRightCorner(n, n) = -Az.transpose();

  // ── Newton para sign(H): S <- (mu S + S^-1 / mu) / 2 ──────────────────────
  // OJO: `mu` es el escalado por DETERMINANTE de cada paso de Newton, distinto
  // de la `c` de arriba (balance escalar Q/G, que se deshace al final). Se
  // llaman distinto a proposito: con el mismo nombre el de dentro tapaba al de
  // fuera y el codigo seguia siendo correcto solo por el ambito.
  bool converged = false;
  bool scaling = true;
  double err_prev = std::numeric_limits<double>::infinity();
  MatrixXd S_next(n2, n2);
  for (int it = 0; it < opt.max_iterations; ++it) {
    const Eigen::PartialPivLU<MatrixXd> lu(S);
    double logdet = 0.0;
    if (!logAbsDet(lu, n2, logdet)) {
      // Un autovalor en cero (o en el eje imaginario, que lo lleva ahi) => el
      // subespacio estable no tiene dimension n y no hay solucion estabilizante.
      res.iterations = it;
      res.reason = "hamiltoniano singular (autovalor en el eje imaginario)";
      return res;
    }
    const double mu = scaling ? std::exp(-logdet / n2) : 1.0;

    S_next.noalias() = 0.5 * (mu * S + lu.inverse() / mu);
    const double s_norm = S.norm();
    const double err = (S_next - S).norm() / std::max(1.0, s_norm);
    S.swap(S_next);
    res.iterations = it + 1;

    if (err < opt.tol) {converged = true; break;}
    // Cerca del punto fijo el escalado deja de ayudar y mete ruido: se apaga.
    if (err < 1e-2) {scaling = false;}
    // Estancamiento: en aritmetica finita el error deja de bajar antes de tol.
    // Un paso que no mejora ya no va a mejorar (la convergencia es cuadratica).
    if (!scaling && err >= err_prev) {converged = true; break;}
    err_prev = err;
  }
  if (!converged) {
    res.reason = "la iteracion de la funcion signo no convergio";
    return res;
  }

  // ── Subespacio estable: (I + S) [I; P] = 0 ────────────────────────────────
  MatrixXd lhs(n2, n);
  lhs.topRows(n) = S.topRightCorner(n, n);
  lhs.bottomRows(n) = S.bottomRightCorner(n, n) + MatrixXd::Identity(n, n);
  MatrixXd rhs(n2, n);
  rhs.topRows(n) = -(S.topLeftCorner(n, n) + MatrixXd::Identity(n, n));
  rhs.bottomRows(n) = -S.bottomLeftCorner(n, n);

  Eigen::ColPivHouseholderQR<MatrixXd> qr(lhs);
  qr.setThreshold(1e-10);
  if (qr.rank() < n) {
    res.reason = "subespacio estable degenerado (rango " +
      std::to_string(qr.rank()) + " < " + std::to_string(n) + ")";
    return res;
  }
  const MatrixXd P_raw = qr.solve(rhs);
  if (!P_raw.allFinite()) {
    res.reason = "solucion no finita";
    return res;
  }

  // P debe salir simetrica por construccion; la asimetria residual es un
  // indicador de condicionamiento util, asi que se mide ANTES de simetrizar.
  const double p_norm = P_raw.norm();
  res.asymmetry = (p_norm > 0.0)
    ? (P_raw - P_raw.transpose()).norm() / p_norm
    : 0.0;
  const MatrixXd Pz = 0.5 * c * (P_raw + P_raw.transpose());

  // ── Verificacion: residuo de la CARE y estabilidad del lazo cerrado ───────
  // Ambas cosas EN COORDENADAS BALANCEADAS. El residuo relativo en las
  // coordenadas originales seria enganoso: al normalizar por la norma global
  // solo mide el bloque del hombro, ocho ordenes por encima del de wrist_3, y
  // daria por buena una K con la muneca completamente equivocada.
  // Los autovalores son invariantes bajo similaridad, asi que los del lazo
  // cerrado balanceado SON los de A - B K.
  const MatrixXd AtP = Az.transpose() * Pz;
  const MatrixXd PGP = Pz * Gz * Pz;
  const MatrixXd resid = AtP + AtP.transpose() - PGP + Qz;
  const double scale = std::max(
    {AtP.norm(), PGP.norm(), Qz.norm(), 1e-12});
  res.residual = resid.norm() / scale;

  // Deshacer el balanceado: P = D^-1 P_z D^-1.
  res.P = dinv.asDiagonal() * Pz * dinv.asDiagonal();
  res.K = Rllt.solve(B.transpose() * res.P);

  if (opt.check_closed_loop) {
    // A_z - B_z K_z = A_z - G_z P_z (misma expresion, una multiplicacion menos).
    const Eigen::EigenSolver<MatrixXd> es(Az - Gz * Pz, /*computeEigenvectors=*/false);
    if (es.info() != Eigen::Success) {
      res.reason = "no se pudo calcular eig(A - B K)";
      res.P.setZero();
      res.K.setZero();
      return res;
    }
    res.max_real_eig = es.eigenvalues().real().maxCoeff();
  }

  if (res.residual > opt.residual_tol) {
    res.reason = "residuo relativo de la CARE " + std::to_string(res.residual) +
      " > " + std::to_string(opt.residual_tol);
    res.P.setZero();
    res.K.setZero();
    return res;
  }
  if (opt.check_closed_loop && !(res.max_real_eig < -opt.stable_margin)) {
    res.reason = "lazo cerrado no estable: max Re(eig) = " +
      std::to_string(res.max_real_eig);
    res.P.setZero();
    res.K.setZero();
    return res;
  }

  res.ok = true;
  res.reason = "ok";
  return res;
}

double controllabilityMargin(const MatrixXd & A, const MatrixXd & B)
{
  const int n = static_cast<int>(A.rows());
  const int m = static_cast<int>(B.cols());
  if (n <= 0 || m <= 0 || A.cols() != n || B.rows() != n) {return 0.0;}

  // Cayley-Hamilton: con m entradas el rango deja de crecer, como muy tarde,
  // en el bloque n - m + 1. Se para en cuanto llega a rango n, que para el par
  // de la FASE 4 ocurre en el segundo bloque.
  const int max_blocks = std::max(1, n - m + 1);
  MatrixXd blk = B;
  MatrixXd kal(n, 0);
  double margin = 0.0;
  for (int k = 0; k < max_blocks; ++k) {
    const int c0 = static_cast<int>(kal.cols());
    kal.conservativeResize(n, c0 + m);
    kal.rightCols(m) = blk;

    const Eigen::JacobiSVD<MatrixXd> svd(kal);
    const auto & sv = svd.singularValues();
    const double smax = sv(0);
    const double smin = sv(std::min<int>(n, static_cast<int>(sv.size())) - 1);
    margin = (smax > 0.0 && sv.size() >= n) ? smin / smax : 0.0;
    if (margin > 1e-12) {return margin;}

    blk = (A * blk).eval();
  }
  return margin;
}

}  // namespace ur5_dyn_control
