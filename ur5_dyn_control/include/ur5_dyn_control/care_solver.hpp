#ifndef UR5_DYN_CONTROL_CARE_SOLVER_HPP
#define UR5_DYN_CONTROL_CARE_SOLVER_HPP

#include <string>

#include <Eigen/Dense>

namespace ur5_dyn_control
{

/// Opciones del solver de la ecuacion algebraica de Riccati (CARE).
struct CareOptions
{
  /// Iteraciones maximas del Newton de la funcion signo.
  int max_iterations = 60;
  /// Convergencia relativa ||S_{k+1} - S_k||_F / ||S_k||_F.
  double tol = 1e-11;
  /// Residuo relativo maximo de la CARE que se acepta como solucion valida.
  double residual_tol = 1e-6;
  /// Se exige max Re(eig(A - B K)) < -stable_margin.
  double stable_margin = 0.0;
  /// Se calcula max Re(eig(A - B K)). Cuesta una descomposicion propia n x n;
  /// ponerlo a false solo tiene sentido si el llamante no va a mirar el dato.
  bool check_closed_loop = true;
};

/// Resultado de solveCare(). Con ok = false, P y K quedan a cero: el llamante
/// NUNCA debe usarlas (la politica de reserva es cosa suya, no del solver).
struct CareResult
{
  Eigen::MatrixXd P;              ///< solucion estabilizante, simetrica (n x n)
  Eigen::MatrixXd K;              ///< K = R^-1 B^T P  (m x n)
  double residual = 0.0;          ///< ||A^T P + P A - P B R^-1 B^T P + Q|| relativo
  double max_real_eig = 0.0;      ///< max Re(eig(A - B K)); solo si check_closed_loop
  double asymmetry = 0.0;         ///< ||P - P^T||_F / ||P||_F antes de simetrizar
  int iterations = 0;             ///< pasos de Newton consumidos
  bool ok = false;
  std::string reason = "sin resolver";
};

/**
 * Resuelve la CARE de tiempo continuo
 *
 *     A^T P + P A - P B R^-1 B^T P + Q = 0,      K = R^-1 B^T P
 *
 * por la FUNCION SIGNO de la matriz hamiltoniana (Roberts, 1971/1980):
 *
 *     H = [  A      -B R^-1 B^T ]        S = sign(H)  via  S <- (S + S^-1)/2
 *         [ -Q      -A^T        ]
 *
 * El subespacio invariante ESTABLE de H es ker(I + S). Si [I; P] es una base de
 * ese subespacio, (I + S) [I; P] = 0 da el sistema sobredeterminado
 *
 *     [ S12      ] P  =  -[ S11 + I ]
 *     [ S22 + I  ]        [ S21     ]
 *
 * que se resuelve por minimos cuadrados (QR con pivoteo de columnas).
 *
 * POR QUE LA FUNCION SIGNO Y NO SCHUR NI AUTOVECTORES.
 *
 *  - Schur ORDENADO es el metodo de referencia, pero Eigen::RealSchur no sabe
 *    reordenar autovalores y el intercambio de bloques 1x1/2x2 habria que
 *    escribirlo entero.
 *  - El metodo de AUTOVECTORES del hamiltoniano (tomar los n autovectores con
 *    Re < 0 y hacer P = X2 X1^-1) se rompe justo en el caso de diseno de este
 *    paquete: con amortiguamiento critico (zeta = 1) el lazo cerrado de cada
 *    junta tiene un POLO DOBLE en -wn, y A - B K en forma companera con
 *    autovalor doble es DEFECTIVA -> solo hay un autovector, X1 queda singular
 *    y P sale basura. Esta comprobado en test_care_solver.cpp.
 *  - La funcion signo solo necesita que H no tenga autovalores en el eje
 *    imaginario (condicion necesaria para que exista solucion estabilizante de
 *    todos modos) y es insensible a que los autovalores esten repetidos: lo que
 *    calcula es el PROYECTOR sobre el subespacio, no una base de autovectores.
 *
 * La iteracion lleva escalado por determinante c_k = |det S_k|^(-1/2n) (se
 * calcula por log|U_ii| de la LU, porque det de una 24x24 desborda), que es lo
 * que la hace converger en ~10 pasos en vez de ~40. El escalado se apaga al
 * acercarse a la convergencia: cerca del punto fijo estorba.
 *
 * Devuelve ok = false (y P, K a cero) si R no es definida positiva, si H es
 * singular, si la iteracion no converge, si el subespacio sale degenerado, si
 * el residuo supera residual_tol o si el lazo cerrado no es estable.
 */
CareResult solveCare(const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
                     const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R,
                     const CareOptions & opt = CareOptions());

/**
 * Margen de controlabilidad del par (A, B): sigma_min / sigma_max de la matriz
 * de Kalman [B, A B, A^2 B, ...], construida bloque a bloque y detenida en
 * cuanto alcanza rango n (anadir mas bloques ya no puede subir el rango).
 *
 * Devuelve 0 si el par no es controlable. Controlable => estabilizable, asi que
 * un margen > 0 CERTIFICA la estabilizabilidad; lo contrario no es cierto (un
 * par no controlable puede seguir siendo estabilizable), y por eso el nodo
 * trata un margen nulo como "no certificado" y no como "no estabilizable".
 *
 * Para el par de la FASE 4, A = [0 I; 0 -M^-1 C] y B = [0; M^-1], ya
 * [B, A B] = [0 M^-1; M^-1 -M^-1 C M^-1] tiene rango 12 siempre que M sea
 * invertible: la unica via realista de perder la certificacion es que M(q) se
 * vuelva numericamente singular, que es justo lo que vigila cond(M).
 */
double controllabilityMargin(const Eigen::MatrixXd & A, const Eigen::MatrixXd & B);

}  // namespace ur5_dyn_control

#endif  // UR5_DYN_CONTROL_CARE_SOLVER_HPP
