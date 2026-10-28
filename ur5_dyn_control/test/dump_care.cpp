// Resuelve una CARE con solveCare para compararla con scipy (test_care_scipy.py,
// B4.2). stdin: n m, luego A (n x n), B (n x m), Q (n x n), R (m x m) por filas.
// stdout: ok y K (m x n) por filas.
#include <iomanip>
#include <iostream>

#include "ur5_dyn_control/care_solver.hpp"

int main()
{
  int n = 0, m = 0;
  if (!(std::cin >> n >> m)) {return 2;}
  auto read = [](int r, int c) {
      Eigen::MatrixXd X(r, c);
      for (int i = 0; i < r; ++i) {for (int j = 0; j < c; ++j) {std::cin >> X(i, j);}}
      return X;
    };
  const Eigen::MatrixXd A = read(n, n), B = read(n, m), Q = read(n, n), R = read(m, m);
  const auto res = ur5_dyn_control::solveCare(A, B, Q, R);
  std::cout << std::setprecision(17) << (res.ok ? 1 : 0) << '\n';
  for (int i = 0; i < res.K.rows(); ++i) {
    for (int j = 0; j < res.K.cols(); ++j) {std::cout << res.K(i, j) << ' ';}
    std::cout << '\n';
  }
  return res.ok ? 0 : 1;
}
