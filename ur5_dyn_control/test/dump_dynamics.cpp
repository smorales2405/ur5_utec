// Vuelca M(q), g(q) y C(q, dq) de Ur5Dynamics para el test cruzado con el
// evaluador offline de Python (test_dynamics_cross.py, B0.4 del plan).
//
// Uso: dump_dynamics <urdf> <gravedad> <tcp_offset_z> <masa> <cx> <cy> <cz>
//                    <Ixx> <Iyy> <Izz> <Ixy> <Ixz> <Iyz>
// stdin : una linea por configuracion, 12 numeros: q(6) dq(6)
// stdout: una linea por configuracion: M (36, por filas), g (6), C (36)
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

#include "ur5_dyn_control/ur5_dynamics.hpp"

int main(int argc, char ** argv)
{
  if (argc != 14) {
    std::cerr << "uso: dump_dynamics <urdf> <g> <tcp> <m> <cx> <cy> <cz> "
                 "<Ixx> <Iyy> <Izz> <Ixy> <Ixz> <Iyz>\n";
    return 2;
  }
  auto num = [&](int i) {return std::strtod(argv[i], nullptr);};
  ur5_dyn_control::ToolInertia tool;
  tool.mass = num(4);
  tool.com = Eigen::Vector3d(num(5), num(6), num(7));
  // Mismo orden y misma construccion que torque_control_node_base.cpp.
  tool.inertia << num(8), num(11), num(12),
    num(11), num(9), num(13),
    num(12), num(13), num(10);
  ur5_dyn_control::Ur5Dynamics dyn(argv[1], num(2), num(3), tool);

  std::cout << std::setprecision(17);
  ur5_dyn_control::Vector6d q, dq;
  while (true) {
    for (int i = 0; i < 6; ++i) {if (!(std::cin >> q[i])) {return 0;}}
    for (int i = 0; i < 6; ++i) {if (!(std::cin >> dq[i])) {return 1;}}
    const auto M = dyn.M(q);
    const auto g = dyn.gravity(q);
    const auto C = dyn.coriolis(q, dq);
    for (int r = 0; r < 6; ++r) {for (int c = 0; c < 6; ++c) {std::cout << M(r, c) << ' ';}}
    for (int r = 0; r < 6; ++r) {std::cout << g[r] << ' ';}
    for (int r = 0; r < 6; ++r) {for (int c = 0; c < 6; ++c) {std::cout << C(r, c) << ' ';}}
    std::cout << '\n';
  }
}
