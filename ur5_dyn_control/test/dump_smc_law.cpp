// Evalua smcLaw (smc_law.hpp) para el test dorado contra la SmcLaw del
// evaluador offline (test_smc_law_cross.py, B4.3 del plan).
//
// Uso: dump_smc_law <urdf> <gravedad> <tcp_offset_z> <masa> <cx> <cy> <cz>
//                   <Ixx> <Iyy> <Izz> <Ixy> <Ixz> <Iyz>
// stdin : una linea por caso, 52 numeros: q dq q_d dq_d ddq_d lambda eta phi
//         (6 cada uno) y alpha_M alpha_C alpha_g use_sat
// stdout: una linea por caso: tau (6), s (6), K (6)
#include <cstdlib>
#include <iomanip>
#include <iostream>

#include "ur5_dyn_control/smc_law.hpp"

int main(int argc, char ** argv)
{
  using ur5_dyn_control::Vector6d;
  if (argc != 14) {
    std::cerr << "uso: dump_smc_law <urdf> <g> <tcp> <m> <cx> <cy> <cz> "
                 "<Ixx> <Iyy> <Izz> <Ixy> <Ixz> <Iyz>\n";
    return 2;
  }
  auto num = [&](int i) {return std::strtod(argv[i], nullptr);};
  ur5_dyn_control::ToolInertia tool;
  tool.mass = num(4);
  tool.com = Eigen::Vector3d(num(5), num(6), num(7));
  tool.inertia << num(8), num(11), num(12),
    num(11), num(9), num(13),
    num(12), num(13), num(10);
  ur5_dyn_control::Ur5Dynamics dyn(argv[1], num(2), num(3), tool);

  auto read6 = [](Vector6d & v) {
      for (int i = 0; i < 6; ++i) {if (!(std::cin >> v[i])) {return false;}}
      return true;
    };
  std::cout << std::setprecision(17);
  while (true) {
    Vector6d q, dq;
    ur5_dyn_control::JointRef ref;
    ur5_dyn_control::SmcGains g;
    if (!read6(q)) {return 0;}
    double use_sat = 1.0;
    if (!(read6(dq) && read6(ref.q) && read6(ref.dq) && read6(ref.ddq) &&
      read6(g.lambda) && read6(g.eta) && read6(g.phi) &&
      (std::cin >> g.alpha_M >> g.alpha_C >> g.alpha_g >> use_sat)))
    {
      return 1;
    }
    g.use_sat = use_sat != 0.0;
    const auto o = ur5_dyn_control::smcLaw(dyn, q, dq, ref, g);
    for (const Vector6d * v : {&o.tau, &o.s, &o.K}) {
      for (int i = 0; i < 6; ++i) {std::cout << (*v)[i] << ' ';}
    }
    std::cout << '\n';
  }
}
