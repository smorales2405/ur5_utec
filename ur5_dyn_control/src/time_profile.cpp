#include "ur5_dyn_control/time_profile.hpp"

#include <algorithm>
#include <stdexcept>

namespace ur5_dyn_control
{

namespace
{

// Quintico "smoothstep" y sus derivadas respecto de x in [0, 1].
//   S(0)=0, S(1)=1, S'(0)=S'(1)=0, S''(0)=S''(1)=0
inline double S(double x) {return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));}
inline double dS(double x) {return 30.0 * x * x * (1.0 - x) * (1.0 - x);}
inline double ddS(double x) {return 60.0 * x * (1.0 + x * (-3.0 + 2.0 * x));}

// Integral de S desde 0 hasta x: INT S = 2.5x^4 - 3x^5 + x^6.  Vale 0.5 en x=1.
inline double intS(double x) {return x * x * x * x * (2.5 + x * (-3.0 + x));}

}  // namespace

ScurveProfile::ScurveProfile(double length, double v_max, double ramp_fraction)
{
  if (!(length > 0.0)) {
    throw std::invalid_argument("ScurveProfile: length debe ser > 0");
  }
  if (!(v_max > 0.0)) {
    throw std::invalid_argument("ScurveProfile: v_max debe ser > 0");
  }
  if (!(ramp_fraction > 0.0) || ramp_fraction > 0.5) {
    throw std::invalid_argument("ScurveProfile: ramp_fraction debe estar en (0, 0.5]");
  }

  L_ = length;
  v_ = v_max;
  Lr_ = ramp_fraction * L_;
  Tr_ = 2.0 * Lr_ / v_;                    // porque la rampa cubre 0.5*v*Tr
  Tp_ = (L_ - 2.0 * Lr_) / v_;             // 0 cuando ramp_fraction = 0.5
  T_ = 2.0 * Tr_ + Tp_;
}

double ScurveProfile::peakAcceleration() const
{
  return v_ * 1.875 / Tr_;                 // max de S'(x) = 30x^2(1-x)^2 es 1.875
}

void ScurveProfile::plateauInterval(double & t0, double & t1) const
{
  t0 = Tr_;
  t1 = Tr_ + Tp_;
}

double ScurveProfile::s(double t) const
{
  if (t <= 0.0) {return 0.0;}
  if (t >= T_) {return L_;}
  if (t < Tr_) {
    return v_ * Tr_ * intS(t / Tr_);
  }
  if (t <= Tr_ + Tp_) {
    return Lr_ + v_ * (t - Tr_);
  }
  const double x = (T_ - t) / Tr_;
  return L_ - v_ * Tr_ * intS(x);
}

double ScurveProfile::sd(double t) const
{
  if (t <= 0.0 || t >= T_) {return 0.0;}
  if (t < Tr_) {
    return v_ * S(t / Tr_);
  }
  if (t <= Tr_ + Tp_) {
    return v_;                              // MESETA: feed exactamente constante
  }
  return v_ * S((T_ - t) / Tr_);
}

double ScurveProfile::sdd(double t) const
{
  if (t <= 0.0 || t >= T_) {return 0.0;}
  if (t < Tr_) {
    return v_ * dS(t / Tr_) / Tr_;
  }
  if (t <= Tr_ + Tp_) {
    return 0.0;
  }
  return -v_ * dS((T_ - t) / Tr_) / Tr_;
}

double ScurveProfile::sddd(double t) const
{
  if (t <= 0.0 || t >= T_) {return 0.0;}
  if (t < Tr_) {
    return v_ * ddS(t / Tr_) / (Tr_ * Tr_);
  }
  if (t <= Tr_ + Tp_) {
    return 0.0;
  }
  return v_ * ddS((T_ - t) / Tr_) / (Tr_ * Tr_);
}

}  // namespace ur5_dyn_control
