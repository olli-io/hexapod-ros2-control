#include "hexa_kinematics_cpp/leg_ik.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace hexa_kinematics {

Point3 forward_kinematics(const JointAngles& angles, const LegSpec& spec) {
  const double th_c = angles[0];
  const double th_f = angles[1];
  const double th_t = angles[2];
  const double r = spec.coxa_len + spec.femur_len * std::cos(th_f) +
                   spec.tibia_len * std::cos(th_f + th_t);
  const double z =
      -spec.femur_len * std::sin(th_f) - spec.tibia_len * std::sin(th_f + th_t);
  return Point3(r * std::cos(th_c), r * std::sin(th_c), z);
}

JointAngles inverse_kinematics(const Point3& target, const LegSpec& spec) {
  const double x = target[0];
  const double y = target[1];
  const double z = target[2];

  // At (x, y) = (0, 0) the foot is on the coxa axis and theta_coxa is
  // undetermined; atan2(0, 0) returns 0. Degenerate but harmless.
  const double th_c = std::atan2(y, x);

  // r_prime < 0 means the foot lies between the coxa pivot and the body centre —
  // the math still produces a valid solution (femur folds back under the body),
  // but it will almost certainly violate servo limits.
  const double r_prime = std::hypot(x, y) - spec.coxa_len;
  const double d = std::hypot(r_prime, z);

  const double f = spec.femur_len;
  const double t = spec.tibia_len;
  if (d > f + t + 1e-6 || d < std::abs(f - t) - 1e-6) {
    std::ostringstream msg;
    msg << std::fixed << std::setprecision(4) << "foot (" << x << ", " << y
        << ", " << z << ") is " << d << " m from the femur joint; reach annulus "
        << "is [" << std::abs(f - t) << ", " << (f + t) << "] m";
    throw UnreachableTarget(msg.str());
  }

  // Floating-point safety: arguments may slip outside [-1, 1] right at the
  // workspace boundary.
  const double cos_beta =
      std::max(-1.0, std::min(1.0, (f * f + d * d - t * t) / (2.0 * f * d)));
  const double cos_gamma =
      std::max(-1.0, std::min(1.0, (f * f + t * t - d * d) / (2.0 * f * t)));

  const double alpha = std::atan2(-z, r_prime);
  const double beta = std::acos(cos_beta);
  const double gamma = std::acos(cos_gamma);

  // Knee-up branch: femur sits above the chord from femur joint to foot.
  const double th_f = alpha - beta;
  const double th_t = M_PI - gamma;
  return {th_c, th_f, th_t};
}

}  // namespace hexa_kinematics
