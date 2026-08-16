// The reachability message is built with snprintf rather than ostringstream, to
// keep <sstream>/<iomanip> off the firmware.

#include "kinematics/leg_ik.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace hexa {

Vec3 forward_kinematics(const JointAngles& angles, const LegSpec& spec) {
  const float th_c = angles[0];
  const float th_f = angles[1];
  const float th_t = angles[2];
  const float r = spec.coxa_len + spec.femur_len * cosf(th_f) +
                  spec.tibia_len * cosf(th_f + th_t);
  const float z =
      -spec.femur_len * sinf(th_f) - spec.tibia_len * sinf(th_f + th_t);
  return Vec3(r * cosf(th_c), r * sinf(th_c), z);
}

JointAngles inverse_kinematics(const Vec3& target, const LegSpec& spec) {
  const float x = target[0];
  const float y = target[1];
  const float z = target[2];

  // On the coxa axis theta_coxa is undetermined; atan2f(0, 0) returns 0.
  const float th_c = atan2f(y, x);

  // r_prime < 0 puts the foot between the coxa pivot and the body centre: still
  // a valid solution (femur folded under the body), but almost certainly outside
  // the servo limits.
  const float r_prime = hypotf(x, y) - spec.coxa_len;
  const float d = hypotf(r_prime, z);

  const float f = spec.femur_len;
  const float t = spec.tibia_len;
  if (d > f + t + 1e-6f || d < fabsf(f - t) - 1e-6f) {
    char msg[128];
    std::snprintf(msg, sizeof(msg),
                  "foot (%.4f, %.4f, %.4f) is %.4f m from the femur joint; "
                  "reach annulus is [%.4f, %.4f] m",
                  static_cast<double>(x), static_cast<double>(y),
                  static_cast<double>(z), static_cast<double>(d),
                  static_cast<double>(fabsf(f - t)),
                  static_cast<double>(f + t));
    throw UnreachableTarget(msg);
  }

  // The arguments may slip outside [-1, 1] right at the workspace boundary.
  const float cos_beta =
      std::max(-1.0f, std::min(1.0f, (f * f + d * d - t * t) / (2.0f * f * d)));
  const float cos_gamma =
      std::max(-1.0f, std::min(1.0f, (f * f + t * t - d * d) / (2.0f * f * t)));

  const float alpha = atan2f(-z, r_prime);
  const float beta = acosf(cos_beta);
  const float gamma = acosf(cos_gamma);

  // Knee-up branch: femur sits above the chord from femur joint to foot.
  const float th_f = alpha - beta;
  const float th_t = static_cast<float>(M_PI) - gamma;
  return {th_c, th_f, th_t};
}

}  // namespace hexa
