#include "gesture/keyframe.hpp"

#include <cmath>

namespace hexa::gesture {

LegPolar polar_from_leg_frame(const Vec3& p_leg, float ground_z) {
  LegPolar out;
  out.angle = std::atan2(p_leg.y, p_leg.x);
  out.reach = std::hypot(p_leg.x, p_leg.y);
  out.height = p_leg.z - ground_z;
  return out;
}

Vec3 leg_frame_from_polar(const LegPolar& polar, float ground_z) {
  return Vec3(polar.reach * std::cos(polar.angle),
              polar.reach * std::sin(polar.angle), ground_z + polar.height);
}

float hermite(float p0, float m0, float p1, float m1, float u) {
  const float u2 = u * u;
  const float u3 = u2 * u;
  const float h00 = 2.0f * u3 - 3.0f * u2 + 1.0f;
  const float h10 = u3 - 2.0f * u2 + u;
  const float h01 = -2.0f * u3 + 3.0f * u2;
  const float h11 = u3 - u2;
  return h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1;
}

}  // namespace hexa::gesture
