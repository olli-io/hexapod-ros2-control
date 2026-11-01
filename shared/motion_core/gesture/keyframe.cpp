#include "gesture/keyframe.hpp"

namespace hexa::gesture {

float hermite(float p0, float m0, float p1, float m1, float u) {
  const float u2 = u * u;
  const float u3 = u2 * u;
  const float u4 = u3 * u;
  const float u5 = u4 * u;
  const float h01 = 10.0f * u3 - 15.0f * u4 + 6.0f * u5;
  const float h10 = u - 6.0f * u3 + 8.0f * u4 - 3.0f * u5;
  const float h11 = -4.0f * u3 + 7.0f * u4 - 3.0f * u5;
  return p0 + (p1 - p0) * h01 + h10 * m0 + h11 * m1;
}

}  // namespace hexa::gesture
