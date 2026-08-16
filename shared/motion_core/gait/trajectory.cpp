#include "gait/trajectory.hpp"

namespace hexa::gait {

Vec3 quartic_bezier(const BezierNodes& points, float t) {
  const float s = 1.0f - t;
  const float b0 = s * s * s * s;
  const float b1 = 4.0f * s * s * s * t;
  const float b2 = 6.0f * s * s * t * t;
  const float b3 = 4.0f * s * t * t * t;
  const float b4 = t * t * t * t;
  return b0 * points[0] + b1 * points[1] + b2 * points[2] + b3 * points[3] +
         b4 * points[4];
}

Vec3 quartic_bezier_dot(const BezierNodes& points, float t) {
  const float s = 1.0f - t;
  // d/dt collapses to 4 * (degree-3 Bernstein over the node differences).
  const Vec3 d0 = points[1] - points[0];
  const Vec3 d1 = points[2] - points[1];
  const Vec3 d2 = points[3] - points[2];
  const Vec3 d3 = points[4] - points[3];
  return 4.0f * (s * s * s * d0 + 3.0f * s * s * t * d1 + 3.0f * s * t * t * d2 +
                 t * t * t * d3);
}

BezierNodes generate_stance_control_nodes(const Vec3& stance_origin,
                                          const Vec3& stride_vector,
                                          float stride_scaler) {
  const Vec3 sep = -stride_vector * stride_scaler * 0.25f;
  BezierNodes nodes;
  for (int k = 0; k < 5; ++k) {
    nodes[k] = stance_origin + static_cast<float>(k) * sep;
  }
  return nodes;
}

}  // namespace hexa::gait
