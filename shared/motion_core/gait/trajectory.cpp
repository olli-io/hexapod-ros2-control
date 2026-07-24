#include "gait/trajectory.hpp"

#include <algorithm>

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
  // d/dt of the Bernstein basis collapses to 4 * (degree-3 Bernstein over the
  // differences of successive control points).
  const Vec3 d0 = points[1] - points[0];
  const Vec3 d1 = points[2] - points[1];
  const Vec3 d2 = points[3] - points[2];
  const Vec3 d3 = points[4] - points[3];
  return 4.0f * (s * s * s * d0 + 3.0f * s * s * t * d1 + 3.0f * s * t * t * d2 +
                 t * t * t * d3);
}

namespace {
// Translation between successive Bezier control nodes that yields a tip velocity
// of `velocity` at a curve endpoint. A quartic's endpoint derivative is
// 4 * (P1 - P0) in curve parameter, and the parameter advances at
// 1 / half_duration, so the separation is velocity * half_duration / 4.
Vec3 node_separation(const Vec3& velocity, float half_duration) {
  return 0.25f * velocity * half_duration;
}
}  // namespace

BezierNodes generate_primary_swing_control_nodes(
    const Vec3& swing_origin, const Vec3& swing_origin_velocity,
    const Vec3& target, float swing_clearance, float swing_width,
    int identity_y_sign, float ascent_time) {
  Vec3 mid = (swing_origin + target) / 2.0f;
  mid[2] = std::max(swing_origin[2], target[2]) + swing_clearance;
  mid[1] += identity_y_sign > 0 ? swing_width : -swing_width;

  const Vec3 sep = node_separation(swing_origin_velocity, ascent_time);

  BezierNodes nodes;
  // C0 at stance->swing join.
  nodes[0] = swing_origin;
  // C1 at stance->swing join.
  nodes[1] = swing_origin + sep;
  // C2 at stance->swing join.
  nodes[2] = swing_origin + 2.0f * sep;
  // C2 at primary->secondary swing join (symmetric apex).
  nodes[3] = (mid + nodes[2]) / 2.0f;
  nodes[3][2] = mid[2];
  // Apex.
  nodes[4] = mid;
  return nodes;
}

BezierNodes generate_secondary_swing_control_nodes(
    const BezierNodes& swing_1_nodes, const Vec3& target,
    const Vec3& target_velocity, float ascent_time, float descent_time) {
  const Vec3 sep = node_separation(target_velocity, descent_time);
  // Both halves are quartics but may span different durations, so mirroring the
  // apex tangent has to be scaled by the duration ratio to keep the real-time
  // velocity equal on both sides of the apex.
  const float duration_ratio = descent_time / ascent_time;

  BezierNodes nodes;
  nodes[0] = swing_1_nodes[4];
  // C1 at primary->secondary swing join (duration-scaled mirror about the apex).
  nodes[1] =
      swing_1_nodes[4] + duration_ratio * (swing_1_nodes[4] - swing_1_nodes[3]);
  // C2 at secondary swing->stance join.
  nodes[2] = target - 2.0f * sep;
  // C1 at secondary swing->stance join.
  nodes[3] = target - sep;
  // C0 at secondary swing->stance join.
  nodes[4] = target;
  return nodes;
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
