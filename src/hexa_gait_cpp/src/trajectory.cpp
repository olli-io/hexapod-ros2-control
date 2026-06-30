#include "hexa_gait_cpp/trajectory.hpp"

#include <algorithm>

namespace hexa_gait {

Vec3 quartic_bezier(const BezierNodes& points, double t) {
  const double s = 1.0 - t;
  const double b0 = s * s * s * s;
  const double b1 = 4.0 * s * s * s * t;
  const double b2 = 6.0 * s * s * t * t;
  const double b3 = 4.0 * s * t * t * t;
  const double b4 = t * t * t * t;
  return b0 * points[0] + b1 * points[1] + b2 * points[2] + b3 * points[3] +
         b4 * points[4];
}

Vec3 quartic_bezier_dot(const BezierNodes& points, double t) {
  const double s = 1.0 - t;
  // d/dt of the Bernstein basis collapses to 4 * (degree-3 Bernstein over the
  // differences of successive control points).
  const Vec3 d0 = points[1] - points[0];
  const Vec3 d1 = points[2] - points[1];
  const Vec3 d2 = points[3] - points[2];
  const Vec3 d3 = points[4] - points[3];
  return 4.0 * (s * s * s * d0 + 3.0 * s * s * t * d1 + 3.0 * s * t * t * d2 +
                t * t * t * d3);
}

namespace {
// Translation between successive Bezier control nodes that yields a tip
// velocity of `velocity` at a curve endpoint. Coefficient 0.125 (half the
// Syropod 0.25): each swing is split into a primary and secondary quartic each
// covering swing_time / 2, so the Bezier parameter advances at 2 / swing_time.
Vec3 node_separation(const Vec3& velocity, double controller_dt,
                     double swing_delta_t) {
  return 0.125 * velocity * (controller_dt / swing_delta_t);
}
}  // namespace

BezierNodes generate_primary_swing_control_nodes(
    const Vec3& swing_origin, const Vec3& swing_origin_velocity,
    const Vec3& target, double swing_clearance, double swing_width,
    int identity_y_sign, double controller_dt, double swing_delta_t) {
  Vec3 mid = (swing_origin + target) / 2.0;
  mid[2] = std::max(swing_origin[2], target[2]) + swing_clearance;
  mid[1] += identity_y_sign > 0 ? swing_width : -swing_width;

  const Vec3 sep =
      node_separation(swing_origin_velocity, controller_dt, swing_delta_t);

  BezierNodes nodes;
  // C0 at stance->swing join.
  nodes[0] = swing_origin;
  // C1 at stance->swing join.
  nodes[1] = swing_origin + sep;
  // C2 at stance->swing join.
  nodes[2] = swing_origin + 2.0 * sep;
  // C2 at primary->secondary swing join (symmetric apex).
  nodes[3] = (mid + nodes[2]) / 2.0;
  nodes[3][2] = mid[2];
  // Apex.
  nodes[4] = mid;
  return nodes;
}

BezierNodes generate_secondary_swing_control_nodes(
    const BezierNodes& swing_1_nodes, const Vec3& target,
    const Vec3& stride_vector, double controller_dt, double swing_delta_t,
    double stance_delta_t) {
  const Vec3 final_velocity = -stride_vector * (stance_delta_t / controller_dt);
  const Vec3 sep = node_separation(final_velocity, controller_dt, swing_delta_t);

  BezierNodes nodes;
  nodes[0] = swing_1_nodes[4];
  // C1 at primary->secondary swing join (mirror about the apex).
  nodes[1] = swing_1_nodes[4] - (swing_1_nodes[3] - swing_1_nodes[4]);
  // C2 at secondary swing->stance join.
  nodes[2] = target - 2.0 * sep;
  // C1 at secondary swing->stance join.
  nodes[3] = target - sep;
  // C0 at secondary swing->stance join.
  nodes[4] = target;
  return nodes;
}

BezierNodes generate_stance_control_nodes(const Vec3& stance_origin,
                                          const Vec3& stride_vector,
                                          double stride_scaler) {
  const Vec3 sep = -stride_vector * stride_scaler * 0.25;
  BezierNodes nodes;
  for (int k = 0; k < 5; ++k) {
    nodes[k] = stance_origin + static_cast<double>(k) * sep;
  }
  return nodes;
}

}  // namespace hexa_gait
