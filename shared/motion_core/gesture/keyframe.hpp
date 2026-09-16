// Keyframe sampling for gestures: the leg-polar currency, the two segment
// shapes, and a sampler over a complete keyframe table. Stateless.
#pragma once

#include <cstddef>

#include "config_generated.hpp"
#include "vec3.hpp"

namespace hexa::gesture {

using ::hexa::config::BodyKeyframe;
using ::hexa::config::GestureTransition;
using ::hexa::config::LegKeyframe;

// One foot in its leg's mount frame: coxa swivel, planar reach, and lift above
// the standing ground plane. The currency gestures.yaml is written in.
struct LegPolar {
  float angle = 0.0f;   // rad, 0 along the mount yaw, positive CCW
  float reach = 0.0f;   // m, planar coxa axis -> tip
  float height = 0.0f;  // m above the ground plane; 0 = planted
};

// ground_z is the standing tip z in the leg frame (the tip-sphere centre, so
// height is exactly the contact lift).
LegPolar polar_from_leg_frame(const Vec3& p_leg, float ground_z);
Vec3 leg_frame_from_polar(const LegPolar& polar, float ground_z);

// Quintic smoothstep — the same curve gait/gaits/base.hpp's ease5 draws.
inline float ease5(float u) {
  return u * u * u * (10.0f + u * (-15.0f + 6.0f * u));
}

// Cubic Hermite on u in [0, 1]; m0 and m1 are the end slopes per unit u.
float hermite(float p0, float m0, float p1, float m1, float u);

// Slope at knot i of a table, per second, for the `continuous` segments around
// it. Catmull-Rom over the neighbouring knots' TIMES, so uneven spacing does
// not skew the curve. Zero at the table's ends, where either adjacent segment
// is `ease`, and where the two secants disagree in sign or either is zero —
// that last rule is what makes two equal consecutive keyframes a hold and
// keeps a turning point from overshooting.
template <typename Key, typename Get>
float knot_slope(const Key* keys, std::size_t n, std::size_t i, Get get) {
  if (i == 0 || i + 1 >= n) {
    return 0.0f;
  }
  if (keys[i].transition != GestureTransition::CONTINUOUS ||
      keys[i + 1].transition != GestureTransition::CONTINUOUS) {
    return 0.0f;
  }
  const float dt_in = keys[i].t - keys[i - 1].t;
  const float dt_out = keys[i + 1].t - keys[i].t;
  if (dt_in <= 0.0f || dt_out <= 0.0f) {
    return 0.0f;
  }
  const float s_in = (get(keys[i]) - get(keys[i - 1])) / dt_in;
  const float s_out = (get(keys[i + 1]) - get(keys[i])) / dt_out;
  if (s_in == 0.0f || s_out == 0.0f || (s_in < 0.0f) != (s_out < 0.0f)) {
    return 0.0f;
  }
  return (get(keys[i + 1]) - get(keys[i - 1])) / (dt_in + dt_out);
}

// Sample one component of a COMPLETE table (implicit start and return already
// in place, every preserve knot resolved) at time t. The segment arriving at
// knot i is shaped by knot i's transition. Before the first knot the first
// value holds; after the last, the last.
template <typename Key, typename Get>
float track_value(const Key* keys, std::size_t n, float t, Get get) {
  if (n == 0) {
    return 0.0f;
  }
  if (t <= keys[0].t) {
    return get(keys[0]);
  }
  if (t >= keys[n - 1].t) {
    return get(keys[n - 1]);
  }
  std::size_t i = 1;
  while (i + 1 < n && t > keys[i].t) {
    ++i;
  }
  const Key& a = keys[i - 1];
  const Key& b = keys[i];
  const float h = b.t - a.t;
  const float u = h > 0.0f ? (t - a.t) / h : 1.0f;
  const float p0 = get(a);
  const float p1 = get(b);
  if (b.transition == GestureTransition::EASE) {
    return p0 + (p1 - p0) * ease5(u);
  }
  // Slopes are per second; scale onto the unit-u segment.
  const float m0 = knot_slope(keys, n, i - 1, get) * h;
  const float m1 = knot_slope(keys, n, i, get) * h;
  return hermite(p0, m0, p1, m1, u);
}

}  // namespace hexa::gesture
