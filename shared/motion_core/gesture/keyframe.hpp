// Keyframe sampling for gestures: the two segment shapes and a sampler over a
// complete keyframe table. Stateless. Leg tracks are sampled per joint, body
// tracks per pose axis; every component stays inside the range its
// neighbouring knots span, which is what lets a per-knot check cover the path.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "config_generated.hpp"
#include "vec3.hpp"

namespace hexa::gesture {

using ::hexa::config::BodyKeyframe;
using ::hexa::config::GestureTransition;
using ::hexa::config::LegKeyframe;

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
// keeps a turning point from overshooting. The Fritsch-Carlson cap (no slope
// past three times the shallower secant) keeps a monotone run inside its knots
// however uneven the steps.
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
  const float m = (get(keys[i + 1]) - get(keys[i - 1])) / (dt_in + dt_out);
  const float cap = 3.0f * std::min(std::fabs(s_in), std::fabs(s_out));
  return std::copysign(std::min(std::fabs(m), cap), m);
}

// Sample one component of a COMPLETE table (implicit start knot in place,
// every hold knot resolved) at time t. The segment arriving at knot i is
// shaped by knot i's transition. Before the first knot the first value holds;
// after the last, the last. A leg track's live knots carry no value: the
// player samples only between two fixed knots here and eases the segments
// next to a live knot itself.
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
