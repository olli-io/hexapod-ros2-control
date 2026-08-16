// Quartic Bezier stance track: 5 evenly spaced control nodes, i.e. a straight
// line swept at constant tip velocity. The swing half is not a Bezier — see
// gaits/base.hpp's swing_arc().
#pragma once

#include <array>

#include "gait/types.hpp"

namespace hexa::gait {

using BezierNodes = std::array<Vec3, 5>;

// B(t) for t in [0, 1].
Vec3 quartic_bezier(const BezierNodes& points, float t);

// dB/dt at t.
Vec3 quartic_bezier_dot(const BezierNodes& points, float t);

// Stance curve (touchdown -> next lift-off). Even spacing along
// -stride_vector * stride_scaler makes the curve affine in the parameter.
BezierNodes generate_stance_control_nodes(const Vec3& stance_origin,
                                          const Vec3& stride_vector,
                                          float stride_scaler = 1.0f);

}  // namespace hexa::gait
