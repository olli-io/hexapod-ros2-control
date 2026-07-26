// Quartic Bezier stance track. Float fork of trajectory.hpp (part 06).
//
// The stance half of a leg's step cycle (touchdown -> next lift-off) is a
// 5-control-node quartic Bezier with evenly spaced nodes, i.e. a straight line
// swept at constant tip velocity. The swing half is not a Bezier — see
// gaits/base.hpp's swing_arc(), which eases along the ground track instead.
#pragma once

#include <array>

#include "gait/types.hpp"

namespace hexa::gait {

using BezierNodes = std::array<Vec3, 5>;

// Evaluate the quartic Bezier curve B(t) for t in [0, 1] (Bernstein basis).
Vec3 quartic_bezier(const BezierNodes& points, float t);

// Evaluate dB/dt of the quartic Bezier curve at t (tests check C1 continuity).
Vec3 quartic_bezier_dot(const BezierNodes& points, float t);

// Stance curve (touchdown -> next lift-off). Nodes are evenly spaced along
// -stride_vector * stride_scaler, which makes the curve exactly affine in the
// parameter: the tip travels at a constant velocity for the whole of stance.
BezierNodes generate_stance_control_nodes(const Vec3& stance_origin,
                                          const Vec3& stride_vector,
                                          float stride_scaler = 1.0f);

}  // namespace hexa::gait
