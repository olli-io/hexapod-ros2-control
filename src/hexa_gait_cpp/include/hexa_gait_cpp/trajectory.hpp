// Quartic Bezier foot-tip trajectory. Port of trajectory.py (itself a port of
// docs/trajectory-generation/control_points.cpp, the Syropod walk-controller
// curves).
//
// One step cycle of a single leg is described by three 5-control-node quartic
// Bezier curves: primary swing (lift-off -> apex), secondary swing (apex ->
// touchdown), and stance (touchdown -> next lift-off). Nodes are placed for C0
// (position), C1 (velocity), and where possible C2 (acceleration) continuity at
// the joins.
//
// Each curve's 5 control nodes are a std::array<Vec3, 5> (P0..P4), matching the
// row-wise semantics of the original (5, 3) numpy arrays.
#pragma once

#include <array>

#include "hexa_gait_cpp/types.hpp"

namespace hexa_gait {

using BezierNodes = std::array<Vec3, 5>;

// Evaluate the quartic Bezier curve B(t) for t in [0, 1] (Bernstein basis).
Vec3 quartic_bezier(const BezierNodes& points, double t);

// Evaluate dB/dt of the quartic Bezier curve at t. Used by tests to check C1
// continuity at curve joins; the engine itself only needs B(t).
Vec3 quartic_bezier_dot(const BezierNodes& points, double t);

// Primary swing curve (lift-off -> apex). swing_origin_velocity carries the C1
// join from stance; supply -stride_vector / swing_time for the analytical
// lift-off velocity. identity_y_sign is +1 for left-side legs, -1 for right;
// swing_width = 0 disables the lateral arch.
BezierNodes generate_primary_swing_control_nodes(
    const Vec3& swing_origin, const Vec3& swing_origin_velocity,
    const Vec3& target, double swing_clearance, double swing_width,
    int identity_y_sign, double controller_dt, double swing_delta_t);

// Secondary swing curve (apex -> touchdown). Joins C2 to the primary at the
// apex and C2 to stance at touchdown via the analytical touchdown velocity.
BezierNodes generate_secondary_swing_control_nodes(
    const BezierNodes& swing_1_nodes, const Vec3& target,
    const Vec3& stride_vector, double controller_dt, double swing_delta_t,
    double stance_delta_t);

// Stance curve (touchdown -> next lift-off). Nodes are evenly spaced along
// -stride_vector * stride_scaler, giving a (near) constant tip velocity
// opposite the body motion.
BezierNodes generate_stance_control_nodes(const Vec3& stance_origin,
                                          const Vec3& stride_vector,
                                          double stride_scaler = 1.0);

}  // namespace hexa_gait
