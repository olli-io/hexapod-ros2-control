// Static stability margin from stance-foot positions. Port of stability.py.
//
// support_polygon_margin returns the signed distance (m) from the CoM ground
// projection to the support-polygon boundary: positive inside, negative
// outside, magnitude = distance to the nearest edge. Pure geometry; not wired
// into the runtime (tests / offline gait evaluation only).
#pragma once

#include <utility>
#include <vector>

namespace hexa_gait {

// stance_feet_xy: body-frame ground-plane (x, y) of the feet in stance.
// com_xy: CoM ground projection (defaults to the body origin).
double support_polygon_margin(
    const std::vector<std::pair<double, double>>& stance_feet_xy,
    std::pair<double, double> com_xy = {0.0, 0.0});

}  // namespace hexa_gait
