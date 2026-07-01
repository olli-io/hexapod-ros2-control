// LegSpec loader. Port of leg_specs.py.
//
// Expands hexa_description's geometry.yaml (which defines only the two reference
// mounts l_front and l_middle, with yaw_deg in degrees) into the full six-leg
// map by the URDF symmetry rules:
//
//   rear  mirrors front about the body y-axis: x -> -x, yaw -> pi - yaw.
//   right mirrors left  about the body x-axis: y -> -y, yaw -> -yaw.
//
// These match the mount_leg macro in hexapod.urdf.xacro: the YAML stays the
// single source of truth, and the URDF and this loader expand it the same way.
#pragma once

#include <array>
#include <map>
#include <string>

#include "hexa_kinematics_cpp/leg_geometry.hpp"

namespace hexa_kinematics {

// Canonical leg order. Fixed at six legs (see CLAUDE.md: leg count is not
// parameterised). Mirror of hexa_kinematics.leg_specs.LEG_NAMES.
inline const std::array<std::string, 6> LEG_NAMES = {
    "l_front", "l_middle", "l_rear", "r_front", "r_middle", "r_rear",
};

// Parse geometry.yaml and return one LegSpec per leg, by name. Segment lengths
// come from leg.*; mount positions come from mounts.l_front / mounts.l_middle
// and are mirrored for the rear and right-side legs.
std::map<std::string, LegSpec> load_leg_specs(const std::string& geometry_yaml_path);

}  // namespace hexa_kinematics
