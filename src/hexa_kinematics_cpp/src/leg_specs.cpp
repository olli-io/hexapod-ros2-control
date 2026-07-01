#include "hexa_kinematics_cpp/leg_specs.hpp"

#include <cmath>

#include "hexa_kinematics_cpp/yaml_util.hpp"

namespace hexa_kinematics {

std::map<std::string, LegSpec> load_leg_specs(
    const std::string& geometry_yaml_path) {
  const YAML::Node cfg = load_file(geometry_yaml_path);
  const YAML::Node leg_cfg = cfg["leg"];
  const YAML::Node mounts = cfg["mounts"];

  const double coxa_len = require_scalar<double>(leg_cfg, "coxa_length", "leg");
  const double femur_len = require_scalar<double>(leg_cfg, "femur_length", "leg");
  const double tibia_len = require_scalar<double>(leg_cfg, "tibia_length", "leg");

  const YAML::Node front = mounts["l_front"];
  const YAML::Node middle = mounts["l_middle"];

  std::map<std::string, LegSpec> out;
  for (const std::string side : {"l", "r"}) {
    for (const std::string name : {"front", "middle", "rear"}) {
      const YAML::Node& ref = (name == "middle") ? middle : front;
      const std::string ctx = "mounts.l_" + name;
      const double ref_yaw =
          require_scalar<double>(ref, "yaw_deg", ctx) * M_PI / 180.0;
      const double ref_x = require_scalar<double>(ref, "x", ctx);
      const double ref_y = require_scalar<double>(ref, "y", ctx);

      const double x_fr = (name == "rear") ? -ref_x : ref_x;
      const double yaw_fr = (name == "rear") ? (M_PI - ref_yaw) : ref_yaw;
      const double mx = x_fr;
      const double my = (side == "r") ? -ref_y : ref_y;
      const double myaw = (side == "r") ? -yaw_fr : yaw_fr;

      LegSpec spec;
      spec.mount_xyz = Point3(mx, my, 0.0);
      spec.mount_yaw = myaw;
      spec.coxa_len = coxa_len;
      spec.femur_len = femur_len;
      spec.tibia_len = tibia_len;
      out[side + "_" + name] = spec;
    }
  }
  return out;
}

}  // namespace hexa_kinematics
