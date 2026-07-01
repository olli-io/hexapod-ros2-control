#include "hexa_kinematics_cpp/joint_config.hpp"

#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "hexa_kinematics_cpp/yaml_util.hpp"

namespace hexa_kinematics {

namespace {

// Per-joint-type intuitive-center field name inside the YAML.
const char* center_field(const std::string& joint_type) {
  if (joint_type == "coxa") return "deg";
  if (joint_type == "femur") return "above_horizontal_deg";
  if (joint_type == "tibia") return "interior_deg";
  throw std::runtime_error("unknown joint type: " + joint_type);
}

// Convert an intuitive per-joint degree value to URDF-convention radians.
double to_urdf_rad(const std::string& joint_type, double deg) {
  if (joint_type == "coxa") return deg * M_PI / 180.0;
  if (joint_type == "femur") return -deg * M_PI / 180.0;
  if (joint_type == "tibia") return M_PI - deg * M_PI / 180.0;
  throw std::runtime_error("unknown joint type: " + joint_type);
}

double degrees(double rad) { return rad * 180.0 / M_PI; }

std::string fmt_deg(double rad) {
  std::ostringstream s;
  s << std::fixed << std::setprecision(2) << degrees(rad);
  return s.str();
}

const std::array<std::string, 3> kJointTypes = {"coxa", "femur", "tibia"};

}  // namespace

std::map<std::string, JointLimits> load_joint_limits(
    const std::string& geometry_path) {
  const YAML::Node raw = load_file(geometry_path);
  const YAML::Node joints = raw["joints"];
  std::map<std::string, JointLimits> out;
  for (const auto& joint_type : kJointTypes) {
    const YAML::Node cfg = joints[joint_type];
    const std::string ctx = "joints." + joint_type;
    const double center_deg =
        require_scalar<double>(cfg, center_field(joint_type), ctx);
    const double lower_deg = require_scalar<double>(cfg, "lower_limit_deg", ctx);
    const double upper_deg = require_scalar<double>(cfg, "upper_limit_deg", ctx);

    const double center = to_urdf_rad(joint_type, center_deg);
    const double a = to_urdf_rad(joint_type, lower_deg);
    const double b = to_urdf_rad(joint_type, upper_deg);
    const double lower = std::min(a, b);
    const double upper = std::max(a, b);

    if (!(lower <= center && center <= upper)) {
      std::ostringstream msg;
      msg << std::fixed << std::setprecision(2) << joint_type << " servo center "
          << center_deg << "deg lies outside limit window [" << lower_deg
          << "deg, " << upper_deg << "deg]";
      throw std::runtime_error(msg.str());
    }

    JointLimits lim;
    lim.center = center;
    lim.lower = lower;
    lim.upper = upper;
    lim.effort = require_scalar<double>(cfg, "effort", ctx);
    lim.velocity = require_scalar<double>(cfg, "velocity", ctx);
    out[joint_type] = lim;
  }
  return out;
}

JointAngles load_standing_pose(const std::string& standing_pose_path,
                               const std::string& geometry_path) {
  const YAML::Node raw = load_file(standing_pose_path);
  const std::map<std::string, JointLimits> limits = load_joint_limits(geometry_path);

  std::array<double, 3> angles{};
  for (std::size_t i = 0; i < kJointTypes.size(); ++i) {
    const std::string& joint_type = kJointTypes[i];
    const YAML::Node cfg = raw[joint_type];
    const double theta = to_urdf_rad(
        joint_type, require_scalar<double>(cfg, center_field(joint_type),
                                           "standing_pose." + joint_type));
    const JointLimits& lim = limits.at(joint_type);
    if (!(lim.lower <= theta && theta <= lim.upper)) {
      throw std::runtime_error("standing pose " + joint_type + " angle " +
                               fmt_deg(theta) + "deg lies outside servo range [" +
                               fmt_deg(lim.lower) + "deg, " + fmt_deg(lim.upper) +
                               "deg]");
    }
    angles[i] = theta;
  }
  return {angles[0], angles[1], angles[2]};
}

std::map<std::string, JointAngles> load_initial_pose(
    const std::string& geometry_path) {
  const YAML::Node raw = load_file(geometry_path);
  const std::map<std::string, JointLimits> limits = load_joint_limits(geometry_path);
  const YAML::Node init = raw["initial_pose"];

  const double femur_theta = to_urdf_rad(
      "femur", require_scalar<double>(init["femur"], "above_horizontal_deg",
                                      "initial_pose.femur"));
  const double tibia_theta = to_urdf_rad(
      "tibia",
      require_scalar<double>(init["tibia"], "interior_deg", "initial_pose.tibia"));
  for (const auto& [joint_type, theta] :
       {std::pair<std::string, double>{"femur", femur_theta},
        std::pair<std::string, double>{"tibia", tibia_theta}}) {
    const JointLimits& lim = limits.at(joint_type);
    if (!(lim.lower <= theta && theta <= lim.upper)) {
      throw std::runtime_error("initial pose " + joint_type + " angle " +
                               fmt_deg(theta) + "deg lies outside servo range [" +
                               fmt_deg(lim.lower) + "deg, " + fmt_deg(lim.upper) +
                               "deg]");
    }
  }

  const JointLimits& coxa_lim = limits.at("coxa");
  const YAML::Node coxa_cfg = init["coxa"];
  std::map<std::string, JointAngles> out;
  for (const std::string side : {"l", "r"}) {
    for (const std::string name : {"front", "middle", "rear"}) {
      const double ref_deg = require_scalar<double>(
          coxa_cfg, (name == "middle") ? "l_middle_deg" : "l_front_deg",
          "initial_pose.coxa");
      const double after_fr = (name == "rear") ? -ref_deg : ref_deg;
      const double after_lr = (side == "r") ? -after_fr : after_fr;
      const double coxa_theta = to_urdf_rad("coxa", after_lr);
      if (!(coxa_lim.lower <= coxa_theta && coxa_theta <= coxa_lim.upper)) {
        throw std::runtime_error(
            "initial pose coxa angle for " + side + "_" + name + " " +
            fmt_deg(coxa_theta) + "deg lies outside servo range [" +
            fmt_deg(coxa_lim.lower) + "deg, " + fmt_deg(coxa_lim.upper) + "deg]");
      }
      out[side + "_" + name] = {coxa_theta, femur_theta, tibia_theta};
    }
  }
  return out;
}

}  // namespace hexa_kinematics
