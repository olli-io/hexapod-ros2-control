#include "hexa_hardware/joint_calibration.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace hexa_hardware {

std::uint16_t JointCalibration::to_pulse_us(double theta_rad) const {
  const double center_us = (us_at_plus_45 + us_at_minus_45) * 0.5;
  const double slope_us_per_rad = (us_at_plus_45 - us_at_minus_45) / (M_PI / 2.0);
  const double us = center_us + (theta_rad - urdf_rad_at_center) * slope_us_per_rad;
  const double clamped = std::clamp(us, static_cast<double>(min_us),
                                    static_cast<double>(max_us));
  return static_cast<std::uint16_t>(std::lround(clamped));
}

namespace {

template <typename T>
T require_scalar(const YAML::Node& node, const std::string& key,
                 const std::string& ctx) {
  if (!node[key]) {
    throw std::runtime_error("hexa_hardware: " + ctx + " missing key '" + key + "'");
  }
  try {
    return node[key].as<T>();
  } catch (const YAML::Exception&) {
    throw std::runtime_error("hexa_hardware: " + ctx + " key '" + key +
                             "' has wrong type");
  }
}

JointPosition parse_joint_position(const std::string& s, const std::string& ctx) {
  if (s == "coxa") return JointPosition::Coxa;
  if (s == "femur") return JointPosition::Femur;
  if (s == "tibia") return JointPosition::Tibia;
  throw std::runtime_error("hexa_hardware: " + ctx +
                           " joint_position must be coxa|femur|tibia, got '" + s + "'");
}

// Map an intuitive per-position angle (degrees, geometry.yaml convention)
// to a URDF radian. Mirrors hexa_description/urdf/hexapod.urdf.xacro.
double intuitive_deg_to_urdf_rad(JointPosition pos, double deg) {
  const double rad = deg * M_PI / 180.0;
  switch (pos) {
    case JointPosition::Coxa:  return rad;
    case JointPosition::Femur: return -rad;
    case JointPosition::Tibia: return M_PI - rad;
  }
  return rad;
}

struct DegAtCenter {
  double coxa = 0.0;
  double femur = 0.0;
  double tibia = 0.0;

  double for_position(JointPosition pos) const {
    switch (pos) {
      case JointPosition::Coxa:  return coxa;
      case JointPosition::Femur: return femur;
      case JointPosition::Tibia: return tibia;
    }
    return 0.0;
  }
};

JointCalibration parse_joint(const YAML::Node& node, const std::string& name,
                             const DegAtCenter& deg_at_center) {
  const std::string ctx = "joints[" + name + "]";
  JointCalibration jc;
  jc.pin = require_scalar<unsigned int>(node, "pin", ctx);
  jc.joint_position =
      parse_joint_position(require_scalar<std::string>(node, "joint_position", ctx), ctx);
  jc.us_at_plus_45 = require_scalar<double>(node, "us_at_plus_45", ctx);
  jc.us_at_minus_45 = require_scalar<double>(node, "us_at_minus_45", ctx);
  jc.urdf_rad_at_center =
      intuitive_deg_to_urdf_rad(jc.joint_position, deg_at_center.for_position(jc.joint_position));
  jc.min_us = static_cast<std::uint16_t>(require_scalar<unsigned int>(node, "min_us", ctx));
  jc.max_us = static_cast<std::uint16_t>(require_scalar<unsigned int>(node, "max_us", ctx));
  if (jc.min_us >= jc.max_us) {
    throw std::runtime_error("hexa_hardware: " + ctx + " min_us must be < max_us");
  }
  if (jc.us_at_plus_45 == jc.us_at_minus_45) {
    throw std::runtime_error("hexa_hardware: " + ctx +
                             " us_at_plus_45 must differ from us_at_minus_45");
  }
  return jc;
}

}  // namespace

HardwareConfig load_hardware_config(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("hexa_hardware: failed to load " + path + ": " + e.what());
  }

  HardwareConfig cfg;

  if (const auto serial = root["serial"]) {
    if (serial["device"]) cfg.serial_device = serial["device"].as<std::string>();
    if (serial["baud"]) cfg.serial_baud = serial["baud"].as<int>();
    if (serial["get_period_ticks"]) {
      cfg.get_period_ticks = serial["get_period_ticks"].as<int>();
    }
  }

  if (const auto relay = root["relay"]) {
    cfg.relay_pin = static_cast<std::uint8_t>(relay["pin"].as<unsigned int>());
    cfg.relay_configured = true;
  }

  if (const auto aux = root["aux"]) {
    for (auto it = aux.begin(); it != aux.end(); ++it) {
      const std::string name = it->first.as<std::string>();
      AuxChannel ch;
      ch.pin = static_cast<std::uint8_t>(it->second["pin"].as<unsigned int>());
      ch.scale = it->second["scale"] ? it->second["scale"].as<double>() : 1.0;
      cfg.aux.emplace(name, ch);
    }
  }

  DegAtCenter deg_at_center;
  if (const auto dac = root["deg_at_center"]) {
    if (dac["coxa"])  deg_at_center.coxa  = dac["coxa"].as<double>();
    if (dac["femur"]) deg_at_center.femur = dac["femur"].as<double>();
    if (dac["tibia"]) deg_at_center.tibia = dac["tibia"].as<double>();
  }

  const auto joints = root["joints"];
  if (!joints || !joints.IsMap()) {
    throw std::runtime_error("hexa_hardware: config missing 'joints' map");
  }
  for (auto it = joints.begin(); it != joints.end(); ++it) {
    const std::string name = it->first.as<std::string>();
    cfg.joints.emplace(name, parse_joint(it->second, name, deg_at_center));
  }

  return cfg;
}

}  // namespace hexa_hardware
