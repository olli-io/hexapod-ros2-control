#include "hexa_hardware/joint_calibration.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace hexa_hardware {

std::uint16_t JointCalibration::to_pulse_us(double theta_rad) const {
  const double center_us = (us_at_plus_45 + us_at_minus_45) * 0.5;
  // Endpoints are magnitudes; `direction` (+1 / -1) is the sole sign source,
  // expressing which way the servo is bolted on (see the header).
  const double slope_us_per_rad =
      std::abs(us_at_plus_45 - us_at_minus_45) / (M_PI / 2.0);
  const double us = center_us +
                    direction * (theta_rad - urdf_rad_at_center) * slope_us_per_rad;
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

// The leg segment each joint drives, keyed by URDF joint name. The 6-leg set
// is fixed, so this table is the authoritative name→segment map: calibration
// needs no redundant position field, and an unknown joint name is rejected.
JointPosition joint_position_from_name(const std::string& name,
                                       const std::string& ctx) {
  static const std::unordered_map<std::string, JointPosition> kPositions = {
      {"l_front_coxa_joint", JointPosition::Coxa},
      {"l_front_femur_joint", JointPosition::Femur},
      {"l_front_tibia_joint", JointPosition::Tibia},
      {"l_middle_coxa_joint", JointPosition::Coxa},
      {"l_middle_femur_joint", JointPosition::Femur},
      {"l_middle_tibia_joint", JointPosition::Tibia},
      {"l_rear_coxa_joint", JointPosition::Coxa},
      {"l_rear_femur_joint", JointPosition::Femur},
      {"l_rear_tibia_joint", JointPosition::Tibia},
      {"r_front_coxa_joint", JointPosition::Coxa},
      {"r_front_femur_joint", JointPosition::Femur},
      {"r_front_tibia_joint", JointPosition::Tibia},
      {"r_middle_coxa_joint", JointPosition::Coxa},
      {"r_middle_femur_joint", JointPosition::Femur},
      {"r_middle_tibia_joint", JointPosition::Tibia},
      {"r_rear_coxa_joint", JointPosition::Coxa},
      {"r_rear_femur_joint", JointPosition::Femur},
      {"r_rear_tibia_joint", JointPosition::Tibia},
  };
  const auto it = kPositions.find(name);
  if (it == kPositions.end()) {
    throw std::runtime_error("hexa_hardware: " + ctx + " unknown joint name '" +
                             name + "'");
  }
  return it->second;
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

// One servo's measured endpoint magnitudes, straight from
// servo_calibration.yaml. Position in the vector encodes the pin: index i is
// pin i+1.
struct CalibrationEndpoints {
  double us_at_plus_45 = 2000.0;
  double us_at_minus_45 = 1000.0;
};

// Parse servo_calibration.yaml. The `calibration_values` list must hold exactly
// one entry per servo pin, in pin order; each entry's own `pin` field is
// user-reference and must match its position (index + 1).
std::vector<CalibrationEndpoints> load_calibration(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("hexa_hardware: failed to load calibration " + path +
                             ": " + e.what());
  }
  const auto values = root["calibration_values"];
  if (!values || !values.IsSequence()) {
    throw std::runtime_error(
        "hexa_hardware: calibration missing 'calibration_values' sequence");
  }
  std::vector<CalibrationEndpoints> out;
  out.reserve(values.size());
  for (std::size_t i = 0; i < values.size(); ++i) {
    const auto& node = values[i];
    const std::string ctx = "calibration_values[" + std::to_string(i) + "]";
    const auto pin = require_scalar<unsigned int>(node, "pin", ctx);
    if (pin != i + 1) {
      throw std::runtime_error("hexa_hardware: " + ctx + " pin " +
                               std::to_string(pin) + " must equal index+1 (" +
                               std::to_string(i + 1) + ")");
    }
    CalibrationEndpoints ep;
    ep.us_at_plus_45 = require_scalar<double>(node, "us_at_plus_45", ctx);
    ep.us_at_minus_45 = require_scalar<double>(node, "us_at_minus_45", ctx);
    out.push_back(ep);
  }
  return out;
}

JointCalibration parse_joint(const YAML::Node& node, const std::string& name,
                             const DegAtCenter& deg_at_center,
                             const std::vector<CalibrationEndpoints>& calibration) {
  const std::string ctx = "joints[" + name + "]";
  JointCalibration jc;
  jc.pin = require_scalar<unsigned int>(node, "pin", ctx);
  jc.joint_position = joint_position_from_name(name, ctx);
  if (jc.pin == 0 || jc.pin > calibration.size()) {
    throw std::runtime_error("hexa_hardware: " + ctx + " pin " +
                             std::to_string(jc.pin) +
                             " has no matching servo_calibration.json entry");
  }
  const auto& ep = calibration[jc.pin - 1];
  jc.us_at_plus_45 = ep.us_at_plus_45;
  jc.us_at_minus_45 = ep.us_at_minus_45;
  jc.urdf_rad_at_center =
      intuitive_deg_to_urdf_rad(jc.joint_position, deg_at_center.for_position(jc.joint_position));
  // Mount orientation. Optional; defaults to +1 (normal). Only ±1 are valid.
  if (node["direction"]) {
    const int dir = require_scalar<int>(node, "direction", ctx);
    if (dir != 1 && dir != -1) {
      throw std::runtime_error("hexa_hardware: " + ctx + " direction must be +1 or -1");
    }
    jc.direction = static_cast<std::int8_t>(dir);
  }
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

HardwareConfig load_hardware_config(const std::string& hardware_path,
                                    const std::string& calibration_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(hardware_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("hexa_hardware: failed to load " + hardware_path +
                             ": " + e.what());
  }

  const auto calibration = load_calibration(calibration_path);

  HardwareConfig cfg;

  if (const auto conn = root["connection"]) {
    if (conn["type"])   cfg.connection.type   = conn["type"].as<std::string>();
    if (conn["device"]) cfg.connection.device = conn["device"].as<std::string>();
    if (conn["baud"])   cfg.connection.baud   = conn["baud"].as<int>();
  }

  if (const auto parser = root["parser"]) {
    if (parser["type"]) cfg.parser.type = parser["type"].as<std::string>();
    if (parser["get_period_ticks"]) {
      cfg.parser.get_period_ticks = parser["get_period_ticks"].as<int>();
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
    cfg.joints.emplace(name,
                       parse_joint(it->second, name, deg_at_center, calibration));
  }

  return cfg;
}

}  // namespace hexa_hardware
