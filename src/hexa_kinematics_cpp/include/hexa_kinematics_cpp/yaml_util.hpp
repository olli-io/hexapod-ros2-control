// Small yaml-cpp helpers shared by the kinematics loaders. Mirrors the
// require_scalar idiom in hexa_hardware/src/joint_calibration.cpp so missing or
// mistyped keys fail fast with a clear message instead of a default value.
#pragma once

#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace hexa_kinematics {

template <typename T>
T require_scalar(const YAML::Node& node, const std::string& key,
                 const std::string& ctx) {
  if (!node[key]) {
    throw std::runtime_error("hexa_kinematics: " + ctx + " missing key '" + key +
                             "'");
  }
  try {
    return node[key].as<T>();
  } catch (const YAML::Exception&) {
    throw std::runtime_error("hexa_kinematics: " + ctx + " key '" + key +
                             "' has wrong type");
  }
}

inline YAML::Node load_file(const std::string& path) {
  try {
    return YAML::LoadFile(path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("hexa_kinematics: failed to load " + path + ": " +
                             e.what());
  }
}

}  // namespace hexa_kinematics
