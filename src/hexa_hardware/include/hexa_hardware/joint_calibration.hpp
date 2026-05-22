// Per-joint mapping from URDF-convention radians to a 14-bit servo pulse
// width (µs).
//
// Calibration is two measured endpoints: the pulse width that drives the
// joint to +π/4 rad, and the pulse width at −π/4 rad. Center and slope
// (with sign) fall out automatically; a reversed mount is expressed by
// swapping the two values.

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

namespace hexa_hardware {

// Which segment of a leg this joint drives. Determines how the shared
// `deg_at_center` table (intuitive degrees, per geometry.yaml) is
// translated to URDF radians at load time:
//   Coxa  — urdf_rad =  deg * π/180     (deg from radial)
//   Femur — urdf_rad = -deg * π/180     (above_horizontal_deg)
//   Tibia — urdf_rad =  π - deg * π/180 (interior_deg)
enum class JointPosition { Coxa, Femur, Tibia };

struct JointCalibration {
  std::uint8_t pin = 0;
  JointPosition joint_position = JointPosition::Coxa;
  // Endpoint pulses are measured in the *servo's* frame — i.e. with the
  // servo shaft at +π/4 and -π/4 from its mechanical center, not the
  // joint at URDF ±π/4. `urdf_rad_at_center` separates the two.
  double us_at_plus_45 = 2000.0;
  double us_at_minus_45 = 1000.0;
  // URDF radian the joint sits at when the servo is at its mechanical
  // center (pulse = (us_at_plus_45 + us_at_minus_45) / 2). Captures the
  // assembly offset between servo horn alignment and URDF zero pose.
  // The loader populates this from the YAML's top-level `deg_at_center`
  // table (intuitive degrees) using `joint_position` to pick the entry
  // and the per-position conversion above.
  double urdf_rad_at_center = 0.0;
  std::uint16_t min_us = 500;
  std::uint16_t max_us = 2500;

  // Convert a commanded URDF radian to a clamped 14-bit pulse-width value.
  std::uint16_t to_pulse_us(double theta_rad) const;
};

struct AuxChannel {
  std::uint8_t pin = 0;
  double scale = 1.0;  // raw 14-bit count * scale = engineering unit
};

struct HardwareConfig {
  std::string serial_device = "/dev/ttyACM0";
  int serial_baud = 115200;
  int get_period_ticks = 10;

  std::uint8_t relay_pin = 0;
  bool relay_configured = false;

  std::unordered_map<std::string, AuxChannel> aux;
  std::unordered_map<std::string, JointCalibration> joints;
};

// Load + validate hardware config from a YAML file. Throws std::runtime_error
// on any parse / shape / value error.
HardwareConfig load_hardware_config(const std::string& path);

}  // namespace hexa_hardware
