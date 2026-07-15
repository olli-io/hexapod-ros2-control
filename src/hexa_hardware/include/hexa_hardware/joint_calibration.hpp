// Per-joint mapping from URDF-convention radians to a 14-bit servo pulse
// width (µs).
//
// Calibration is two measured endpoint *magnitudes* — the pulse widths at
// +π/4 and −π/4 from the servo's mechanical center — plus a `direction`
// (+1 / -1) that captures which way the servo is bolted on. Center pulse and
// slope magnitude fall out of the endpoints; `direction` is the sole sign
// source, so a mirror-mounted servo sets `direction: -1` (endpoint ordering
// no longer carries sign).

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

namespace hexa_hardware {

// Which segment of a leg this joint drives, derived at load time from the
// joint name (`..._coxa_joint` / `..._femur_joint` / `..._tibia_joint`).
// Determines how the shared `deg_at_center` table (intuitive degrees, per
// geometry.yaml) is translated to URDF radians at load time:
//   Coxa  — urdf_rad =  deg * π/180     (deg from radial)
//   Femur — urdf_rad = -deg * π/180     (above_horizontal_deg)
//   Tibia — urdf_rad =  π - deg * π/180 (interior_deg)
enum class JointPosition { Coxa, Femur, Tibia };

struct JointCalibration {
  std::uint8_t pin = 0;
  JointPosition joint_position = JointPosition::Coxa;
  // Endpoint pulses are measured in the *servo's* frame — i.e. with the
  // servo shaft at +π/4 and -π/4 from its mechanical center, not the
  // joint at URDF ±π/4. `urdf_rad_at_center` separates the two. Only their
  // magnitude matters; travel direction lives in `direction` below.
  double us_at_plus_45 = 2000.0;
  double us_at_minus_45 = 1000.0;
  // Mount orientation: +1 = joint moves the same way as URDF-positive,
  // -1 = servo is mirror-mounted and travels the opposite way. Because the
  // URDF keeps identical joint axes for left and right legs, a physically
  // mirror-assembled servo needs -1 to realize a URDF-positive command.
  std::int8_t direction = 1;
  // URDF radian the joint sits at when the servo is at its mechanical
  // center (pulse = (us_at_plus_45 + us_at_minus_45) / 2). Captures the
  // assembly offset between servo horn alignment and URDF zero pose.
  // The loader populates this from the YAML's top-level `deg_at_center`
  // table (intuitive degrees) using `joint_position` (derived from the joint
  // name) to pick the entry and the per-position conversion above.
  double urdf_rad_at_center = 0.0;
  std::uint16_t min_us = 500;
  std::uint16_t max_us = 2500;

  // Convert a commanded URDF radian to a clamped 14-bit pulse-width value.
  std::uint16_t to_pulse_us(double theta_rad) const;
};

// Physical-layer settings; the factory reads `type` to pick a concrete
// Transport. `device`/`baud` are interpreted per-transport (e.g. baud
// is honoured for true UART, ignored by USB-CDC, irrelevant for I2C).
struct ConnectionConfig {
  std::string type = "uart";
  std::string device = "/dev/ttyACM0";
  int baud = 115200;
};

// Board-protocol settings; the factory reads `type` to pick a concrete
// BoardProtocol. `get_period_ticks` paces auxiliary GETs against the
// read() cycle and is consumed by HexaHardware, not by the protocol.
struct ParserConfig {
  std::string type = "servo2040";
  int get_period_ticks = 10;
};

struct HardwareConfig {
  ConnectionConfig connection;
  ParserConfig parser;

  std::unordered_map<std::string, JointCalibration> joints;
};

// Load + validate hardware config. `hardware_path` is the YAML with wiring,
// connection/parser, `deg_at_center`, and per-joint pin / direction /
// min_us / max_us. The relay pin and battery-telemetry units are board-owned
// (fixed protocol contract), so they are not configured here.
// `calibration_path` is the sibling servo_calibration.yaml,
// whose `calibration_values` list supplies each servo's endpoint pulse widths
// (indexed by pin: pin N → entry N-1). Throws std::runtime_error on any
// parse / shape / value error.
HardwareConfig load_hardware_config(const std::string& hardware_path,
                                    const std::string& calibration_path);

}  // namespace hexa_hardware
