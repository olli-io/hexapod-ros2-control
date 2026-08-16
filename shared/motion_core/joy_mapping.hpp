// Pure mapping from a gamepad snapshot (raw int16 axes + a button bitmask) to
// high-level commands: the mode FSM, eased yaw / wiggle, persistent height,
// two-press init/revert, record folding, and the gait / animation cyclers.
//
// The per-mode function->key bindings are pre-resolved into JoyKeyRef tables in
// config_generated.hpp, so there is no runtime string handling. There is no
// /teleop/owner arbitration: the firmware always owns the sole gamepad.
#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include "bt_teleop.hpp"

namespace hexa::teleop {

enum class Mode : std::uint8_t { Posture, Gait, Animation };

// Runtime-mutable stick scaling: the velocity ceiling is per-gait, so the caller
// rewrites these on every accepted gait switch.
struct JoyConfig {
  float gait_linear_max = 0.0f;
  float gait_angular_z_max = 0.0f;
};

// Rising-edge trackers, eased offsets, the recorded posture baseline and the
// cycler indices.
struct JoyState {
  Mode mode = Mode::Posture;
  bool prev_gait_mode = false;
  bool prev_posture_mode = false;
  bool prev_animation_mode = false;
  bool prev_init = false;
  bool prev_record = false;
  float yaw_current = 0.0f;
  float wiggle_amount = 0.0f;
  // Survives release and a mode toggle.
  float height_current = 0.0f;
  // Captured by a rising-edge record press; bleeds through into GAIT mode, reset
  // by init when non-default.
  float recorded_x = 0.0f;
  float recorded_y = 0.0f;
  float recorded_z = 0.0f;
  float recorded_roll = 0.0f;
  float recorded_pitch = 0.0f;
  float recorded_yaw = 0.0f;
  bool reverting = false;
  bool prev_gait_prev = false;
  bool prev_gait_next = false;
  int current_gait_idx = 0;
  bool prev_animation_prev = false;
  bool prev_animation_next = false;
  int current_animation_idx = 0;
  std::string_view animation_name = "";
};

// One tick's high-level command. The two optional outputs carry a has_* flag,
// since "" (restore the default animation stack) is not "no change".
struct JoyOutput {
  float linear_x = 0.0f;
  float linear_y = 0.0f;
  float angular_z = 0.0f;
  float pose_x = 0.0f;
  float pose_y = 0.0f;
  float pose_z = 0.0f;
  float pose_yaw = 0.0f;
  float pose_roll = 0.0f;
  float pose_pitch = 0.0f;
  bool mode_changed = false;
  bool init_request = false;
  bool has_gait_select = false;
  std::string_view gait_select = "";
  bool has_animation_name = false;
  std::string_view animation_name = "";
};

// |v| < deadband collapses to 0; what survives is rescaled onto [0, 1] so the
// output leaves centre continuously.
float apply_deadband(float value, float deadband);

// Fit a stick triple to the reachable velocity envelope: hold the commanded
// direction and map deflection linearly onto 0 -> boundary, so full deflection
// lands exactly on it and no stick travel is dead.
std::array<float, 3> fit_drive_to_envelope(float drive_x, float drive_y,
                                           float drive_yaw);

// Map one gamepad snapshot to a high-level command, advancing `state`. `axes` is
// kNumAxes raw int16 values in the Linux-joystick convention (l2/r2 rest = +max,
// pressed = -max); `buttons` is a bitmask, bit i == button i.
JoyOutput map_joy(const std::int16_t axes[bt_teleop::kNumAxes],
                  std::uint32_t buttons, const JoyConfig& cfg, JoyState& state,
                  float dt);

// "posture"/"gait"/"animation" -> Mode; anything else is Gait.
Mode mode_from_string(std::string_view name);

}  // namespace hexa::teleop
