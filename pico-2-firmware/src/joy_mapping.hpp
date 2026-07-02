// Pure mapping from a gamepad snapshot to high-level commands (plan part 07).
//
// Float fork of hexa_teleop/joy_mapping.py — the same mode FSM, eased yaw /
// wiggle, persistent height, two-press init/revert, record folding, and gait /
// animation cyclers, converted double->float for the RP2350. The Python library
// is unit-testable without rclpy; this fork is likewise ROS-free and drives
// straight off the bt_teleop snapshot (raw int16 axes + a button bitmask).
//
// The per-mode function->key bindings are pre-resolved into JoyKeyRef tables in
// config_generated.hpp (baked from teleop_joy.yaml), so there is no runtime
// string handling: map_joy resolves a function to its physical input by table
// lookup. Arbitration (/teleop/owner) is dropped — the firmware always "owns"
// the sole gamepad, so it always publishes (overview part 00, plan part 07).
#pragma once

#include <cstdint>
#include <string_view>

#include "bt_teleop.hpp"  // kNumAxes

namespace hexa::teleop {

// Input mode. Mirrors joy_mapping's POSTURE / GAIT / ANIMATION string constants.
enum class Mode : std::uint8_t { Posture, Gait, Animation };

// Runtime-mutable stick scaling. The gait's per-leg velocity ceiling is
// gait-specific, so main() rewrites gait_linear_max on every accepted gait
// switch (mirrors teleop_joy's dataclasses.replace on /cmd_gait).
struct JoyConfig {
  float gait_linear_max = 0.0f;
  float gait_angular_z_max = 0.0f;
};

// Persistent mapping state. Rising-edge trackers, eased offsets, the recorded
// posture baseline, and cycler indices — a direct port of joy_mapping.JoyState.
struct JoyState {
  Mode mode = Mode::Posture;
  bool prev_gait_mode = false;
  bool prev_posture_mode = false;
  bool prev_animation_mode = false;
  bool prev_init = false;
  bool prev_record = false;
  float yaw_current = 0.0f;
  float wiggle_amount = 0.0f;
  // Persistent body-height offset — survives release and a mode toggle.
  float height_current = 0.0f;
  // Persistent posture baseline captured by a rising-edge record press; bleeds
  // through into GAIT mode, reset by init when non-default.
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
  // Active animation selection: "" when ANIMATION mode is not in effect.
  std::string_view animation_name = "";
};

// One tick's high-level command. Mirrors joy_mapping.JoyOutput; the two
// "optional" outputs use a has_* flag (None -> has_*=false) since a value of ""
// (restore default animation stack) is distinct from "no change".
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

// Deadband helper — |v| < deadband collapses to 0.
float apply_deadband(float value, float deadband);

// Map one gamepad snapshot to a high-level command, advancing `state`.
//   axes    — kNumAxes raw int16 values (Linux-joystick convention, scaled by
//             1/32767 internally; l2/r2 rest = +max, pressed = -max).
//   buttons — bitmask, bit i set == button index i pressed.
//   dt      — tick period in seconds (eased offsets, height integration).
JoyOutput map_joy(const std::int16_t axes[bt_teleop::kNumAxes],
                  std::uint32_t buttons, const JoyConfig& cfg, JoyState& state,
                  float dt);

// Resolve kInitialMode ("posture"/"gait"/"animation") to a Mode (default Gait).
Mode mode_from_string(std::string_view name);

}  // namespace hexa::teleop
