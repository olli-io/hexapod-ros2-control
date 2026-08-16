// Target-agnostic 200 Hz control brain: the whole velocity / posture / compose /
// IK pipeline plus the failsafe supervisor, in one class with no Pico SDK, no
// ROS, no I/O. The caller owns the hardware seam — input, servo out, clock — and
// each tick samples those, feeds a TickInput and applies the TickResult.
//
// A link-time swap, not #ifdef soup: the Pico firmware, the Gazebo bridge and
// hexa_locomotion each compose their own impls around this same source, so the
// tick logic is compiled unchanged for every target.
#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include "bt_teleop.hpp"
#include "config_generated.hpp"
#include "control.hpp"
#include "gait/engine.hpp"
#include "gait/limits.hpp"
#include "joy_mapping.hpp"
#include "kinematics/body_transform.hpp"
#include "leg_index.hpp"
#include "pipeline_config.hpp"
#include "posture/posture.hpp"
#include "servo_out.hpp"
#include "supervisor.hpp"

namespace hexa::pipeline {

// The tick contract shared by every caller: a 5 ms period (the engine's
// controller_dt) and 1 ms of slack before an interval counts as an overrun.
inline constexpr std::uint64_t kTickPeriodUs = 5'000;    // 5 ms -> 200 Hz
inline constexpr std::uint64_t kTickMarginUs = 1'000;    // overrun slack
inline constexpr float kDt = 0.005f;                     // engine tick, seconds

// The command seam: this tick's velocity/pose/gait/animation intent, decoupled
// from where it came from. The Pico and the Gazebo /joy bridge produce it with
// map_joy(); hexa_locomotion builds it from /cmd_vel + /cmd_gait +
// /gait/initialize + /body/pose + /animation/mode.
using CommandIntent = hexa::teleop::JoyOutput;

struct TickInput {
  std::uint64_t now_us = 0;              // monotonic clock
  const std::int16_t* axes = nullptr;    // bt_teleop::kNumAxes raw int16 entries
  std::uint32_t buttons = 0;             // button bitmask (bit i == button i)
  bool bt_connected = false;             // input link up
  std::uint64_t last_input_us = 0;       // freshness for the watchdog (0 = none)
  bool battery_valid = false;            // a fresh battery sample is present
  float battery_v = 0.0f;                // decoded pack voltage (iff valid)
  bool hardware_fault = false;           // over-current trip latched: the engine
                                         //   latches FAULT, the rail disarms
  float dt = kDt;                        // tick period, seconds
};

// Which branch the init-button edge drove the engine down (for the caller log).
enum class InitAction { kNone, kInitialized, kFoldRequested };

// Everything the caller needs to drive the outputs and log this tick.
struct TickResult {
  // 18 joint angles, URDF-convention rad, leg-major/segment-minor — the order of
  // servo_out's pin table and the sim controller's joints: list alike. Holds the
  // per-leg last-good angle across an UnreachableTarget.
  float theta[servo_out::kNumJoints] = {};
  int unreachable = 0;                   // legs that held last-good this tick

  // force_zero was already applied to the command inside tick(). The caller
  // drives the relay + LED off this, and undervolt_stage off whatever alarm
  // channel it has (/buzzer/play on the Pi, the status LED on the Pico).
  hexa::supervisor::Decision decision{};
  bool relay_energized = false;          // == decision.relay_energized

  // Rung 2 queued a fold and the engine accepted it: a one-tick log edge.
  bool undervolt_fold_requested = false;

  hexa::gait::EngineState engine_state = hexa::gait::EngineState::FOLDED;
  float master_phase = 0.0f;
  bool walking = false;                  // shaped-command non-zero (posture gate)

  // Raw teleop intent for an on-board face policy: pre-shaping, so gaze tracks
  // the stick rather than the slew. The pose-mode expressions need the body shift
  // as well as the tilt to tell them apart.
  float cmd_vx = 0.0f, cmd_vy = 0.0f, cmd_wz = 0.0f;
  float pose_x = 0.0f, pose_y = 0.0f;
  float pose_roll = 0.0f, pose_pitch = 0.0f, pose_yaw = 0.0f;

  // Teleop events — the caller's log surface.
  bool mode_changed = false;
  hexa::teleop::Mode mode = hexa::teleop::Mode::Posture;
  bool init_request = false;
  InitAction init_action = InitAction::kNone;
  bool has_gait_select = false;
  std::string gait_select;               // requested strategy (accepted or not)
  bool gait_accepted = false;            // switch allowed in the current state
  float gait_linear_max = 0.0f;          // new stick cap iff accepted
  float gait_angular_z_max = 0.0f;       // ditto, the yaw half
  bool has_animation_name = false;
  std::string animation_name;            // requested animation mode
  bool animation_accepted = false;       // known animation (else warn-and-ignore)
};

class Pipeline {
 public:
  // The baked config: FOLDED cold start, default gait, initial teleop mode,
  // standing-pose last-good seed.
  Pipeline();

  // Runtime-supplied geometry + tuning (hexa_locomotion's YAML path). The
  // supervisor and joy config stay baked, being out of that scope.
  explicit Pipeline(const PipelineConfig& config);

  // Joy path: map the gamepad snapshot in `in` through map_joy() using the
  // pipeline's own JoyConfig/JoyState, then run the core tick.
  TickResult tick(const TickInput& in);

  // Core tick: supervisor -> control.shape -> engine.update -> posture.update ->
  // compose/IK on an already-mapped command. `in` supplies the
  // clock/link/battery/dt seam; its axes/buttons are ignored here. Records the
  // tick edge for jitter accounting.
  TickResult tick(const CommandIntent& cmd, const TickInput& in);

  // For the caller's heartbeat / diagnostics.
  const hexa::gait::Engine& engine() const { return *engine_; }
  hexa::gait::Engine& engine() { return *engine_; }
  const hexa::supervisor::Supervisor& supervisor() const { return supervisor_; }
  const hexa::posture::PostureController& posture() const { return posture_; }
  // Solved from the standing-pose scalars; seeds the caller's pre-first-tick
  // joint dump.
  const std::array<hexa::JointAngles, hexa::kNumLegs>& standing_pose() const {
    return standing_pose_;
  }

 private:
  // Engine foot targets -> theta_, via apply_body_pose -> body_to_leg ->
  // inverse_kinematics, holding last-good on UnreachableTarget. Returns the
  // unreachable-leg count.
  int compose_gait(const std::map<std::string, hexa::gait::LegOutput>& out,
                   const hexa::BodyPose& body_pose);

  // Solved before engine_, which takes it by reference. Declaration order is
  // initialization order.
  std::array<hexa::JointAngles, hexa::kNumLegs> standing_pose_;
  // The lever arms a yaw command acts through, which Control shapes the envelope
  // against. Declared before control_ so it is initialized first.
  std::map<std::string, hexa::Vec3> nominal_stance_;
  std::unique_ptr<hexa::gait::Engine> engine_;
  hexa::gait::VelocityCaps caps_;
  hexa::control::Control control_;
  hexa::teleop::JoyConfig joycfg_;
  hexa::teleop::JoyState joystate_;
  hexa::posture::PostureController posture_;
  hexa::supervisor::Supervisor supervisor_;

  // Leg geometry for compose_gait's IK, held so a runtime PipelineConfig's
  // geometry threads through to the per-leg solve.
  std::array<hexa::config::LegSpec, hexa::kNumLegs> leg_specs_;

  // Persisted across ticks, seeded with the standing pose so the first tick's
  // held legs are valid.
  float theta_[servo_out::kNumJoints];
};

}  // namespace hexa::pipeline
