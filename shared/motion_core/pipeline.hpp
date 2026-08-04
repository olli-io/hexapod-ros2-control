// Target-agnostic 200 Hz control brain (plan part 10, Tier 2).
//
// The whole velocity / posture / compose / IK pipeline plus the failsafe
// supervisor, factored out of main.cpp into ONE class with no Pico SDK, no ROS,
// no I/O. The caller owns the hardware-touching seam — input (bt_teleop / a
// scripted axes source / a ROS /joy bridge), servo out (the Chica UART / a sim
// publisher / a log), and the clock (time_us_64 / std::chrono / a ROS clock) —
// and each tick it samples those, feeds a TickInput, and applies the TickResult
// (drive the servos from `theta`, the relay from `relay_energized`, the LED from
// `decision.led`, and log off the event flags).
//
// This is the "link-time swap, not #ifdef soup" the plan calls for: the Pico
// firmware (main.cpp) composes the Pico impls around this; the Gazebo bridge
// (hexa_pico_bridge/firmware_bridge_node.cpp) composes ROS impls around the very
// same source. The tick logic — every line that could carry a port bug — is
// compiled unchanged for both targets and exercised off-target by the host
// harness (test/host/test_pipeline.cpp).
#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include "bt_teleop.hpp"                   // bt_teleop::kNumAxes
#include "config_generated.hpp"            // hexa::config::LegSpec (leg_specs_)
#include "control.hpp"                     // hexa::control::Control
#include "gait/engine.hpp"                 // hexa::gait::Engine, EngineState
#include "gait/limits.hpp"                 // hexa::gait::VelocityCaps
#include "joy_mapping.hpp"                 // hexa::teleop::JoyConfig / JoyState / Mode
#include "kinematics/body_transform.hpp"   // hexa::BodyPose
#include "leg_index.hpp"                   // hexa::kNumLegs
#include "pipeline_config.hpp"             // hexa::pipeline::PipelineConfig
#include "posture/posture.hpp"             // hexa::posture::PostureController
#include "servo_out.hpp"                   // servo_out::kNumJoints
#include "supervisor.hpp"                  // hexa::supervisor::Supervisor

namespace hexa::pipeline {

// The 200 Hz tick contract shared by every caller: 5 ms period (matches the ROS
// node rate and the engine controller_dt), 1 ms of slack before an inter-tick
// interval counts as a deadline overrun. kDt is the same period in seconds, fed
// to the engine / filters.
inline constexpr std::uint64_t kTickPeriodUs = 5'000;    // 5 ms -> 200 Hz
inline constexpr std::uint64_t kTickMarginUs = 1'000;    // overrun slack
inline constexpr float kDt = 0.005f;                     // engine tick, seconds

// The high-level command the tick consumes: the velocity/pose/gait/animation
// intent for this tick, decoupled from *where* it came from. The Pico and the
// Gazebo /joy bridge produce it with map_joy() off a gamepad snapshot; the ROS
// hexa_locomotion node builds it from /cmd_vel + /cmd_gait + /gait/initialize +
// /body/pose + /animation/mode. Both feed the same core tick — this is the
// command seam (see the two-arg tick() overload below). It is exactly
// map_joy's output type, so the joy path passes its result straight through.
using CommandIntent = hexa::teleop::JoyOutput;

// What the caller sampled for this tick from its input / clock / telemetry seam.
struct TickInput {
  std::uint64_t now_us = 0;              // monotonic clock (caller's seam)
  const std::int16_t* axes = nullptr;    // bt_teleop::kNumAxes raw int16 entries
  std::uint32_t buttons = 0;             // button bitmask (bit i == button i)
  bool bt_connected = false;             // input link up
  std::uint64_t last_input_us = 0;       // freshness for the watchdog (0 = none)
  bool battery_valid = false;            // a fresh battery sample is present
  float battery_v = 0.0f;                // decoded pack voltage (iff valid)
  bool hardware_fault = false;           // board over-current trip latched (the
                                         //   host read a non-zero STATUS): the
                                         //   engine latches FAULT, rail disarmed
  float dt = kDt;                        // tick period, seconds
};

// Which branch the init-button edge drove the engine down (for the caller log).
enum class InitAction { kNone, kInitialized, kFoldRequested };

// Everything the caller needs to drive the outputs and log this tick.
struct TickResult {
  // 18 joint angles, URDF-convention rad, leg-major/segment-minor order
  // (l_front,l_middle,l_rear,r_front,r_middle,r_rear x coxa,femur,tibia) —
  // identical to servo_out's pin table AND the sim controller's joints: list, so
  // the servo sink and the Gazebo publisher both consume it directly. Holds the
  // per-leg last-good angle across an UnreachableTarget (no command spike).
  float theta[servo_out::kNumJoints] = {};
  int unreachable = 0;                   // legs that held last-good this tick

  // Failsafe / telemetry / LED decision. force_zero was already applied to the
  // command inside tick(); the caller drives the relay + LED off this, and
  // decision.undervolt_stage off whatever alarm channel it has (hexa_hardware's
  // /buzzer/play publisher on the Pi, the status LED on the Pico).
  hexa::supervisor::Decision decision{};
  bool relay_energized = false;          // == decision.relay_energized

  // Rung 2 queued a fold this tick AND the engine accepted it. One-tick edge for
  // the caller's log; false if the request was refused.
  bool undervolt_fold_requested = false;

  // Engine / posture snapshot for logging + diagnostics.
  hexa::gait::EngineState engine_state = hexa::gait::EngineState::FOLDED;
  float master_phase = 0.0f;
  bool walking = false;                  // shaped-command non-zero (posture gate)

  // Raw teleop intent surfaced for an on-board face policy — the unshaped
  // command velocity and the user body pose, mirroring the ROS face's /cmd_vel
  // and /body/pose inputs (pre-shaping, so gaze tracks the stick not the slew).
  float cmd_vx = 0.0f, cmd_vy = 0.0f, cmd_wz = 0.0f;
  float pose_roll = 0.0f, pose_pitch = 0.0f, pose_yaw = 0.0f;

  // Teleop events — the caller's log surface (main's [teleop]/[safety] lines).
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
  // Builds the engine / control / posture / supervisor from the baked config
  // (config_generated.hpp) exactly as main.cpp used to: FOLDED cold-start,
  // default gait, initial teleop mode, standing-pose last-good seed. Delegates
  // to the PipelineConfig ctor with PipelineConfig::baked().
  Pipeline();

  // Build the engine / control / posture from a runtime-supplied geometry +
  // tuning config (hexa_locomotion's YAML path). The supervisor and joy config
  // stay baked (out of the geometry+tuning scope). PipelineConfig::baked()
  // reproduces the no-arg behavior exactly.
  explicit Pipeline(const PipelineConfig& config);

  // Joy-path convenience: map the raw gamepad snapshot in `in` (axes/buttons)
  // through map_joy() using the pipeline's own JoyConfig/JoyState, then run the
  // core tick. The Pico firmware and the /joy Gazebo bridge use this overload.
  TickResult tick(const TickInput& in);

  // Core control tick — the command seam. Runs supervisor -> control.shape ->
  // engine.update -> posture.update -> compose/IK on an already-mapped command,
  // independent of input source. hexa_locomotion builds `cmd` from ROS topics
  // (/cmd_vel etc.) and calls this directly; the joy overload above builds `cmd`
  // via map_joy and delegates here. `in` supplies the clock/link/battery/dt seam
  // (its axes/buttons are ignored on this path). Records the tick edge for jitter
  // accounting (supervisor.record_tick), so the caller need only feed now_us once.
  TickResult tick(const CommandIntent& cmd, const TickInput& in);

  // Accessors for the caller's heartbeat / diagnostics (main's [gait]/[safety]
  // heartbeat reads engine state + master phase and the supervisor tick stats).
  const hexa::gait::Engine& engine() const { return *engine_; }
  hexa::gait::Engine& engine() { return *engine_; }
  const hexa::supervisor::Supervisor& supervisor() const { return supervisor_; }
  const hexa::posture::PostureController& posture() const { return posture_; }
  // Per-leg resting joint angles, solved from the standing-pose scalars. The
  // caller uses these to seed its own pre-first-tick joint dump.
  const std::array<hexa::JointAngles, hexa::kNumLegs>& standing_pose() const {
    return standing_pose_;
  }

 private:
  // Compose the engine's per-leg body-frame foot targets into theta_ (the exact
  // ik_node.cpp ordering: apply_body_pose -> body_to_leg -> inverse_kinematics),
  // holding last-good on UnreachableTarget. Returns the unreachable-leg count.
  int compose_gait(const std::map<std::string, hexa::gait::LegOutput>& out,
                   const hexa::BodyPose& body_pose);

  // Solved before engine_ — make_default_engine takes it by reference, and the
  // theta_ seed below reads it. Declaration order is initialization order.
  std::array<hexa::JointAngles, hexa::kNumLegs> standing_pose_;
  // Standing foot positions, solved from standing_pose_. These are the lever arms
  // a yaw command acts through, so Control shapes the envelope against them.
  // Declared here so it is initialized before control_.
  std::map<std::string, hexa::Vec3> nominal_stance_;
  std::unique_ptr<hexa::gait::Engine> engine_;
  hexa::gait::VelocityCaps caps_;
  hexa::control::Control control_;
  hexa::teleop::JoyConfig joycfg_;
  hexa::teleop::JoyState joystate_;
  hexa::posture::PostureController posture_;
  hexa::supervisor::Supervisor supervisor_;

  // Leg geometry for compose_gait's IK (was config::kLegSpecs; now held so a
  // runtime PipelineConfig's geometry threads through to the per-leg solve).
  std::array<hexa::config::LegSpec, hexa::kNumLegs> leg_specs_;

  // Last-good joint angles, persisted across ticks (seeded with the standing
  // pose so the very first tick's held legs are valid).
  float theta_[servo_out::kNumJoints];
};

}  // namespace hexa::pipeline
