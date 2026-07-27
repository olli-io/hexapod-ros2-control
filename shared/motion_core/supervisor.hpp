// Integration supervisor — failsafes, telemetry debounce, status LED, health
// metrics (plan part 09).
//
// The robot's safety/health coordinator, kept as PURE logic (no Pico SDK, no
// I/O) so it unit-tests off-target like the rest of the port. main.cpp feeds it
// per-tick observations (time, BT-link freshness, engine posture, battery
// samples) and applies the decisions it returns — zero the cmd_vel, drive the
// servo-rail relay, set the status LED. It folds several ROS-side mechanisms
// into one onboard unit:
//   - the hexa_webteleop input watchdog (stale input -> zero velocity),
//   - the hexa_display BatteryMonitor (hysteresis-debounced), widened into the
//     three-rung undervoltage ladder (beep -> fold+cut -> hard cutoff),
//   - the relay-arming discipline the real bringup enforces by launch order,
//   - a status-LED policy standing in for the dropped hexa_display face.
//
// The IK guard (hold last-good angle on UnreachableTarget) and the boot-FOLDED
// startup ordering — the other two failsafes in the part 09 scope — already
// live in the compose step and the engine's cold-start; this module owns the
// rest.
#pragma once

#include <cstdint>

namespace hexa::supervisor {

// Onboard status-LED cadence. main.cpp turns a pattern into GPIO toggling
// against the wall clock. Mirrors the part 09 spec:
//   slow blink  — idle / standing (armed, feet planted, not walking) or
//                 pre-link scanning at boot,
//   solid       — BT linked and walking (gait active),
//   fast blink  — fault: any rung of the undervoltage ladder, or a lost link
//                 mid-operation.
enum class LedPattern { kSlowBlink, kSolid, kFastBlink };

// The undervoltage ladder — three escalating responses to a draining pack:
//
//   kWarn   — beep, nothing else. The robot stays drivable on purpose, so it can
//             be walked home.
//   kFold   — command a fold and safe-stop. The rail is cut by the clean-fold
//             edge once parked, so the robot goes down under control.
//   kCutoff — drop the rail now, whatever the posture, and refuse to re-arm.
//
// MONOTONIC within a power cycle. Cutting the rail unloads the pack and the
// voltage rebounds, so a ladder that de-escalated would cut, re-arm, sag and cut
// again. The latch is in-memory: a power cycle is the only reset.
enum class UndervoltStage : std::uint8_t {
  kNone = 0,
  kWarn = 1,
  kFold = 2,
  kCutoff = 3,
};

// Hysteresis-debounced battery flags — float port of hexa_display's
// BatteryMonitor, widened to the three rungs above. A threshold of 0 disables
// that rung (shipped default: the divider scale is uncalibrated). A flag raises
// only after the voltage stays below the threshold for hold_s seconds, and
// clears once it rises above threshold + hysteresis (no hold on the way up).
//
// Bidirectional by design: the Supervisor owns the latching. A sag that recovers
// before hold_s never arms a rung, which is what keeps load transients out.
class BatteryMonitor {
 public:
  BatteryMonitor(float warning_v, float fold_v, float cutoff_v,
                 float hysteresis_v, float hold_s);

  // Feed a fresh voltage sample taken at monotonic time t_s (seconds). Call
  // only when a real reading is available; between samples the flags hold.
  void update(float voltage_v, float t_s);

  bool warn() const { return warn_active_; }
  bool fold() const { return fold_active_; }
  bool cutoff() const { return cutoff_active_; }

 private:
  // One debounced threshold. below_since is valid only while has_since is set
  // (stands in for the reference's Optional[float]).
  struct Flag {
    bool active = false;
    bool has_since = false;
    float below_since = 0.0f;
  };
  void step(Flag& f, float voltage, float t, float threshold) const;

  float warning_v_;
  float fold_v_;
  float cutoff_v_;
  float hysteresis_v_;
  float hold_s_;
  bool warn_active_ = false;
  bool fold_active_ = false;
  bool cutoff_active_ = false;
  Flag warn_;
  Flag fold_;
  Flag cutoff_;
};

// Static configuration, sourced from the baked config (config_generated.hpp)
// plus the loop's tick geometry.
struct Config {
  float input_timeout_s;          // stale-input watchdog window
  float battery_warning_v;        // ladder rung 1 (beep);      0 disables
  float battery_fold_v;           // ladder rung 2 (fold+cut);  0 disables
  float battery_cutoff_v;         // ladder rung 3 (cut+latch); 0 disables
  float battery_hysteresis_v;
  float battery_hold_s;
  std::uint64_t tick_period_us;   // nominal control-tick period
  std::uint64_t tick_margin_us;   // slack before an interval counts as an overrun
};

// Per-tick observation from the live system (every field main already holds).
struct Observation {
  std::uint64_t now_us;         // monotonic time (time_us_64)
  bool bt_connected;            // bt_teleop::connected()
  std::uint64_t last_input_us;  // bt_teleop::last_data_us() (0 = no frame yet)
  bool armable;                 // engine is in a state the rail may be closed
                                //   in (anything but FAULT) — the arm gate
  bool folded;                  // engine == FOLDED. Its *rising* edge is a
                                //   completed park (FOLDING -> FOLDED), the
                                //   safe moment to drop the rail. Being folded
                                //   at boot is not a park, so the level alone
                                //   does not disarm.
  bool walking;                 // gait active (non-zero cmd_vel in a gait state)
  bool battery_valid;           // a fresh battery sample is present this tick
  float battery_v;              // decoded pack voltage (valid iff battery_valid)
  bool fault;                   // hardware over-current trip latched: disarm the
                                //   rail now and sound the alarm cadence
};

// Decisions main applies this tick.
struct Decision {
  bool input_stale;       // watchdog fired (stale/lost BT link)
  bool force_zero;        // main must zero cmd_vel this tick — the aggregate
                          //   safe-stop: stale input OR the ladder at kFold+.
                          //   NOT set by kWarn, which stays drivable.
  bool relay_energized;   // drive servo_out::set_relay(...)

  // Undervoltage ladder (see UndervoltStage). `undervolt_stage` is latched;
  // `request_fold` is a one-tick edge on first reaching kFold, so the caller
  // queues exactly one fold rather than re-asking every tick.
  UndervoltStage undervolt_stage;
  bool request_fold;

  bool undervolt_cutoff;  // == (undervolt_stage == kCutoff); rail latched off
  bool fault;             // aggregate fault (drives the fast-blink LED)
  LedPattern led;
};

class Supervisor {
 public:
  explicit Supervisor(const Config& cfg);

  // Run the failsafe/telemetry policy for one tick.
  Decision step(const Observation& obs);

  // Tick-jitter accounting — call once per control tick with the loop clock.
  // Kept separate from step() so it can be sampled at exactly the tick the
  // scheduler fires, independent of the (possibly rate-limited) observation.
  void record_tick(std::uint64_t now_us);

  struct TickStats {
    std::uint32_t count = 0;         // inter-tick intervals recorded
    std::uint64_t last_dt_us = 0;    // most recent interval
    std::uint64_t min_dt_us = 0;
    std::uint64_t max_dt_us = 0;
    std::uint32_t overruns = 0;      // intervals over period + margin
  };
  const TickStats& tick_stats() const { return tick_; }

  bool relay_armed() const { return relay_armed_; }

  // Monotonic for the life of this object — see UndervoltStage.
  UndervoltStage undervolt_stage() const { return stage_; }

 private:
  Config cfg_;
  BatteryMonitor battery_;
  bool relay_armed_ = false;
  bool prev_folded_ = false;  // for the FOLDING -> FOLDED park edge

  // Latched ladder rung. In-memory only: a power cycle is the sole reset.
  UndervoltStage stage_ = UndervoltStage::kNone;
  bool fold_requested_ = false;  // the kFold edge has already been emitted

  TickStats tick_;
  bool have_last_tick_ = false;
  std::uint64_t last_tick_us_ = 0;
};

}  // namespace hexa::supervisor
