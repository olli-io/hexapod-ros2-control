// The robot's safety/health coordinator: the stale-input watchdog, the
// hysteresis-debounced undervoltage ladder, the relay-arming discipline and the
// status-LED policy. PURE logic (no Pico SDK, no I/O) — main.cpp feeds it
// per-tick observations and applies the decisions it returns.
//
// The other two failsafes live elsewhere: the IK guard in the compose step, the
// boot-FOLDED ordering in the engine's cold start.
#pragma once

#include <cstdint>

namespace hexa::supervisor {

// main.cpp turns a pattern into GPIO toggling against the wall clock.
//   slow blink — idle / standing, or pre-link scanning at boot,
//   solid      — BT linked and walking,
//   fast blink — fault: any undervoltage rung, or a link lost mid-operation.
enum class LedPattern { kSlowBlink, kSolid, kFastBlink };

// Three escalating responses to a draining pack:
//
//   kWarn   — beep only; the robot stays drivable so it can be walked home.
//   kFold   — command a fold and safe-stop; the clean-fold edge cuts the rail.
//   kCutoff — drop the rail now, whatever the posture, and refuse to re-arm.
//
// MONOTONIC within a power cycle: cutting the rail unloads the pack and the
// voltage rebounds, so a ladder that de-escalated would cut, re-arm, sag and cut
// again. The latch is in-memory.
enum class UndervoltStage : std::uint8_t {
  kNone = 0,
  kWarn = 1,
  kFold = 2,
  kCutoff = 3,
};

// Hysteresis-debounced flags, one per rung. A threshold of 0 disables that rung
// (the shipped default — the divider scale is uncalibrated). A flag raises only
// after hold_s below the threshold and clears above threshold + hysteresis, with
// no hold on the way up, so a sag that recovers in time never arms a rung.
// Bidirectional by design: the Supervisor owns the latching.
class BatteryMonitor {
 public:
  BatteryMonitor(float warning_v, float fold_v, float cutoff_v,
                 float hysteresis_v, float hold_s);

  // Call only with a real reading; between samples the flags hold.
  void update(float voltage_v, float t_s);

  bool warn() const { return warn_active_; }
  bool fold() const { return fold_active_; }
  bool cutoff() const { return cutoff_active_; }

 private:
  // below_since is valid only while has_since is set.
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

struct Observation {
  std::uint64_t now_us;         // monotonic time (time_us_64)
  bool bt_connected;            // bt_teleop::connected()
  std::uint64_t last_input_us;  // bt_teleop::last_data_us() (0 = no frame yet)
  bool armable;                 // the arm gate: any engine state but FAULT
  bool folded;                  // engine == FOLDED. The *rising* edge is a
                                //   completed park and the safe moment to drop
                                //   the rail; folded at boot is not a park, so
                                //   the level alone does not disarm.
  bool walking;                 // gait active (non-zero cmd_vel in a gait state)
  bool battery_valid;           // a fresh battery sample is present this tick
  float battery_v;              // decoded pack voltage (valid iff battery_valid)
  bool fault;                   // over-current trip latched: disarm now
};

// Decisions main applies this tick.
struct Decision {
  bool input_stale;       // watchdog fired (stale/lost BT link)
  bool force_zero;        // aggregate safe-stop: stale input OR the ladder at
                          //   kFold+. Not kWarn, which stays drivable.
  bool relay_energized;   // drive servo_out::set_relay(...)

  // `undervolt_stage` is latched; `request_fold` is a one-tick edge on first
  // reaching kFold, so the caller queues exactly one fold.
  UndervoltStage undervolt_stage;
  bool request_fold;

  bool undervolt_cutoff;  // == (undervolt_stage == kCutoff); rail latched off
  bool fault;             // aggregate fault (drives the fast-blink LED)
  LedPattern led;
};

class Supervisor {
 public:
  explicit Supervisor(const Config& cfg);

  Decision step(const Observation& obs);

  // Tick-jitter accounting, separate from step() so it samples exactly when the
  // scheduler fires rather than when an observation happens to arrive.
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

  UndervoltStage undervolt_stage() const { return stage_; }

 private:
  Config cfg_;
  BatteryMonitor battery_;
  bool relay_armed_ = false;
  bool prev_folded_ = false;  // for the FOLDING -> FOLDED park edge

  UndervoltStage stage_ = UndervoltStage::kNone;
  bool fold_requested_ = false;  // the kFold edge has already been emitted

  TickStats tick_;
  bool have_last_tick_ = false;
  std::uint64_t last_tick_us_ = 0;
};

}  // namespace hexa::supervisor
