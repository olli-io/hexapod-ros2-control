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
//   - the hexa_display BatteryMonitor (hysteresis-debounced low/critical),
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
//   fast blink  — fault: low/critical battery, or a lost link mid-operation.
enum class LedPattern { kSlowBlink, kSolid, kFastBlink };

// Hysteresis-debounced battery flags — float port of hexa_display's
// BatteryMonitor. A threshold of 0 disables that flag entirely (shipped
// default: the divider scale is uncalibrated). A flag raises only after the
// voltage stays below the threshold for hold_s seconds, and clears once it
// rises above threshold + hysteresis (good news is immediate — no hold on the
// way up).
class BatteryMonitor {
 public:
  BatteryMonitor(float warning_v, float critical_v, float hysteresis_v,
                 float hold_s);

  // Feed a fresh voltage sample taken at monotonic time t_s (seconds). Call
  // only when a real reading is available; between samples the flags hold.
  void update(float voltage_v, float t_s);

  bool low() const { return low_; }
  bool critical() const { return critical_; }

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
  float critical_v_;
  float hysteresis_v_;
  float hold_s_;
  bool low_ = false;
  bool critical_ = false;
  Flag warn_;
  Flag crit_;
};

// Static configuration, sourced from the baked config (config_generated.hpp)
// plus the loop's tick geometry.
struct Config {
  float input_timeout_s;          // stale-input watchdog window
  float battery_warning_v;        // 0 disables
  float battery_critical_v;       // 0 disables
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
  bool stood;                   // engine reached a standing posture (past
                                //   INITIALIZE, feet planted) — the arm gate
  bool folded;                  // engine == FOLDED (pre-init / post-fold: the
                                //   safe moment to drop the rail)
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
                          //   safe-stop: stale input OR a low/critical battery.
                          //   The engine then settles (pauses → reseats) rather
                          //   than walking on stale input or a weak pack.
  bool relay_energized;   // drive servo_out::set_relay(...)
  bool battery_low;       // debounced warning flag
  bool battery_critical;  // debounced critical flag
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

 private:
  Config cfg_;
  BatteryMonitor battery_;
  bool relay_armed_ = false;

  TickStats tick_;
  bool have_last_tick_ = false;
  std::uint64_t last_tick_us_ = 0;
};

}  // namespace hexa::supervisor
