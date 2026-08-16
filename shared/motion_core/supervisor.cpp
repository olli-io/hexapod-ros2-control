#include "supervisor.hpp"

namespace hexa::supervisor {

BatteryMonitor::BatteryMonitor(float warning_v, float fold_v, float cutoff_v,
                               float hysteresis_v, float hold_s)
    : warning_v_(warning_v),
      fold_v_(fold_v),
      cutoff_v_(cutoff_v),
      hysteresis_v_(hysteresis_v),
      hold_s_(hold_s) {}

void BatteryMonitor::step(Flag& f, float voltage, float t,
                          float threshold) const {
  if (threshold <= 0.0f) {  // disabled
    f.active = false;
    f.has_since = false;
    return;
  }
  if (f.active) {
    if (voltage > threshold + hysteresis_v_) {  // recovered
      f.active = false;
      f.has_since = false;
    }
    return;
  }
  if (voltage < threshold) {
    if (!f.has_since) {
      f.below_since = t;
      f.has_since = true;
    }
    if (t - f.below_since >= hold_s_) {
      f.active = true;
    }
    return;
  }
  f.has_since = false;  // back above threshold before the hold elapsed
}

void BatteryMonitor::update(float voltage_v, float t_s) {
  step(warn_, voltage_v, t_s, warning_v_);
  step(fold_, voltage_v, t_s, fold_v_);
  step(cutoff_, voltage_v, t_s, cutoff_v_);
  warn_active_ = warn_.active;
  fold_active_ = fold_.active;
  cutoff_active_ = cutoff_.active;
}

Supervisor::Supervisor(const Config& cfg)
    : cfg_(cfg),
      battery_(cfg.battery_warning_v, cfg.battery_fold_v, cfg.battery_cutoff_v,
               cfg.battery_hysteresis_v, cfg.battery_hold_s) {}

Decision Supervisor::step(const Observation& obs) {
  if (obs.battery_valid) {
    battery_.update(obs.battery_v, static_cast<float>(obs.now_us) * 1e-6f);
  }

  // Take the deepest rung the flags justify, then latch it. Reading the deepest
  // rather than chaining them means a config with only cutoff_v set still
  // escalates correctly.
  UndervoltStage observed = UndervoltStage::kNone;
  if (battery_.warn()) observed = UndervoltStage::kWarn;
  if (battery_.fold()) observed = UndervoltStage::kFold;
  if (battery_.cutoff()) observed = UndervoltStage::kCutoff;
  if (observed > stage_) {
    stage_ = observed;
  }
  // Separate from stage_ so kFold -> kCutoff does not re-fire a fold the cutoff
  // is about to make moot.
  const bool request_fold = stage_ >= UndervoltStage::kFold && !fold_requested_;
  if (request_fold) {
    fold_requested_ = true;
  }
  const bool batt_warn = stage_ >= UndervoltStage::kWarn;
  const bool batt_fold = stage_ >= UndervoltStage::kFold;
  const bool batt_cutoff = stage_ == UndervoltStage::kCutoff;

  // A live link that stops delivering frames for input_timeout_s is stale, as is
  // a clean disconnect. Main then zeroes the command so the engine settles rather
  // than latching the last velocity.
  bool input_stale;
  if (!obs.bt_connected) {
    input_stale = true;
  } else if (obs.last_input_us == 0) {
    input_stale = true;  // connected but no frame observed yet
  } else {
    const float since_s =
        static_cast<float>(obs.now_us - obs.last_input_us) * 1e-6f;
    input_stale = since_s >= cfg_.input_timeout_s;
  }

  // Arm once the link is up and the engine is armable, FOLDED included, so the
  // robot comes up energized and the INITIALIZE ladder runs with the rail live.
  // The consumer staggers the servo energize leg by leg (hexa::EnergizeSweep) so
  // closing the relay is not an inrush spike.
  //
  // Disarm on the *rising* edge of `folded` (a completed park), a stage-3 cutoff,
  // or a latched over-current. Folded at boot is not a park — the rail is not
  // armed yet, so the edge is a no-op. A stale link deliberately does NOT drop
  // the rail: cutting servo power mid-stance would collapse the robot.
  //
  // Rung 2 folds rather than cutting, so the legs fold under power, and rung 3 is
  // the backstop if that fold never completes. Re-arming is blocked from rung 2
  // on: unloading the rail lets the voltage rebound, and a gate reading only the
  // live flags would re-arm into the same sag.
  const bool fold_completed = obs.folded && !prev_folded_;
  prev_folded_ = obs.folded;
  if (relay_armed_) {
    if (fold_completed || batt_cutoff || obs.fault) {
      relay_armed_ = false;
    }
  } else if (obs.bt_connected && obs.armable && !batt_fold && !obs.fault) {
    relay_armed_ = true;
  }

  // Zero the command on a stale link or the ladder at rung 2+, so the robot
  // settles while the fold runs. Rung 1 is absent: a warning that immobilizes the
  // robot strands it.
  const bool force_zero = input_stale || batt_fold;

  // Any ladder rung is a fault; a lost link only once armed, so pre-link boot
  // scanning stays a calm slow blink.
  const bool bt_lost = relay_armed_ && !obs.bt_connected;
  const bool fault = batt_warn || bt_lost || obs.fault;

  // Gated on the safe-stop, so the LED cannot read solid off a latched velocity
  // that is being discarded.
  LedPattern led;
  if (fault) {
    led = LedPattern::kFastBlink;
  } else if (obs.bt_connected && obs.walking && !force_zero) {
    led = LedPattern::kSolid;
  } else {
    led = LedPattern::kSlowBlink;
  }

  Decision d;
  d.input_stale = input_stale;
  d.force_zero = force_zero;
  d.relay_energized = relay_armed_;
  d.undervolt_stage = stage_;
  d.request_fold = request_fold;
  d.undervolt_cutoff = batt_cutoff;
  d.fault = fault;
  d.led = led;
  return d;
}

void Supervisor::record_tick(std::uint64_t now_us) {
  if (!have_last_tick_) {
    have_last_tick_ = true;
    last_tick_us_ = now_us;
    return;
  }
  const std::uint64_t dt = now_us - last_tick_us_;
  last_tick_us_ = now_us;
  tick_.last_dt_us = dt;
  if (tick_.count == 0) {
    tick_.min_dt_us = dt;
    tick_.max_dt_us = dt;
  } else {
    if (dt < tick_.min_dt_us) tick_.min_dt_us = dt;
    if (dt > tick_.max_dt_us) tick_.max_dt_us = dt;
  }
  ++tick_.count;
  if (dt > cfg_.tick_period_us + cfg_.tick_margin_us) {
    ++tick_.overruns;
  }
}

}  // namespace hexa::supervisor
