#include "hexa_hardware/hexa_hardware.hpp"

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <filesystem>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <stdexcept>

#include "hexa_hardware/hardware_factory.hpp"
#include "hexa_hardware/servo2040_protocol.hpp"

namespace hexa_hardware {

namespace {

constexpr auto kLogger = "hexa_hardware";

// Per-GET reply deadline for the aux poll. Generous because it no longer costs
// anything on the control path: a board that never answers just delays the next
// poll by this much on the aux thread. See HexaHardware::poll_aux.
constexpr int kAuxReadTimeoutMs = 50;

// How long the aux thread will block waiting for a subscription callback before
// looping round to re-check the poll deadline and the shutdown flag. Bounds the
// jitter on the aux poll period; also the reason this loop uses spin_once rather
// than spin_some, which returns immediately when idle and would busy-spin a core
// against the 200 Hz control thread.
constexpr auto kAuxSpinTimeout = std::chrono::milliseconds(10);

// True if every joint declares the position command + position/velocity state
// interfaces that the URDF promises.
bool joint_interfaces_ok(const hardware_interface::ComponentInfo& j) {
  if (j.command_interfaces.size() != 1 ||
      j.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
    return false;
  }
  bool has_pos = false, has_vel = false;
  for (const auto& s : j.state_interfaces) {
    if (s.name == hardware_interface::HW_IF_POSITION) has_pos = true;
    if (s.name == hardware_interface::HW_IF_VELOCITY) has_vel = true;
  }
  return has_pos && has_vel;
}

// Wire form of hexa::supervisor::UndervoltStage, which this package cannot
// include (hexa_hardware links no motion_core).
constexpr std::uint8_t kUndervoltWarn = 1;
constexpr std::uint8_t kUndervoltCutoff = 3;

// Events hexa_buzzer knows. Kept in step with the `events:` map in
// hexa_buzzer/config/buzzer.yaml, which is pytest-covered against this list.
// What each one sounds like is that file's business, not ours.
constexpr const char* kTuneUp = "up";
constexpr const char* kTuneFault = "fault";
constexpr const char* kTuneUndervolt = "undervolt";

}  // namespace

hardware_interface::CallbackReturn HexaHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Config lives in hexa_description/config (co-located with geometry/tuning):
  // hardware.yaml (wiring) + servo_calibration.yaml (endpoint pulse widths).
  // The URDF can override either via <param name="config_path"> /
  // <param name="calibration_path"> if a robot needs a different calibration
  // (test rig, second build, …).
  const auto share_config = std::filesystem::path(
      ament_index_cpp::get_package_share_directory("hexa_description")) / "config";
  std::string config_path;
  if (const auto it = info_.hardware_parameters.find("config_path");
      it != info_.hardware_parameters.end() && !it->second.empty()) {
    config_path = it->second;
  } else {
    config_path = (share_config / "hardware.yaml").string();
  }
  std::string calibration_path;
  if (const auto it = info_.hardware_parameters.find("calibration_path");
      it != info_.hardware_parameters.end() && !it->second.empty()) {
    calibration_path = it->second;
  } else {
    calibration_path = (share_config / "servo_calibration.yaml").string();
  }
  try {
    config_ = load_hardware_config(config_path, calibration_path);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger), "Config load failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Optional overrides from <hardware> params.
  if (const auto it = info_.hardware_parameters.find("connection_device");
      it != info_.hardware_parameters.end()) {
    config_.connection.device = it->second;
  }
  if (const auto it = info_.hardware_parameters.find("connection_baud");
      it != info_.hardware_parameters.end()) {
    config_.connection.baud = std::stoi(it->second);
  }

  try {
    transport_ = make_transport(config_);
    board_ = make_board_protocol(config_, *transport_);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger), "Hardware backend init failed: %s",
                 e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joints_.clear();
  joints_.reserve(info_.joints.size());
  for (const auto& j : info_.joints) {
    if (!joint_interfaces_ok(j)) {
      RCLCPP_FATAL(rclcpp::get_logger(kLogger),
                   "Joint '%s' must declare position command + position/velocity state",
                   j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto cal_it = config_.joints.find(j.name);
    if (cal_it == config_.joints.end()) {
      RCLCPP_FATAL(rclcpp::get_logger(kLogger),
                   "Joint '%s' has no entry in servo config", j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    JointSlot slot;
    slot.name = j.name;
    slot.cal = cal_it->second;

    // Seed position from the URDF initial_value so the first read() doesn't
    // jump from 0.
    if (!j.state_interfaces.empty()) {
      for (const auto& s : j.state_interfaces) {
        if (s.name == hardware_interface::HW_IF_POSITION) {
          const auto v_it = s.parameters.find("initial_value");
          if (v_it != s.parameters.end()) {
            try {
              const double v = std::stod(v_it->second);
              slot.pos = slot.prev_pos = slot.cmd = v;
            } catch (...) {}
          }
        }
      }
    }
    joints_.push_back(std::move(slot));
  }

  // Build the consecutive-pin batch view once; pin assignment is static.
  // cal.pin is the 1-based silkscreen number (1..18, enforced in
  // joint_calibration.cpp); the board's wire protocol is 0-based (SERVO1 == index
  // 0, see firmware main.h cmdPins). Convert silkscreen -> logical index once
  // here so the sort / consecutive-run logic and the SET frame's start index all
  // operate in the board's 0-based space. cal.pin >= 1 is guaranteed, so pin - 1
  // never underflows.
  pin_order_.clear();
  pin_order_.reserve(joints_.size());
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    pin_order_.push_back(
        {static_cast<std::uint8_t>(joints_[i].cal.pin - 1), i});
  }
  std::sort(pin_order_.begin(), pin_order_.end(),
            [](const PinEntry& a, const PinEntry& b) { return a.pin < b.pin; });
  for (std::size_t i = 1; i < pin_order_.size(); ++i) {
    if (pin_order_[i].pin == pin_order_[i - 1].pin) {
      RCLCPP_FATAL(rclcpp::get_logger(kLogger),
                   "Duplicate servo pin %u (joints '%s' and '%s')",
                   pin_order_[i].pin,
                   joints_[pin_order_[i - 1].joint_idx].name.c_str(),
                   joints_[pin_order_[i].joint_idx].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // The same view grouped into legs (ordered by lowest pin) — the granularity
  // the energize sweep staggers at.
  std::vector<std::string> joint_names;
  joint_names.reserve(joints_.size());
  for (const auto& j : joints_) joint_names.push_back(j.name);
  try {
    leg_order_ = build_leg_order(joint_names, pin_order_);
    // Frame plans. `full_runs_` is what every tick costs once the sweep is done
    // — under the shipped wiring (board indices 0..17, all consecutive) that is
    // one 18-value SET, exactly what the link carried before the sweep existed.
    // `leg_runs_` is only for the ramp, while some legs must stay limp.
    full_runs_ = build_pin_runs(pin_order_);
    leg_runs_.clear();
    leg_runs_.reserve(leg_order_.size());
    for (const auto& leg : leg_order_) {
      leg_runs_.push_back(build_pin_runs(pin_order_, leg.pin_order_idx));
    }
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger), "Leg grouping failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  batch_.reserve(joints_.size());

  // Steady-state fast path. SETALL carries no start/count header — it is exactly
  // "all 18 servos, board index 0..17" — so it applies only to the flat harness,
  // and only while every clamp stays inside the range it can encode. That second
  // condition is not cosmetic: SETALL clamps a sub-500 µs pulse *up* to 500 and
  // drives it, whereas a SET below 500 means "hold last position" in firmware, so
  // a tighter per-servo override would silently change meaning. Either way the
  // fallback is the 39-byte SET the code has always sent.
  setall_joint_idx_.clear();
  if (!is_flat_pin_map(pin_order_, kSetAllServoCount)) {
    RCLCPP_INFO(rclcpp::get_logger(kLogger),
                "Compact SETALL frame unavailable: wiring is not the flat "
                "board map 0..%zu; using SET frames",
                kSetAllServoCount - 1);
  } else {
    constexpr std::uint16_t kSetAllMinUs = kSetAllPulseBaseUs;
    constexpr std::uint16_t kSetAllMaxUs = kSetAllPulseBaseUs + kSetAllValueMax;
    const auto* out_of_range = [&]() -> const JointSlot* {
      for (const auto& e : pin_order_) {
        const auto& j = joints_[e.joint_idx];
        if (j.cal.min_us < kSetAllMinUs || j.cal.max_us > kSetAllMaxUs) return &j;
      }
      return nullptr;
    }();
    if (out_of_range != nullptr) {
      RCLCPP_INFO(rclcpp::get_logger(kLogger),
                  "Compact SETALL frame unavailable: joint '%s' clamps to "
                  "[%u, %u] µs, outside the encodable [%u, %u]; using SET frames",
                  out_of_range->name.c_str(), out_of_range->cal.min_us,
                  out_of_range->cal.max_us, kSetAllMinUs, kSetAllMaxUs);
    } else {
      setall_joint_idx_.reserve(pin_order_.size());
      for (const auto& e : pin_order_) setall_joint_idx_.push_back(e.joint_idx);
      RCLCPP_INFO(rclcpp::get_logger(kLogger),
                  "Steady-state pose goes out as one %zu-byte SETALL frame "
                  "(fits the board's 32-byte UART RX FIFO)",
                  kSetAllFrameBytes);
    }
  }

  sweep_ = hexa::EnergizeSweep(
      static_cast<float>(config_.init.sweep_leg_interval_ms) * 1e-3f);
  if (sweep_.interval_s() > 0.0f) {
    std::string order;
    for (const auto& leg : leg_order_) {
      if (!order.empty()) order += ", ";
      order += leg.name;
    }
    RCLCPP_INFO(rclcpp::get_logger(kLogger),
                "Energize sweep: %zu legs, %d ms apart, in pin order (%s)",
                leg_order_.size(), config_.init.sweep_leg_interval_ms,
                order.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger(kLogger),
                "Energize sweep disabled — every leg comes up at once");
  }

  if (config_.parser.aux_period_ms > 0) {
    RCLCPP_INFO(rclcpp::get_logger(kLogger),
                "Aux poll (battery + fault STATUS) every %d ms on the aux "
                "thread; the control cycle issues no GETs",
                config_.parser.aux_period_ms);
  } else {
    RCLCPP_INFO(rclcpp::get_logger(kLogger),
                "Aux poll disabled — no battery telemetry, and an over-current "
                "trip will not be reported");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
HexaHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> out;
  out.reserve(joints_.size() * 2);
  for (auto& j : joints_) {
    out.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.pos);
    out.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.vel);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface>
HexaHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> out;
  out.reserve(joints_.size());
  for (auto& j : joints_) {
    out.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.cmd);
  }
  return out;
}

hardware_interface::CallbackReturn HexaHardware::on_configure(
    const rclcpp_lifecycle::State& /*previous*/) {
  try {
    transport_->open();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger), "Transport open failed: %s",
                 e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!aux_node_) {
    aux_node_ = std::make_shared<rclcpp::Node>("hexa_hardware_aux");
    battery_pub_ = aux_node_->create_publisher<sensor_msgs::msg::BatteryState>(
        "~/battery_state", rclcpp::SensorDataQoS());
    // Latched so a late-joining consumer sees the current fault state; the
    // engine (hexa_locomotion) latches FAULT off the first true it sees.
    fault_pub_ = aux_node_->create_publisher<std_msgs::msg::Bool>(
        "/hardware/fault", rclcpp::QoS(1).transient_local());
    // Buzzer requests for hexa_buzzer, which owns the PWM. Latched, and that is
    // load-bearing rather than symmetry: `up` goes out from on_activate(),
    // which can beat the buzzer node's subscription matching, and a volatile
    // reader would drop the one tune that says the robot is ready. The price is
    // that restarting hexa_buzzer alone replays the last tune once.
    buzzer_pub_ = aux_node_->create_publisher<std_msgs::msg::String>(
        "/buzzer/play", rclcpp::QoS(1).transient_local());
    // Relay-arm intent from the locomotion supervisor. The callback only stores
    // the value (atomic); apply_relay() on the CM thread owns the transport.
    relay_sub_ = aux_node_->create_subscription<std_msgs::msg::Bool>(
        "/hardware/relay_cmd", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::Bool::SharedPtr m) {
          relay_cmd_.store(m->data);
        });
    // Undervoltage ladder from the locomotion supervisor. Latched, so a
    // restarted hardware node re-learns a cutoff instead of coming up willing
    // to close the rail on a condemned pack.
    undervolt_sub_ = aux_node_->create_subscription<std_msgs::msg::UInt8>(
        "/hardware/undervoltage", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::UInt8::SharedPtr m) {
          on_undervoltage(m->data);
        });
    aux_spin_run_ = true;
    aux_spin_thread_ = std::thread([this]() {
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(aux_node_);
      const auto period =
          std::chrono::milliseconds(config_.parser.aux_period_ms);
      auto next_poll = std::chrono::steady_clock::now() + period;
      while (aux_spin_run_.load() && rclcpp::ok()) {
        // Blocks until a callback arrives or the timeout expires, so this
        // thread sleeps instead of competing with the control cycle for CPU.
        exec.spin_once(kAuxSpinTimeout);
        if (config_.parser.aux_period_ms <= 0) continue;
        const auto now = std::chrono::steady_clock::now();
        if (now < next_poll) continue;
        // Re-base off `now` rather than accumulating: a poll that overruns its
        // period (a board timing out) must not leave a backlog to catch up on.
        next_poll = now + period;
        poll_aux();
      }
    });
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HexaHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous*/) {
  // Do NOT energise the rail here: activating the component must not power the
  // servos on its own. Start the rail OFF and let apply_relay() close it once
  // the locomotion supervisor asks (relay_cmd_ — true on a live link in any
  // non-FAULT engine state, so normally within a second of launch, while the
  // engine is still FOLDED). The energize sweep then brings the legs up one at
  // a time. Same path serves cold start and over-current recovery.
  relay_on_ = false;
  clear_latch_pending_.store(false);
  // The set_servo_power(false) below clears the board latch, so a fault the aux
  // poll saw before activation is answered here. Retract it on the topic too:
  // /hardware/fault is transient_local, so a stale `true` would outlive this and
  // hold the engine in FAULT — and poll_aux, finding faulted_ already false,
  // never publishes the retraction itself.
  if (faulted_.exchange(false) && fault_pub_) {
    std_msgs::msg::Bool fm;
    fm.data = false;
    fault_pub_->publish(fm);
  }
  sweep_.disarm();
  relay_cmd_.store(false);
  try {
    board_->set_servo_power(false);  // known-off baseline (also clears any latch)
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger), "Servo power off failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Reset commands to current state so the first write() doesn't snap.
  for (auto& j : joints_) {
    j.cmd = j.pos;
    j.prev_pos = j.pos;
    j.vel = 0.0;
  }

  // "The robot is up." Everything above succeeded, so the servo link is open
  // and the stack is live — the last milestone of a cold start that a human
  // standing next to a folded, limp robot cannot otherwise see. Deliberately
  // here rather than on the relay closing: the rail is a separate, later
  // decision (a gamepad Start), and it has the legs moving to announce it.
  request_tune(kTuneUp);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HexaHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous*/) {
  if (transport_ && transport_->is_open()) {
    try {
      board_->set_servo_power(false);
    } catch (const std::exception& e) {
      RCLCPP_WARN(rclcpp::get_logger(kLogger), "Servo power off failed: %s", e.what());
    }
  }
  relay_on_ = false;
  sweep_.disarm();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HexaHardware::on_cleanup(
    const rclcpp_lifecycle::State& /*previous*/) {
  aux_spin_run_ = false;
  if (aux_spin_thread_.joinable()) aux_spin_thread_.join();
  battery_pub_.reset();
  buzzer_pub_.reset();
  aux_node_.reset();
  if (transport_) transport_->close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HexaHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  const double dt = period.seconds();
  for (auto& j : joints_) {
    j.vel = dt > 0.0 ? (j.pos - j.prev_pos) / dt : 0.0;
    j.prev_pos = j.pos;
  }

  // Nothing here blocks. Aux telemetry and the fault STATUS are round trips and
  // live on the aux thread (poll_aux); this reads only what that thread has
  // already latched into atomics. The single frame apply_relay() may emit is a
  // write, which returns as soon as the kernel accepts the bytes.
  apply_relay();
  return hardware_interface::return_type::OK;
}

void HexaHardware::poll_aux() {
  if (!board_ || !transport_ || !transport_->is_open()) return;

  // One GET fetches both voltage and current in engineering units (the board
  // owns the units); publish only on a good read.
  if (battery_pub_) {
    float voltage_v = 0.0f;
    float current_a = 0.0f;
    if (board_->read_battery(voltage_v, current_a, kAuxReadTimeoutMs)) {
      sensor_msgs::msg::BatteryState msg;
      msg.header.stamp = aux_node_->now();
      msg.voltage = voltage_v;
      msg.current = current_a;
      msg.present = true;
      battery_pub_->publish(msg);
    }
  }

  // Poll the board's latched fault register. On a fresh trip: publish the fault
  // so the engine latches FAULT, log the trip current, and ask the CM thread for
  // the SET RELAY 0 that clears the board's sticky latch (and resets its
  // staged-pose flag) — every relay frame stays on that thread, and the request
  // is answered within one 5 ms tick. Once STATUS reads clean again, drop the
  // published fault so the operator's Start (recovery) is honoured.
  bool tripped = false;
  float trip_amps = 0.0f;
  if (!board_->read_status(tripped, trip_amps, kAuxReadTimeoutMs)) return;

  if (tripped && !faulted_.load()) {
    faulted_.store(true);
    clear_latch_pending_.store(true);
    if (fault_pub_) {
      std_msgs::msg::Bool fm;
      fm.data = true;
      fault_pub_->publish(fm);
    }
    RCLCPP_ERROR(rclcpp::get_logger(kLogger),
                 "Over-current trip: %.1f A. Servo rail disabled; "
                 "reposition feet and press Start to recover.",
                 static_cast<double>(trip_amps));
    // A trip drops the whole robot on the spot and the log is the only
    // other place it shows up, so say it out loud. Once per trip edge —
    // the latch keeps this branch from re-firing until STATUS reads clean.
    request_tune(kTuneFault);
  } else if (!tripped && faulted_.load()) {
    faulted_.store(false);
    if (fault_pub_) {
      std_msgs::msg::Bool fm;
      fm.data = false;
      fault_pub_->publish(fm);
    }
    RCLCPP_INFO(rclcpp::get_logger(kLogger),
                "Fault latch cleared; ready to re-initialize.");
  }
}

void HexaHardware::apply_relay() {
  if (!board_) return;
  // Force the rail off while a trip or the undervoltage cutoff is latched;
  // otherwise follow the locomotion supervisor's arm intent. SET RELAY 1 closes
  // the relay with every servo limp — it never moves a servo on its own — so
  // there is nothing to stage first; write() drives the legs afterwards,
  // staggered by the sweep armed here. The undervoltage latch is checked here,
  // not only upstream, because this process drives SET RELAY: it is what makes
  // rung 3 hold against a locomotion restart republishing a stale arm intent.
  try {
    // Answer a trip first and unconditionally: this frame is what clears the
    // board's sticky latch, so STATUS can read clean and the robot can be
    // re-armed. Running it ahead of the ladder also leaves relay_on_ false, so
    // the ladder below (desired is false — faulted_ is set) then does nothing.
    if (clear_latch_pending_.exchange(false)) {
      board_->set_servo_power(false);
      relay_on_ = false;
      sweep_.disarm();
    }
    const bool desired =
        relay_cmd_.load() && !faulted_.load() && !undervolt_cutoff_.load();
    if (desired && !relay_on_) {
      board_->set_servo_power(true);
      relay_on_ = true;
      sweep_.arm();
    } else if (!desired && relay_on_) {
      board_->set_servo_power(false);
      relay_on_ = false;
      sweep_.disarm();
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger(kLogger), *aux_node_->get_clock(),
                         1000, "Relay drive failed: %s", e.what());
  }
}

void HexaHardware::request_tune(const char* tune) {
  // Null before on_activate builds the aux node, and on every deactivated
  // cycle after it. Silence is the correct answer either way.
  if (!buzzer_pub_) return;
  std_msgs::msg::String msg;
  msg.data = tune;
  buzzer_pub_->publish(msg);
}

void HexaHardware::on_undervoltage(std::uint8_t stage) {
  undervolt_stage_.store(stage);
  if (stage >= kUndervoltCutoff) {
    undervolt_cutoff_.store(true);  // sticky; apply_relay() reads it every cycle
  }

  // Announce each rung once, on its rising edge. The topic is latched, so a
  // restart of this node replays the current rung — track the edge here rather
  // than assume it from the topic.
  if (stage <= undervolt_beeped_) return;
  const std::uint8_t previous = undervolt_beeped_;
  undervolt_beeped_ = stage;

  if (stage == kUndervoltWarn) {
    RCLCPP_WARN(rclcpp::get_logger(kLogger),
                "Undervoltage rung 1/3: pack low. Sounding the buzzer; the "
                "robot is still drivable — walk it back and charge it.");
    request_tune(kTuneUndervolt);
    return;
  }

  // Rungs 2 and 3 announce themselves by folding or sitting down. Beep only if
  // rung 1 never got the chance — a pack falling off a cliff, or warning_v
  // disabled — so a collapse is never silent.
  if (previous < kUndervoltWarn) {
    request_tune(kTuneUndervolt);
  }
  RCLCPP_ERROR(rclcpp::get_logger(kLogger),
               "Undervoltage rung %u/3: servo rail cut%s.",
               static_cast<unsigned>(stage),
               stage >= kUndervoltCutoff
                   ? " and latched off — power-cycle the robot after charging"
                   : " once folded");
}

hardware_interface::return_type HexaHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  // The echo is unconditional: /joint_states and the velocity derivative must
  // stay continuous whether or not a joint is being driven this tick.
  for (auto& j : joints_) j.pos = j.cmd;

  if (!transport_ || !transport_->is_open() || leg_order_.empty()) {
    return hardware_interface::return_type::OK;
  }
  // Servo SETs sent while the relay is open are discarded by the board (there is
  // no pre-relay staging), so don't burn the link on them.
  if (!relay_on_) return hardware_interface::return_type::OK;

  // Advance the inrush stagger: only the first `live` legs of the pin-ordered
  // table are driven, the rest stay limp until their turn comes round.
  const int live = sweep_.step(static_cast<float>(period.seconds()));
  const std::size_t n_legs =
      std::min(static_cast<std::size_t>(live), leg_order_.size());

  // Once every leg is live, drive the whole table in one frame — the stagger is
  // an inrush measure, not a wire format, and leaving it in place would split
  // every steady-state tick into one frame per leg for the rest of the session.
  // Same fallback the Pico takes (servo_out::command_legs -> command_all once
  // n_legs >= kNumLegs).
  //
  // The ramp cannot use SETALL: the board stages every channel in the frame, so
  // one SETALL energizes all 18 servos at once and defeats the stagger, whose
  // whole point is that un-commanded channels stay limp until their turn. Per-leg
  // SETs are 9 bytes each and well inside the FIFO anyway; only the whole-table
  // frame was ever at risk of losing its tail.
  try {
    if (n_legs >= leg_order_.size()) {
      if (!setall_joint_idx_.empty()) {
        send_setall();
      } else {
        send_runs(full_runs_);
      }
    } else {
      for (std::size_t leg = 0; leg < n_legs; ++leg) {
        send_runs(leg_runs_[leg]);
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger(kLogger), *aux_node_->get_clock(),
                          1000, "SET write failed: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

void HexaHardware::send_runs(const std::vector<PinRun>& runs) {
  for (const auto& run : runs) {
    batch_.clear();
    for (const std::size_t joint_idx : run.joint_idx) {
      const auto& j = joints_[joint_idx];
      batch_.push_back(j.cal.to_pulse_us(j.cmd));
    }
    board_->send_servo_positions(run.start_pin, batch_);
  }
}

void HexaHardware::send_setall() {
  batch_.clear();
  for (const std::size_t joint_idx : setall_joint_idx_) {
    const auto& j = joints_[joint_idx];
    batch_.push_back(j.cal.to_pulse_us(j.cmd));
  }
  board_->send_all_servo_positions(batch_);
}

}  // namespace hexa_hardware

PLUGINLIB_EXPORT_CLASS(hexa_hardware::HexaHardware,
                       hardware_interface::SystemInterface)
