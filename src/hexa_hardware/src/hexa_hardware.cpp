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

namespace hexa_hardware {

namespace {

constexpr auto kLogger = "hexa_hardware";

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
    // Relay-arm intent from the locomotion supervisor. The callback only stores
    // the value (atomic); apply_relay() on the CM thread owns the transport.
    relay_sub_ = aux_node_->create_subscription<std_msgs::msg::Bool>(
        "/hardware/relay_cmd", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::Bool::SharedPtr m) {
          relay_cmd_.store(m->data);
        });
    aux_spin_run_ = true;
    aux_spin_thread_ = std::thread([this]() {
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(aux_node_);
      while (aux_spin_run_.load() && rclcpp::ok()) {
        exec.spin_some(std::chrono::milliseconds(50));
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
  faulted_ = false;
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
  read_tick_ = 0;
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

  // Battery GETs are rate-limited; the SET path owns the bus most of the time.
  // One GET fetches both voltage and current in engineering units (the board
  // owns the units); publish only on a good read.
  if (config_.parser.get_period_ticks > 0 &&
      ++read_tick_ >= config_.parser.get_period_ticks) {
    read_tick_ = 0;

    if (battery_pub_ && board_) {
      float voltage_v = 0.0f;
      float current_a = 0.0f;
      if (board_->read_battery(voltage_v, current_a, 50)) {
        sensor_msgs::msg::BatteryState msg;
        msg.header.stamp = aux_node_->now();
        msg.voltage = voltage_v;
        msg.current = current_a;
        msg.present = true;
        battery_pub_->publish(msg);
      }
    }

    // Poll the board's latched fault register. On a fresh trip: clear the sticky
    // latch (SET RELAY 0, which also resets the board's staged-pose flag),
    // publish the fault so the engine latches FAULT, and log the trip current.
    // Once STATUS reads clean again, drop the published fault so the operator's
    // Start (recovery) is honoured.
    if (board_) {
      bool tripped = false;
      float trip_amps = 0.0f;
      if (board_->read_status(tripped, trip_amps, 50)) {
        if (tripped && !faulted_) {
          faulted_ = true;
          try {
            board_->set_servo_power(false);
          } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger(kLogger),
                        "Latch clear (SET RELAY 0) failed: %s", e.what());
          }
          relay_on_ = false;
          sweep_.disarm();
          if (fault_pub_) {
            std_msgs::msg::Bool fm;
            fm.data = true;
            fault_pub_->publish(fm);
          }
          RCLCPP_ERROR(rclcpp::get_logger(kLogger),
                       "Over-current trip: %.1f A. Servo rail disabled; "
                       "reposition feet and press Start to recover.",
                       static_cast<double>(trip_amps));
        } else if (!tripped && faulted_) {
          faulted_ = false;
          if (fault_pub_) {
            std_msgs::msg::Bool fm;
            fm.data = false;
            fault_pub_->publish(fm);
          }
          RCLCPP_INFO(rclcpp::get_logger(kLogger),
                      "Fault latch cleared; ready to re-initialize.");
        }
      }
    }
  }

  apply_relay();
  return hardware_interface::return_type::OK;
}

void HexaHardware::apply_relay() {
  if (!board_) return;
  // Force the rail off while a trip is latched; otherwise follow the locomotion
  // supervisor's arm intent. SET RELAY 1 closes the relay with every servo limp
  // — it never moves a servo on its own — so there is nothing to stage first;
  // write() drives the legs afterwards, staggered by the sweep armed here.
  const bool desired = relay_cmd_.load() && !faulted_;
  try {
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

  // Once every leg is live, drive the whole table as one frame plan — the
  // stagger is an inrush measure, not a wire format, and leaving it in place
  // would split every steady-state tick into one frame per leg for the rest of
  // the session. The last leg in pin order is the one that pays for that, and
  // it is the one that goes missing. Same fallback the Pico takes
  // (servo_out::command_legs -> command_all once n_legs >= kNumLegs).
  try {
    if (n_legs >= leg_order_.size()) {
      send_runs(full_runs_);
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

}  // namespace hexa_hardware

PLUGINLIB_EXPORT_CLASS(hexa_hardware::HexaHardware,
                       hardware_interface::SystemInterface)
