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

  // Config path defaults to this package's installed hardware.yaml; the
  // URDF can override via <param name="config_path"> if a robot needs to
  // run against a different calibration (test rig, second build, …).
  std::string config_path;
  if (const auto it = info_.hardware_parameters.find("config_path");
      it != info_.hardware_parameters.end() && !it->second.empty()) {
    config_path = it->second;
  } else {
    config_path = (std::filesystem::path(
                       ament_index_cpp::get_package_share_directory("hexa_hardware")) /
                   "config" / "hardware.yaml")
                      .string();
  }
  try {
    config_ = load_hardware_config(config_path);
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
  pin_order_.clear();
  pin_order_.reserve(joints_.size());
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    pin_order_.push_back({joints_[i].cal.pin, i});
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
  // Energise the servo rail before any motion command lands.
  if (config_.relay_configured) {
    try {
      board_->send_digital(config_.relay_pin, true);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger(kLogger), "Relay on failed: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
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
  if (config_.relay_configured && transport_ && transport_->is_open()) {
    try {
      board_->send_digital(config_.relay_pin, false);
    } catch (const std::exception& e) {
      RCLCPP_WARN(rclcpp::get_logger(kLogger), "Relay off failed: %s", e.what());
    }
  }
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

  // Aux GETs are rate-limited; the SET path owns the bus most of the time.
  if (config_.parser.get_period_ticks > 0 &&
      ++read_tick_ >= config_.parser.get_period_ticks && !config_.aux.empty()) {
    read_tick_ = 0;

    const auto v_it = config_.aux.find("battery_voltage");
    const auto i_it = config_.aux.find("battery_current");
    if (v_it != config_.aux.end() && battery_pub_ && board_) {
      std::vector<std::uint16_t> raw;
      if (board_->read_aux(v_it->second.pin, 1, raw, 50) && raw.size() == 1) {
        sensor_msgs::msg::BatteryState msg;
        msg.header.stamp = aux_node_->now();
        msg.voltage = static_cast<float>(raw[0] * v_it->second.scale);
        msg.current = std::numeric_limits<float>::quiet_NaN();
        if (i_it != config_.aux.end()) {
          std::vector<std::uint16_t> raw_i;
          if (board_->read_aux(i_it->second.pin, 1, raw_i, 50) &&
              raw_i.size() == 1) {
            msg.current = static_cast<float>(raw_i[0] * i_it->second.scale);
          }
        }
        msg.present = true;
        battery_pub_->publish(msg);
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HexaHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (!transport_ || !transport_->is_open() || pin_order_.empty()) {
    return hardware_interface::return_type::OK;
  }
  // Walk the pin-sorted index, accumulating into runs of consecutive pins,
  // then emit one SET frame per run.
  std::vector<std::uint16_t> batch;
  batch.reserve(joints_.size());
  std::size_t i = 0;
  while (i < pin_order_.size()) {
    const std::uint8_t run_start = pin_order_[i].pin;
    batch.clear();
    std::size_t k = i;
    while (k < pin_order_.size() &&
           pin_order_[k].pin == run_start + (k - i)) {
      auto& j = joints_[pin_order_[k].joint_idx];
      const std::uint16_t us = j.cal.to_pulse_us(j.cmd);
      batch.push_back(us);
      j.pos = j.cmd;  // echo: state mirrors the most recent command
      ++k;
    }
    try {
      board_->send_servo_positions(run_start, batch);
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger(kLogger), *aux_node_->get_clock(),
                            1000, "SET write failed: %s", e.what());
      return hardware_interface::return_type::ERROR;
    }
    i = k;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace hexa_hardware

PLUGINLIB_EXPORT_CLASS(hexa_hardware::HexaHardware,
                       hardware_interface::SystemInterface)
