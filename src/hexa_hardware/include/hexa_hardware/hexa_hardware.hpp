#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include "hexa_hardware/joint_calibration.hpp"
#include "hexa_hardware/servo_bus.hpp"

namespace hexa_hardware {

class HexaHardware : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HexaHardware)

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params) override;
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous) override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous) override;
  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // Per-joint runtime state, ordered to match info_.joints.
  struct JointSlot {
    std::string name;
    JointCalibration cal;
    double cmd = 0.0;       // most recent commanded radian
    double pos = 0.0;       // echoed position state
    double vel = 0.0;       // numerical derivative
    double prev_pos = 0.0;
  };
  // Sorted-by-pin view used to build consecutive-pin SET batches.
  struct PinEntry {
    std::uint8_t pin;
    std::size_t joint_idx;
  };

  HardwareConfig config_;
  std::vector<JointSlot> joints_;
  std::vector<PinEntry> pin_order_;

  ServoBus bus_;

  // Internal node, used solely to publish aux sensor readings (battery,
  // currents). Spun on a private thread so the executor doesn't need to
  // know about us.
  std::shared_ptr<rclcpp::Node> aux_node_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::BatteryState>> battery_pub_;
  std::thread aux_spin_thread_;
  std::atomic<bool> aux_spin_run_{false};

  int read_tick_ = 0;
};

}  // namespace hexa_hardware
