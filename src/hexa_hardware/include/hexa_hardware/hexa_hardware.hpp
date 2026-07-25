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
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include "energize_sweep.hpp"  // shared/motion_core (build-interface include)
#include "hexa_hardware/board_protocol.hpp"
#include "hexa_hardware/joint_calibration.hpp"
#include "hexa_hardware/leg_order.hpp"
#include "hexa_hardware/transport.hpp"

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
  // Drive the physical relay toward relay_cmd_, forcing it off while a fault is
  // latched, and arm/disarm the energize sweep on the edge. Transport is touched
  // only here / read() / write() — all on the controller-manager thread — so the
  // relay_cmd_ subscription callback merely stores the intent.
  void apply_relay();
  // Emit one SET frame per run, each carrying the calibrated pulse widths of the
  // joints it covers. Throws whatever the transport throws.
  void send_runs(const std::vector<PinRun>& runs);
  // Per-joint runtime state, ordered to match info_.joints.
  struct JointSlot {
    std::string name;
    JointCalibration cal;
    double cmd = 0.0;       // most recent commanded radian
    double pos = 0.0;       // echoed position state
    double vel = 0.0;       // numerical derivative
    double prev_pos = 0.0;
  };

  HardwareConfig config_;
  std::vector<JointSlot> joints_;
  // Sorted-by-pin view used to build consecutive-pin SET batches, plus the same
  // view grouped into legs (ordered by lowest pin) for the energize sweep.
  std::vector<PinEntry> pin_order_;
  std::vector<LegGroup> leg_order_;
  // Frame plans, precomputed in on_init because the wiring fixes them. The
  // steady state uses the whole-view plan (a single frame under the shipped,
  // fully consecutive harness); the sweep ramp walks the per-leg plans instead,
  // which costs one frame per live leg but leaves the rest of the servos limp.
  std::vector<PinRun> full_runs_;
  std::vector<std::vector<PinRun>> leg_runs_;
  std::vector<std::uint16_t> batch_;  // reused SET payload buffer

  // Built once in on_init from cfg.connection / cfg.parser; transport
  // owns the link, protocol holds a reference into it.
  std::unique_ptr<Transport> transport_;
  std::unique_ptr<BoardProtocol> board_;

  // Internal node, used solely to publish aux sensor readings (battery,
  // currents). Spun on a private thread so the executor doesn't need to
  // know about us.
  std::shared_ptr<rclcpp::Node> aux_node_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::BatteryState>> battery_pub_;
  std::thread aux_spin_thread_;
  std::atomic<bool> aux_spin_run_{false};

  // Fault / relay-recovery handshake (real-board over-current path).
  // /hardware/fault publishes the board's latched trip; /hardware/relay_cmd is
  // the locomotion supervisor's arm intent. relay_cmd_ is written by the aux
  // executor thread and read on the CM thread, hence atomic; all transport
  // access (SET RELAY) stays on the CM thread in apply_relay().
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> fault_pub_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> relay_sub_;
  std::atomic<bool> relay_cmd_{false};   // desired arm state from locomotion
  bool relay_on_ = false;                // physical relay currently energised
  bool faulted_ = false;                 // board trip latched (until STATUS clean)

  // Inrush stagger: at the relay OFF->ON edge the legs are driven one at a time
  // (config_.init.sweep_leg_interval_ms apart) instead of all 18 servos in one
  // tick. Re-seeded from the config in on_init.
  hexa::EnergizeSweep sweep_{0.0f};

  int read_tick_ = 0;
};

}  // namespace hexa_hardware
