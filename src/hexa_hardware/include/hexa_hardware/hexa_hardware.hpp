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
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

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
  // latched, and arm/disarm the energize sweep on the edge. Called from read(),
  // so every relay SET stays on the controller-manager thread alongside the
  // servo SETs — the aux thread only ever *asks* for a cut, via the atomics
  // below. Each call is one 5-byte frame, never a round trip.
  void apply_relay();

  // Poll the board's aux registers — battery telemetry, then the latched fault
  // STATUS — and publish what changed. Runs on the aux thread, every
  // config_.parser.aux_period_ms.
  //
  // This is the only place that issues GETs, and GETs are blocking round trips:
  // the host writes a request and waits on the board's reply, which costs
  // whatever the Servo2040's own loop takes to turn around. That is why it lives
  // here and not in read(). On the controller-manager thread it shared a 5 ms
  // budget (200 Hz) with the whole control cycle, so every poll risked a
  // "controller_manager: Overrun detected!" and a bad link cost a 50 ms timeout
  // per GET — twenty missed cycles. Off that thread, a slow or silent board
  // costs stale telemetry and nothing else.
  void poll_aux();

  // Handle an undervoltage rung (aux executor thread): request the buzzer tune
  // on the rung-1 edge, set the sticky cutoff latch at rung 3. Never touches the
  // transport — apply_relay() on the CM thread reads the latch.
  void on_undervoltage(std::uint8_t stage);

  // Ask hexa_buzzer to sound a tune. Not a beep — a request for one: the buzzer
  // hangs off the Pi's hardware PWM and hexa_buzzer is the node that owns it.
  //
  // Best-effort by construction. No buzzer node running, none fitted, or no PWM
  // mounted all mean silence and nothing else, and none of them is visible from
  // here. The buzzer is optional hardware and must never be able to fail a
  // control-path call.
  void request_tune(const char* tune);
  // Emit one SET frame per run, each carrying the calibrated pulse widths of the
  // joints it covers. Throws whatever the transport throws.
  void send_runs(const std::vector<PinRun>& runs);
  // Emit the whole pose as one compact SETALL frame, in board index order.
  // Only callable once setall_joint_idx_ is populated (see on_init). Throws
  // whatever the transport throws.
  void send_setall();
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
  // Steady-state fast path: JointSlot indices in board index order, one per
  // servo, filled in on_init only when the harness and the per-servo clamps let
  // the board's SETALL frame express the pose. Empty means "fall back to
  // full_runs_" — SETALL carries no start/count header, so it is all or nothing.
  std::vector<std::size_t> setall_joint_idx_;
  std::vector<std::uint16_t> batch_;  // reused SET / SETALL payload buffer

  // Built once in on_init from cfg.connection / cfg.parser; transport
  // owns the link, protocol holds a reference into it.
  std::unique_ptr<Transport> transport_;
  std::unique_ptr<BoardProtocol> board_;

  // Internal node, used solely to publish aux sensor readings (battery,
  // currents). Spun on a private thread so the executor doesn't need to
  // know about us. That same thread also runs poll_aux() — every blocking
  // board round trip in this component happens on it, never on the control
  // cycle.
  std::shared_ptr<rclcpp::Node> aux_node_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::BatteryState>> battery_pub_;
  std::thread aux_spin_thread_;
  std::atomic<bool> aux_spin_run_{false};

  // Fault / relay-recovery handshake (real-board over-current path).
  // /hardware/fault publishes the board's latched trip; /hardware/relay_cmd is
  // the locomotion supervisor's arm intent. relay_cmd_ and faulted_ are written
  // by the aux thread and read on the CM thread, hence atomic; all transport
  // access (SET RELAY) stays on the CM thread in apply_relay().
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> fault_pub_;
  // Tune requests to hexa_buzzer (/buzzer/play). Published from both the aux
  // thread (trip, undervoltage) and the CM thread (on_activate); rclcpp
  // publishers are thread-safe, so neither needs a lock.
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> buzzer_pub_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> relay_sub_;
  std::atomic<bool> relay_cmd_{false};   // desired arm state from locomotion
  bool relay_on_ = false;                // physical relay currently energised (CM thread)
  std::atomic<bool> faulted_{false};     // board trip latched (until STATUS clean)
  // Set by poll_aux() on a trip edge, consumed by apply_relay() on the next CM
  // tick. A trip must be answered with SET RELAY 0 *unconditionally*: that frame
  // is what clears the board's sticky latch, and without it STATUS never reads
  // clean again and the robot can never be re-armed. The relay ladder in
  // apply_relay() only sends it on an ON->OFF edge, so a trip observed with
  // relay_on_ already false would otherwise go unanswered.
  std::atomic<bool> clear_latch_pending_{false};

  // Undervoltage rung from the locomotion supervisor (/hardware/undervoltage;
  // 0 none, 1 warn, 2 fold, 3 cutoff). This node owns the two rungs that need
  // it specifically: rung 1 requests the buzzer tune, and
  // rung 3 sets a local rail latch, so the cut survives a locomotion restart
  // republishing a stale relay_cmd_. In-memory — restarting this process (a
  // power cycle, in practice) is the reset.
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::UInt8>> undervolt_sub_;
  std::atomic<std::uint8_t> undervolt_stage_{0};  // written by the aux thread
  std::atomic<bool> undervolt_cutoff_{false};     // sticky: rung 3 seen
  std::uint8_t undervolt_beeped_{0};  // highest rung already announced (aux thread)

  // Inrush stagger: at the relay OFF->ON edge the legs are driven one at a time
  // (config_.init.sweep_leg_interval_ms apart) instead of all 18 servos in one
  // tick. Re-seeded from the config in on_init.
  hexa::EnergizeSweep sweep_{0.0f};
};

}  // namespace hexa_hardware
