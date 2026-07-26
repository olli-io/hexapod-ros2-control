// Gazebo-in-the-loop firmware seam smoke.
//
// Runs the Pico 2 W firmware's control brain — the exact same
// hexa::pipeline::Pipeline source compiled for the RP2350 — against the
// simulated hexapod, with no hardware.
//
// The control brain itself (velocity -> gait -> posture -> compose/IK) is shared
// verbatim with hexa_locomotion, which is the production sim locomotion path and
// already runs that brain against Gazebo off /cmd_vel. What this bridge uniquely
// exercises are the two firmware-specific seams hexa_locomotion deliberately
// bypasses, giving them an in-Gazebo smoke before the Pico is flashed:
//
//   - Input seam: map_joy. Subscribe /joy (sensor_msgs/Joy) and convert it into
//     the exact raw int16 axes[] / button bitmask the firmware's bt_teleop
//     emits, then call the pipeline's joy overload so map_joy runs identically to
//     on-hardware. (hexa_locomotion instead builds a CommandIntent from /cmd_vel
//     and calls the core tick, skipping map_joy.) /joy is the layout the existing
//     joy_publisher already produces (teleop_joy.yaml base block); the firmware
//     applies the axis signs itself, so we only rescale [-1,1] -> int16.
//   - Config seam: baked constexpr. The build bakes config_generated.hpp via
//     gen_config.py — the same constants the RP2350 compiles — instead of
//     loading geometry.yaml + tuning.yaml at runtime the way hexa_locomotion
//     does.
//
// Everything else is a straight tap of the shared pipeline:
//
//   - Output seam: tap the pipeline at the JointAngles stage (before to_pulse_us
//     — the Pico's servo_out) and publish std_msgs/Float64MultiArray (radians)
//     on /joint_group_position_controller/commands. The firmware's joint order
//     (l_front,l_middle,l_rear,r_front,r_middle,r_rear x coxa,femur,tibia) is
//     identical to the controller's joints: list (ros2_controllers.yaml), so the
//     array publishes directly with no remap.
//   - Clock seam: the node clock (sim time under use_sim_time) stands in for
//     time_us_64(), keeping the tick in lockstep with Gazebo (see on_tick).
//
// Launch it alongside `ros2 launch hexa_simulation sim.launch.py`: the firmware
// brain, driven through its own joy + baked-config seams, walks the Gazebo
// hexapod.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "bt_teleop.hpp"     // axis / button indices + int16 extremes
#include "gait/engine.hpp"   // state_value for logging
#include "pipeline.hpp"      // the shared firmware control brain
#include "servo_out.hpp"     // kNumJoints

namespace {

using Joy = sensor_msgs::msg::Joy;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;

// A neutral gamepad snapshot in the firmware's raw int16 layout: sticks/dpad
// centered, analog triggers released at +max, no buttons. Matches bt_teleop's
// idle state, so a missing / empty /joy reads as "no input" rather than garbage.
void fill_neutral(std::array<std::int16_t, bt_teleop::kNumAxes>& axes) {
  axes.fill(0);
  axes[bt_teleop::kL2] = bt_teleop::kAxisMax;
  axes[bt_teleop::kR2] = bt_teleop::kAxisMax;
}

// Rescale a /joy float in [-1, 1] to the firmware's int16 axis convention
// (map_joy divides by 32767 internally). Clamped so a driver reporting slightly
// past the rail can't overflow.
std::int16_t to_axis_i16(float v) {
  const float scaled = std::lround(v * static_cast<float>(bt_teleop::kAxisMax));
  const float clamped =
      std::min(std::max(scaled, static_cast<float>(bt_teleop::kAxisMin)),
               static_cast<float>(bt_teleop::kAxisMax));
  return static_cast<std::int16_t>(clamped);
}

class FirmwareBridgeNode : public rclcpp::Node {
 public:
  FirmwareBridgeNode() : rclcpp::Node("firmware_bridge") {
    // Publish the same command interface gz_ros2_control exposes (see
    // hexa_simulation/config/ros2_controllers.yaml).
    const std::string command_topic = declare_parameter<std::string>(
        "command_topic", "/joint_group_position_controller/commands");
    const std::string joy_topic =
        declare_parameter<std::string>("joy_topic", "/joy");

    fill_neutral(axes_);

    pub_ = create_publisher<Float64MultiArray>(command_topic, 10);
    sub_ = create_subscription<Joy>(
        joy_topic, 10,
        [this](Joy::SharedPtr msg) { on_joy(*msg); });

    // Control tick on the SIM clock (use_sim_time). The Pico paces off
    // time_us_64() — real wall time — but Gazebo runs on sim time and its
    // real-time factor drifts from 1.0, so a wall-clock timer here would deliver
    // the command stream mispaced against the physics: global body motions (the
    // stand/sit body-Z ramp, the idle breathing bob) would stutter across all
    // legs at once. Driving the timer and now_us() off get_clock() keeps the
    // pipeline in lockstep with the controller_manager (also use_sim_time), just
    // as the ROS gait/ik nodes do (create_timer + get_clock()).
    boot_ = get_clock()->now();
    timer_ = rclcpp::create_timer(
        this, get_clock(),
        std::chrono::microseconds(hexa::pipeline::kTickPeriodUs),
        [this]() { on_tick(); });

    RCLCPP_INFO(get_logger(),
                "firmware bridge up: /joy -> Pipeline -> %s (200 Hz). Press the "
                "init (start) button to stand, then drive.",
                command_topic.c_str());
  }

 private:
  // Monotonic microseconds since construction — the bridge's clock seam. Reads
  // the node clock (sim time under use_sim_time), so the pipeline advances in
  // lockstep with Gazebo instead of drifting against it on wall time.
  std::uint64_t now_us() const {
    const std::int64_t ns = (get_clock()->now() - boot_).nanoseconds();
    return ns > 0 ? static_cast<std::uint64_t>(ns / 1000) : 0;
  }

  void on_joy(const Joy& msg) {
    std::array<std::int16_t, bt_teleop::kNumAxes> axes;
    fill_neutral(axes);
    const std::size_t na =
        std::min<std::size_t>(msg.axes.size(), bt_teleop::kNumAxes);
    for (std::size_t i = 0; i < na; ++i) {
      axes[i] = to_axis_i16(msg.axes[i]);
    }
    std::uint32_t buttons = 0;
    const std::size_t nb =
        std::min<std::size_t>(msg.buttons.size(), bt_teleop::kNumButtons);
    for (std::size_t i = 0; i < nb; ++i) {
      if (msg.buttons[i] != 0) {
        buttons |= (1u << i);
      }
    }
    axes_ = axes;
    buttons_ = buttons;
    last_joy_us_ = now_us();
    have_joy_ = true;
  }

  void on_tick() {
    const std::uint64_t t = now_us();

    // Link freshness for the firmware watchdog: "connected" once /joy has
    // arrived and stayed fresh within the input-timeout window. A stalled
    // publisher (or none) reads as a lost link -> the pipeline force-zeroes the
    // command and settles, exactly as on-hardware.
    const std::uint64_t timeout_us = static_cast<std::uint64_t>(
        hexa::config::kInputTimeoutS * 1e6f);
    const bool fresh =
        have_joy_ && (t - last_joy_us_) < timeout_us;

    hexa::pipeline::TickInput in;
    in.now_us = t;
    in.axes = axes_.data();
    in.buttons = buttons_;
    in.bt_connected = fresh;
    in.last_input_us = have_joy_ ? last_joy_us_ : 0;
    in.battery_valid = false;  // no battery telemetry in sim
    in.battery_v = 0.0f;
    in.dt = hexa::pipeline::kDt;

    const hexa::pipeline::TickResult res = pipeline_.tick(in);
    log_events(res);

    // Firmware joint order == controller joints: order, so publish directly.
    Float64MultiArray out;
    out.data.resize(servo_out::kNumJoints);
    for (int i = 0; i < servo_out::kNumJoints; ++i) {
      out.data[static_cast<std::size_t>(i)] = static_cast<double>(res.theta[i]);
    }
    pub_->publish(out);
  }

  // Narrate the teleop / safety events the pipeline resolved (the sim analogue
  // of main.cpp's [teleop] / [safety] logs). Rate-limited edges only.
  void log_events(const hexa::pipeline::TickResult& res) {
    if (res.mode_changed) {
      RCLCPP_INFO(get_logger(), "mode changed");
    }
    if (res.init_request) {
      using IA = hexa::pipeline::InitAction;
      if (res.init_action == IA::kInitialized) {
        RCLCPP_INFO(get_logger(), "init: FOLDED -> INITIALIZE");
      } else if (res.init_action == IA::kFoldRequested) {
        RCLCPP_INFO(get_logger(), "init: fold requested (state=%s)",
                    hexa::gait::state_value(res.engine_state).c_str());
      }
    }
    if (res.has_gait_select) {
      if (res.gait_accepted) {
        RCLCPP_INFO(get_logger(),
                    "gait -> %s (linear_max=%.3f m/s, angular_max=%.3f rad/s)",
                    res.gait_select.c_str(),
                    static_cast<double>(res.gait_linear_max),
                    static_cast<double>(res.gait_angular_z_max));
      } else {
        RCLCPP_INFO(get_logger(), "gait -> %s dropped (state=%s)",
                    res.gait_select.c_str(),
                    hexa::gait::state_value(res.engine_state).c_str());
      }
    }
    if (res.has_animation_name && !res.animation_accepted) {
      RCLCPP_WARN(get_logger(), "animation=%s dropped (unknown)",
                  res.animation_name.c_str());
    }
    if (res.unreachable > 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "%d/6 legs unreachable (holding last-good)",
                           res.unreachable);
    }
  }

  hexa::pipeline::Pipeline pipeline_;

  std::array<std::int16_t, bt_teleop::kNumAxes> axes_{};
  std::uint32_t buttons_ = 0;
  bool have_joy_ = false;
  std::uint64_t last_joy_us_ = 0;

  rclcpp::Time boot_;
  rclcpp::Subscription<Joy>::SharedPtr sub_;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FirmwareBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
