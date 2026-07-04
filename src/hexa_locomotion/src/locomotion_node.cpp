// Consolidated single-node locomotion controller (the ROS seam around the shared
// control brain, shared/hexa_pipeline).
//
// Mirrors the Pi Pico firmware's single 200 Hz loop, but with ROS seams instead
// of the gamepad / servo hardware:
//   - Input  seam: build a hexa::pipeline::CommandIntent from /cmd_vel (velocity)
//     plus the discrete command topics (/cmd_gait, /gait/initialize, /body/pose,
//     /animation/mode), and call the pipeline's CORE tick directly — bypassing
//     map_joy, so the node is /cmd_vel-native (Nav2 / twist_mux / teleop_twist).
//   - Output seam: publish the pipeline's 18 joint angles (radians) as
//     std_msgs/Float64MultiArray on /joint_group_position_controller/commands,
//     folding the old ik_node + joint_command_bridge. The firmware joint order
//     already equals the controller's joints: list, so no remap.
//   - Clock seam: a ROS timer + get_clock() (sim time under use_sim_time), so the
//     tick stays in lockstep with controller_manager / Gazebo.
//   - Diagnostics: re-publish /gait/state (the one locomotion-chain topic the
//     hexa_display face sink consumes). /cmd_vel, /body/pose, /animation/mode
//     reach the face straight from teleop, unchanged.
//
// Replaces the control_node -> gait_node -> posture_node -> ik_node ->
// joint_command_bridge process chain with one process, one clock, one dt.

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <hexa_interfaces/msg/body_pose.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "gait/engine.hpp"   // hexa::gait::state_value
#include "pipeline.hpp"      // the shared control brain + CommandIntent
#include "pipeline_config_loader.hpp"
#include "servo_out.hpp"     // servo_out::kNumJoints

namespace {

using Twist = geometry_msgs::msg::Twist;
using Empty = std_msgs::msg::Empty;
using StringMsg = std_msgs::msg::String;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;
using BodyPoseMsg = hexa_interfaces::msg::BodyPose;

class LocomotionNode : public rclcpp::Node {
 public:
  LocomotionNode()
      : rclcpp::Node("locomotion_node"),
        pipeline_(hexa::locomotion::load_pipeline_config(*this)) {
    command_topic_ = declare_parameter<std::string>(
        "command_topic", "/joint_group_position_controller/commands");
    publish_diagnostics_ = declare_parameter<bool>("publish_diagnostics", false);

    boot_ = get_clock()->now();

    pub_cmd_ = create_publisher<Float64MultiArray>(command_topic_, 10);
    // /gait/state is the only locomotion-chain topic the face sink consumes; the
    // rest of its inputs (/cmd_vel, /body/pose, /animation/mode) come from teleop.
    pub_state_ = create_publisher<StringMsg>("/gait/state", 10);

    sub_vel_ = create_subscription<Twist>(
        "/cmd_vel", 10, [this](Twist::SharedPtr m) { on_cmd_vel(*m); });
    sub_pose_ = create_subscription<BodyPoseMsg>(
        "/body/pose", 10, [this](BodyPoseMsg::SharedPtr m) { last_pose_ = *m; });
    sub_init_ = create_subscription<Empty>(
        "/gait/initialize", 10,
        [this](Empty::SharedPtr) { init_pending_ = true; });
    // /cmd_gait and /animation/mode are latched by teleop (transient_local,
    // depth 1) so a late-joining node catches the last selection.
    const auto latched = rclcpp::QoS(1).transient_local();
    sub_gait_ = create_subscription<StringMsg>(
        "/cmd_gait", latched, [this](StringMsg::SharedPtr m) {
          gait_name_ = m->data;
          gait_pending_ = true;
        });
    sub_anim_ = create_subscription<StringMsg>(
        "/animation/mode", latched, [this](StringMsg::SharedPtr m) {
          anim_name_ = m->data;
          anim_pending_ = true;
        });

    timer_ = rclcpp::create_timer(
        this, get_clock(),
        std::chrono::microseconds(hexa::pipeline::kTickPeriodUs),
        [this]() { on_tick(); });

    RCLCPP_INFO(get_logger(),
                "hexa_locomotion up: /cmd_vel -> Pipeline -> %s (200 Hz). "
                "Send /gait/initialize to stand, then drive with /cmd_vel.",
                command_topic_.c_str());
  }

 private:
  // Monotonic microseconds since construction (sim time under use_sim_time), so
  // the pipeline advances in lockstep with the controller / Gazebo.
  std::uint64_t now_us() const {
    const std::int64_t ns = (get_clock()->now() - boot_).nanoseconds();
    return ns > 0 ? static_cast<std::uint64_t>(ns / 1000) : 0;
  }

  void on_cmd_vel(const Twist& m) {
    last_twist_ = m;
    last_cmd_vel_us_ = now_us();
    have_cmd_vel_ = true;
  }

  void on_tick() {
    const std::uint64_t t = now_us();

    // Build the command intent from the latest topic state. Velocity is in m/s
    // already (Twist is physical units) — no stick scaling. The discrete
    // commands fire for a single tick on the edge their topic arrived.
    hexa::pipeline::CommandIntent cmd;
    cmd.linear_x = static_cast<float>(last_twist_.linear.x);
    cmd.linear_y = static_cast<float>(last_twist_.linear.y);
    cmd.angular_z = static_cast<float>(last_twist_.angular.z);
    cmd.pose_x = static_cast<float>(last_pose_.x);
    cmd.pose_y = static_cast<float>(last_pose_.y);
    cmd.pose_z = static_cast<float>(last_pose_.z);
    cmd.pose_roll = static_cast<float>(last_pose_.roll);
    cmd.pose_pitch = static_cast<float>(last_pose_.pitch);
    cmd.pose_yaw = static_cast<float>(last_pose_.yaw);
    if (init_pending_) {
      cmd.init_request = true;
      init_pending_ = false;
    }
    if (gait_pending_) {
      cmd.has_gait_select = true;
      cmd.gait_select = gait_name_;  // string_view into gait_name_ (outlives tick)
      gait_pending_ = false;
    }
    if (anim_pending_) {
      cmd.has_animation_name = true;
      cmd.animation_name = anim_name_;
      anim_pending_ = false;
    }

    // /cmd_vel-staleness watchdog: a dead velocity publisher settles the gait
    // (force_zero), mirroring the firmware supervisor's input timeout. An
    // already-zero command is unaffected, so an idle robot is fine.
    const std::uint64_t timeout_us =
        static_cast<std::uint64_t>(hexa::config::kInputTimeoutS * 1e6f);
    const bool fresh = have_cmd_vel_ && (t - last_cmd_vel_us_) < timeout_us;

    hexa::pipeline::TickInput in;
    in.now_us = t;
    in.axes = nullptr;  // core tick ignores axes/buttons (no map_joy on this path)
    in.buttons = 0;
    in.bt_connected = fresh;
    in.last_input_us = have_cmd_vel_ ? last_cmd_vel_us_ : 0;
    in.battery_valid = false;  // no battery telemetry in sim
    in.battery_v = 0.0f;
    in.dt = hexa::pipeline::kDt;

    const hexa::pipeline::TickResult res = pipeline_.tick(cmd, in);
    log_events(res);

    // Firmware joint order == controller joints: order, so publish directly.
    Float64MultiArray out;
    out.data.resize(servo_out::kNumJoints);
    for (int i = 0; i < servo_out::kNumJoints; ++i) {
      out.data[static_cast<std::size_t>(i)] = static_cast<double>(res.theta[i]);
    }
    pub_cmd_->publish(out);

    // Re-publish /gait/state on change for the face sink.
    const std::string state = hexa::gait::state_value(res.engine_state);
    if (state != last_state_) {
      StringMsg sm;
      sm.data = state;
      pub_state_->publish(sm);
      last_state_ = state;
    }
  }

  // Narrate the teleop / safety edges the pipeline resolved (the ROS analogue of
  // the firmware's [teleop]/[safety] logs). Rate-limited edges only.
  void log_events(const hexa::pipeline::TickResult& res) {
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
        RCLCPP_INFO(get_logger(), "gait -> %s (linear_max=%.3f m/s)",
                    res.gait_select.c_str(),
                    static_cast<double>(res.gait_linear_max));
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
  std::string command_topic_;
  bool publish_diagnostics_ = false;

  // Latest topic state.
  Twist last_twist_;
  BodyPoseMsg last_pose_;
  bool have_cmd_vel_ = false;
  std::uint64_t last_cmd_vel_us_ = 0;
  bool init_pending_ = false;
  bool gait_pending_ = false;
  std::string gait_name_;
  bool anim_pending_ = false;
  std::string anim_name_;
  std::string last_state_;

  rclcpp::Time boot_;
  rclcpp::Subscription<Twist>::SharedPtr sub_vel_;
  rclcpp::Subscription<BodyPoseMsg>::SharedPtr sub_pose_;
  rclcpp::Subscription<Empty>::SharedPtr sub_init_;
  rclcpp::Subscription<StringMsg>::SharedPtr sub_gait_;
  rclcpp::Subscription<StringMsg>::SharedPtr sub_anim_;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr pub_cmd_;
  rclcpp::Publisher<StringMsg>::SharedPtr pub_state_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocomotionNode>());
  rclcpp::shutdown();
  return 0;
}
