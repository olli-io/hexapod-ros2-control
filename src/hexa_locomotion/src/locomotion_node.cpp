// ROS seam around the shared control brain (shared/motion_core): one 200 Hz
// tick from /cmd_vel + the discrete command topics to joint commands. Calls the
// core tick directly, bypassing map_joy. See README.md.

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <hexa_interfaces/msg/body_pose.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "gait/engine.hpp"
#include "pipeline.hpp"
#include "pipeline_config_loader.hpp"
#include "servo_out.hpp"

namespace {

using Twist = geometry_msgs::msg::Twist;
using Empty = std_msgs::msg::Empty;
using BoolMsg = std_msgs::msg::Bool;
using StringMsg = std_msgs::msg::String;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;
using UInt8Msg = std_msgs::msg::UInt8;
using BatteryState = sensor_msgs::msg::BatteryState;
using BodyPoseMsg = hexa_interfaces::msg::BodyPose;
using Trigger = std_srvs::srv::Trigger;

class LocomotionNode : public rclcpp::Node {
 public:
  LocomotionNode()
      : rclcpp::Node("locomotion_node"),
        pipeline_(std::make_unique<hexa::pipeline::Pipeline>(
            hexa::locomotion::load_pipeline_config(*this))) {
    command_topic_ = declare_parameter<std::string>(
        "command_topic", "/joint_group_position_controller/commands");
    publish_diagnostics_ = declare_parameter<bool>("publish_diagnostics", false);

    boot_ = get_clock()->now();

    pub_cmd_ = create_publisher<Float64MultiArray>(command_topic_, 10);
    // Report topics publish on change only, so they are latched for a subscriber
    // that joins after the boot value went out.
    const auto latched = rclcpp::QoS(1).transient_local();
    pub_state_ = create_publisher<StringMsg>("/gait/state", latched);
    // The leg set / preset the engine has APPLIED, not the one requested:
    // /cmd_gait and /cmd_preset are latched, so a refused request stays on them.
    pub_leg_set_ = create_publisher<StringMsg>("/gait/leg_set", latched);
    pub_preset_ = create_publisher<StringMsg>("/gait/preset", latched);
    pub_relay_ = create_publisher<BoolMsg>("/hardware/relay_cmd", latched);
    // UndervoltStage as uint8; escalate-only, never counts down.
    pub_undervolt_ = create_publisher<UInt8Msg>("/hardware/undervoltage", latched);

    sub_vel_ = create_subscription<Twist>(
        "/cmd_vel", 10, [this](Twist::SharedPtr m) { on_cmd_vel(*m); });
    sub_pose_ = create_subscription<BodyPoseMsg>(
        "/body/pose", 10, [this](BodyPoseMsg::SharedPtr m) { last_pose_ = *m; });
    sub_init_ = create_subscription<Empty>(
        "/gait/initialize", 10,
        [this](Empty::SharedPtr) { init_pending_ = true; });
    // Level, not edge: stays true while the board is tripped. The engine latches.
    sub_fault_ = create_subscription<BoolMsg>(
        "/hardware/fault", latched,
        [this](BoolMsg::SharedPtr m) { fault_level_ = m->data; });
    battery_topic_ = declare_parameter<std::string>(
        "battery_topic", "/hexa_hardware_aux/battery_state");
    sub_battery_ = create_subscription<BatteryState>(
        battery_topic_, rclcpp::SensorDataQoS(),
        [this](BatteryState::SharedPtr m) {
          battery_v_ = m->voltage;
          battery_unconsumed_ = true;
        });
    sub_gait_ = create_subscription<StringMsg>(
        "/cmd_gait", latched, [this](StringMsg::SharedPtr m) {
          gait_name_ = m->data;
          gait_pending_ = true;
        });
    // Re-asserted every tick, not on the edge: request_preset is idempotent, and
    // this is what brings a restarted node back on the operator's preset.
    sub_preset_ = create_subscription<StringMsg>(
        "/cmd_preset", latched, [this](StringMsg::SharedPtr m) {
          preset_name_ = m->data;
          have_preset_ = true;
        });
    sub_anim_ = create_subscription<StringMsg>(
        "/animation/mode", latched, [this](StringMsg::SharedPtr m) {
          anim_name_ = m->data;
          anim_pending_ = true;
        });

    // Same single-threaded executor as on_tick(), so the swap needs no lock.
    srv_reload_ = create_service<Trigger>(
        "~/reload_config",
        [this](const std::shared_ptr<Trigger::Request> req,
               std::shared_ptr<Trigger::Response> res) {
          on_reload_config(*req, *res);
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
    // Preset before gait: a gait of the other leg set is refused, and both
    // topics land in the same tick when the operator picks a preset.
    if (have_preset_) {
      cmd.has_preset_select = true;
      cmd.preset_select = preset_name_;
    }
    if (gait_pending_) {
      cmd.has_gait_select = true;
      cmd.gait_select = gait_name_;
      gait_pending_ = false;
    }
    if (anim_pending_) {
      cmd.has_animation_name = true;
      cmd.animation_name = anim_name_;
      anim_pending_ = false;
    }

    // A stale /cmd_vel publisher settles the gait via the supervisor's input timeout.
    const std::uint64_t timeout_us =
        static_cast<std::uint64_t>(hexa::config::kInputTimeoutS * 1e6f);
    const bool fresh = have_cmd_vel_ && (t - last_cmd_vel_us_) < timeout_us;

    hexa::pipeline::TickInput in;
    in.now_us = t;
    in.axes = nullptr;
    in.buttons = 0;
    in.bt_connected = fresh;
    in.last_input_us = have_cmd_vel_ ? last_cmd_vel_us_ : 0;
    // Consume-once: re-presenting a ~10 Hz sample at 200 Hz would run the
    // undervoltage debounce against stale data, and keep counting after the
    // publisher died.
    in.battery_valid = battery_unconsumed_;
    in.battery_v = battery_unconsumed_ ? battery_v_ : 0.0f;
    battery_unconsumed_ = false;
    in.hardware_fault = fault_level_;
    in.dt = hexa::pipeline::kDt;

    const hexa::pipeline::TickResult res = pipeline_->tick(cmd, in);
    log_events(res);

    // Firmware joint order equals the controller's joints: list, so no remap.
    Float64MultiArray out;
    out.data.resize(servo_out::kNumJoints);
    for (int i = 0; i < servo_out::kNumJoints; ++i) {
      out.data[static_cast<std::size_t>(i)] = static_cast<double>(res.theta[i]);
    }
    pub_cmd_->publish(out);

    const std::string state = hexa::gait::state_value(res.engine_state);
    if (state != last_state_) {
      StringMsg sm;
      sm.data = state;
      pub_state_->publish(sm);
      if (state == "fault") {
        RCLCPP_WARN(get_logger(),
                    "FAULT latched (over-current) — servo rail disabled. "
                    "Reposition the feet to the folded pose, then press Start.");
      }
      last_state_ = state;
    }

    if (res.preset != last_preset_) {
      StringMsg pm;
      pm.data = res.preset;
      pub_preset_->publish(pm);
      RCLCPP_INFO(get_logger(), "preset -> %s", res.preset.c_str());
      last_preset_ = res.preset;
    }
    const std::string leg_set = hexa::gait::leg_set_value(res.leg_set);
    if (leg_set != last_leg_set_) {
      StringMsg lm;
      lm.data = leg_set;
      pub_leg_set_->publish(lm);
      RCLCPP_INFO(get_logger(), "leg set -> %s", leg_set.c_str());
      last_leg_set_ = leg_set;
    }

    if (!have_relay_ || res.relay_energized != last_relay_) {
      BoolMsg rm;
      rm.data = res.relay_energized;
      pub_relay_->publish(rm);
      last_relay_ = res.relay_energized;
      have_relay_ = true;
    }

    publish_undervolt_stage(res);
  }

  // Floored at the last stage sent, independently of the supervisor's latch: a
  // reload swaps in a pipeline that restarts at kNone, and this node outlives it.
  void publish_undervolt_stage(const hexa::pipeline::TickResult& res) {
    using Stage = hexa::supervisor::UndervoltStage;
    const Stage stage = res.decision.undervolt_stage;
    if (have_undervolt_ && stage <= last_undervolt_) {
      return;
    }
    UInt8Msg um;
    um.data = static_cast<std::uint8_t>(stage);
    pub_undervolt_->publish(um);
    last_undervolt_ = stage;
    have_undervolt_ = true;

    switch (stage) {
      case Stage::kNone:
        break;
      case Stage::kWarn:
        RCLCPP_WARN(get_logger(),
                    "UNDERVOLTAGE rung 1/3 (%.2f V): pack low — buzzer sounding. "
                    "Still drivable; walk the robot back and charge it.",
                    static_cast<double>(battery_v_));
        break;
      case Stage::kFold:
        RCLCPP_ERROR(get_logger(),
                     "UNDERVOLTAGE rung 2/3 (%.2f V): folding and cutting the "
                     "servo rail. Command zeroed%s.",
                     static_cast<double>(battery_v_),
                     res.undervolt_fold_requested
                         ? ""
                         : " — engine REFUSED the fold, rung 3 will cut anyway");
        break;
      case Stage::kCutoff:
        RCLCPP_ERROR(get_logger(),
                     "UNDERVOLTAGE rung 3/3 (%.2f V): servo rail cut now and "
                     "latched off. Power-cycle the robot after charging.",
                     static_cast<double>(battery_v_));
        break;
    }
  }

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
    if (res.gait_blocked_by_posture) {
      RCLCPP_WARN(get_logger(),
                  "leg-set change dropped — the body pose never returned to "
                  "neutral. Centre the posture sticks and ask again.");
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

  // Unlike startup, a bad YAML keeps the current pipeline rather than falling
  // back to baked: a mid-session swap to stale config must never be silent.
  void on_reload_config(const Trigger::Request&, Trigger::Response& res) {
    // A fresh supervisor starts at kNone and would re-arm the rail on a
    // condemned pack; the cutoff latch must survive until a power cycle.
    if (last_undervolt_ >= hexa::supervisor::UndervoltStage::kFold) {
      res.success = false;
      res.message =
          "config reload refused: undervoltage cutoff latched (rung " +
          std::to_string(static_cast<int>(last_undervolt_)) +
          "/3). Charge the pack and power-cycle the robot.";
      RCLCPP_ERROR(get_logger(), "%s", res.message.c_str());
      return;
    }
    const std::string share =
        ament_index_cpp::get_package_share_directory("hexa_description");
    const std::string geometry_path = share + "/config/geometry.yaml";
    const std::string tuning_path = share + "/config/tuning.yaml";
    try {
      auto cfg = hexa::locomotion::load_pipeline_config_from_yaml(geometry_path,
                                                                  tuning_path);
      pipeline_ = std::make_unique<hexa::pipeline::Pipeline>(cfg);
      last_state_.clear();
      last_leg_set_.clear();
      last_preset_.clear();
      have_relay_ = false;
      res.success = true;
      res.message = "reloaded config (gait=" + cfg.default_gait +
                    "); pipeline reset to FOLDED — send /gait/initialize to stand";
      RCLCPP_INFO(get_logger(), "%s", res.message.c_str());
    } catch (const std::exception& ex) {
      res.success = false;
      res.message = std::string("config reload failed (") + ex.what() +
                    "); keeping current pipeline";
      RCLCPP_ERROR(get_logger(), "%s", res.message.c_str());
    }
  }

  std::unique_ptr<hexa::pipeline::Pipeline> pipeline_;
  std::string command_topic_;
  bool publish_diagnostics_ = false;

  Twist last_twist_;
  BodyPoseMsg last_pose_;
  bool have_cmd_vel_ = false;
  std::uint64_t last_cmd_vel_us_ = 0;
  bool init_pending_ = false;
  bool gait_pending_ = false;
  std::string gait_name_;
  bool have_preset_ = false;
  std::string preset_name_;
  bool anim_pending_ = false;
  std::string anim_name_;
  std::string last_state_;
  std::string last_leg_set_;
  std::string last_preset_;
  bool fault_level_ = false;
  bool last_relay_ = false;
  bool have_relay_ = false;
  std::string battery_topic_;
  float battery_v_ = 0.0f;
  bool battery_unconsumed_ = false;
  hexa::supervisor::UndervoltStage last_undervolt_ =
      hexa::supervisor::UndervoltStage::kNone;
  bool have_undervolt_ = false;

  rclcpp::Time boot_;
  rclcpp::Subscription<Twist>::SharedPtr sub_vel_;
  rclcpp::Subscription<BodyPoseMsg>::SharedPtr sub_pose_;
  rclcpp::Subscription<Empty>::SharedPtr sub_init_;
  rclcpp::Subscription<BoolMsg>::SharedPtr sub_fault_;
  rclcpp::Subscription<BatteryState>::SharedPtr sub_battery_;
  rclcpp::Subscription<StringMsg>::SharedPtr sub_gait_;
  rclcpp::Subscription<StringMsg>::SharedPtr sub_preset_;
  rclcpp::Subscription<StringMsg>::SharedPtr sub_anim_;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr pub_cmd_;
  rclcpp::Publisher<StringMsg>::SharedPtr pub_state_;
  rclcpp::Publisher<StringMsg>::SharedPtr pub_leg_set_;
  rclcpp::Publisher<StringMsg>::SharedPtr pub_preset_;
  rclcpp::Publisher<BoolMsg>::SharedPtr pub_relay_;
  rclcpp::Publisher<UInt8Msg>::SharedPtr pub_undervolt_;
  rclcpp::Service<Trigger>::SharedPtr srv_reload_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocomotionNode>());
  rclcpp::shutdown();
  return 0;
}
