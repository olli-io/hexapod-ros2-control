// Gait engine ROS node. Port of gait_node.py.
//
// Subscribes to /gait/params (last-write-wins), /gait/initialize, and
// /body/pose (height axis only); publishes /legs/targets and /gait/state at
// 50 Hz. Builds an Engine at init from hexa_description's YAML (single source of
// truth for body geometry and standing pose) and its own ROS parameters
// (engine-internal knobs; defaults mirror config/gait.yaml, which the launch
// files pass as the param source). All gait logic lives in the pure Engine;
// this file owns only the ROS plumbing.

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/point.hpp>
#include <hexa_interfaces/msg/body_pose.hpp>
#include <hexa_interfaces/msg/gait_params.hpp>
#include <hexa_interfaces/msg/leg_state.hpp>
#include <hexa_interfaces/msg/leg_targets.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include "hexa_gait_cpp/engine.hpp"
#include "hexa_gait_cpp/gaits/registry.hpp"
#include "hexa_gait_cpp/kinematics.hpp"
#include "hexa_gait_cpp/types.hpp"

namespace {

constexpr double kPublishRateHz = 50.0;

double load_coxa_to_bottom(const std::string& geometry_path) {
  const YAML::Node raw = YAML::LoadFile(geometry_path);
  return raw["body"]["coxa_to_bottom"].as<double>();
}

class GaitNode : public rclcpp::Node {
 public:
  GaitNode() : rclcpp::Node("gait_node") {
    const std::string desc_share =
        ament_index_cpp::get_package_share_directory("hexa_description") +
        "/config";
    const std::string geometry = desc_share + "/geometry.yaml";
    const std::string standing = desc_share + "/standing_pose.yaml";

    std::string default_gait;
    cfg_ = read_engine_config(default_gait);

    auto nominal = hexa_gait::nominal_stance_from_yaml(geometry, standing);
    auto initial = hexa_gait::initial_stance_from_yaml(geometry);
    const double coxa_to_bottom = load_coxa_to_bottom(geometry);
    auto leg_contexts = hexa_gait::build_leg_contexts(geometry, standing);
    auto leg_specs = hexa_gait::kin::load_leg_specs(geometry);
    auto reseat_geometry =
        hexa_gait::reseat_geometry_from_yaml(geometry, standing);

    engine_ = std::make_unique<hexa_gait::Engine>(
        cfg_, hexa_gait::strategies().at(default_gait)(), default_gait,
        std::move(nominal), std::move(initial), coxa_to_bottom,
        std::move(leg_contexts), std::make_optional(std::move(leg_specs)),
        std::make_optional(reseat_geometry));

    gait_name_ = default_gait;

    sub_params_ = create_subscription<hexa_interfaces::msg::GaitParams>(
        "/gait/params", 10,
        [this](hexa_interfaces::msg::GaitParams::SharedPtr msg) {
          on_params(*msg);
        });
    sub_init_ = create_subscription<std_msgs::msg::Empty>(
        "/gait/initialize", 10,
        [this](std_msgs::msg::Empty::SharedPtr msg) { on_init(*msg); });
    sub_body_pose_ = create_subscription<hexa_interfaces::msg::BodyPose>(
        "/body/pose", 10,
        [this](hexa_interfaces::msg::BodyPose::SharedPtr msg) {
          engine_->set_target_height(msg->z);
        });

    pub_targets_ =
        create_publisher<hexa_interfaces::msg::LegTargets>("/legs/targets", 10);
    pub_state_ = create_publisher<std_msgs::msg::String>("/gait/state", 10);

    // Wall timer at the publish rate; dt is computed from the ROS clock inside
    // tick() so the engine integrates in sim time when use_sim_time is set.
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / kPublishRateHz),
        [this]() { tick(); });

    RCLCPP_INFO(get_logger(),
                "gait_node up: strategy=%s, stride_length=%.3f m, "
                "min_swing_time=%.2f s, max_swing_time=%.2f s, "
                "step_height=%.3f m",
                default_gait.c_str(), cfg_.stride_length, cfg_.min_swing_time,
                cfg_.max_swing_time, cfg_.step_height);
  }

 private:
  // Declare and read the engine knobs from this node's ros params. Defaults
  // mirror config/gait.yaml; the launch files pass that file (plus the "gait"
  // tuning-overlay block, layered last) as the param source, so a bare
  // `ros2 run` still boots on these defaults. The nested initialize:/reseat:
  // blocks in the YAML surface as dotted parameter names.
  hexa_gait::EngineConfig read_engine_config(std::string& default_gait_out) {
    hexa_gait::EngineConfig cfg;
    cfg.stride_length = declare_parameter<double>("stride_length", 0.1);
    cfg.min_swing_time = declare_parameter<double>("min_swing_time", 0.30);
    cfg.max_swing_time = declare_parameter<double>("max_swing_time", 0.4);
    cfg.step_height = declare_parameter<double>("step_height", 0.08);
    cfg.swing_width = declare_parameter<double>("swing_width", 0.0);
    cfg.controller_dt = declare_parameter<double>("controller_dt", 0.005);
    cfg.cmd_zero_tol = declare_parameter<double>("cmd_zero_tol", 1.0e-4);
    cfg.pause_debounce_delay =
        declare_parameter<double>("pause_debounce_delay", 0.4);
    cfg.pause_to_reseat_delay =
        declare_parameter<double>("pause_to_reseat_delay", 0.2);
    cfg.gait_change_pause_to_reseat_delay =
        declare_parameter<double>("gait_change_pause_to_reseat_delay", 0.1);
    cfg.max_reset_time = declare_parameter<double>("max_reset_time", 1.2);
    cfg.init_pair_swing_time =
        declare_parameter<double>("initialize.pair_swing_time", 0.4);
    cfg.init_lift_body_time =
        declare_parameter<double>("initialize.lift_body_time", 0.6);
    cfg.init_swing_clearance =
        declare_parameter<double>("initialize.swing_clearance", 0.04);
    cfg.init_place_feet_clearance =
        declare_parameter<double>("initialize.place_feet_clearance", 0.012);
    cfg.reseat_pose_settle_delay =
        declare_parameter<double>("reseat.pose_settle_delay", 0.5);
    cfg.reseat_height_change_threshold =
        declare_parameter<double>("reseat.height_change_threshold", 0.002);
    cfg.reseat_pair_swing_time =
        declare_parameter<double>("reseat.pair_swing_time", 0.2);
    cfg.reseat_pair_dwell_time =
        declare_parameter<double>("reseat.pair_dwell_time", 0.05);
    cfg.reseat_swing_clearance =
        declare_parameter<double>("reseat.swing_clearance", 0.025);

    default_gait_out = declare_parameter<std::string>("default_gait", "tripod");
    if (hexa_gait::strategies().find(default_gait_out) ==
        hexa_gait::strategies().end()) {
      throw std::runtime_error("default_gait=" + default_gait_out +
                               " not in STRATEGIES");
    }
    return cfg;
  }

  void on_init(const std_msgs::msg::Empty&) {
    if (engine_->start_initialize()) {
      RCLCPP_INFO(get_logger(),
                  "start-button trigger received: FOLDED -> INITIALIZE");
      return;
    }
    // Anywhere other than FOLDED: queue a fold request, consumed the next time
    // the engine sits in STAND at zero height.
    if (engine_->request_fold()) {
      RCLCPP_INFO(get_logger(),
                  "start-button trigger received: fold requested (engine in %s)",
                  hexa_gait::state_name(engine_->state()).c_str());
      return;
    }
    RCLCPP_INFO(get_logger(),
                "start-button trigger ignored: engine in state %s",
                hexa_gait::state_name(engine_->state()).c_str());
  }

  void on_params(const hexa_interfaces::msg::GaitParams& msg) {
    // Strategy switch arrives folded into GaitParams.gait_name. gait_name_
    // tracks the last *handled* request so a rejected swap is not retried on
    // every message; the engine's strategy_name stays the source of truth.
    if (!msg.gait_name.empty() && msg.gait_name != gait_name_) {
      if (engine_->set_strategy(msg.gait_name)) {
        if (engine_->pending_strategy_name().has_value() &&
            engine_->pending_strategy_name().value() == msg.gait_name) {
          RCLCPP_INFO(get_logger(),
                      "gait change to '%s' pending — pause-and-reseat sequence "
                      "running (engine in %s)",
                      msg.gait_name.c_str(),
                      hexa_gait::state_name(engine_->state()).c_str());
        } else {
          RCLCPP_INFO(get_logger(), "gait strategy switched to '%s'",
                      msg.gait_name.c_str());
        }
      } else {
        RCLCPP_WARN(get_logger(),
                    "gait change to '%s' dropped — gait locked during "
                    "engagement (engine in %s); request not retried",
                    msg.gait_name.c_str(),
                    hexa_gait::state_name(engine_->state()).c_str());
      }
      gait_name_ = msg.gait_name;
    }
    linear_x_ = msg.linear_x;
    linear_y_ = msg.linear_y;
    angular_z_ = msg.angular_z;
  }

  void tick() {
    const int64_t now_ns = get_clock()->now().nanoseconds();
    double dt;
    if (!last_tick_ns_.has_value()) {
      dt = 1.0 / kPublishRateHz;
    } else {
      dt = static_cast<double>(now_ns - last_tick_ns_.value()) * 1e-9;
      if (dt <= 0.0) {
        // /clock can rewind during sim resets; skip this tick.
        last_tick_ns_ = now_ns;
        return;
      }
    }
    last_tick_ns_ = now_ns;

    // DEBUG: trace engine state transitions. Remove once verified.
    const hexa_gait::EngineState prev_state = engine_->state();
    const auto out = engine_->update(dt, {linear_x_, linear_y_}, angular_z_);
    if (engine_->state() != prev_state) {
      RCLCPP_INFO(get_logger(), "[gait-debug] %s -> %s",
                  hexa_gait::state_name(prev_state).c_str(),
                  hexa_gait::state_name(engine_->state()).c_str());
    }

    hexa_interfaces::msg::LegTargets msg;
    msg.header.stamp = now();
    // msg.legs is a fixed-size std::array<LegState, 6>; fill by LEG_NAMES index.
    for (std::size_t i = 0; i < hexa_gait::LEG_NAMES.size(); ++i) {
      const std::string& name = hexa_gait::LEG_NAMES[i];
      const hexa_gait::LegOutput& leg = out.at(name);
      hexa_interfaces::msg::LegState state;
      state.leg_name = name;
      geometry_msgs::msg::Point point;
      point.x = leg.foot_target[0];
      point.y = leg.foot_target[1];
      point.z = leg.foot_target[2];
      state.foot_target = point;
      state.phase = leg.phase;
      state.stance = leg.stance;
      msg.legs[i] = state;
    }
    msg.master_phase = engine_->master_phase();
    pub_targets_->publish(msg);

    std_msgs::msg::String state_msg;
    state_msg.data = hexa_gait::state_value(engine_->state());
    pub_state_->publish(state_msg);
  }

  hexa_gait::EngineConfig cfg_;
  std::unique_ptr<hexa_gait::Engine> engine_;

  std::string gait_name_;
  double linear_x_ = 0.0;
  double linear_y_ = 0.0;
  double angular_z_ = 0.0;

  rclcpp::Subscription<hexa_interfaces::msg::GaitParams>::SharedPtr sub_params_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_init_;
  rclcpp::Subscription<hexa_interfaces::msg::BodyPose>::SharedPtr sub_body_pose_;
  rclcpp::Publisher<hexa_interfaces::msg::LegTargets>::SharedPtr pub_targets_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<int64_t> last_tick_ns_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GaitNode>());
  rclcpp::shutdown();
  return 0;
}
