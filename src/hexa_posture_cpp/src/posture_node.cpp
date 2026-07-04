// ROS glue for the hexa_posture controller. Port of posture_node.py.
//
// Subscribes to the user pose (/body/pose), the gait engine state
// (/gait/state), the active gait params (/gait/params), the per-leg foot
// targets (/legs/targets), and the animation mode selection (/animation/mode).
// On a fixed 200 Hz timer it runs the animation stack with the current context,
// cross-fades the gait animations in/out with an activation slewed off the
// engine state, composes the result with the user pose under a layered clamp,
// and publishes on /body/pose_target.
//
// The gait state drives two things: (1) the posture-active gate — in
// folded/initialize/folding the feet are not at the nominal footprint, so a
// body-pose offset would be nonsense and the node emits IDENTITY; (2) the
// gait-animation activation crossfade — the animations ramp in/out as the
// engine engages/settles rather than stepping at the /cmd_vel edge. All
// pose/animation/signal math lives in the pure library; this file owns only the
// ROS plumbing.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <hexa_interfaces/msg/body_pose.hpp>
#include <hexa_interfaces/msg/gait_params.hpp>
#include <hexa_interfaces/msg/leg_targets.hpp>
#include <std_msgs/msg/string.hpp>

#include "hexa_posture_cpp/animations.hpp"
#include "hexa_posture_cpp/pose.hpp"
#include "hexa_posture_cpp/signals.hpp"
#include "hexa_posture_cpp/types.hpp"

namespace {

using hexa_posture::AnimationContext;
using hexa_posture::AnimationPtr;
using hexa_posture::BodyPose;
using hexa_posture::PoseLimits;
using hexa_posture::Stack;
using hexa_posture::Vec3;

using BodyPoseMsg = hexa_interfaces::msg::BodyPose;
using GaitParams = hexa_interfaces::msg::GaitParams;
using LegTargets = hexa_interfaces::msg::LegTargets;
using StringMsg = std_msgs::msg::String;

constexpr double kPublishRateHz = 200.0;
constexpr double kDegToRad = M_PI / 180.0;

BodyPose msg_to_pose(const BodyPoseMsg& m) {
  return BodyPose{m.x, m.y, m.z, m.roll, m.pitch, m.yaw};
}

BodyPoseMsg pose_to_msg(const BodyPose& p, const rclcpp::Time& stamp) {
  BodyPoseMsg out;
  out.header.stamp = stamp;  // rclcpp::Time -> builtin_interfaces::msg::Time
  // frame_id intentionally left blank: the pose is an offset in the body frame,
  // not a transform into a named TF frame.
  out.x = p.x;
  out.y = p.y;
  out.z = p.z;
  out.roll = p.roll;
  out.pitch = p.pitch;
  out.yaw = p.yaw;
  return out;
}

// Split "a, b ,c" into {"a", "b", "c"} with each token trimmed.
std::vector<std::string> split_trim(const std::string& s) {
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    const auto begin = tok.find_first_not_of(" \t");
    if (begin == std::string::npos) {
      out.emplace_back("");
      continue;
    }
    const auto end = tok.find_last_not_of(" \t");
    out.push_back(tok.substr(begin, end - begin + 1));
  }
  return out;
}

class PostureNode : public rclcpp::Node {
 public:
  PostureNode() : rclcpp::Node("posture_node") {
    // --- Parameters (defaults mirror tuning.yaml's posture_node block /
    // the Python node) ---
    const auto enabled = declare_string_array(
        "enabled_animations", hexa_posture::DEFAULT_ANIMATIONS);
    const auto animation_mode_names = declare_string_array(
        "animation_mode_animations",
        hexa_posture::DEFAULT_ANIMATION_MODE_ANIMATIONS);
    const double gait_sway_gain = declare_parameter("gait_sway_gain", 1.0);
    const double gait_sway_strength =
        declare_parameter("gait_sway_strength", 0.4);
    const double gait_bounce_arc_height =
        declare_parameter("gait_bounce_arc_height", 0.02);
    const double gait_bounce_step_height_ref =
        declare_parameter("gait_bounce_step_height_ref", 0.08);
    const double vbr_z =
        declare_parameter("vertical_body_roll_z_amplitude", 0.02);
    const double vbr_pitch_rad =
        declare_parameter("vertical_body_roll_pitch_amplitude_deg", 10.0) *
        kDegToRad;
    const double vbr_phase =
        declare_parameter("vertical_body_roll_phase_offset", 0.0);
    const double hbr_y =
        declare_parameter("horizontal_body_roll_y_amplitude", 0.02);
    const double hbr_yaw_rad =
        declare_parameter("horizontal_body_roll_yaw_amplitude_deg", 10.0) *
        kDegToRad;
    const double hbr_phase =
        declare_parameter("horizontal_body_roll_phase_offset", 0.0);
    const double br3d_z = declare_parameter("body_roll_3d_z_amplitude", 0.02);
    const double br3d_pitch_rad =
        declare_parameter("body_roll_3d_pitch_amplitude_deg", 10.0) * kDegToRad;
    const double br3d_y = declare_parameter("body_roll_3d_y_amplitude", 0.02);
    const double br3d_yaw_rad =
        declare_parameter("body_roll_3d_yaw_amplitude_deg", 10.0) * kDegToRad;
    const double br3d_h_phase =
        declare_parameter("body_roll_3d_horizontal_phase_offset", 0.25);
    const double br3d_pitch_phase =
        declare_parameter("body_roll_3d_pitch_phase_offset", 0.0);
    const double br3d_yaw_phase =
        declare_parameter("body_roll_3d_yaw_phase_offset", 0.0);
    centroid_tau_ = declare_parameter("support_centroid_tau", 0.1);
    swing_lift_tau_ = declare_parameter("swing_lift_tau", 0.04);

    // Linear slew rate (1/s) of the gait-animation activation crossfade. The
    // animation contribution ramps in/out over ~1/rate seconds when the engine
    // engages/disengages the gait, instead of stepping at the /cmd_vel edge.
    activation_slew_rate_ =
        declare_parameter("gait_activation_slew_rate", 4.0);
    // Per-axis budget reserved for the animation layer in the layered clamp, so
    // a dialed-in user posture can never asymmetrically clip the animation. The
    // user's static-posture range shrinks by these amounts (limits - reserve).
    anim_reserve_.x = declare_parameter("animation_reserve_x", 0.02);
    anim_reserve_.y = declare_parameter("animation_reserve_y", 0.02);
    anim_reserve_.z = declare_parameter("animation_reserve_z", 0.03);
    anim_reserve_.roll = declare_parameter("animation_reserve_roll", 0.20);
    anim_reserve_.pitch = declare_parameter("animation_reserve_pitch", 0.20);
    anim_reserve_.yaw = declare_parameter("animation_reserve_yaw", 0.20);

    // Parameterized animations override the default-constructed factory
    // instances; still/breathing are never overridden.
    const std::map<std::string, AnimationPtr> overrides = {
        {"gait_sway", std::make_shared<hexa_posture::GaitSway>(
                          gait_sway_gain, gait_sway_strength)},
        {"gait_bounce", std::make_shared<hexa_posture::GaitBounce>(
                            gait_bounce_arc_height, gait_bounce_step_height_ref)},
        {"vertical_body_roll", std::make_shared<hexa_posture::VerticalBodyRoll>(
                                   vbr_z, vbr_pitch_rad, vbr_phase)},
        {"horizontal_body_roll",
         std::make_shared<hexa_posture::HorizontalBodyRoll>(hbr_y, hbr_yaw_rad,
                                                            hbr_phase)},
        {"body_roll_3d", std::make_shared<hexa_posture::BodyRoll3D>(
                             br3d_z, br3d_pitch_rad, br3d_y, br3d_yaw_rad,
                             br3d_h_phase, br3d_pitch_phase, br3d_yaw_phase)},
    };

    default_stack_ = hexa_posture::build_animation_stack(enabled, overrides);
    // Per-animation stacks for ANIMATION mode: each entry yields a dedicated
    // stack of "still" + the named animation(s), so gait_sway/gait_bounce do
    // not bleed in while the user is demoing a body animation. Comma-separated
    // entries compose multiple animations into one stack.
    for (const auto& name : animation_mode_names) {
      std::vector<std::string> layer_names = {"still"};
      for (auto& n : split_trim(name)) {
        layer_names.push_back(n);
      }
      animation_stacks_.emplace(
          name, hexa_posture::build_animation_stack(layer_names, overrides));
    }

    RCLCPP_INFO(get_logger(), "animations enabled: %s",
                join(enabled).c_str());
    RCLCPP_INFO(get_logger(), "animation-mode animations available: %s",
                join(animation_mode_names).c_str());

    // --- Subscriptions / publisher ---
    sub_pose_ = create_subscription<BodyPoseMsg>(
        "/body/pose", 10,
        [this](BodyPoseMsg::SharedPtr msg) { user_pose_ = msg_to_pose(*msg); });
    sub_gait_state_ = create_subscription<StringMsg>(
        "/gait/state", 10,
        [this](StringMsg::SharedPtr msg) { gait_state_ = msg->data; });
    sub_gait_params_ = create_subscription<GaitParams>(
        "/gait/params", 10, [this](GaitParams::SharedPtr msg) {
          // Only the name matters here; keep the previous value on an empty
          // string so a stray default-constructed message doesn't blank the
          // gate.
          if (!msg->gait_name.empty()) {
            gait_name_ = msg->gait_name;
          }
        });
    sub_targets_ = create_subscription<LegTargets>(
        "/legs/targets", 10,
        [this](LegTargets::SharedPtr msg) { on_leg_targets(*msg); });
    // transient_local so we always pick up the latest animation-mode selection
    // from teleop even if this node starts after the user picked one.
    rclcpp::QoS animation_qos(rclcpp::KeepLast(1));
    animation_qos.transient_local();
    sub_animation_mode_ = create_subscription<StringMsg>(
        "/animation/mode", animation_qos,
        [this](StringMsg::SharedPtr msg) { on_animation_mode(msg->data); });

    pub_target_ = create_publisher<BodyPoseMsg>("/body/pose_target", 10);

    const auto period = std::chrono::duration<double>(1.0 / kPublishRateHz);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this] { tick(); });
  }

 private:
  std::vector<std::string> declare_string_array(
      const std::string& name, const std::vector<std::string>& fallback) {
    auto value = declare_parameter(name, fallback);
    if (value.empty()) {
      return fallback;
    }
    return value;
  }

  static std::string join(const std::vector<std::string>& v) {
    std::ostringstream oss;
    oss << "[";
    for (std::size_t i = 0; i < v.size(); ++i) {
      oss << (i ? ", " : "") << v[i];
    }
    oss << "]";
    return oss.str();
  }

  void on_animation_mode(const std::string& new_mode) {
    if (!new_mode.empty() &&
        animation_stacks_.find(new_mode) == animation_stacks_.end()) {
      RCLCPP_WARN(get_logger(), "unknown animation mode '%s'",
                  new_mode.c_str());
      return;
    }
    if (new_mode == animation_mode_) {
      return;
    }
    animation_mode_ = new_mode;
    if (!new_mode.empty()) {
      RCLCPP_INFO(get_logger(), "animation mode active: %s", new_mode.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "animation mode cleared");
    }
  }

  void on_leg_targets(const LegTargets& msg) {
    std::array<Vec3, hexa_posture::LEG_COUNT> feet{};
    std::array<bool, hexa_posture::LEG_COUNT> stance{};
    stance.fill(false);
    const std::size_t n = std::min<std::size_t>(msg.legs.size(), feet.size());
    for (std::size_t i = 0; i < n; ++i) {
      const auto& leg = msg.legs[i];
      feet[i] = Vec3(leg.foot_target.x, leg.foot_target.y, leg.foot_target.z);
      stance[i] = leg.stance;
    }
    const auto raw = hexa_posture::stance_centroid_xy(feet, stance);
    if (raw.has_value()) {
      latest_raw_centroid_ = raw;
    }
    const auto lift = hexa_posture::max_swing_lift_z(feet, stance);
    if (lift.has_value()) {
      latest_raw_swing_lift_ = lift;
    }
    double phase = std::fmod(msg.master_phase, 1.0);
    if (phase < 0.0) {
      phase += 1.0;
    }
    master_phase_ = phase;
  }

  void step_filters(double dt) {
    support_centroid_xy_ = hexa_posture::lpf_step_xy(
        support_centroid_xy_, latest_raw_centroid_, centroid_tau_, dt);
    swing_lift_z_ = hexa_posture::lpf_step_scalar(
        swing_lift_z_, latest_raw_swing_lift_, swing_lift_tau_, dt);
  }

  void tick() {
    const rclcpp::Time now = get_clock()->now();
    const int64_t now_ns = now.nanoseconds();
    double dt;
    if (!last_tick_ns_.has_value()) {
      dt = 1.0 / kPublishRateHz;
    } else {
      dt = std::max((now_ns - *last_tick_ns_) * 1e-9, 0.0);
    }
    last_tick_ns_ = now_ns;
    step_filters(dt);

    if (!hexa_posture::is_posture_active(gait_state_)) {
      // Engine is FOLDED / INITIALIZE / FOLDING (or no state seen yet): hold
      // IDENTITY until posture is meaningful again. Reset the crossfade so a
      // re-engage from a fold always ramps the gait animations in from idle.
      activation_ = 0.0;
      pub_target_->publish(pose_to_msg(hexa_posture::IDENTITY, now));
      return;
    }

    // Crossfade the gait-animation contribution in/out. Key off the engine's
    // real state (not the raw /cmd_vel edge), so the fade-out finishes as the
    // feet settle to the nominal stance rather than ~0.4 s earlier.
    const double activation_target =
        hexa_posture::is_gait_engaged(gait_state_) ? 1.0 : 0.0;
    activation_ = hexa_posture::slew_toward(activation_, activation_target,
                                            activation_slew_rate_, dt);

    AnimationContext ctx;
    ctx.t = now_ns * 1e-9;
    ctx.gait_name = gait_name_;
    ctx.support_centroid_xy = support_centroid_xy_;
    ctx.swing_lift_z = swing_lift_z_;
    ctx.master_phase = master_phase_;

    // Evaluate the stack twice — gait-active (walking=true) and idle
    // (walking=false, i.e. breathing) — and cross-fade. Animations self-gate on
    // ctx.walking, so this cleanly separates the two contributions without a
    // per-animation activation term (animations stay pure).
    const Stack& stack = animation_mode_.empty()
                             ? default_stack_
                             : animation_stacks_.at(animation_mode_);
    ctx.walking = true;
    const BodyPose gait_out = stack(ctx);
    ctx.walking = false;
    const BodyPose idle_out = stack(ctx);
    const BodyPose animated = hexa_posture::lerp(idle_out, gait_out, activation_);

    // Layered clamp: user posture and animation each get their own budget so a
    // dialed-in posture never asymmetrically clips the animation.
    const BodyPose target = hexa_posture::compose_layered(
        user_pose_, animated, limits_, anim_reserve_);
    pub_target_->publish(pose_to_msg(target, now));
  }

  // --- Latest inputs ---
  BodyPose user_pose_ = hexa_posture::IDENTITY;
  // Empty until /gait/state arrives — treated as inactive (emits IDENTITY).
  // Drives both the posture-active gate and the gait-animation crossfade.
  std::string gait_state_;
  std::optional<std::string> gait_name_;
  std::optional<std::pair<double, double>> support_centroid_xy_;
  std::optional<std::pair<double, double>> latest_raw_centroid_;
  std::optional<double> swing_lift_z_;
  std::optional<double> latest_raw_swing_lift_;
  std::optional<double> master_phase_;
  std::optional<int64_t> last_tick_ns_;

  // --- Config / derived state ---
  double centroid_tau_ = 0.1;
  double swing_lift_tau_ = 0.04;
  // Gait-animation activation crossfade in [0, 1] and its slew rate (1/s).
  double activation_ = 0.0;
  double activation_slew_rate_ = 4.0;
  Stack default_stack_;
  std::map<std::string, Stack> animation_stacks_;
  std::string animation_mode_;
  PoseLimits limits_;
  // Per-axis budget reserved for the animation layer in the layered clamp.
  PoseLimits anim_reserve_;

  rclcpp::Subscription<BodyPoseMsg>::SharedPtr sub_pose_;
  rclcpp::Subscription<StringMsg>::SharedPtr sub_gait_state_;
  rclcpp::Subscription<GaitParams>::SharedPtr sub_gait_params_;
  rclcpp::Subscription<LegTargets>::SharedPtr sub_targets_;
  rclcpp::Subscription<StringMsg>::SharedPtr sub_animation_mode_;
  rclcpp::Publisher<BodyPoseMsg>::SharedPtr pub_target_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PostureNode>());
  rclcpp::shutdown();
  return 0;
}
