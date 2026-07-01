// Adapter: sensor_msgs/JointState -> std_msgs/Float64MultiArray. Port of
// joint_command_bridge.py.
//
// The canonical kinematics output is /joint_commands (JointState) — semantic,
// self-describing, joint names included. ros2_control position group
// controllers take Float64MultiArray in a fixed joint order set by the
// controller's YAML config. This bridge indexes each joint's latest position by
// name and emits the fixed-order array.
//
// Holding latest-per-joint values means a partial JointState update (e.g. one
// leg) doesn't zero unrelated joints. Joint order and topics are parameterised
// so the same bridge serves sim and any future real-robot controller.

#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace {

using JointState = sensor_msgs::msg::JointState;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;

// 18 joint names, leg-major / segment-minor — the same order as the IK node's
// JOINT_ORDER and the Python DEFAULT_JOINT_ORDER.
std::vector<std::string> default_joint_order() {
  static const std::array<std::string, 6> legs = {
      "l_front", "l_middle", "l_rear", "r_front", "r_middle", "r_rear"};
  static const std::array<std::string, 3> segments = {"coxa", "femur", "tibia"};
  std::vector<std::string> out;
  out.reserve(legs.size() * segments.size());
  for (const auto& leg : legs) {
    for (const auto& seg : segments) {
      out.push_back(leg + "_" + seg + "_joint");
    }
  }
  return out;
}

class JointCommandBridge : public rclcpp::Node {
 public:
  JointCommandBridge() : rclcpp::Node("joint_command_bridge") {
    declare_parameter<std::string>("input_topic", "/joint_commands");
    declare_parameter<std::string>(
        "output_topic", "/joint_group_position_controller/commands");
    declare_parameter<std::vector<std::string>>("joint_order",
                                                default_joint_order());

    const std::string in_topic = get_parameter("input_topic").as_string();
    const std::string out_topic = get_parameter("output_topic").as_string();
    joint_order_ = get_parameter("joint_order").as_string_array();
    for (const auto& j : joint_order_) {
      positions_[j] = 0.0;
    }

    sub_ = create_subscription<JointState>(
        in_topic, 10, [this](JointState::SharedPtr msg) { on_joints(*msg); });
    pub_ = create_publisher<Float64MultiArray>(out_topic, 10);

    RCLCPP_INFO(get_logger(),
                "bridging %s (JointState) -> %s (Float64MultiArray) for %zu "
                "joints",
                in_topic.c_str(), out_topic.c_str(), joint_order_.size());
  }

 private:
  void on_joints(const JointState& msg) {
    const std::size_t n = std::min(msg.name.size(), msg.position.size());
    for (std::size_t i = 0; i < n; ++i) {
      const auto it = positions_.find(msg.name[i]);
      if (it != positions_.end()) {
        it->second = msg.position[i];
      }
    }
    Float64MultiArray out;
    out.data.reserve(joint_order_.size());
    for (const auto& j : joint_order_) {
      out.data.push_back(positions_[j]);
    }
    pub_->publish(out);
  }

  std::vector<std::string> joint_order_;
  std::map<std::string, double> positions_;

  rclcpp::Subscription<JointState>::SharedPtr sub_;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr pub_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointCommandBridge>());
  rclcpp::shutdown();
  return 0;
}
