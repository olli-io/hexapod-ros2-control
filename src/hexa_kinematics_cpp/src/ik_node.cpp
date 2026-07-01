// ROS glue for the hexa_kinematics IK library. Port of ik_node.py.
//
// Sits at the convergence of the gait chain (foot targets in the nominal body
// frame, published on /legs/targets) and the body-pose chain (BodyPose offset
// published on /body/pose_target by hexa_posture).
//
// Per /legs/targets tick, for each leg:
//   1. apply_body_pose — re-express the foot target in the pose-offset frame.
//   2. body_to_leg into the leg's coxa-mount frame.
//   3. inverse_kinematics to recover (coxa, femur, tibia) joint angles.
//
// Emit an 18-entry sensor_msgs/JointState on /joint_commands in the URDF joint
// order (see JOINT_ORDER). All kinematics math lives in the pure library; this
// file owns only the ROS plumbing.

#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <hexa_interfaces/msg/body_pose.hpp>
#include <hexa_interfaces/msg/leg_targets.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "hexa_kinematics_cpp/body_transform.hpp"
#include "hexa_kinematics_cpp/leg_ik.hpp"
#include "hexa_kinematics_cpp/leg_specs.hpp"

namespace {

using BodyPoseMsg = hexa_interfaces::msg::BodyPose;
using LegTargets = hexa_interfaces::msg::LegTargets;
using JointState = sensor_msgs::msg::JointState;

// 18 joint names, leg-major / segment-minor, matching the URDF and the Python
// JOINT_ORDER. Built from LEG_NAMES so the order stays single-sourced.
std::vector<std::string> make_joint_order() {
  static const std::array<std::string, 3> segments = {"coxa", "femur", "tibia"};
  std::vector<std::string> out;
  out.reserve(hexa_kinematics::LEG_NAMES.size() * segments.size());
  for (const auto& leg : hexa_kinematics::LEG_NAMES) {
    for (const auto& seg : segments) {
      out.push_back(leg + "_" + seg + "_joint");
    }
  }
  return out;
}

hexa_kinematics::BodyPose msg_to_pose(const BodyPoseMsg& m) {
  hexa_kinematics::BodyPose p;
  p.x = m.x;
  p.y = m.y;
  p.z = m.z;
  p.roll = m.roll;
  p.pitch = m.pitch;
  p.yaw = m.yaw;
  return p;
}

class IKNode : public rclcpp::Node {
 public:
  IKNode() : rclcpp::Node("ik_node") {
    const std::string geometry_yaml =
        ament_index_cpp::get_package_share_directory("hexa_description") +
        "/config/geometry.yaml";
    legs_ = hexa_kinematics::load_leg_specs(geometry_yaml);
    for (const auto& name : hexa_kinematics::LEG_NAMES) {
      if (legs_.find(name) == legs_.end()) {
        throw std::runtime_error("geometry.yaml is missing leg: " + name);
      }
    }
    RCLCPP_INFO(get_logger(), "loaded %zu leg specs from %s", legs_.size(),
                geometry_yaml.c_str());

    joint_order_ = make_joint_order();
    // Hold the last successful joint angles so a transient UnreachableTarget on
    // one leg doesn't zero unrelated joints (or this leg's own joints).
    for (const auto& j : joint_order_) {
      last_angles_[j] = 0.0;
    }

    // Latest /body/pose_target. Default to identity so we can produce joint
    // commands the moment /legs/targets starts arriving — even if hexa_posture
    // hasn't published yet.
    body_pose_ = hexa_kinematics::IDENTITY_BODY_POSE;

    sub_pose_ = create_subscription<BodyPoseMsg>(
        "/body/pose_target", 10,
        [this](BodyPoseMsg::SharedPtr msg) { body_pose_ = msg_to_pose(*msg); });
    sub_legs_ = create_subscription<LegTargets>(
        "/legs/targets", 10,
        [this](LegTargets::SharedPtr msg) { on_legs(*msg); });
    pub_joints_ = create_publisher<JointState>("/joint_commands", 10);
  }

 private:
  void on_legs(const LegTargets& msg) {
    std::map<std::string, double> angles = last_angles_;
    for (const auto& leg_state : msg.legs) {
      const auto it = legs_.find(leg_state.leg_name);
      if (it == legs_.end()) {
        RCLCPP_WARN(get_logger(), "unknown leg_name '%s' — skipping",
                    leg_state.leg_name.c_str());
        continue;
      }
      const hexa_kinematics::Point3 target_body(leg_state.foot_target.x,
                                                leg_state.foot_target.y,
                                                leg_state.foot_target.z);
      const hexa_kinematics::Point3 target_offset =
          hexa_kinematics::apply_body_pose(target_body, body_pose_);
      const hexa_kinematics::Point3 target_leg =
          hexa_kinematics::body_to_leg(target_offset, it->second);
      try {
        const hexa_kinematics::JointAngles a =
            hexa_kinematics::inverse_kinematics(target_leg, it->second);
        angles[leg_state.leg_name + "_coxa_joint"] = a[0];
        angles[leg_state.leg_name + "_femur_joint"] = a[1];
        angles[leg_state.leg_name + "_tibia_joint"] = a[2];
      } catch (const hexa_kinematics::UnreachableTarget& exc) {
        RCLCPP_WARN(get_logger(),
                    "%s: IK unreachable (%s); holding last angles",
                    leg_state.leg_name.c_str(), exc.what());
        continue;
      }
    }

    last_angles_ = angles;

    JointState out;
    out.header.stamp = now();
    out.name = joint_order_;
    out.position.reserve(joint_order_.size());
    for (const auto& j : joint_order_) {
      out.position.push_back(angles[j]);
    }
    pub_joints_->publish(out);
  }

  std::map<std::string, hexa_kinematics::LegSpec> legs_;
  std::vector<std::string> joint_order_;
  std::map<std::string, double> last_angles_;
  hexa_kinematics::BodyPose body_pose_;

  rclcpp::Subscription<BodyPoseMsg>::SharedPtr sub_pose_;
  rclcpp::Subscription<LegTargets>::SharedPtr sub_legs_;
  rclcpp::Publisher<JointState>::SharedPtr pub_joints_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKNode>());
  rclcpp::shutdown();
  return 0;
}
