// Internal pose types for the posture controller. Port of hexa_posture/pose.py.
//
// The ROS interface is hexa_interfaces/msg/BodyPose; this header provides a
// ROS-free analogue so the library code (animations, clamps, composition) links
// without rclcpp and is unit-testable in isolation. posture_node.cpp is the only
// place that converts between the ROS msg and these structs.
#pragma once

namespace hexa_posture {

// 6-DOF body pose offset from nominal — translations in metres, rotations in
// radians. REP-103 body frame (+x fwd, +y left, +z up), intrinsic XYZ rotation
// order. Mirrors BodyPose.msg field-for-field.
struct BodyPose {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  bool operator==(const BodyPose&) const = default;
};

// Zero offset. Mirrors hexa_posture.pose.IDENTITY.
inline constexpr BodyPose IDENTITY{};

// Per-axis symmetric clamp envelope for the final pose target. A blunt first cut
// at safety: a geometry-aware clamp belongs in the IK node; this static envelope
// is a cheap upstream guard against runaway animation/teleop inputs.
struct PoseLimits {
  double x = 0.05;      // m
  double y = 0.05;      // m
  double z = 0.04;      // m, max body lift/drop
  double roll = 0.30;   // rad (~17 deg)
  double pitch = 0.30;  // rad
  double yaw = 0.50;    // rad (~29 deg)
};

// Component-wise sum. Valid only for small offsets: full SE(3) composition
// doesn't commute and intrinsic-XYZ Euler angles don't add. The posture stack
// stays in the small-angle regime, so additive composition is good enough.
BodyPose add(const BodyPose& a, const BodyPose& b);

// Uniform component-wise scalar multiply.
BodyPose scale(const BodyPose& p, double k);

// Component-wise linear interpolation: a + t*(b - a). t is not clamped, but the
// posture node only ever passes t in [0, 1] (the activation crossfade).
BodyPose lerp(const BodyPose& a, const BodyPose& b, double t);

// Clamp each axis to [-limit, +limit].
BodyPose clamp(const BodyPose& pose, const PoseLimits& limits);

// Layered clamp: give the static user pose and the (AC) animation offset each
// their own budget so a dialed-in posture can never asymmetrically clip the
// animation. The user pose is clamped to (limits - anim_reserve, floored at 0)
// per axis, the animation to +/-anim_reserve, then summed and clamped to limits
// as a final guard (inert while user_env + anim_reserve <= limits). Trade-off:
// the user's static range shrinks by anim_reserve per axis.
BodyPose compose_layered(const BodyPose& user, const BodyPose& animated,
                         const PoseLimits& limits, const PoseLimits& anim_reserve);

}  // namespace hexa_posture
