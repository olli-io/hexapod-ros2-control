// Body-pose value type + small-offset algebra for the posture stack (plan
// part 08).
//
// Float fork of hexa_posture/pose.py. The Python module carries its own
// ROS-free BodyPose analogue so the animation / clamp / composition code stays
// importable without rclpy; here the equivalent role is played by the
// kinematics BodyPose (kinematics/body_transform.hpp) — the two structs are
// field-for-field identical (x, y, z, roll, pitch, yaw floats), and the whole
// point of the posture stack is to produce the BodyPose the compose step then
// feeds to apply_body_pose. Reusing the one struct avoids a redundant twin and
// a conversion at the seam.
//
// The add()/scale()/clamp() helpers live in namespace hexa::posture to mirror
// the Python pose module surface without colliding with the kinematics free
// functions.
#pragma once

#include "kinematics/body_transform.hpp"  // hexa::BodyPose, IDENTITY_BODY_POSE

namespace hexa::posture {

// The posture stack's pose currency IS the kinematics body-pose offset.
using BodyPose = ::hexa::BodyPose;

inline constexpr BodyPose IDENTITY{};

// Component-wise sum. Valid only for small offsets: full SE(3) composition
// doesn't commute and intrinsic-XYZ Euler angles don't add, but the posture
// stack stays in the small-angle regime (centimetre / single-digit-degree
// amplitudes), so additive composition is good enough. Mirrors pose.add.
BodyPose add(const BodyPose& a, const BodyPose& b);

// Uniform per-component scale. Mirrors pose.scale.
BodyPose scale(const BodyPose& p, float k);

// Per-axis symmetric clamp envelope for the final pose target. A blunt safety
// first cut (the real reachable envelope is geometry-dependent); a cheap
// upstream guard against runaway animation / teleop inputs. Mirrors
// pose.PoseLimits defaults exactly.
struct PoseLimits {
  float x = 0.05f;      // m
  float y = 0.05f;      // m
  float z = 0.04f;      // m, max body lift/drop
  float roll = 0.30f;   // rad (~17 deg)
  float pitch = 0.30f;  // rad
  float yaw = 0.50f;    // rad (~29 deg)
};

// Clamp each axis to [-limit, +limit]. Mirrors pose.clamp.
BodyPose clamp(const BodyPose& pose, const PoseLimits& limits);

}  // namespace hexa::posture
