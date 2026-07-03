// Golden-trace test for the float kinematics port (plan part 05).
//
// Diffs the firmware's float FK/IK/body-transform (src/kinematics, namespace
// hexa) against the untouched double hexa_kinematics_cpp engine (namespace
// hexa_kinematics, Eigen). Both are compiled into this one binary; they live in
// different namespaces so there is no ODR clash. The two libraries are fed the
// SAME geometry (the generated config::kLegSpecs, mirrored into the double
// LegSpec) so any divergence is purely the double->float conversion.
//
// This target deliberately bridges float and double, so the build drops
// -Wdouble-promotion for it (see test/host/CMakeLists.txt); the firmware build
// is the -Wdouble-promotion gate for the port sources themselves.
//
// Covered:
//   - FK/IK round-trip fk(ik(p)) ~= p within 1e-4 m (float port, self-consistent)
//   - float FK vs double FK over a joint-angle sweep (<= 1e-4 m)
//   - float IK vs double IK over a reachable target sweep (<= 1e-3 rad)
//   - float body_to_leg/leg_to_body/apply_body_pose vs double (<= 1e-4 m)
//   - full compose pipeline (nominal -> apply_body_pose -> body_to_leg -> IK)
//     across all six legs and several BodyPoses (<= 1e-3 rad), mirroring ik_node
//   - an unreachable target throws UnreachableTarget in both engines

#include <array>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "config_generated.hpp"
#include "kinematics/body_transform.hpp"
#include "kinematics/leg_ik.hpp"
#include "leg_index.hpp"

#include "hexa_kinematics_cpp/body_transform.hpp"
#include "hexa_kinematics_cpp/leg_ik.hpp"

namespace cfg = hexa::config;
namespace ref = hexa_kinematics;

namespace {

constexpr double kFkTol = 1e-4;   // metres (float FK vs double FK / round-trip)
constexpr double kIkTol = 1e-3;   // radians (float IK vs double IK)

// Mirror a generated (float) LegSpec into the double reference LegSpec, feeding
// both engines byte-identical geometry (up to float->double widening).
ref::LegSpec to_ref(const cfg::LegSpec& s) {
  ref::LegSpec d;
  d.mount_xyz = ref::Point3(static_cast<double>(s.mount_xyz.x),
                            static_cast<double>(s.mount_xyz.y),
                            static_cast<double>(s.mount_xyz.z));
  d.mount_yaw = static_cast<double>(s.mount_yaw);
  d.coxa_len = static_cast<double>(s.coxa_len);
  d.femur_len = static_cast<double>(s.femur_len);
  d.tibia_len = static_cast<double>(s.tibia_len);
  return d;
}

ref::JointAngles ref_standing() {
  return {static_cast<double>(cfg::kStandingPose[0]),
          static_cast<double>(cfg::kStandingPose[1]),
          static_cast<double>(cfg::kStandingPose[2])};
}

// Nominal standing foot target in the body frame for one leg (double reference):
// FK of the standing pose in the leg frame, lifted into the body frame. This is
// the nominal_stance the compose pipeline sweeps around.
ref::Point3 ref_nominal(const cfg::LegSpec& s) {
  const ref::LegSpec d = to_ref(s);
  return ref::leg_to_body(ref::forward_kinematics(ref_standing(), d), d);
}

}  // namespace

TEST(Kinematics, FkIkRoundTrip) {
  // Reachable leg-frame targets around the nominal standing foot: fk(ik(p)) ~= p.
  const cfg::LegSpec& s = cfg::kLegSpecs[0];
  const hexa::Vec3 base =
      hexa::forward_kinematics(cfg::kStandingPose, s);  // leg-frame nominal foot
  for (float dx = -0.04f; dx <= 0.04f; dx += 0.02f) {
    for (float dy = -0.04f; dy <= 0.04f; dy += 0.02f) {
      for (float dz = -0.04f; dz <= 0.04f; dz += 0.02f) {
        const hexa::Vec3 p(base.x + dx, base.y + dy, base.z + dz);
        const hexa::JointAngles a = hexa::inverse_kinematics(p, s);
        const hexa::Vec3 q = hexa::forward_kinematics(a, s);
        EXPECT_NEAR(static_cast<double>(q.x), static_cast<double>(p.x), kFkTol);
        EXPECT_NEAR(static_cast<double>(q.y), static_cast<double>(p.y), kFkTol);
        EXPECT_NEAR(static_cast<double>(q.z), static_cast<double>(p.z), kFkTol);
      }
    }
  }
}

TEST(Kinematics, GoldenForwardKinematics) {
  const cfg::LegSpec& s = cfg::kLegSpecs[0];
  const ref::LegSpec d = to_ref(s);
  for (float c = -1.0f; c <= 1.0f; c += 0.5f) {
    for (float f = -1.5f; f <= 0.5f; f += 0.5f) {
      for (float t = 0.5f; t <= 3.0f; t += 0.5f) {
        const hexa::JointAngles fa = {c, f, t};
        const ref::JointAngles da = {static_cast<double>(c),
                                     static_cast<double>(f),
                                     static_cast<double>(t)};
        const hexa::Vec3 fp = hexa::forward_kinematics(fa, s);
        const ref::Point3 dp = ref::forward_kinematics(da, d);
        EXPECT_NEAR(static_cast<double>(fp.x), dp[0], kFkTol);
        EXPECT_NEAR(static_cast<double>(fp.y), dp[1], kFkTol);
        EXPECT_NEAR(static_cast<double>(fp.z), dp[2], kFkTol);
      }
    }
  }
}

TEST(Kinematics, GoldenInverseKinematics) {
  // Sweep reachable leg-frame targets around each leg's nominal foot and diff
  // the float IK against the double IK, angle by angle.
  for (int leg = 0; leg < hexa::kNumLegs; ++leg) {
    const cfg::LegSpec& s = cfg::kLegSpecs[static_cast<std::size_t>(leg)];
    const ref::LegSpec d = to_ref(s);
    const hexa::Vec3 base = hexa::forward_kinematics(cfg::kStandingPose, s);
    for (float dx = -0.05f; dx <= 0.05f; dx += 0.025f) {
      for (float dy = -0.05f; dy <= 0.05f; dy += 0.025f) {
        for (float dz = -0.05f; dz <= 0.05f; dz += 0.025f) {
          const hexa::Vec3 fp(base.x + dx, base.y + dy, base.z + dz);
          const ref::Point3 dp(static_cast<double>(fp.x),
                               static_cast<double>(fp.y),
                               static_cast<double>(fp.z));
          const hexa::JointAngles fa = hexa::inverse_kinematics(fp, s);
          const ref::JointAngles da = ref::inverse_kinematics(dp, d);
          EXPECT_NEAR(static_cast<double>(fa[0]), da[0], kIkTol) << "leg " << leg;
          EXPECT_NEAR(static_cast<double>(fa[1]), da[1], kIkTol) << "leg " << leg;
          EXPECT_NEAR(static_cast<double>(fa[2]), da[2], kIkTol) << "leg " << leg;
        }
      }
    }
  }
}

TEST(Kinematics, GoldenBodyTransforms) {
  for (int leg = 0; leg < hexa::kNumLegs; ++leg) {
    const cfg::LegSpec& s = cfg::kLegSpecs[static_cast<std::size_t>(leg)];
    const ref::LegSpec d = to_ref(s);
    for (float x = -0.2f; x <= 0.2f; x += 0.1f) {
      for (float y = -0.2f; y <= 0.2f; y += 0.1f) {
        for (float z = -0.1f; z <= 0.1f; z += 0.1f) {
          const hexa::Vec3 fp(x, y, z);
          const ref::Point3 dp(static_cast<double>(x), static_cast<double>(y),
                               static_cast<double>(z));
          const hexa::Vec3 fbl = hexa::body_to_leg(fp, s);
          const ref::Point3 dbl = ref::body_to_leg(dp, d);
          EXPECT_NEAR(static_cast<double>(fbl.x), dbl[0], kFkTol);
          EXPECT_NEAR(static_cast<double>(fbl.y), dbl[1], kFkTol);
          EXPECT_NEAR(static_cast<double>(fbl.z), dbl[2], kFkTol);

          const hexa::Vec3 flb = hexa::leg_to_body(fp, s);
          const ref::Point3 dlb = ref::leg_to_body(dp, d);
          EXPECT_NEAR(static_cast<double>(flb.x), dlb[0], kFkTol);
          EXPECT_NEAR(static_cast<double>(flb.y), dlb[1], kFkTol);
          EXPECT_NEAR(static_cast<double>(flb.z), dlb[2], kFkTol);
        }
      }
    }
  }
}

TEST(Kinematics, GoldenApplyBodyPose) {
  // A handful of representative 6-DOF poses (matches the demo tilt magnitudes).
  const std::array<hexa::BodyPose, 5> poses = {{
      hexa::IDENTITY_BODY_POSE,
      {0.0f, 0.0f, -0.02f, 0.0f, 0.0873f, 0.0f},   // z -2 cm, pitch +5deg
      {0.01f, -0.015f, 0.0f, 0.0873f, 0.0f, 0.0f}, // xy shift, roll +5deg
      {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1745f},     // yaw +10deg
      {0.02f, 0.02f, -0.03f, 0.05f, -0.05f, 0.08f},
  }};
  const hexa::Vec3 p_nominal(0.15f, 0.05f, -0.12f);
  const ref::Point3 dp(static_cast<double>(p_nominal.x),
                       static_cast<double>(p_nominal.y),
                       static_cast<double>(p_nominal.z));
  for (const auto& fpose : poses) {
    ref::BodyPose dpose;
    dpose.x = static_cast<double>(fpose.x);
    dpose.y = static_cast<double>(fpose.y);
    dpose.z = static_cast<double>(fpose.z);
    dpose.roll = static_cast<double>(fpose.roll);
    dpose.pitch = static_cast<double>(fpose.pitch);
    dpose.yaw = static_cast<double>(fpose.yaw);
    const hexa::Vec3 fq = hexa::apply_body_pose(p_nominal, fpose);
    const ref::Point3 dq = ref::apply_body_pose(dp, dpose);
    EXPECT_NEAR(static_cast<double>(fq.x), dq[0], kFkTol);
    EXPECT_NEAR(static_cast<double>(fq.y), dq[1], kFkTol);
    EXPECT_NEAR(static_cast<double>(fq.z), dq[2], kFkTol);
  }
}

TEST(Kinematics, GoldenComposePipeline) {
  // The full ik_node path: nominal stance -> apply_body_pose -> body_to_leg ->
  // IK, for every leg under several static BodyPoses. This is the on-target
  // verification path (main.cpp compose_stance) diffed against the double engine.
  const std::array<hexa::BodyPose, 4> poses = {{
      hexa::IDENTITY_BODY_POSE,
      {0.0f, 0.0f, -0.02f, 0.0f, 0.0873f, 0.0f},   // z -2 cm, pitch +5deg
      {0.015f, -0.015f, 0.0f, 0.0873f, 0.0f, 0.0f},
      {0.0f, 0.0f, -0.01f, 0.03f, -0.03f, 0.0873f},
  }};
  for (const auto& fpose : poses) {
    ref::BodyPose dpose;
    dpose.x = static_cast<double>(fpose.x);
    dpose.y = static_cast<double>(fpose.y);
    dpose.z = static_cast<double>(fpose.z);
    dpose.roll = static_cast<double>(fpose.roll);
    dpose.pitch = static_cast<double>(fpose.pitch);
    dpose.yaw = static_cast<double>(fpose.yaw);
    for (int leg = 0; leg < hexa::kNumLegs; ++leg) {
      const cfg::LegSpec& s = cfg::kLegSpecs[static_cast<std::size_t>(leg)];
      const ref::LegSpec d = to_ref(s);

      const hexa::Vec3 fnom =
          hexa::leg_to_body(hexa::forward_kinematics(cfg::kStandingPose, s), s);
      const hexa::Vec3 fleg =
          hexa::body_to_leg(hexa::apply_body_pose(fnom, fpose), s);
      const hexa::JointAngles fa = hexa::inverse_kinematics(fleg, s);

      const ref::Point3 dnom = ref_nominal(s);
      const ref::Point3 dleg =
          ref::body_to_leg(ref::apply_body_pose(dnom, dpose), d);
      const ref::JointAngles da = ref::inverse_kinematics(dleg, d);

      EXPECT_NEAR(static_cast<double>(fa[0]), da[0], kIkTol) << "leg " << leg;
      EXPECT_NEAR(static_cast<double>(fa[1]), da[1], kIkTol) << "leg " << leg;
      EXPECT_NEAR(static_cast<double>(fa[2]), da[2], kIkTol) << "leg " << leg;
    }
  }
}

TEST(Kinematics, ComposeIdentityReproducesStandingPose) {
  // With an identity BodyPose the compose pipeline must round-trip back to the
  // standing joint angles for every leg (nominal = leg_to_body(fk(standing))).
  for (int leg = 0; leg < hexa::kNumLegs; ++leg) {
    const cfg::LegSpec& s = cfg::kLegSpecs[static_cast<std::size_t>(leg)];
    const hexa::Vec3 nom =
        hexa::leg_to_body(hexa::forward_kinematics(cfg::kStandingPose, s), s);
    const hexa::Vec3 in_leg =
        hexa::body_to_leg(hexa::apply_body_pose(nom, hexa::IDENTITY_BODY_POSE), s);
    const hexa::JointAngles a = hexa::inverse_kinematics(in_leg, s);
    EXPECT_NEAR(static_cast<double>(a[0]),
                static_cast<double>(cfg::kStandingPose[0]), kIkTol);
    EXPECT_NEAR(static_cast<double>(a[1]),
                static_cast<double>(cfg::kStandingPose[1]), kIkTol);
    EXPECT_NEAR(static_cast<double>(a[2]),
                static_cast<double>(cfg::kStandingPose[2]), kIkTol);
  }
}

TEST(Kinematics, UnreachableTargetThrows) {
  const cfg::LegSpec& s = cfg::kLegSpecs[0];
  const ref::LegSpec d = to_ref(s);
  // Far beyond the reach annulus (femur + tibia = 0.214 m).
  const hexa::Vec3 far(1.0f, 0.0f, 0.0f);
  const ref::Point3 dfar(1.0, 0.0, 0.0);
  EXPECT_THROW(hexa::inverse_kinematics(far, s), hexa::UnreachableTarget);
  EXPECT_THROW(ref::inverse_kinematics(dfar, d), ref::UnreachableTarget);

  // Inside the inner annulus hole (|femur - tibia| = 0.054 m from femur joint):
  // put the foot right at the coxa reach so d ~= 0.
  const hexa::Vec3 inner(s.coxa_len, 0.0f, 0.0f);
  EXPECT_THROW(hexa::inverse_kinematics(inner, s), hexa::UnreachableTarget);
}
