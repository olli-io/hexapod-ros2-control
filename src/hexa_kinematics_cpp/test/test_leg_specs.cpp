// Port of hexa_kinematics/test/test_leg_specs.py.
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <set>
#include <string>

#include "hexa_kinematics_cpp/leg_specs.hpp"

namespace {

using hexa_kinematics::LEG_NAMES;
using hexa_kinematics::load_leg_specs;
using hexa_kinematics::Point3;

// Minimal geometry.yaml mirroring the structure expected by the loader. Numbers
// are easy to verify by eye. Matches the pytest tmp_path fixture.
const char* kGeometryYaml = R"(
leg:
  coxa_length: 0.04
  femur_length: 0.07
  tibia_length: 0.10
mounts:
  l_front:  {x: 0.08, y: 0.07, yaw_deg: 45}
  l_middle: {x: 0.00, y: 0.09, yaw_deg: 90}
)";

const double kFrontYaw = 45.0 * M_PI / 180.0;
const double kMiddleYaw = 90.0 * M_PI / 180.0;

std::string writeFixture() {
  const std::string path = std::string(::testing::TempDir()) + "/geometry.yaml";
  std::ofstream(path) << kGeometryYaml;
  return path;
}

void expectClosePoint(const Point3& a, const Point3& b, double tol = 1e-6) {
  EXPECT_NEAR(a[0], b[0], tol);
  EXPECT_NEAR(a[1], b[1], tol);
  EXPECT_NEAR(a[2], b[2], tol);
}

TEST(LegSpecs, LoadsAllSixLegs) {
  const auto legs = load_leg_specs(writeFixture());
  std::set<std::string> keys;
  for (const auto& [name, spec] : legs) keys.insert(name);
  std::set<std::string> expected(LEG_NAMES.begin(), LEG_NAMES.end());
  EXPECT_EQ(keys, expected);
}

TEST(LegSpecs, SegmentLengthsPropagate) {
  const auto spec = load_leg_specs(writeFixture()).at("l_front");
  EXPECT_NEAR(spec.coxa_len, 0.04, 1e-12);
  EXPECT_NEAR(spec.femur_len, 0.07, 1e-12);
  EXPECT_NEAR(spec.tibia_len, 0.10, 1e-12);
}

TEST(LegSpecs, LeftLegsMatchReferenceMounts) {
  const auto legs = load_leg_specs(writeFixture());
  expectClosePoint(legs.at("l_front").mount_xyz, Point3(0.08, 0.07, 0.0));
  EXPECT_NEAR(legs.at("l_front").mount_yaw, kFrontYaw, 1e-12);
  expectClosePoint(legs.at("l_middle").mount_xyz, Point3(0.0, 0.09, 0.0));
  EXPECT_NEAR(legs.at("l_middle").mount_yaw, kMiddleYaw, 1e-12);
}

TEST(LegSpecs, RearMirrorsFrontAboutYAxis) {
  const auto legs = load_leg_specs(writeFixture());
  // rear: x -> -x, yaw -> pi - yaw
  expectClosePoint(legs.at("l_rear").mount_xyz, Point3(-0.08, 0.07, 0.0));
  EXPECT_NEAR(legs.at("l_rear").mount_yaw, M_PI - kFrontYaw, 1e-12);
}

TEST(LegSpecs, RightMirrorsLeftAboutXAxis) {
  const auto legs = load_leg_specs(writeFixture());
  // right: y -> -y, yaw -> -yaw (applied after the front/rear mirror)
  expectClosePoint(legs.at("r_front").mount_xyz, Point3(0.08, -0.07, 0.0));
  EXPECT_NEAR(legs.at("r_front").mount_yaw, -kFrontYaw, 1e-12);
  expectClosePoint(legs.at("r_rear").mount_xyz, Point3(-0.08, -0.07, 0.0));
  EXPECT_NEAR(legs.at("r_rear").mount_yaw, -(M_PI - kFrontYaw), 1e-12);
  expectClosePoint(legs.at("r_middle").mount_xyz, Point3(0.0, -0.09, 0.0));
  EXPECT_NEAR(legs.at("r_middle").mount_yaw, -kMiddleYaw, 1e-12);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
