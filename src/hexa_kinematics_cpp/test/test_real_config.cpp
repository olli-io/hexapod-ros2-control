// Port of hexa_kinematics/test/test_real_config.py.
//
// Schema-drift smoke tests against the installed hexa_description YAML.
// Complements test_leg_specs.cpp (which verifies mirror logic with
// self-documenting numbers): these load the real geometry.yaml /
// standing_pose.yaml and assert only that the loaders accept the real schema.
// Renamed keys or missing fields would break these even if the inline fixtures
// still pass. Skips if hexa_description is not on the prefix path.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <set>
#include <string>

#include <ament_index_cpp/get_package_prefix.hpp>  // PackageNotFoundError
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "hexa_kinematics_cpp/joint_config.hpp"
#include "hexa_kinematics_cpp/leg_specs.hpp"

namespace {

std::string descriptionConfigDir() {
  try {
    return ament_index_cpp::get_package_share_directory("hexa_description") +
           "/config";
  } catch (const ament_index_cpp::PackageNotFoundError&) {
    return "";
  }
}

TEST(RealConfig, LoadLegSpecsAgainstRealGeometry) {
  const std::string dir = descriptionConfigDir();
  if (dir.empty()) GTEST_SKIP() << "hexa_description is not installed";
  const auto specs = hexa_kinematics::load_leg_specs(dir + "/geometry.yaml");

  std::set<std::string> keys;
  for (const auto& [name, spec] : specs) keys.insert(name);
  std::set<std::string> expected(hexa_kinematics::LEG_NAMES.begin(),
                                 hexa_kinematics::LEG_NAMES.end());
  EXPECT_EQ(keys, expected);

  for (const auto& [name, spec] : specs) {
    EXPECT_GT(spec.coxa_len, 0.0) << name;
    EXPECT_GT(spec.femur_len, 0.0) << name;
    EXPECT_GT(spec.tibia_len, 0.0) << name;
    EXPECT_TRUE(std::isfinite(spec.mount_xyz[0]) &&
                std::isfinite(spec.mount_xyz[1]) &&
                std::isfinite(spec.mount_xyz[2]))
        << name;
    EXPECT_TRUE(std::isfinite(spec.mount_yaw)) << name;
  }
}

TEST(RealConfig, LoadJointLimitsAgainstRealGeometry) {
  const std::string dir = descriptionConfigDir();
  if (dir.empty()) GTEST_SKIP() << "hexa_description is not installed";
  const auto limits = hexa_kinematics::load_joint_limits(dir + "/geometry.yaml");

  std::set<std::string> keys;
  for (const auto& [joint_type, lim] : limits) keys.insert(joint_type);
  EXPECT_EQ(keys, (std::set<std::string>{"coxa", "femur", "tibia"}));

  for (const auto& [joint_type, lim] : limits) {
    EXPECT_LE(lim.lower, lim.center) << joint_type;
    EXPECT_LE(lim.center, lim.upper) << joint_type;
    EXPECT_GT(lim.effort, 0.0) << joint_type;
    EXPECT_GT(lim.velocity, 0.0) << joint_type;
  }
}

TEST(RealConfig, LoadStandingPoseAgainstRealYaml) {
  const std::string dir = descriptionConfigDir();
  if (dir.empty()) GTEST_SKIP() << "hexa_description is not installed";
  const auto angles = hexa_kinematics::load_standing_pose(
      dir + "/standing_pose.yaml", dir + "/geometry.yaml");
  for (double a : angles) EXPECT_TRUE(std::isfinite(a));

  const auto limits = hexa_kinematics::load_joint_limits(dir + "/geometry.yaml");
  const std::array<std::string, 3> types = {"coxa", "femur", "tibia"};
  for (std::size_t i = 0; i < types.size(); ++i) {
    const auto& lim = limits.at(types[i]);
    EXPECT_LE(lim.lower, angles[i]) << types[i];
    EXPECT_LE(angles[i], lim.upper) << types[i];
  }
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
