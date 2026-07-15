#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "hexa_hardware/joint_calibration.hpp"

namespace hh = hexa_hardware;

namespace {

// Write `contents` to a uniquely-named temp file and return its path. Caller
// removes it.
std::string write_temp(const std::string& stem, const std::string& contents) {
  const auto path = std::filesystem::temp_directory_path() / stem;
  std::ofstream f(path);
  f << contents;
  f.close();
  return path.string();
}

// Build a servo_calibration.yaml body from endpoint magnitudes; pins are
// assigned 1..N in order to satisfy the loader's pin == index+1 rule.
std::string cal_yaml(const std::vector<std::pair<int, int>>& endpoints) {
  std::string s = "calibration_values:\n";
  for (std::size_t i = 0; i < endpoints.size(); ++i) {
    s += "  - { pin: " + std::to_string(i + 1) +
         ", us_at_plus_45: " + std::to_string(endpoints[i].first) +
         ", us_at_minus_45: " + std::to_string(endpoints[i].second) + " }\n";
  }
  return s;
}

// Load from an inline hardware.yaml + calibration.yaml pair, cleaning up both.
hh::HardwareConfig load(const std::string& stem, const std::string& hw_yaml,
                        const std::string& calibration_yaml) {
  const auto hw = write_temp(stem + "_hw.yaml", hw_yaml);
  const auto cal = write_temp(stem + "_cal.yaml", calibration_yaml);
  hh::HardwareConfig cfg;
  try {
    cfg = hh::load_hardware_config(hw, cal);
  } catch (...) {
    std::filesystem::remove(hw);
    std::filesystem::remove(cal);
    throw;
  }
  std::filesystem::remove(hw);
  std::filesystem::remove(cal);
  return cfg;
}

}  // namespace

TEST(JointCalibration, CenterPulseAtZero) {
  hh::JointCalibration jc;
  // Defaults: 1000 / 2000 → center = 1500 at θ = 0.
  EXPECT_EQ(jc.to_pulse_us(0.0), 1500);
}

TEST(JointCalibration, HitsCalibratedEndpoints) {
  hh::JointCalibration jc;
  EXPECT_EQ(jc.to_pulse_us(+M_PI / 4.0), 2000);
  EXPECT_EQ(jc.to_pulse_us(-M_PI / 4.0), 1000);
}

TEST(JointCalibration, DirectionInvertsTravel) {
  // direction = -1 mirrors travel about the center; endpoints stay magnitudes.
  hh::JointCalibration jc;
  jc.direction = -1;
  EXPECT_EQ(jc.to_pulse_us(0.0), 1500);
  EXPECT_EQ(jc.to_pulse_us(+M_PI / 4.0), 1000);
  EXPECT_EQ(jc.to_pulse_us(-M_PI / 4.0), 2000);
}

TEST(JointCalibration, EndpointOrderingDoesNotCarrySign) {
  // Slope is |plus - minus|, so swapping the endpoint magnitudes no longer
  // reverses direction — only `direction` does.
  hh::JointCalibration jc;
  jc.us_at_plus_45 = 1000;
  jc.us_at_minus_45 = 2000;
  EXPECT_EQ(jc.to_pulse_us(+M_PI / 4.0), 2000);
  EXPECT_EQ(jc.to_pulse_us(-M_PI / 4.0), 1000);
}

TEST(JointCalibration, AsymmetricEndpoints) {
  // Servo with measured trim: +45° at 1900, -45° at 1100 → center 1500,
  // but slope = (1900 - 1100) / (π/2) = 509.3 µs/rad.
  hh::JointCalibration jc;
  jc.us_at_plus_45 = 1900;
  jc.us_at_minus_45 = 1100;
  EXPECT_EQ(jc.to_pulse_us(+M_PI / 4.0), 1900);
  EXPECT_EQ(jc.to_pulse_us(-M_PI / 4.0), 1100);
  EXPECT_EQ(jc.to_pulse_us(0.0), 1500);
}

TEST(JointCalibration, ClampToMinMax) {
  hh::JointCalibration jc;
  jc.min_us = 1000;
  jc.max_us = 2000;
  EXPECT_EQ(jc.to_pulse_us(+10.0), 2000);
  EXPECT_EQ(jc.to_pulse_us(-10.0), 1000);
}

TEST(JointCalibration, AssemblyOffsetShiftsCenter) {
  // Servo centered (1500 µs) when URDF angle is +0.3 rad — i.e. the
  // joint sits at +0.3 rad in its at-rest assembled state.
  hh::JointCalibration jc;
  jc.urdf_rad_at_center = 0.3;
  EXPECT_EQ(jc.to_pulse_us(0.3), 1500);
  EXPECT_EQ(jc.to_pulse_us(0.3 + M_PI / 4.0), 2000);
  EXPECT_EQ(jc.to_pulse_us(0.3 - M_PI / 4.0), 1000);
}

TEST(LoadHardwareConfig, ParsesYaml) {
  const std::string hw = R"(connection:
  type: uart
  device: /dev/ttyUSB0
  baud: 230400
parser:
  type: servo2040
  get_period_ticks: 5
deg_at_center:
  coxa: 30.0
  femur: 35.0
  tibia: 68.0
joints:
  l_front_coxa_joint:  { pin: 1, min_us: 600, max_us: 2400 }
  l_front_femur_joint: { pin: 2, min_us: 600, max_us: 2400 }
  l_front_tibia_joint: { pin: 3, min_us: 600, max_us: 2400 }
)";
  // Endpoint magnitudes come from the calibration file (pin → index-1).
  const auto cfg = load("parses", hw,
                        cal_yaml({{1900, 1100}, {2000, 1000}, {2000, 1000}}));
  EXPECT_EQ(cfg.connection.type, "uart");
  EXPECT_EQ(cfg.connection.device, "/dev/ttyUSB0");
  EXPECT_EQ(cfg.connection.baud, 230400);
  EXPECT_EQ(cfg.parser.type, "servo2040");
  EXPECT_EQ(cfg.parser.get_period_ticks, 5);

  ASSERT_EQ(cfg.joints.count("l_front_coxa_joint"), 1u);
  const auto& coxa = cfg.joints.at("l_front_coxa_joint");
  EXPECT_EQ(coxa.joint_position, hh::JointPosition::Coxa);
  EXPECT_DOUBLE_EQ(coxa.us_at_plus_45, 1900.0);
  EXPECT_DOUBLE_EQ(coxa.us_at_minus_45, 1100.0);
  // coxa: urdf_rad = deg * π/180
  EXPECT_DOUBLE_EQ(coxa.urdf_rad_at_center, 30.0 * M_PI / 180.0);
  EXPECT_EQ(coxa.min_us, 600);
  EXPECT_EQ(coxa.max_us, 2400);

  ASSERT_EQ(cfg.joints.count("l_front_femur_joint"), 1u);
  const auto& femur = cfg.joints.at("l_front_femur_joint");
  EXPECT_EQ(femur.joint_position, hh::JointPosition::Femur);
  EXPECT_DOUBLE_EQ(femur.us_at_plus_45, 2000.0);
  // femur: urdf_rad = -deg * π/180
  EXPECT_DOUBLE_EQ(femur.urdf_rad_at_center, -35.0 * M_PI / 180.0);

  ASSERT_EQ(cfg.joints.count("l_front_tibia_joint"), 1u);
  const auto& tibia = cfg.joints.at("l_front_tibia_joint");
  EXPECT_EQ(tibia.joint_position, hh::JointPosition::Tibia);
  // tibia: urdf_rad = π - deg * π/180
  EXPECT_DOUBLE_EQ(tibia.urdf_rad_at_center, M_PI - 68.0 * M_PI / 180.0);
}

TEST(LoadHardwareConfig, DegAtCenterOptional) {
  // Missing `deg_at_center` block means all positions default to 0 →
  // urdf_rad_at_center is 0 for coxa/femur and π for tibia.
  const std::string hw = R"(joints:
  l_front_coxa_joint:  { pin: 1, min_us: 600, max_us: 2400 }
  l_front_tibia_joint: { pin: 2, min_us: 600, max_us: 2400 }
)";
  const auto cfg = load("dac", hw, cal_yaml({{2000, 1000}, {2000, 1000}}));
  EXPECT_DOUBLE_EQ(cfg.joints.at("l_front_coxa_joint").urdf_rad_at_center, 0.0);
  EXPECT_DOUBLE_EQ(cfg.joints.at("l_front_tibia_joint").urdf_rad_at_center, M_PI);
}

TEST(LoadHardwareConfig, MapsJointNameToPosition) {
  // The segment comes from the name→position table, not a separate field.
  const std::string hw = R"(joints:
  r_rear_femur_joint: { pin: 1, min_us: 600, max_us: 2400 }
)";
  const auto cfg = load("derive", hw, cal_yaml({{2000, 1000}}));
  EXPECT_EQ(cfg.joints.at("r_rear_femur_joint").joint_position,
            hh::JointPosition::Femur);
}

TEST(LoadHardwareConfig, RejectsUnknownJointName) {
  // A name outside the canonical 6-leg set has no segment mapping.
  const std::string hw = R"(joints:
  mystery_joint: { pin: 1, min_us: 600, max_us: 2400 }
)";
  EXPECT_THROW(load("noseg", hw, cal_yaml({{2000, 1000}})), std::runtime_error);
}

TEST(LoadHardwareConfig, RejectsEqualEndpoints) {
  // The endpoints-differ check now runs on the merged calibration values.
  const std::string hw = R"(joints:
  l_front_coxa_joint: { pin: 1, min_us: 600, max_us: 2400 }
)";
  EXPECT_THROW(load("equal", hw, cal_yaml({{1500, 1500}})), std::runtime_error);
}

TEST(LoadHardwareConfig, DirectionDefaultsToPlusOne) {
  const std::string hw = R"(joints:
  l_front_coxa_joint: { pin: 1, min_us: 600, max_us: 2400 }
)";
  const auto cfg = load("dir_def", hw, cal_yaml({{2000, 1000}}));
  EXPECT_EQ(cfg.joints.at("l_front_coxa_joint").direction, 1);
}

TEST(LoadHardwareConfig, ParsesReversedDirection) {
  const std::string hw = R"(joints:
  l_front_coxa_joint: { pin: 1, direction: -1, min_us: 600, max_us: 2400 }
)";
  const auto cfg = load("dir_rev", hw, cal_yaml({{2000, 1000}}));
  EXPECT_EQ(cfg.joints.at("l_front_coxa_joint").direction, -1);
}

TEST(LoadHardwareConfig, RejectsBadDirection) {
  const std::string hw = R"(joints:
  l_front_coxa_joint: { pin: 1, direction: 2, min_us: 600, max_us: 2400 }
)";
  EXPECT_THROW(load("dir_bad", hw, cal_yaml({{2000, 1000}})), std::runtime_error);
}

TEST(LoadHardwareConfig, RejectsPinWithoutCalibrationEntry) {
  // Joint pin 5 has no matching entry in a 3-entry calibration array.
  const std::string hw = R"(joints:
  l_front_coxa_joint: { pin: 5, min_us: 600, max_us: 2400 }
)";
  EXPECT_THROW(
      load("nopin", hw, cal_yaml({{2000, 1000}, {2000, 1000}, {2000, 1000}})),
      std::runtime_error);
}

TEST(LoadHardwareConfig, RejectsMissingCalibrationArray) {
  const std::string hw = R"(joints:
  l_front_coxa_joint: { pin: 1, min_us: 600, max_us: 2400 }
)";
  EXPECT_THROW(load("nocal", hw, "not_calibration_values: []\n"),
               std::runtime_error);
}

TEST(LoadHardwareConfig, RejectsPinIndexMismatch) {
  // Second entry claims pin 3 but sits at index 1 (pin 2).
  const std::string cal = R"(calibration_values:
  - { pin: 1, us_at_plus_45: 2000, us_at_minus_45: 1000 }
  - { pin: 3, us_at_plus_45: 2000, us_at_minus_45: 1000 }
)";
  const std::string hw = R"(joints:
  l_front_coxa_joint:  { pin: 1, min_us: 600, max_us: 2400 }
  l_front_femur_joint: { pin: 2, min_us: 600, max_us: 2400 }
)";
  EXPECT_THROW(load("mismatch", hw, cal), std::runtime_error);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
