#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>

#include "hexa_hardware/joint_calibration.hpp"

namespace hh = hexa_hardware;

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

TEST(JointCalibration, SwappingEndpointsInvertsDirection) {
  hh::JointCalibration jc;
  jc.us_at_plus_45 = 1000;
  jc.us_at_minus_45 = 2000;
  EXPECT_EQ(jc.to_pulse_us(+M_PI / 4.0), 1000);
  EXPECT_EQ(jc.to_pulse_us(-M_PI / 4.0), 2000);
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
  const auto path = std::filesystem::temp_directory_path() / "hexa_hw_test.yaml";
  {
    std::ofstream f(path);
    f << R"(connection:
  type: uart
  device: /dev/ttyUSB0
  baud: 230400
parser:
  type: servo2040
  get_period_ticks: 5
relay:
  pin: 7
aux:
  battery_voltage: { pin: 30, scale: 0.01 }
deg_at_center:
  coxa: 30.0
  femur: 35.0
  tibia: 68.0
joints:
  test_coxa:
    pin: 0
    joint_position: coxa
    us_at_plus_45: 1900
    us_at_minus_45: 1100
    min_us: 600
    max_us: 2400
  test_femur:
    pin: 1
    joint_position: femur
    us_at_plus_45: 2000
    us_at_minus_45: 1000
    min_us: 600
    max_us: 2400
  test_tibia:
    pin: 2
    joint_position: tibia
    us_at_plus_45: 2000
    us_at_minus_45: 1000
    min_us: 600
    max_us: 2400
)";
  }
  const auto cfg = hh::load_hardware_config(path.string());
  EXPECT_EQ(cfg.connection.type, "uart");
  EXPECT_EQ(cfg.connection.device, "/dev/ttyUSB0");
  EXPECT_EQ(cfg.connection.baud, 230400);
  EXPECT_EQ(cfg.parser.type, "servo2040");
  EXPECT_EQ(cfg.parser.get_period_ticks, 5);
  EXPECT_TRUE(cfg.relay_configured);
  EXPECT_EQ(cfg.relay_pin, 7);
  ASSERT_EQ(cfg.aux.count("battery_voltage"), 1u);
  EXPECT_EQ(cfg.aux.at("battery_voltage").pin, 30);

  ASSERT_EQ(cfg.joints.count("test_coxa"), 1u);
  const auto& coxa = cfg.joints.at("test_coxa");
  EXPECT_EQ(coxa.joint_position, hh::JointPosition::Coxa);
  EXPECT_DOUBLE_EQ(coxa.us_at_plus_45, 1900.0);
  EXPECT_DOUBLE_EQ(coxa.us_at_minus_45, 1100.0);
  // coxa: urdf_rad = deg * π/180
  EXPECT_DOUBLE_EQ(coxa.urdf_rad_at_center, 30.0 * M_PI / 180.0);
  EXPECT_EQ(coxa.min_us, 600);
  EXPECT_EQ(coxa.max_us, 2400);

  ASSERT_EQ(cfg.joints.count("test_femur"), 1u);
  const auto& femur = cfg.joints.at("test_femur");
  EXPECT_EQ(femur.joint_position, hh::JointPosition::Femur);
  // femur: urdf_rad = -deg * π/180
  EXPECT_DOUBLE_EQ(femur.urdf_rad_at_center, -35.0 * M_PI / 180.0);

  ASSERT_EQ(cfg.joints.count("test_tibia"), 1u);
  const auto& tibia = cfg.joints.at("test_tibia");
  EXPECT_EQ(tibia.joint_position, hh::JointPosition::Tibia);
  // tibia: urdf_rad = π - deg * π/180
  EXPECT_DOUBLE_EQ(tibia.urdf_rad_at_center, M_PI - 68.0 * M_PI / 180.0);

  std::filesystem::remove(path);
}

TEST(LoadHardwareConfig, DegAtCenterOptional) {
  // Missing `deg_at_center` block means all positions default to 0 →
  // urdf_rad_at_center is 0 for coxa/femur and π for tibia.
  const auto path = std::filesystem::temp_directory_path() / "hexa_hw_dac.yaml";
  {
    std::ofstream f(path);
    f << R"(joints:
  j_coxa:
    pin: 0
    joint_position: coxa
    us_at_plus_45: 2000
    us_at_minus_45: 1000
    min_us: 600
    max_us: 2400
  j_tibia:
    pin: 1
    joint_position: tibia
    us_at_plus_45: 2000
    us_at_minus_45: 1000
    min_us: 600
    max_us: 2400
)";
  }
  const auto cfg = hh::load_hardware_config(path.string());
  EXPECT_DOUBLE_EQ(cfg.joints.at("j_coxa").urdf_rad_at_center, 0.0);
  EXPECT_DOUBLE_EQ(cfg.joints.at("j_tibia").urdf_rad_at_center, M_PI);
  std::filesystem::remove(path);
}

TEST(LoadHardwareConfig, RejectsMissingJointPosition) {
  const auto path = std::filesystem::temp_directory_path() / "hexa_hw_nopos.yaml";
  {
    std::ofstream f(path);
    f << R"(joints:
  j:
    pin: 0
    us_at_plus_45: 2000
    us_at_minus_45: 1000
    min_us: 600
    max_us: 2400
)";
  }
  EXPECT_THROW(hh::load_hardware_config(path.string()), std::runtime_error);
  std::filesystem::remove(path);
}

TEST(LoadHardwareConfig, RejectsBadJointPosition) {
  const auto path = std::filesystem::temp_directory_path() / "hexa_hw_badpos.yaml";
  {
    std::ofstream f(path);
    f << R"(joints:
  j:
    pin: 0
    joint_position: elbow
    us_at_plus_45: 2000
    us_at_minus_45: 1000
    min_us: 600
    max_us: 2400
)";
  }
  EXPECT_THROW(hh::load_hardware_config(path.string()), std::runtime_error);
  std::filesystem::remove(path);
}

TEST(LoadHardwareConfig, RejectsEqualEndpoints) {
  const auto path = std::filesystem::temp_directory_path() / "hexa_hw_equal.yaml";
  {
    std::ofstream f(path);
    f << R"(joints:
  j:
    pin: 0
    joint_position: coxa
    us_at_plus_45: 1500
    us_at_minus_45: 1500
    min_us: 600
    max_us: 2400
)";
  }
  EXPECT_THROW(hh::load_hardware_config(path.string()), std::runtime_error);
  std::filesystem::remove(path);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
