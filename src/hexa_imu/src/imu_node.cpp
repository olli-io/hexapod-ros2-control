// MPU6500 on the Pi's I2C bus -> attitude filter -> /imu/data. Real robot only;
// the sim publishes the same topic from a Gazebo IMU sensor. See README.md.

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "attitude_filter.hpp"
#include "mpu6500.hpp"

namespace {

using Imu = sensor_msgs::msg::Imu;
using Trigger = std_srvs::srv::Trigger;
using Clock = std::chrono::steady_clock;

class ImuNode : public rclcpp::Node {
 public:
  ImuNode()
      : rclcpp::Node("imu_node"),
        device_(device_config()),
        filter_(hexa::imu::AttitudeFilterConfig{
            declare_parameter<double>("filter_tau", 0.5),
            declare_parameter<double>("accel_reject", 0.15)}) {
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    const auto mount =
        declare_parameter<std::vector<double>>("mount_rpy", {0.0, 0.0, 0.0});
    if (mount.size() != 3) {
      throw std::invalid_argument("mount_rpy must have 3 entries");
    }
    mount_ = {mount[0], mount[1], mount[2]};
    bias_duration_s_ = declare_parameter<double>("bias_duration_s", 2.0);
    const double orientation_sd =
        declare_parameter<double>("orientation_stddev", 0.01);
    const double gyro_sd = declare_parameter<double>("gyro_stddev", 0.002);
    const double accel_sd = declare_parameter<double>("accel_stddev", 0.05);
    const double rate_hz = declare_parameter<double>("rate_hz", 200.0);

    msg_.header.frame_id = frame_id_;
    for (std::size_t i = 0; i < 3; ++i) {
      msg_.angular_velocity_covariance[i * 4] = gyro_sd * gyro_sd;
      msg_.linear_acceleration_covariance[i * 4] = accel_sd * accel_sd;
    }
    msg_.orientation_covariance[0] = orientation_sd * orientation_sd;
    msg_.orientation_covariance[4] = orientation_sd * orientation_sd;
    // Yaw is gyro-only and drifts.
    msg_.orientation_covariance[8] = 1.0e3;

    pub_ = create_publisher<Imu>("/imu/data", rclcpp::SensorDataQoS());
    srv_calibrate_ = create_service<Trigger>(
        "~/calibrate_gyro",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> res) {
          start_bias();
          res->success = true;
          res->message = "collecting gyro bias; keep the robot still";
        });

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz), [this]() { on_tick(); });
    start_bias();
    RCLCPP_INFO(get_logger(), "hexa_imu up: %s @ 0x%02x -> /imu/data (%.0f Hz)",
                get_parameter("bus").as_string().c_str(),
                static_cast<int>(get_parameter("address").as_int()), rate_hz);
  }

 private:
  hexa::imu::Mpu6500Config device_config() {
    hexa::imu::Mpu6500Config c;
    c.bus = declare_parameter<std::string>("bus", c.bus);
    c.address = static_cast<int>(declare_parameter<int>("address", c.address));
    c.accel_range_g = static_cast<int>(
        declare_parameter<int>("accel_range_g", c.accel_range_g));
    c.gyro_range_dps = static_cast<int>(
        declare_parameter<int>("gyro_range_dps", c.gyro_range_dps));
    c.dlpf_hz = static_cast<int>(declare_parameter<int>("dlpf_hz", c.dlpf_hz));
    c.sample_rate_hz = static_cast<int>(
        declare_parameter<int>("sample_rate_hz", c.sample_rate_hz));
    return c;
  }

  void start_bias() {
    bias_sum_ = {};
    bias_count_ = 0;
    bias_until_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                     std::chrono::duration<double>(bias_duration_s_));
    filter_.reset();
  }

  void on_tick() {
    if (!device_.is_open()) {
      try {
        device_.open();
        RCLCPP_INFO(get_logger(), "MPU6500 configured");
      } catch (const std::exception& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "MPU6500 unavailable (%s); retrying", ex.what());
        return;
      }
    }

    hexa::imu::RawSample raw;
    try {
      raw = device_.read();
    } catch (const std::exception& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "MPU6500 read failed (%s); reopening", ex.what());
      device_.close();
      return;
    }
    const auto now = Clock::now();
    const auto accel =
        hexa::imu::rotate_rpy(raw.accel, mount_[0], mount_[1], mount_[2]);
    auto gyro = hexa::imu::rotate_rpy(raw.gyro, mount_[0], mount_[1], mount_[2]);

    if (now < bias_until_) {
      for (std::size_t i = 0; i < 3; ++i) {
        bias_sum_[i] += gyro[i];
      }
      ++bias_count_;
      last_ = now;
      return;
    }
    if (bias_count_ > 0) {
      for (std::size_t i = 0; i < 3; ++i) {
        bias_[i] = bias_sum_[i] / bias_count_;
      }
      RCLCPP_INFO(get_logger(), "gyro bias [%.4f %.4f %.4f] rad/s from %d samples",
                  bias_[0], bias_[1], bias_[2], bias_count_);
      bias_count_ = 0;
    }
    for (std::size_t i = 0; i < 3; ++i) {
      gyro[i] -= bias_[i];
    }

    const double dt = std::chrono::duration<double>(now - last_).count();
    last_ = now;
    const hexa::imu::Attitude& att = filter_.update(accel, gyro, dt);
    const auto q = hexa::imu::quaternion_from_rpy(att.roll, att.pitch, att.yaw);

    msg_.header.stamp = get_clock()->now();
    msg_.orientation.x = q[0];
    msg_.orientation.y = q[1];
    msg_.orientation.z = q[2];
    msg_.orientation.w = q[3];
    msg_.angular_velocity.x = gyro[0];
    msg_.angular_velocity.y = gyro[1];
    msg_.angular_velocity.z = gyro[2];
    msg_.linear_acceleration.x = accel[0];
    msg_.linear_acceleration.y = accel[1];
    msg_.linear_acceleration.z = accel[2];
    pub_->publish(msg_);
  }

  hexa::imu::Mpu6500 device_;
  hexa::imu::AttitudeFilter filter_;
  std::string frame_id_;
  std::array<double, 3> mount_{};
  double bias_duration_s_ = 0.0;
  std::array<double, 3> bias_{};
  std::array<double, 3> bias_sum_{};
  int bias_count_ = 0;
  Clock::time_point bias_until_{};
  Clock::time_point last_{};
  Imu msg_;
  rclcpp::Publisher<Imu>::SharedPtr pub_;
  rclcpp::Service<Trigger>::SharedPtr srv_calibrate_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuNode>());
  rclcpp::shutdown();
  return 0;
}
