# hexa_imu

MPU6500 driver for the Raspberry Pi. Reads the chip over I²C1 with raw Linux
`i2c-dev` ioctls (no external library), runs a complementary attitude filter,
and publishes `/imu/data` for body stabilization in
[`hexa_locomotion`](../hexa_locomotion/README.md). Real robot only: the sim
publishes the same topic from a Gazebo IMU sensor
([`hexa_simulation`](../hexa_simulation/README.md)).

## Layout

- `src/mpu6500.{hpp,cpp}` — the chip: WHO_AM_I check (`0x70`, or `0x71` for an
  MPU9250), range, DLPF and rate setup, and a 14-byte burst read in one
  repeated-start transaction. `decode_burst` and `scales_for` are free
  functions, so the test needs no device.
- `src/attitude_filter.{hpp,cpp}` — complementary filter. The gyro integrates;
  the accelerometer's gravity direction trims roll and pitch below
  `1/filter_tau`. The trim is skipped while `|accel|` is off g by more than
  `accel_reject`. Yaw is gyro-only and drifts. Also the mount rotation and the
  quaternion helper.
- `src/imu_node.cpp` — the ROS glue.
- `config/imu.yaml` — every parameter, with `enabled` for the launch gate.

## Topics

- **`/imu/data`** (`sensor_msgs/Imu`, sensor QoS, frame `imu_link`) —
  orientation, bias-corrected angular velocity and acceleration, in the body
  axes after `mount_rpy`. Yaw covariance is large; it drifts.

Service **`~/calibrate_gyro`** (`std_srvs/Trigger`) — collect the gyro bias
again over `bias_duration_s`. Keep the robot still.

## Behaviour

- At start the node averages the gyro over `bias_duration_s` and publishes
  nothing until it is done. The robot is on its belly at boot, so it is still.
- An open or read error is logged (throttled), the bus is closed, and the next
  tick tries again. The node never exits on a missing chip.
- `mount_rpy` rotates the sensor axes into `imu_link`, which is aligned with
  `base_link`. A small trim here also levels a mount that is not flat.

## Hardware

VCC to 3V3, GND to GND, SDA to GPIO2 (pin 3), SCL to GPIO3 (pin 5), AD0 to GND
for address `0x68`. Enable `dtparam=i2c_arm=on` and
`dtparam=i2c_arm_baudrate=400000`; see
[`docs/pi-setup.md`](../../docs/pi-setup.md).

## Tests

`test/test_imu.cpp` — burst decode and scales, unsupported ranges, the
filter's seed, gyro integration and drift trim, the accel reject, the mount
rotation and the quaternion.
