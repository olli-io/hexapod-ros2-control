# syntax=docker/dockerfile:1.7
#
# Robot (production) image for the hexapod (Raspberry Pi 3, ARM64).
# Two-stage: a fat builder on ros-base (colcon + ament + headers), a slim
# runtime on ros-core that only carries the compiled install/ tree plus its
# runtime apt deps. ros-core is the smallest ROS tier; every runtime ROS
# package is installed explicitly below, so apt pulls the transitive deps
# (tf2/urdf/kdl via robot-state-publisher, the ros2_control runtime via
# controller-manager) that ros-base would otherwise have pre-baked.
#
# Build on the workstation via `./hexa deploy build`, which wraps:
#   docker buildx build --platform linux/arm64 -f robot.Dockerfile ...

# ---------------------------------------------------------------------------
# Stage 1 — builder
# ---------------------------------------------------------------------------
FROM ros:jazzy-ros-base AS builder

ENV DEBIAN_FRONTEND=noninteractive

# Build tooling + every build-time dep declared across the prod packages.
# hexa_simulation is deliberately excluded from the build below, so its
# Gazebo deps (ros-gz, gz-ros2-control) do not need to be installed here.
# build-essential, cmake, git and colcon come baked into ros-base; only the
# ROS/apt deps the base does *not* carry are listed explicitly.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ament-cmake \
        ros-jazzy-ament-cmake-gtest \
        ros-jazzy-ament-index-cpp \
        ros-jazzy-controller-manager \
        ros-jazzy-hardware-interface \
        ros-jazzy-joint-state-broadcaster \
        ros-jazzy-pluginlib \
        ros-jazzy-position-controllers \
        ros-jazzy-rclcpp \
        ros-jazzy-rclcpp-lifecycle \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-rosidl-default-generators \
        ros-jazzy-rosidl-default-runtime \
        ros-jazzy-sensor-msgs \
        ros-jazzy-xacro \
        libyaml-cpp-dev \
        libusb-1.0-0-dev \
        python3-numpy \
        python3-serial \
        python3-yaml \
        python3-aiohttp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY src/ /workspace/src/

# Drop the sim-only package before building so its Gazebo deps don't have to
# exist in this image. The runtime stage uses --merge-install, not the
# symlinked dev layout, so install/ is fully self-contained afterwards.
RUN rm -rf /workspace/src/hexa_simulation

RUN . /opt/ros/jazzy/setup.sh \
    && colcon build \
        --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /workspace/build /workspace/log

# ---------------------------------------------------------------------------
# Stage 2 — runtime
# ---------------------------------------------------------------------------
FROM ros:jazzy-ros-core AS runtime

ENV DEBIAN_FRONTEND=noninteractive

# Runtime-only equivalents of the builder apt set, on the ros-core base. No
# compilers, no headers, no Gazebo, no GUI. Each ros-jazzy-* below pulls its
# own transitive runtime deps, so ros-core carries nothing it doesn't need.
# yaml-cpp is the runtime .so the hexa_hardware plugin links against.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libyaml-cpp0.8 \
        libusb-1.0-0 \
        python3-numpy \
        python3-serial \
        python3-yaml \
        python3-aiohttp \
        ros-jazzy-ament-index-cpp \
        ros-jazzy-controller-manager \
        ros-jazzy-hardware-interface \
        ros-jazzy-joint-state-broadcaster \
        ros-jazzy-pluginlib \
        ros-jazzy-position-controllers \
        ros-jazzy-rclcpp \
        ros-jazzy-rclcpp-lifecycle \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-rosidl-default-runtime \
        ros-jazzy-sensor-msgs \
        ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY --from=builder /workspace/install/ /workspace/install/
COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["ros2", "launch", "hexa_bringup", "bringup.launch.py"]
