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
# Asymmetry to note: python3-gpiozero / python3-lgpio are installed in the
# runtime stage only. Building an ament_python package never imports it, so the
# builder has no use for them and they would only pad this layer.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ccache \
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
# The target-agnostic control brain and face policy live outside src/ and are
# compiled directly by hexa_locomotion / hexa_pico_bridge / hexa_display via
# ${REPO_ROOT}/shared/..., so the build context needs them too.
COPY shared/ /workspace/shared/

# Drop the sim-only package and the standalone host test harness before
# building — the former's Gazebo deps don't have to exist in this image, the
# latter is its own CMake project that colcon would otherwise pick up out of
# shared/ and that installs nothing. The runtime stage uses --merge-install,
# not the symlinked dev layout, so install/ is fully self-contained afterwards.
RUN rm -rf /workspace/src/hexa_simulation /workspace/shared/motion_core/test

# Pinned, not defaulted: ccache 4.x picks $XDG_CACHE_HOME/ccache, which would
# land outside the mount below and be discarded with the layer.
ENV CCACHE_DIR=/root/.ccache

# Cache mounts, not layer content: every compile here is emulated when
# cross-built from x86, so incremental state is worth carrying across builds.
# sharing=locked on build/ — concurrent builds would corrupt one tree.
# `./hexa deploy build --fresh` drops both.
RUN --mount=type=cache,target=/root/.ccache,sharing=shared \
    --mount=type=cache,target=/workspace/build,sharing=locked \
    . /opt/ros/jazzy/setup.sh \
    && colcon build \
        --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
                     -DCMAKE_C_COMPILER_LAUNCHER=ccache \
                     -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
    && ccache -s \
    && rm -rf /workspace/log

# ---------------------------------------------------------------------------
# Stage 2 — runtime
# ---------------------------------------------------------------------------
FROM ros:jazzy-ros-core AS runtime

ENV DEBIAN_FRONTEND=noninteractive

# Runtime-only equivalents of the builder apt set, on the ros-core base. No
# compilers, no headers, no Gazebo, no GUI. Each ros-jazzy-* below pulls its
# own transitive runtime deps, so ros-core carries nothing it doesn't need.
# yaml-cpp is the runtime .so the hexa_hardware plugin links against.
# python3-gpiozero + python3-lgpio drive hexa_buttons' two front-panel GPIO
# buttons. lgpio is the chardev-based backend; gpiozero >= 2.0 is what carries
# Pi 5 board data (noble ships 2.0.1). Both live in universe.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libyaml-cpp0.8 \
        libusb-1.0-0 \
        python3-numpy \
        python3-serial \
        python3-yaml \
        python3-aiohttp \
        python3-gpiozero \
        python3-lgpio \
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

# Belt and braces. hexa_buttons already passes an explicit LGPIOFactory, so this
# changes nothing today — but gpiozero's default factory search ends in
# `native`, which maps BCM283x registers directly and on a Pi 5's RP1 reads
# garbage rather than failing. Pinning it here means any future gpiozero use in
# this image cannot land there silently.
ENV GPIOZERO_PIN_FACTORY=lgpio

WORKDIR /workspace

COPY --from=builder /workspace/install/ /workspace/install/
COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["ros2", "launch", "hexa_bringup", "bringup.launch.py"]
