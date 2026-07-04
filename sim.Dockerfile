# syntax=docker/dockerfile:1.7
FROM osrf/ros:jazzy-desktop

ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=dev

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-controller-manager \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-xacro \
        ros-jazzy-joint-state-publisher-gui \
        python3-colcon-common-extensions \
        python3-serial \
        python3-vcstool \
        python3-aiohttp \
        build-essential \
        git \
        vim \
        less \
        sudo \
        usbutils \
    && rm -rf /var/lib/apt/lists/*

# ── Pi Pico firmware toolchain (pi-pico-firmware/) ─────────────────────────
# The RP2350 firmware builds inside this same container — no host ARM toolchain
# needed. We bake the cross toolchain + a pinned Pico SDK + Bluepad32 + picotool
# so `hexa deploy --pico` (and a raw cmake build) works offline. See
# pi-pico-firmware/README.md "Toolchain".
#
# PICO_SDK_REF is fixed at 2.1.1 (RP2350 + Bluepad32 require >= 2.1.0).
# (BLUEPAD32_REF is declared lower, next to its clone, so bumping it doesn't
# invalidate the cached ARM-toolchain and Pico SDK layers.)
ARG PICO_SDK_REF=2.1.1

RUN apt-get update && apt-get install -y --no-install-recommends \
        gcc-arm-none-eabi \
        libnewlib-arm-none-eabi \
        libstdc++-arm-none-eabi-newlib \
        cmake \
        pkg-config \
        libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

# Pico SDK (baked, pinned), with only the submodules the firmware pulls in:
# tinyusb (USB-CDC stdio), cyw43-driver + lwip (cyw43_arch_threadsafe_background
# / onboard LED), btstack + mbedtls (Bluepad32 HID host). The firmware's
# pico_sdk_import.cmake picks this up via PICO_SDK_PATH.
ENV PICO_SDK_PATH=/opt/pico-sdk
RUN git clone --depth 1 --branch ${PICO_SDK_REF} \
        https://github.com/raspberrypi/pico-sdk ${PICO_SDK_PATH} \
    && git -C ${PICO_SDK_PATH} submodule update --init --depth 1 \
        lib/tinyusb lib/cyw43-driver lib/lwip lib/btstack lib/mbedtls

# Bluepad32 (Bluetooth gamepad, part 02): the bare Pico SDK "platform" C API.
# Baked and pointed at by BLUEPAD32_ROOT so the firmware CMakeLists finds it
# without a repo submodule (its btstack_config.h + sdkconfig.h stay the sync
# point). Pinned to 4.2.0 — the release that added Pico 2 W (RP2350) support
# (needs Pico SDK 2.1). Override with `--build-arg BLUEPAD32_REF=<tag>`.
ARG BLUEPAD32_REF=4.2.0
ENV BLUEPAD32_ROOT=/opt/bluepad32
RUN git clone --depth 1 --branch ${BLUEPAD32_REF} \
        --recurse-submodules --shallow-submodules \
        https://github.com/ricardoquesada/bluepad32 ${BLUEPAD32_ROOT}

# picotool: pico_add_extra_outputs runs find_package(picotool) to package the
# RP2350 .uf2/.bin. Build it against the SDK and install so the firmware build
# resolves it offline (no per-build FetchContent). picotool_DIR is the config
# location cmake --install writes to.
ENV picotool_DIR=/usr/local/picotool
RUN git clone --depth 1 --branch ${PICO_SDK_REF} \
        https://github.com/raspberrypi/picotool /tmp/picotool \
    && cmake -S /tmp/picotool -B /tmp/picotool/build \
    && cmake --build /tmp/picotool/build -j"$(nproc)" \
    && cmake --install /tmp/picotool/build \
    && rm -rf /tmp/picotool

# Ubuntu 24.04 ships a default `ubuntu` user/group at UID/GID 1000. Rename
# it to ${USERNAME} (preserving the UID) so bind-mounted files stay owned by
# the host user; fall back to fresh creation if the base image ever changes.
RUN if id ubuntu >/dev/null 2>&1; then \
        groupmod --new-name ${USERNAME} ubuntu \
        && usermod --login ${USERNAME} --move-home --home /home/${USERNAME} ubuntu \
        && usermod --uid ${USER_UID} ${USERNAME} \
        && groupmod --gid ${USER_GID} ${USERNAME} ; \
    else \
        groupadd --gid ${USER_GID} ${USERNAME} \
        && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME} ; \
    fi \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

USER ${USERNAME}
WORKDIR /workspace

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo '[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash' >> /home/${USERNAME}/.bashrc \
    && echo '[ -f /workspace/docker/aliases.sh ] && source /workspace/docker/aliases.sh' >> /home/${USERNAME}/.bashrc

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
