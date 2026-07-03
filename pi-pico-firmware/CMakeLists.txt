cmake_minimum_required(VERSION 3.13)

# ── Board / SDK ────────────────────────────────────────────────────────────
# Raspberry Pi Pico 2 W (RP2350, dual Cortex-M33, single-precision FPU).
# Bluepad32 + RP2350 require Pico SDK >= 2.1 (see README).
set(PICO_BOARD pico2_w CACHE STRING "Target board")

# ── enable_usb_debugging ───────────────────────────────────────────────────
# USB-CDC debug logging (all HEXA_DBG output + the USB stdio stack itself). ON
# for bench development; turn OFF (-DHEXA_ENABLE_USB_DEBUGGING=OFF) for on-robot
# / low-jitter runs so a blocking CDC write never stalls the 200 Hz control tick.
option(HEXA_ENABLE_USB_DEBUGGING "USB-CDC debug logging" ON)

# Pull in the SDK (locate via PICO_SDK_PATH, or fetch the pinned tag from git).
include(pico_sdk_import.cmake)

# ── Bluepad32 (Bluetooth gamepad, part 02) ─────────────────────────────────
# Bare Pico SDK integration: the uni.h C "platform" API, NOT the Arduino
# BP32/ControllerPtr API (that one needs the arduino-pico core). Vendor
# Bluepad32 as a git submodule under external/bluepad32, or point BLUEPAD32_ROOT
# at a checkout. Needs a Bluepad32 with RP2350 / Pico 2 W support and Pico
# SDK >= 2.1. Included before project()/pico_sdk_init() per its import contract.
if (DEFINED ENV{BLUEPAD32_ROOT})
    set(BLUEPAD32_ROOT $ENV{BLUEPAD32_ROOT})
else ()
    set(BLUEPAD32_ROOT ${CMAKE_CURRENT_LIST_DIR}/external/bluepad32)
endif ()
message(STATUS "Bluepad32 root: ${BLUEPAD32_ROOT}")
include(${BLUEPAD32_ROOT}/src/components/bluepad32/bluepad32_import.cmake)

project(hexa_pico_firmware C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

# ── Build-time config codegen (part 04) ────────────────────────────────────
# The Pico has no filesystem, so the repo's runtime YAML config is baked into
# src/config_generated.hpp by tools/gen_config.py before the build. Same header
# the host test harness (test/host) generates, so host and target share one
# baked config. Re-runs whenever the generator or any source YAML changes.
find_program(PYTHON3 python3 REQUIRED)
set(HEXA_REPO_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)
set(HEXA_GEN_CONFIG ${CMAKE_CURRENT_LIST_DIR}/src/config_generated.hpp)
set(HEXA_CONFIG_YAMLS
    ${HEXA_REPO_ROOT}/src/hexa_description/config/geometry.yaml
    ${HEXA_REPO_ROOT}/src/hexa_description/config/standing_pose.yaml
    ${HEXA_REPO_ROOT}/src/hexa_gait_cpp/config/gait.yaml
    ${HEXA_REPO_ROOT}/src/hexa_teleop/config/teleop_joy.yaml
    ${HEXA_REPO_ROOT}/src/hexa_posture/config/posture.yaml
    ${HEXA_REPO_ROOT}/src/hexa_control/config/control.yaml
    ${HEXA_REPO_ROOT}/src/hexa_hardware/config/hardware.yaml
    ${HEXA_REPO_ROOT}/src/hexa_webteleop/config/webteleop.yaml
    ${HEXA_REPO_ROOT}/src/hexa_display/config/display.yaml)
add_custom_command(
    OUTPUT ${HEXA_GEN_CONFIG}
    COMMAND ${PYTHON3} ${CMAKE_CURRENT_LIST_DIR}/tools/gen_config.py
            --repo-root ${HEXA_REPO_ROOT} --out ${HEXA_GEN_CONFIG}
    DEPENDS ${CMAKE_CURRENT_LIST_DIR}/tools/gen_config.py ${HEXA_CONFIG_YAMLS}
    COMMENT "Generating config_generated.hpp from repo YAMLs"
    VERBATIM)
add_custom_target(hexa_gen_config DEPENDS ${HEXA_GEN_CONFIG})

# ── Firmware executable ────────────────────────────────────────────────────
add_executable(hexa_pico
    src/main.cpp
    src/pipeline.cpp                 # target-agnostic 50 Hz control brain (part 10)
    src/bt_teleop.cpp
    src/joy_mapping.cpp              # gamepad -> high-level command (part 07)
    src/control.cpp                  # velocity shaping (scale + slew) (part 07)
    src/servo_out.cpp                # Servo 2040 Chica link over UART (part 03)
    src/kinematics/leg_ik.cpp        # FK/IK, float fork (part 05)
    src/kinematics/body_transform.cpp # body<->leg + BodyPose compose (part 05)
    src/gait/clock.cpp               # gait engine, float fork (part 06)
    src/gait/trajectory.cpp
    src/gait/gaits/base.cpp
    src/gait/gaits/registry.cpp
    src/gait/engagement.cpp
    src/gait/pause.cpp
    src/gait/stand_transition.cpp
    src/gait/reseat.cpp
    src/gait/limits.cpp
    src/gait/engine.cpp
    src/posture/pose.cpp             # body-pose algebra, float fork (part 08)
    src/posture/animations.cpp       # animation stack (part 08)
    src/posture/posture.cpp          # posture controller / signal derivation (part 08)
    src/supervisor.cpp               # failsafes / telemetry / status LED (part 09)
)
add_dependencies(hexa_pico hexa_gen_config)

# btstack_config.h lives in the firmware root; BTstack (pulled in by Bluepad32)
# locates it on the include path. src/ carries the shared foundations
# (vec3.hpp, leg_index.hpp, generated config — part 04).
target_include_directories(hexa_pico PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/src)

# C++ exceptions: IK throws UnreachableTarget, reseat throws invalid_argument
# (see overview part 00 / kinematics part 05). Enabled globally so the forked
# libs behave identically to the ROS2 sources.
target_compile_options(hexa_pico PRIVATE -fexceptions)

# Single-precision FPU: keep doubles from silently sneaking in.
target_compile_options(hexa_pico PRIVATE -Wdouble-promotion)

# enable_usb_debugging: seen by src/dbg.hpp's HEXA_DBG / HEXA_DBG_INIT macros.
target_compile_definitions(hexa_pico PRIVATE
    HEXA_ENABLE_USB_DEBUGGING=$<BOOL:${HEXA_ENABLE_USB_DEBUGGING}>)

target_link_libraries(hexa_pico
    pico_stdlib                             # core runtime, timers, GPIO, flash
    hardware_uart                           # Servo 2040 Chica link (part 03)
    pico_cyw43_arch_threadsafe_background   # CYW43 driver + onboard LED; runs the
                                            # BTstack run loop in the background so
                                            # the cooperative loop stays clean
    bluepad32                               # Bluepad32 + BTstack HID host (part 02)
)

# Debug channel: USB-CDC only (no UART stdio — GP0/GP1 UART is reserved for the
# Servo 2040 Chica link in part 03). Gated by enable_usb_debugging: OFF drops the
# USB stdio stack entirely (no enumeration, no blocking CDC writes).
pico_enable_stdio_usb(hexa_pico $<BOOL:${HEXA_ENABLE_USB_DEBUGGING}>)
pico_enable_stdio_uart(hexa_pico 0)

# Emit .uf2 / .bin / .hex / map / disassembly.
pico_add_extra_outputs(hexa_pico)
