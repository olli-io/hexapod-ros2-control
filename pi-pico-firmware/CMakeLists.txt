cmake_minimum_required(VERSION 3.13)

# ── Board / SDK ────────────────────────────────────────────────────────────
# Raspberry Pi Pico 2 W (RP2350, dual Cortex-M33, single-precision FPU).
# Bluepad32 + RP2350 require Pico SDK >= 2.1 (see README).
set(PICO_BOARD pico2_w CACHE STRING "Target board")

# C++ exceptions: the shared control brain throws/catches (IK UnreachableTarget,
# reseat/control invalid_argument — see part 00/05). The Pico SDK compiles C++
# with -fno-exceptions by default (via pico_cxx_options, pulled in by
# pico_stdlib), which would override a bare -fexceptions on our target. Flip the
# SDK's own toggle BEFORE pico_sdk_init() so it never adds -fno-exceptions and
# the forked libs behave like the ROS2 sources. RTTI stays off (-fno-rtti) — GCC
# exception type-matching doesn't need it, and it saves flash.
set(PICO_CXX_ENABLE_EXCEPTIONS 1)

# ── enable_usb_debugging ───────────────────────────────────────────────────
# USB-CDC debug logging (all HEXA_DBG output + the USB stdio stack itself). ON
# for bench development; turn OFF (-DHEXA_ENABLE_USB_DEBUGGING=OFF) for on-robot
# / low-jitter runs so a blocking CDC write never stalls the 200 Hz control tick.
option(HEXA_ENABLE_USB_DEBUGGING "USB-CDC debug logging" ON)

# Pull in the SDK (locate via PICO_SDK_PATH, or fetch the pinned tag from git).
include(pico_sdk_import.cmake)

# ── Bluepad32 (Bluetooth gamepad, part 02) ─────────────────────────────────
# Bare Pico SDK integration: the uni.h C "platform" API, NOT the Arduino
# BP32/ControllerPtr API (that one needs the arduino-pico core). Point
# BLUEPAD32_ROOT at a Bluepad32 checkout (the hexa-sim image bakes 4.2.0 and
# exports it; see sim.Dockerfile). Needs Bluepad32 with Pico 2 W / RP2350
# support and Pico SDK >= 2.1.
#
# Integration mirrors Bluepad32's own examples/pico_w: there is NO
# "import.cmake" — BTstack comes from the Pico SDK (BTSTACK_ROOT, set before
# project()), and the component is pulled in with add_subdirectory() *after*
# pico_sdk_init() (it defines the `bluepad32` target). See the component wiring
# just above target_link_libraries below.
if (DEFINED ENV{BLUEPAD32_ROOT})
    set(BLUEPAD32_ROOT $ENV{BLUEPAD32_ROOT})
else ()
    set(BLUEPAD32_ROOT ${CMAKE_CURRENT_LIST_DIR}/external/bluepad32)
endif ()
message(STATUS "Bluepad32 root: ${BLUEPAD32_ROOT}")
set(BTSTACK_ROOT ${PICO_SDK_PATH}/lib/btstack)

project(hexa_pico_firmware C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

# ── Build-time config codegen (part 04) ────────────────────────────────────
# The Pico has no filesystem, so the repo's runtime YAML config is baked into
# shared/motion_core/config_generated.hpp by motion_core/tools/gen_config.py
# before the build. Same header the host test harness and the ROS bridge /
# hexa_locomotion builds generate, so every target shares one baked config.
# Re-runs whenever the generator or any source YAML changes.
find_program(PYTHON3 python3 REQUIRED)
set(HEXA_REPO_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)
set(HEXA_SHARED ${HEXA_REPO_ROOT}/shared/motion_core)

set(HEXA_DISPLAY_CORE ${HEXA_REPO_ROOT}/shared/display_core)
set(HEXA_GEN_CONFIG ${HEXA_SHARED}/config_generated.hpp)
set(HEXA_CONFIG_YAMLS
    ${HEXA_REPO_ROOT}/src/hexa_description/config/geometry.yaml
    ${HEXA_REPO_ROOT}/src/hexa_description/config/tuning.yaml
    ${HEXA_REPO_ROOT}/src/hexa_teleop/config/teleop_joy.yaml
    ${HEXA_REPO_ROOT}/src/hexa_description/config/hardware.yaml
    ${HEXA_REPO_ROOT}/src/hexa_description/config/servo_calibration.yaml
    ${HEXA_REPO_ROOT}/src/hexa_webteleop/config/webteleop.yaml
    ${HEXA_REPO_ROOT}/src/hexa_display/config/display.yaml)
add_custom_command(
    OUTPUT ${HEXA_GEN_CONFIG}
    COMMAND ${PYTHON3} ${HEXA_SHARED}/tools/gen_config.py
            --repo-root ${HEXA_REPO_ROOT} --out ${HEXA_GEN_CONFIG}
    DEPENDS ${HEXA_SHARED}/tools/gen_config.py ${HEXA_CONFIG_YAMLS}
    COMMENT "Generating config_generated.hpp from repo YAMLs"
    VERBATIM)
add_custom_target(hexa_gen_config DEPENDS ${HEXA_GEN_CONFIG})

# ── Firmware executable ────────────────────────────────────────────────────
# The target-agnostic control brain lives in ../shared/motion_core (shared with
# the host tests, hexa_pico_bridge, and hexa_locomotion). This target composes
# the Pico hardware seam (src/main.cpp, bt_teleop.cpp, servo_out.cpp) around it.
add_executable(hexa_pico
    src/main.cpp
    src/bt_teleop.cpp
    src/servo_out.cpp                # Servo 2040 Chica link over UART (part 03)
    ${HEXA_SHARED}/pipeline.cpp      # target-agnostic 200 Hz control brain (part 10)
    ${HEXA_SHARED}/pipeline_config.cpp  # geometry+tuning config seam (baked() factory)
    ${HEXA_SHARED}/joy_mapping.cpp   # gamepad -> high-level command (part 07)
    ${HEXA_SHARED}/control.cpp       # velocity shaping (scale + slew) (part 07)
    ${HEXA_SHARED}/kinematics/leg_ik.cpp        # FK/IK, float fork (part 05)
    ${HEXA_SHARED}/kinematics/body_transform.cpp # body<->leg + BodyPose compose (part 05)
    ${HEXA_SHARED}/gait/clock.cpp    # gait engine, float fork (part 06)
    ${HEXA_SHARED}/gait/trajectory.cpp
    ${HEXA_SHARED}/gait/gaits/base.cpp
    ${HEXA_SHARED}/gait/gaits/registry.cpp
    ${HEXA_SHARED}/gait/engagement.cpp
    ${HEXA_SHARED}/gait/stand_transition.cpp
    ${HEXA_SHARED}/gait/reseat.cpp
    ${HEXA_SHARED}/gait/limits.cpp
    ${HEXA_SHARED}/gait/engine.cpp
    ${HEXA_SHARED}/gait/reversal.cpp
    ${HEXA_SHARED}/posture/pose.cpp        # body-pose algebra, float fork (part 08)
    ${HEXA_SHARED}/posture/animations.cpp  # animation stack (part 08)
    ${HEXA_SHARED}/posture/posture.cpp     # posture controller / signal derivation (part 08)
    ${HEXA_SHARED}/supervisor.cpp    # failsafes / telemetry / status LED (part 09)
)
add_dependencies(hexa_pico hexa_gen_config)

# Face: the shared display core (shared/display_core) + the Pico panel transport.
# Compiled directly, like motion_core, so the eyes rasterize bit-identically to
# the ROS node. u8g2 is globbed exactly as the ROS package does — the SDK already
# applies -ffunction-sections/-fdata-sections and --gc-sections, so the unused
# device drivers in u8g2_d_setup.c drop at link time.
file(GLOB U8G2_SRCS "${HEXA_DISPLAY_CORE}/u8g2/*.c")
target_sources(hexa_pico PRIVATE
    src/face.cpp                 # core0 policy + core1 render loop
    src/button.cpp               # front-panel button -> battery screen / pairing
    src/Sh1122PanelPico.cpp      # SH1122 over hardware SPI
    ${HEXA_DISPLAY_CORE}/core/EyeAnim.cpp        # eased gaze/blink state machine
    ${HEXA_DISPLAY_CORE}/core/EyeRaster.cpp      # 1-bit eye rasterizer
    ${HEXA_DISPLAY_CORE}/core/ExpressionController.cpp  # expressionName()/gazeName()
    ${HEXA_DISPLAY_CORE}/expression_policy.cpp      # pure expression/gaze policy
    ${HEXA_DISPLAY_CORE}/face_animation.cpp         # breathing/idling sequences
    ${HEXA_DISPLAY_CORE}/face_animation_runner.cpp  # face-animation clock
    ${HEXA_DISPLAY_CORE}/text_screen.cpp            # button's battery screen
    ${HEXA_DISPLAY_CORE}/fonts/hexa_text_font.c     # bundled Pixel Operator
    ${U8G2_SRCS})

# btstack_config.h lives in the firmware root; BTstack (pulled in by Bluepad32)
# locates it on the include path. ../shared/motion_core carries the shared
# foundations (vec3.hpp, leg_index.hpp, generated config — part 04) and the
# control brain; src/ still holds the Pico-only seam headers (dbg.hpp).
target_include_directories(hexa_pico PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${HEXA_SHARED}
    ${HEXA_DISPLAY_CORE}       # expression_policy / face_animation / frame_compare
    ${HEXA_DISPLAY_CORE}/core  # EyeAnim / EyeRaster / Expression
    ${HEXA_DISPLAY_CORE}/u8g2)

# C++ exceptions are enabled via PICO_CXX_ENABLE_EXCEPTIONS at the top of this
# file (the SDK-level toggle). Keep -fexceptions explicit on the target too as a
# belt-and-suspenders against any INTERFACE option ordering.
target_compile_options(hexa_pico PRIVATE -fexceptions)

# Single-precision FPU: keep doubles from silently sneaking in.
target_compile_options(hexa_pico PRIVATE -Wdouble-promotion)

# enable_usb_debugging: seen by src/dbg.hpp's HEXA_DBG / HEXA_DBG_INIT macros.
# U8X8_WITH_USER_PTR gives each u8x8 a user pointer (Sh1122PanelPico stashes
# `this` there). It changes struct u8x8_struct's layout, so it must reach the
# u8g2 C core and the panel identically — defining it once on the single target
# that compiles both makes a mismatch unrepresentable.
target_compile_definitions(hexa_pico PRIVATE
    HEXA_ENABLE_USB_DEBUGGING=$<BOOL:${HEXA_ENABLE_USB_DEBUGGING}>
    U8X8_WITH_USER_PTR)

# ── Bluepad32 component (part 02) ──────────────────────────────────────────
# Build the vendored Bluepad32 as a sub-library (the `bluepad32` target); it
# links BTstack from the Pico SDK. Its C sources #include the app-provided
# btstack_config.h + sdkconfig.h off the include path, so expose the firmware
# root (which holds both) to it with a directory-scoped include *before* the
# add_subdirectory — that propagates into the sub-library. The btstack/bluedroid
# include dirs mirror Bluepad32's examples/pico_w.
include_directories(
    ${CMAKE_CURRENT_LIST_DIR}                              # btstack_config.h + sdkconfig.h
    ${BTSTACK_ROOT}/src
    ${BTSTACK_ROOT}/3rd-party/bluedroid/encoder/include
    ${BTSTACK_ROOT}/3rd-party/bluedroid/decoder/include)
add_subdirectory(${BLUEPAD32_ROOT}/src/components/bluepad32 libbluepad32)

target_link_libraries(hexa_pico
    pico_stdlib                             # core runtime, timers, GPIO, flash
    hardware_uart                           # Servo 2040 Chica link (part 03)
    hardware_spi                            # SH1122 face panel on core1 (part 11)
    pico_multicore                          # core1 runs cyw43/BTstack (Bluepad32)
    # CYW43 driver + onboard LED. pico_cyw43_arch_none is the BT-only arch
    # (CYW43_LWIP=0, no Wi-Fi): despite the name it still runs the
    # threadsafe_background async context, so the BTstack run loop is serviced
    # in the background and the cooperative loop stays clean. It also matches the
    # arch Bluepad32's component links, so there's no cyw43_arch clash.
    pico_cyw43_arch_none
    pico_btstack_classic                    # BR/EDR HID host (PS4/PS5, part 02)
    pico_btstack_cyw43                      # BTstack <-> CYW43 HCI transport
    bluepad32                               # Bluepad32 gamepad platform (part 02)
)

# Debug channel: USB-CDC only (no UART stdio — GP0/GP1 UART is reserved for the
# Servo 2040 Chica link in part 03). Gated by enable_usb_debugging: OFF drops the
# USB stdio stack entirely (no enumeration, no blocking CDC writes).
pico_enable_stdio_usb(hexa_pico $<BOOL:${HEXA_ENABLE_USB_DEBUGGING}>)
pico_enable_stdio_uart(hexa_pico 0)

# Emit .uf2 / .bin / .hex / map / disassembly.
pico_add_extra_outputs(hexa_pico)
