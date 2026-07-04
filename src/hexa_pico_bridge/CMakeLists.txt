cmake_minimum_required(VERSION 3.16)
project(hexa_pico_bridge CXX)

# ── Gazebo-in-the-loop firmware bridge (plan part 10, Tier 3) ───────────────
#
# Links the target-agnostic control-brain sources (shared/hexa_pipeline) DIRECTLY
# — the same float source compiled for the RP2350 — and runs the real 200 Hz
# control tick against the simulated hexapod. The only thing that changes vs the
# Pico build is the hardware seam: input comes from /joy instead of Bluepad32,
# output goes to the sim controller instead of the Chica UART, and the clock is a
# std::chrono steady clock instead of time_us_64(). No #ifdefs — a link-time swap
# (see shared/hexa_pipeline/pipeline.hpp).

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ── Shared control-brain source tree ────────────────────────────────────────
# The target-agnostic firmware brain lives in shared/hexa_pipeline at the
# workspace root (shared with the Pico build, the host tests, and
# hexa_locomotion). REPO_ROOT feeds the config generator the same repo YAMLs
# every target bakes from. FW_SRC is kept as the variable name for the sources.
set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(FW_SRC ${REPO_ROOT}/shared/hexa_pipeline)
set(GEN_CONFIG ${FW_SRC}/config_generated.hpp)

# ── Build-time config codegen ───────────────────────────────────────────────
# Same baked config_generated.hpp the Pico firmware and the host harness use, so
# the sim brain runs the identical constants. Regenerated whenever the generator
# or any source YAML changes.
find_program(PYTHON3 python3 REQUIRED)
set(CONFIG_YAMLS
  ${REPO_ROOT}/src/hexa_description/config/geometry.yaml
  ${REPO_ROOT}/src/hexa_description/config/tuning.yaml
  ${REPO_ROOT}/src/hexa_teleop/config/teleop_joy.yaml
  ${REPO_ROOT}/src/hexa_hardware/config/hardware.yaml
  ${REPO_ROOT}/src/hexa_webteleop/config/webteleop.yaml
  ${REPO_ROOT}/src/hexa_display/config/display.yaml)
add_custom_command(
  OUTPUT ${GEN_CONFIG}
  COMMAND ${PYTHON3} ${FW_SRC}/tools/gen_config.py
          --repo-root ${REPO_ROOT} --out ${GEN_CONFIG}
  DEPENDS ${FW_SRC}/tools/gen_config.py ${CONFIG_YAMLS}
  COMMENT "Generating config_generated.hpp from repo YAMLs (hexa_pico_bridge)"
  VERBATIM)
add_custom_target(hexa_pico_bridge_gen_config DEPENDS ${GEN_CONFIG})

# ── Firmware pipeline sources ───────────────────────────────────────────────
# Everything the 50 Hz tick needs, minus the Pico-only seam impls (bt_teleop.cpp,
# servo_out.cpp, main.cpp). pipeline.cpp includes bt_teleop.hpp / servo_out.hpp
# only for their pure header constants (kNumAxes / kNumJoints), so there is no
# link dependency on the Pico UART / Bluepad32 code.
set(FW_PIPELINE_SOURCES
  ${FW_SRC}/pipeline.cpp
  ${FW_SRC}/control.cpp
  ${FW_SRC}/joy_mapping.cpp
  ${FW_SRC}/supervisor.cpp
  ${FW_SRC}/kinematics/leg_ik.cpp
  ${FW_SRC}/kinematics/body_transform.cpp
  ${FW_SRC}/gait/clock.cpp
  ${FW_SRC}/gait/trajectory.cpp
  ${FW_SRC}/gait/gaits/base.cpp
  ${FW_SRC}/gait/gaits/registry.cpp
  ${FW_SRC}/gait/engagement.cpp
  ${FW_SRC}/gait/pause.cpp
  ${FW_SRC}/gait/stand_transition.cpp
  ${FW_SRC}/gait/reseat.cpp
  ${FW_SRC}/gait/limits.cpp
  ${FW_SRC}/gait/engine.cpp
  ${FW_SRC}/posture/pose.cpp
  ${FW_SRC}/posture/animations.cpp
  ${FW_SRC}/posture/posture.cpp)

add_executable(firmware_bridge_node
  src/firmware_bridge_node.cpp
  ${FW_PIPELINE_SOURCES})
add_dependencies(firmware_bridge_node hexa_pico_bridge_gen_config)
target_include_directories(firmware_bridge_node PRIVATE ${FW_SRC})
# The firmware libs throw (IK UnreachableTarget, reseat invalid_argument); build
# with exceptions exactly as the Pico + host builds do.
target_compile_options(firmware_bridge_node PRIVATE -Wall -Wextra -fexceptions)
ament_target_dependencies(firmware_bridge_node rclcpp sensor_msgs std_msgs)

install(TARGETS firmware_bridge_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
