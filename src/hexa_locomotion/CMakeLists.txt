cmake_minimum_required(VERSION 3.16)
project(hexa_locomotion CXX)

# ── Consolidated single-node locomotion controller ─────────────────────────
#
# Links the target-agnostic control-brain sources (shared/hexa_pipeline) DIRECTLY
# — the same float source the Pico firmware and hexa_pico_bridge compile — and
# wraps them in one 200 Hz ROS node with /cmd_vel-native seams. The only thing
# that differs vs the Pico build is the seam: input is /cmd_vel + the discrete
# command topics (not a gamepad), config is loaded from geometry.yaml /
# tuning.yaml at startup (PipelineConfig), and output is the sim/robot controller.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hexa_interfaces REQUIRED)
find_package(hexa_kinematics_cpp REQUIRED)
find_package(hexa_gait_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
# The double hexa_*_cpp loader headers (used only by the runtime config loader)
# include Eigen; the firmware brain itself is Eigen-free.
find_package(Eigen3 QUIET)

# ── Shared control-brain source tree ────────────────────────────────────────
set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(FW_SRC ${REPO_ROOT}/shared/hexa_pipeline)
set(GEN_CONFIG ${FW_SRC}/config_generated.hpp)

# ── Build-time config codegen (baked defaults for PipelineConfig::baked()) ───
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
  COMMENT "Generating config_generated.hpp from repo YAMLs (hexa_locomotion)"
  VERBATIM)
add_custom_target(hexa_locomotion_gen_config DEPENDS ${GEN_CONFIG})

# ── Firmware pipeline sources (minus the Pico-only seam impls) ───────────────
set(FW_PIPELINE_SOURCES
  ${FW_SRC}/pipeline.cpp
  ${FW_SRC}/pipeline_config.cpp
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

add_executable(locomotion_node
  src/locomotion_node.cpp
  src/pipeline_config_loader.cpp
  ${FW_PIPELINE_SOURCES})
add_dependencies(locomotion_node hexa_locomotion_gen_config)
target_include_directories(locomotion_node PRIVATE ${FW_SRC} src)
# The firmware libs throw (IK UnreachableTarget, reseat invalid_argument); build
# with exceptions exactly as the Pico + host + bridge builds do.
target_compile_options(locomotion_node PRIVATE -Wall -Wextra -fexceptions)
ament_target_dependencies(locomotion_node
  rclcpp ament_index_cpp geometry_msgs std_msgs hexa_interfaces
  hexa_kinematics_cpp hexa_gait_cpp)
target_link_libraries(locomotion_node yaml-cpp)
if(TARGET Eigen3::Eigen)
  target_link_libraries(locomotion_node Eigen3::Eigen)
else()
  target_include_directories(locomotion_node PRIVATE /usr/include/eigen3)
endif()

install(TARGETS locomotion_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
