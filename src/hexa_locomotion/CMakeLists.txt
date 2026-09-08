cmake_minimum_required(VERSION 3.16)
project(hexa_locomotion CXX)

# Compiles shared/motion_core directly (link-time seam swap, same source as the
# Pico firmware and hexa_pico_bridge). See README.md.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hexa_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(FW_SRC ${REPO_ROOT}/shared/motion_core)
set(GEN_CONFIG ${FW_SRC}/config_generated.hpp)

# Baked defaults for PipelineConfig::baked(); the fallback when the YAML load fails.
find_program(PYTHON3 python3 REQUIRED)
set(CONFIG_YAMLS
  ${REPO_ROOT}/src/hexa_description/config/geometry.yaml
  ${REPO_ROOT}/src/hexa_description/config/tuning.yaml
  ${REPO_ROOT}/src/hexa_teleop/config/teleop_joy.yaml
  ${REPO_ROOT}/src/hexa_description/config/hardware.yaml
  ${REPO_ROOT}/src/hexa_description/config/servo_calibration.yaml
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

# motion_core minus the Pico-only seam impls.
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
  ${FW_SRC}/gait/stand_transition.cpp
  ${FW_SRC}/gait/reseat.cpp
  ${FW_SRC}/gait/limits.cpp
  ${FW_SRC}/gait/engine.cpp
  ${FW_SRC}/gait/reversal.cpp
  ${FW_SRC}/posture/pose.cpp
  ${FW_SRC}/posture/animations.cpp
  ${FW_SRC}/posture/posture.cpp)

add_executable(locomotion_node
  src/locomotion_node.cpp
  src/pipeline_config_loader.cpp
  ${FW_PIPELINE_SOURCES})
add_dependencies(locomotion_node hexa_locomotion_gen_config)
target_include_directories(locomotion_node PRIVATE ${FW_SRC} src)
# motion_core throws (IK UnreachableTarget, reseat invalid_argument).
target_compile_options(locomotion_node PRIVATE -Wall -Wextra -fexceptions)
ament_target_dependencies(locomotion_node
  rclcpp ament_index_cpp geometry_msgs std_msgs sensor_msgs std_srvs hexa_interfaces)
target_link_libraries(locomotion_node yaml-cpp)

install(TARGETS locomotion_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

# Parity test: runtime loader == baked().
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_config_loader
    test/test_config_loader.cpp
    src/pipeline_config_loader.cpp
    ${FW_PIPELINE_SOURCES})
  if(TARGET test_config_loader)
    add_dependencies(test_config_loader hexa_locomotion_gen_config)
    target_include_directories(test_config_loader PRIVATE ${FW_SRC} src)
    target_compile_options(test_config_loader PRIVATE -Wall -Wextra -fexceptions)
    # The exact YAMLs gen_config.py bakes from, so the diff is over identical inputs.
    target_compile_definitions(test_config_loader PRIVATE
      GEOMETRY_YAML="${REPO_ROOT}/src/hexa_description/config/geometry.yaml"
      TUNING_YAML="${REPO_ROOT}/src/hexa_description/config/tuning.yaml")
    ament_target_dependencies(test_config_loader
      rclcpp ament_index_cpp)
    target_link_libraries(test_config_loader yaml-cpp)
  endif()
endif()

ament_package()
