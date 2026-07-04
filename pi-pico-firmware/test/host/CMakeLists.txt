cmake_minimum_required(VERSION 3.16)

# ── Host test harness for the Pico face (part 11) ──
#
# STANDALONE host-native build (not the Pico SDK build), like
# shared/motion_core/test. It exercises the firmware face config path —
# display.yaml -> gen_config.py -> config_generated.hpp -> face::buildPolicyConfig()
# -> the shared policy library — off-target. The EyeAnim/EyeRaster core, the
# Pico SPI panel and the core1 render loop are hardware and not covered here; the
# eye policy itself is covered by hexa_display's gtest.
#
# Build (host, or inside `hexa sim`):
#   cmake -S . -B build && cmake --build build && ctest --test-dir build
project(hexa_pico_face_host_test CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(REPO_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../..)
set(SHARED ${REPO_ROOT}/shared/motion_core)
set(DISPLAY_CORE ${REPO_ROOT}/shared/display_core)
set(FW_SRC ${CMAKE_CURRENT_LIST_DIR}/../../src)
set(GEN_CONFIG ${SHARED}/config_generated.hpp)

# Regenerate the baked config before building (mirrors the Pico + motion_core
# builds) so the face bake under test tracks display.yaml.
find_program(PYTHON3 python3 REQUIRED)
add_custom_command(
  OUTPUT ${GEN_CONFIG}
  COMMAND ${PYTHON3} ${SHARED}/tools/gen_config.py
          --repo-root ${REPO_ROOT} --out ${GEN_CONFIG}
  DEPENDS ${SHARED}/tools/gen_config.py
          ${REPO_ROOT}/src/hexa_display/config/display.yaml
  COMMENT "Generating config_generated.hpp from repo YAMLs"
  VERBATIM)
add_custom_target(gen_config DEPENDS ${GEN_CONFIG})

enable_testing()

# The face policy library is pure (no rclcpp, no Pico SDK): the vendored eye-name
# core + the expression/gaze policy + the face-animation clock.
add_executable(test_face_policy
  test_face_policy.cpp
  ${DISPLAY_CORE}/expression_policy.cpp
  ${DISPLAY_CORE}/face_animation.cpp
  ${DISPLAY_CORE}/face_animation_runner.cpp
  ${DISPLAY_CORE}/core/ExpressionController.cpp)
add_dependencies(test_face_policy gen_config)
target_include_directories(test_face_policy PRIVATE
  ${SHARED} ${DISPLAY_CORE} ${DISPLAY_CORE}/core ${FW_SRC})
target_compile_options(test_face_policy PRIVATE -Wall -Wextra -fexceptions)
add_test(NAME test_face_policy COMMAND test_face_policy)
