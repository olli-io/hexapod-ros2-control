cmake_minimum_required(VERSION 3.16)

# ── Host golden-trace / unit-test harness (plan part 04, extended by 05-10) ──
#
# A STANDALONE, host-native CMake project — deliberately NOT the Pico SDK build.
# It compiles the forked pure libs (float) with the system compiler, no Pico SDK,
# so the whole port's logic (float conversion, kinematics, gait, posture, teleop
# mapping) is unit- and golden-testable off-target. Part 04 stands up the seam
# with vec3 + generated-config tests; parts 05-08 add the forked libs and the
# golden-trace diff against the double ROS2 engines (see README.md, Tier 1).
#
# Build (host, or inside `hexa sim`):
#   cmake -S . -B build && cmake --build build && ctest --test-dir build
project(hexa_pico_host_test CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Firmware sources live two levels up (pi-pico-firmware/src). The generated
# config header is emitted there by tools/gen_config.py.
set(FW_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
set(FW_SRC ${FW_ROOT}/src)
set(GEN_CONFIG ${FW_SRC}/config_generated.hpp)

# ── Regenerate config_generated.hpp from the repo YAMLs before building ──
# Mirrors the Pico build's pre-build codegen so host and target share one baked
# config. Re-runs whenever the generator or any source YAML changes.
find_program(PYTHON3 python3 REQUIRED)
set(REPO_ROOT ${FW_ROOT}/..)
file(GLOB_RECURSE CONFIG_YAMLS
  ${REPO_ROOT}/src/hexa_description/config/geometry.yaml
  ${REPO_ROOT}/src/hexa_description/config/standing_pose.yaml
  ${REPO_ROOT}/src/hexa_gait_cpp/config/gait.yaml
  ${REPO_ROOT}/src/hexa_teleop/config/teleop_joy.yaml
  ${REPO_ROOT}/src/hexa_posture/config/posture.yaml
  ${REPO_ROOT}/src/hexa_control/config/control.yaml
  ${REPO_ROOT}/src/hexa_hardware/config/hardware.yaml
  ${REPO_ROOT}/src/hexa_webteleop/config/webteleop.yaml
  ${REPO_ROOT}/src/hexa_display/config/display.yaml)
add_custom_command(
  OUTPUT ${GEN_CONFIG}
  COMMAND ${PYTHON3} ${FW_ROOT}/tools/gen_config.py
          --repo-root ${REPO_ROOT} --out ${GEN_CONFIG}
  DEPENDS ${FW_ROOT}/tools/gen_config.py ${CONFIG_YAMLS}
  COMMENT "Generating config_generated.hpp from repo YAMLs"
  VERBATIM)
add_custom_target(gen_config DEPENDS ${GEN_CONFIG})

# ── GoogleTest ──
find_package(GTest REQUIRED)
include(GoogleTest)
enable_testing()

# Common settings for every host test target.
function(hexa_host_test name)
  add_executable(${name} ${ARGN})
  add_dependencies(${name} gen_config)
  target_include_directories(${name} PRIVATE ${FW_SRC})
  target_compile_options(${name} PRIVATE -Wall -Wextra -Wdouble-promotion)
  target_link_libraries(${name} PRIVATE GTest::gtest GTest::gtest_main)
  gtest_discover_tests(${name})
endfunction()

hexa_host_test(test_vec3 test_vec3.cpp)
hexa_host_test(test_config test_config.cpp)

# ── Integration supervisor (part 09) ──
# Failsafe / telemetry / status-LED policy: pure logic, no Pico SDK, so it runs
# through hexa_host_test() under -Wdouble-promotion like the rest of the port.
hexa_host_test(test_supervisor test_supervisor.cpp ${FW_SRC}/supervisor.cpp)

# Human-readable config dump (not a test): prints the 6 symmetry-expanded leg
# specs and derived velocity caps for the part-04 eyeball check.
add_executable(dump_config dump_config.cpp)
add_dependencies(dump_config gen_config)
target_include_directories(dump_config PRIVATE ${FW_SRC})
target_compile_options(dump_config PRIVATE -Wall -Wextra -Wdouble-promotion)

# ── Golden-trace tests (parts 05-08) ──
# As each pure lib is forked into ../../src (kinematics/, gait/, posture/), add
# its gtest suite here, plus the golden-trace driver that diffs the float port
# against the double hexa_kinematics_cpp / hexa_gait_cpp engines (~1e-3 rad).
# See README.md (Tier 1).

# Kinematics (part 05). Compiles the float port (../../src/kinematics) alongside
# the untouched double hexa_kinematics_cpp sources (namespace hexa_kinematics,
# Eigen) and diffs FK/IK/body-transform/compose traces. This target is NOT built
# through hexa_host_test(): it deliberately bridges float<->double, so it drops
# -Wdouble-promotion (the firmware build is that gate for the port sources) and
# needs Eigen + the double engine's headers.
set(KIN_SRC ${FW_SRC}/kinematics)
set(REF_KIN ${REPO_ROOT}/src/hexa_kinematics_cpp)

find_package(Eigen3 QUIET)

add_executable(test_kinematics
  test_kinematics.cpp
  ${KIN_SRC}/leg_ik.cpp
  ${KIN_SRC}/body_transform.cpp
  ${REF_KIN}/src/leg_ik.cpp
  ${REF_KIN}/src/body_transform.cpp)
add_dependencies(test_kinematics gen_config)
target_include_directories(test_kinematics PRIVATE ${FW_SRC} ${REF_KIN}/include)
target_compile_options(test_kinematics PRIVATE -Wall -Wextra)
target_link_libraries(test_kinematics PRIVATE GTest::gtest GTest::gtest_main)
if (TARGET Eigen3::Eigen)
  target_link_libraries(test_kinematics PRIVATE Eigen3::Eigen)
else()
  # Fall back to the conventional system include path (Arch/Debian: eigen3).
  target_include_directories(test_kinematics PRIVATE /usr/include/eigen3)
endif()
gtest_discover_tests(test_kinematics)

# ── Gait engine (part 06) ──
# The forked float engine (src/gait) plus its float kinematics dependency. This
# list is shared by the double-promotion-gated unit test and the golden-trace
# diff below.
set(GAIT_SRC ${FW_SRC}/gait)
set(FW_GAIT_SOURCES
  ${GAIT_SRC}/clock.cpp
  ${GAIT_SRC}/trajectory.cpp
  ${GAIT_SRC}/gaits/base.cpp
  ${GAIT_SRC}/gaits/registry.cpp
  ${GAIT_SRC}/engagement.cpp
  ${GAIT_SRC}/pause.cpp
  ${GAIT_SRC}/stand_transition.cpp
  ${GAIT_SRC}/reseat.cpp
  ${GAIT_SRC}/limits.cpp
  ${GAIT_SRC}/engine.cpp
  ${KIN_SRC}/leg_ik.cpp
  ${KIN_SRC}/body_transform.cpp)

# Behavioural unit test: float-only, so it runs through hexa_host_test() and is
# compiled under -Wdouble-promotion (the same gate as the firmware build). A
# clean compile here proves the port never silently promotes to double.
hexa_host_test(test_gait_unit test_gait_unit.cpp ${FW_GAIT_SOURCES})
target_compile_options(test_gait_unit PRIVATE -fexceptions)

# Golden-trace diff: compiles the float port alongside the untouched double
# hexa_gait_cpp engine (namespace hexa_gait, Eigen + yaml-cpp) and diffs a full
# cmd_vel trace. Like test_kinematics it deliberately bridges float<->double, so
# it drops -Wdouble-promotion and needs Eigen + yaml-cpp + the double engine's
# and kinematics' headers/sources.
set(REF_GAIT ${REPO_ROOT}/src/hexa_gait_cpp)
find_package(yaml-cpp QUIET)

add_executable(test_gait
  test_gait.cpp
  ${FW_GAIT_SOURCES}
  ${REF_GAIT}/src/clock.cpp
  ${REF_GAIT}/src/trajectory.cpp
  ${REF_GAIT}/src/gaits/base.cpp
  ${REF_GAIT}/src/gaits/registry.cpp
  ${REF_GAIT}/src/engagement.cpp
  ${REF_GAIT}/src/pause.cpp
  ${REF_GAIT}/src/stand_transition.cpp
  ${REF_GAIT}/src/reseat.cpp
  ${REF_GAIT}/src/limits.cpp
  ${REF_GAIT}/src/engine.cpp
  # The double engine's YAML builders reference the kinematics description loader
  # even though this test never calls them; compile it (and yaml-cpp) to satisfy
  # the linker.
  ${REF_KIN}/src/leg_ik.cpp
  ${REF_KIN}/src/body_transform.cpp
  ${REF_KIN}/src/description_loader.cpp)
add_dependencies(test_gait gen_config)
target_include_directories(test_gait PRIVATE
  ${FW_SRC} ${REF_GAIT}/include ${REF_KIN}/include)
target_compile_options(test_gait PRIVATE -Wall -Wextra -fexceptions)
target_link_libraries(test_gait PRIVATE GTest::gtest GTest::gtest_main yaml-cpp)
if (TARGET Eigen3::Eigen)
  target_link_libraries(test_gait PRIVATE Eigen3::Eigen)
else()
  target_include_directories(test_gait PRIVATE /usr/include/eigen3)
endif()
gtest_discover_tests(test_gait)

# ── Control velocity shaping (part 07) ──
# BodyVelocityLimiter slew + Control scale-to-envelope / reset wiring. Float
# only, so it runs through hexa_host_test() under -Wdouble-promotion. Needs the
# gait limits (scale_to_envelope + caps) it shapes against.
hexa_host_test(test_control test_control.cpp
  ${FW_SRC}/control.cpp
  ${GAIT_SRC}/limits.cpp)
target_compile_options(test_control PRIVATE -fexceptions)

# ── Teleop joy mapping (part 07) ──
# Golden-trace parity of the float map_joy port against the Python
# hexa_teleop.joy_mapping reference. gen_joy_golden.py imports the pure
# reference and bakes the trace + expected outputs into joy_golden_generated.hpp
# (emitted into the build tree) from the same YAMLs as the firmware config.
set(JOY_GOLDEN ${CMAKE_CURRENT_BINARY_DIR}/joy_golden_generated.hpp)
add_custom_command(
  OUTPUT ${JOY_GOLDEN}
  COMMAND ${PYTHON3} ${CMAKE_CURRENT_LIST_DIR}/gen_joy_golden.py
          --repo-root ${REPO_ROOT} --out ${JOY_GOLDEN}
  DEPENDS ${CMAKE_CURRENT_LIST_DIR}/gen_joy_golden.py
          ${REPO_ROOT}/src/hexa_teleop/hexa_teleop/joy_mapping.py
          ${REPO_ROOT}/src/hexa_teleop/config/teleop_joy.yaml
          ${REPO_ROOT}/src/hexa_gait_cpp/config/gait.yaml
          ${REPO_ROOT}/src/hexa_posture/config/posture.yaml
  COMMENT "Generating joy_golden_generated.hpp from the Python map_joy reference"
  VERBATIM)
add_custom_target(gen_joy_golden DEPENDS ${JOY_GOLDEN})

# map_joy is float-only (config_generated + caps), so it runs under
# -Wdouble-promotion too.
hexa_host_test(test_joy_mapping test_joy_mapping.cpp
  ${FW_SRC}/joy_mapping.cpp
  ${GAIT_SRC}/limits.cpp)
add_dependencies(test_joy_mapping gen_joy_golden)
target_include_directories(test_joy_mapping PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# ── Posture (part 08) ──
# The float posture port (src/posture) plus its gait/kinematics dependencies
# (LegOutput, EngineState, state_value, pymod). Float-only, so it runs through
# hexa_host_test() under -Wdouble-promotion. The golden trace is baked from the
# real pure Python hexa_posture animation stack by gen_posture_golden.py.
set(POSTURE_SRC ${FW_SRC}/posture)
set(FW_POSTURE_SOURCES
  ${POSTURE_SRC}/pose.cpp
  ${POSTURE_SRC}/animations.cpp
  ${POSTURE_SRC}/posture.cpp)

set(POSTURE_GOLDEN ${CMAKE_CURRENT_BINARY_DIR}/posture_golden_generated.hpp)
set(REF_POSTURE ${REPO_ROOT}/src/hexa_posture/hexa_posture)
add_custom_command(
  OUTPUT ${POSTURE_GOLDEN}
  COMMAND ${PYTHON3} ${CMAKE_CURRENT_LIST_DIR}/gen_posture_golden.py
          --repo-root ${REPO_ROOT} --out ${POSTURE_GOLDEN}
  DEPENDS ${CMAKE_CURRENT_LIST_DIR}/gen_posture_golden.py
          ${REF_POSTURE}/pose.py
          ${REF_POSTURE}/animations/base.py
          ${REF_POSTURE}/animations/still.py
          ${REF_POSTURE}/animations/breathing.py
          ${REF_POSTURE}/animations/gait_sway.py
          ${REF_POSTURE}/animations/gait_bounce.py
          ${REF_POSTURE}/animations/vertical_body_roll.py
          ${REF_POSTURE}/animations/horizontal_body_roll.py
          ${REF_POSTURE}/animations/body_roll_3d.py
          ${REPO_ROOT}/src/hexa_posture/config/posture.yaml
  COMMENT "Generating posture_golden_generated.hpp from the Python reference"
  VERBATIM)
add_custom_target(gen_posture_golden DEPENDS ${POSTURE_GOLDEN})

hexa_host_test(test_posture test_posture.cpp
  ${FW_POSTURE_SOURCES}
  ${FW_GAIT_SOURCES})
add_dependencies(test_posture gen_posture_golden)
target_include_directories(test_posture PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_compile_options(test_posture PRIVATE -fexceptions)

# ── Pipeline integration (part 10, Tier 2/3 seam) ──
# Drives the whole target-agnostic control brain (src/pipeline.cpp) — teleop ->
# velocity shaping -> gait -> posture -> compose/IK + supervisor — natively,
# the exact source the Pico firmware and the Gazebo bridge compile. Float-only,
# so it runs through hexa_host_test() under -Wdouble-promotion like the rest of
# the port. This is the off-target proof that the extraction composes and the
# full chain stands, walks, and safe-stops; per-stage fidelity is the golden
# suites' job.
hexa_host_test(test_pipeline test_pipeline.cpp
  ${FW_SRC}/pipeline.cpp
  ${FW_SRC}/control.cpp
  ${FW_SRC}/joy_mapping.cpp
  ${FW_SRC}/supervisor.cpp
  ${FW_POSTURE_SOURCES}
  ${FW_GAIT_SOURCES})
target_compile_options(test_pipeline PRIVATE -fexceptions)
