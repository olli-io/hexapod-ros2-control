cmake_minimum_required(VERSION 3.16)
project(hexa_display CXX C)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hexa_interfaces REQUIRED)

# The pure policy + the vendored eye core / u8g2 C core live in the shared
# display core (shared/display_core), compiled directly here — the same source
# the Pico firmware and the firmware host test build, so the eyes rasterize
# bit-identically across targets (mirrors how hexa_locomotion consumes
# shared/motion_core). This package keeps only the rclcpp nodes and the Linux
# SH1122 panel driver. See shared/display_core/README.md for provenance.
set(DISPLAY_CORE ${CMAKE_CURRENT_SOURCE_DIR}/../../shared/display_core)

# Vendored u8g2 C core (framework-independent), compiled into its own library
# so --gc-sections can drop the unused device drivers. u8g2's 38 MB font-data
# blobs are intentionally not vendored — the text mode draws with the repo's
# own generated font (shared/display_core/fonts, from tools/gen_font.py).
file(GLOB U8G2_SRCS "${DISPLAY_CORE}/u8g2/*.c")
add_library(u8g2_core STATIC ${U8G2_SRCS})
target_include_directories(u8g2_core PUBLIC "${DISPLAY_CORE}/u8g2")

# Platform-free pieces vendored from the firmware tree:
#   EyeAnim/EyeRaster        - the animation + rasterizer (also host-tested)
#   ExpressionController.cpp - only for expressionName()/gazeName(), the single
#                              source of truth for the on-wire enum names
set(CORE_SRCS
  "${DISPLAY_CORE}/core/EyeAnim.cpp"
  "${DISPLAY_CORE}/core/EyeRaster.cpp"
  "${DISPLAY_CORE}/core/ExpressionController.cpp"
)

# Pure support library: the shared expression/gaze policy + face-animation
# sequences/clock + text screen (from shared/display_core) and the Linux
# SH1122 panel driver.
# No rclcpp — every piece is unit-testable standalone (the gtest suites link
# this directly). Built on the vendored eye core + u8g2.
add_library(hexa_display_support STATIC
  ${DISPLAY_CORE}/expression_policy.cpp
  ${DISPLAY_CORE}/face_animation.cpp
  ${DISPLAY_CORE}/face_animation_runner.cpp
  ${DISPLAY_CORE}/text_screen.cpp
  ${DISPLAY_CORE}/fonts/hexa_text_font.c
  src/Sh1122Panel.cpp
  ${CORE_SRCS}
)
target_include_directories(hexa_display_support PUBLIC
  src
  "${DISPLAY_CORE}"
  "${DISPLAY_CORE}/core"
)
target_link_libraries(hexa_display_support u8g2_core)

# ROS node executable: the only component that touches rclcpp. One process runs
# the policy and rasterizes the OLED — the former hexa_display/hexa_face topic
# hop is now an in-process call.
add_executable(display_node src/display_node.cpp)
target_link_libraries(display_node hexa_display_support)
ament_target_dependencies(display_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  hexa_interfaces
)

# Sim-only terminal eye emulator: a live mirror of display_node that rasterizes
# the eyes into the terminal (Unicode quadrant / Braille blocks) instead of the
# SH1122. Reuses the same policy + eye core via hexa_display_support; no panel /
# SPI / GPIO. Built in every image but launched only by hand (`hexa sim face`) —
# never by robot.launch.py, so the robot runtime is unaffected.
add_executable(face_sim src/face_sim.cpp)
target_link_libraries(face_sim hexa_display_support)
ament_target_dependencies(face_sim
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  hexa_interfaces
)

install(TARGETS display_node face_sim DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # gtest ports of the former pytest suites plus the renderer's own suite. All
  # headless: the pure support library links without rclcpp or a device.
  set(display_tests
    test_dirty_band
    test_expression_policy
    test_face_animation
    test_face_animation_runner
    test_face
    test_text_screen
  )
  foreach(test_name ${display_tests})
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    if(TARGET ${test_name})
      target_link_libraries(${test_name} hexa_display_support)
    endif()
  endforeach()
endif()

ament_package()
