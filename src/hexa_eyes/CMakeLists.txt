cmake_minimum_required(VERSION 3.16)
project(hexa_eyes CXX C)

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
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# This package adds no policy of its own: it compiles the same shared policy and
# animation core as hexa_display (shared/display_core, plus its Linux u8g2 panel
# and gesture-expression sources, used here only to draw the text screen into a
# buffer), and swaps the rasterizer and panel driver for shared/eyes_core. Both
# trees are compiled directly, so hexa_display itself is untouched.
set(DISPLAY_CORE ${CMAKE_CURRENT_SOURCE_DIR}/../../shared/display_core)
set(EYES_CORE    ${CMAKE_CURRENT_SOURCE_DIR}/../../shared/eyes_core)
set(DISPLAY_SRC  ${CMAKE_CURRENT_SOURCE_DIR}/../hexa_display/src)

file(GLOB U8G2_SRCS "${DISPLAY_CORE}/u8g2/*.c")
add_library(eyes_u8g2_core STATIC ${U8G2_SRCS})
target_include_directories(eyes_u8g2_core PUBLIC "${DISPLAY_CORE}/u8g2")

add_library(hexa_eyes_support STATIC
  ${DISPLAY_CORE}/expression_policy.cpp
  ${DISPLAY_CORE}/face_animation.cpp
  ${DISPLAY_CORE}/face_animation_runner.cpp
  ${DISPLAY_CORE}/text_screen.cpp
  ${DISPLAY_CORE}/fonts/hexa_text_font.c
  ${DISPLAY_CORE}/core/EyeAnim.cpp
  ${DISPLAY_CORE}/core/ExpressionController.cpp
  ${DISPLAY_SRC}/Sh1122Panel.cpp
  ${DISPLAY_SRC}/gesture_expressions.cpp
  ${EYES_CORE}/image.cpp
  ${EYES_CORE}/eyes.cpp
  ${EYES_CORE}/sh1122.cpp
  src/eyes_face.cpp
)
target_include_directories(hexa_eyes_support PUBLIC
  src
  "${DISPLAY_SRC}"
  "${DISPLAY_CORE}"
  "${DISPLAY_CORE}/core"
  "${EYES_CORE}"
)
target_link_libraries(hexa_eyes_support eyes_u8g2_core yaml-cpp)

add_executable(eyes_node src/eyes_node.cpp)
target_link_libraries(eyes_node hexa_eyes_support)
ament_target_dependencies(eyes_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  hexa_interfaces
  ament_index_cpp
)

install(TARGETS eyes_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_eyes_face test/test_eyes_face.cpp)
  if(TARGET test_eyes_face)
    target_link_libraries(test_eyes_face hexa_eyes_support)
  endif()
endif()

ament_package()
