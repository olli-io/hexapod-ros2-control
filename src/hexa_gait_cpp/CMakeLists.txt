cmake_minimum_required(VERSION 3.16)
project(hexa_gait_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hexa_interfaces REQUIRED)
find_package(hexa_kinematics_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Pure engine library: no ROS, no rclcpp. Mirrors the Python package's
# contract that every gait module is importable/unit-testable standalone.
add_library(${PROJECT_NAME} SHARED
  src/clock.cpp
  src/trajectory.cpp
  src/gaits/base.cpp
  src/gaits/common.cpp
  src/gaits/registry.cpp
  src/pause.cpp
  src/initialize.cpp
  src/fold.cpp
  src/reseat.cpp
  src/stability.cpp
  src/limits.cpp
  src/engagement.cpp
  src/engine.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen yaml-cpp)
# The real kinematics library (replaces the former kinematics_stub.hpp). Its
# exported surface is ROS-free (Eigen + yaml-cpp only), so the engine library
# stays free of rclcpp.
ament_target_dependencies(${PROJECT_NAME} hexa_kinematics_cpp)

# ROS node executable: the only component that touches rclcpp.
add_executable(gait_node src/gait_node.cpp)
target_link_libraries(gait_node ${PROJECT_NAME})
ament_target_dependencies(gait_node
  ament_index_cpp
  rclcpp
  geometry_msgs
  std_msgs
  hexa_interfaces
  hexa_kinematics_cpp
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS gait_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # TODO(tests): port the 15 pytest suites from hexa_gait to ament_cmake_gtest
  # as a separate task. The pure engine library above links without ROS, so the
  # gtest targets can exercise it directly (mirror hexa_hardware's test setup).
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  ament_index_cpp
  rclcpp
  geometry_msgs
  std_msgs
  hexa_interfaces
  hexa_kinematics_cpp
  Eigen3
  yaml-cpp
)
ament_package()
