cmake_minimum_required(VERSION 3.16)
project(hexa_kinematics_cpp)

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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hexa_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Pure kinematics library: no ROS, no rclcpp. Mirrors the Python package's
# contract that the kinematics modules are importable/unit-testable standalone.
add_library(${PROJECT_NAME} SHARED
  src/leg_ik.cpp
  src/leg_specs.cpp
  src/body_transform.cpp
  src/joint_config.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen yaml-cpp)

# ROS node executables: the only components that touch rclcpp.
add_executable(ik_node src/ik_node.cpp)
target_link_libraries(ik_node ${PROJECT_NAME})
ament_target_dependencies(ik_node
  ament_index_cpp
  rclcpp
  geometry_msgs
  sensor_msgs
  hexa_interfaces
)

add_executable(joint_command_bridge src/joint_command_bridge.cpp)
target_link_libraries(joint_command_bridge ${PROJECT_NAME})
ament_target_dependencies(joint_command_bridge
  rclcpp
  sensor_msgs
  std_msgs
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
  TARGETS ik_node joint_command_bridge
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # The pure library links without ROS, so the gtest targets exercise it
  # directly (mirror hexa_hardware's test setup). Linking ${PROJECT_NAME}
  # propagates the Eigen and yaml-cpp include paths transitively.
  ament_add_gtest(test_leg_ik test/test_leg_ik.cpp)
  target_link_libraries(test_leg_ik ${PROJECT_NAME})

  ament_add_gtest(test_leg_specs test/test_leg_specs.cpp)
  target_link_libraries(test_leg_specs ${PROJECT_NAME})

  ament_add_gtest(test_body_transform test/test_body_transform.cpp)
  target_link_libraries(test_body_transform ${PROJECT_NAME})

  ament_add_gtest(test_body_pose test/test_body_pose.cpp)
  target_link_libraries(test_body_pose ${PROJECT_NAME})

  # Loads the installed hexa_description config, so it needs ament_index_cpp;
  # it GTEST_SKIPs if that package is not on the prefix path.
  ament_add_gtest(test_real_config test/test_real_config.cpp)
  target_link_libraries(test_real_config ${PROJECT_NAME})
  ament_target_dependencies(test_real_config ament_index_cpp)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
# Only the pure library is consumed downstream (the node executables are not),
# so export only what its public headers need: Eigen (Point3 = Eigen::Vector3d)
# and yaml-cpp (yaml_util.hpp). This keeps the exported surface ROS-free so the
# hexa_gait_cpp engine library can depend on us without pulling in rclcpp.
ament_export_dependencies(
  Eigen3
  yaml-cpp
)
ament_package()
