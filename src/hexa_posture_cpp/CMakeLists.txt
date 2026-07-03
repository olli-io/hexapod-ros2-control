cmake_minimum_required(VERSION 3.16)
project(hexa_posture_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hexa_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

# Pure posture library: no ROS, no rclcpp. Mirrors the Python package's contract
# that pose/animations/signals are importable and unit-testable standalone.
add_library(${PROJECT_NAME} SHARED
  src/pose.cpp
  src/animations.cpp
  src/signals.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# ROS node executable: the only component that touches rclcpp.
add_executable(posture_node src/posture_node.cpp)
target_link_libraries(posture_node ${PROJECT_NAME})
ament_target_dependencies(posture_node
  rclcpp
  geometry_msgs
  std_msgs
  hexa_interfaces
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
  TARGETS posture_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # The pure library links without ROS, so the gtest targets exercise it
  # directly. Linking ${PROJECT_NAME} propagates the Eigen include paths
  # transitively.
  ament_add_gtest(test_pose test/test_pose.cpp)
  target_link_libraries(test_pose ${PROJECT_NAME})

  ament_add_gtest(test_animations test/test_animations.cpp)
  target_link_libraries(test_animations ${PROJECT_NAME})

  ament_add_gtest(test_signals test/test_signals.cpp)
  target_link_libraries(test_signals ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
# Only the pure library is consumed downstream; export just what its public
# headers need (Eigen, for Vec3 = Eigen::Vector3d).
ament_export_dependencies(Eigen3)
ament_package()
