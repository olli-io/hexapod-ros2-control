cmake_minimum_required(VERSION 3.16)
project(hexa_imu CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Pure support library: register decode, i2c-dev driver and attitude filter.
# No rclcpp, so the gtest suite links it without a ROS graph or a device.
add_library(hexa_imu_support STATIC
  src/attitude_filter.cpp
  src/mpu6500.cpp
)
target_include_directories(hexa_imu_support PUBLIC src)

add_executable(imu_node src/imu_node.cpp)
target_link_libraries(imu_node hexa_imu_support)
ament_target_dependencies(imu_node rclcpp sensor_msgs std_srvs)

install(TARGETS imu_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_imu test/test_imu.cpp)
  if(TARGET test_imu)
    target_link_libraries(test_imu hexa_imu_support)
  endif()
endif()

ament_package()
