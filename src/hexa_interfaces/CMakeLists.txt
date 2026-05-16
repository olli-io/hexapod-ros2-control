cmake_minimum_required(VERSION 3.8)
project(hexa_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add new msg/srv/action files here as they're introduced. Keep the list
# tight — only ship interfaces that have a concrete producer/consumer.
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/BodyPose.msg
  msg/LegState.msg
  msg/LegTargets.msg
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
