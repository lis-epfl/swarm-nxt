cmake_minimum_required(VERSION 3.8)
project(swarmnxt_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
  px4_msgs
)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Plane.msg"
  "msg/ControllerCommand.msg"
  "msg/Trigger.msg"
  "msg/DroneState.msg"
  "srv/GetPlanes.srv"
  DEPENDENCIES ${DEPENDENCIES}
)

ament_package()
