cmake_minimum_required(VERSION 3.5)
project(swarmnxt_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(DEPENDENCIES
  geometry_msgs
  mavros_msgs 
  std_msgs
)
# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Plane.msg"
  "msg/ControllerCommand.msg"
  "msg/Trigger.msg"
  "msg/DroneState.msg"
  DEPENDENCIES ${DEPENDENCIES}
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()