cmake_minimum_required(VERSION 3.5)
project(swarmnxt_msgs)

# Find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

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

# Install the generated files
install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

ament_package()