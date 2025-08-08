cmake_minimum_required(VERSION 3.5)
project(drone_state_manager_ros2)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(swarmnxt_msgs REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/drone_state_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

# Ament Package
ament_package()
