cmake_minimum_required(VERSION 3.5)
project(goal_giver_ros2)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(multi_agent_planner_msgs REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/goal_giver.py
  DESTINATION lib/${PROJECT_NAME}
)

# Ament Package
ament_package()
