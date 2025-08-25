cmake_minimum_required(VERSION 3.8)
project(logger_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/logger_ros2/logger_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()