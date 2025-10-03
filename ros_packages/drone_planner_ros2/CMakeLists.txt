cmake_minimum_required(VERSION 3.8)
project(drone_planner_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(DEPENDENCIES
  rclcpp
  builtin_interfaces
  geometry_msgs
  sensor_msgs
  nav_msgs
  px4_ros_com
  Eigen3
  tf2_geometry_msgs
)

include_directories(
  include/drone_planner_ros2
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

add_library(drone_planner_class src/drone_planner.cpp)
add_executable(drone_planner_node src/drone_planner_node.cpp)
target_link_libraries(drone_planner_node drone_planner_class)

ament_target_dependencies(drone_planner_class
  ${DEPENDENCIES}
)

ament_target_dependencies(drone_planner_node
${DEPENDENCIES})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
  drone_planner_class
  LIBRARY DESTINATION lib
)

install(TARGETS
drone_planner_node
DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
