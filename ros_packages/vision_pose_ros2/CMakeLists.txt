cmake_minimum_required(VERSION 3.8)
project(vision_pose_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_compile_options(-g) # compile options for debugging
add_compile_options(-O3)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(optitrack_multiplexer_ros2_msgs REQUIRED)
find_package(swarmnxt_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(DEPENDENCIES
  rclcpp
  builtin_interfaces
  geometry_msgs
  nav_msgs
  px4_msgs
  optitrack_multiplexer_ros2_msgs
  swarmnxt_msgs)

# source selection, placement and conversions: no ROS dependency
add_library(vision_pose_core STATIC src/vision_pose_core.cpp)
target_include_directories(vision_pose_core PUBLIC include/vision_pose_ros2)
target_link_libraries(vision_pose_core Eigen3::Eigen)

add_executable(vision_pose_node
  src/vision_pose_node.cpp
  src/vision_pose.cpp)
target_link_libraries(vision_pose_node vision_pose_core)
ament_target_dependencies(vision_pose_node ${DEPENDENCIES})

install(TARGETS
  vision_pose_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # offline check of the logic on synthetic data (no ROS graph):
  # colcon test --packages-select vision_pose_ros2
  add_executable(vision_pose_selfcheck test/vision_pose_selfcheck.cpp)
  target_link_libraries(vision_pose_selfcheck vision_pose_core)
  add_test(NAME vision_pose_selfcheck COMMAND vision_pose_selfcheck)
endif()

ament_package()
