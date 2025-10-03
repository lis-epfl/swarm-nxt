cmake_minimum_required(VERSION 3.8)
project(swarmnxt_controller_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(swarmnxt_msgs REQUIRED)
find_package(multi_agent_planner_msgs REQUIRED)

set(DEPENDENCIES
  rclcpp
  builtin_interfaces
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  px4_msgs
  tf2_geometry_msgs
  std_msgs
  swarmnxt_msgs
  multi_agent_planner_msgs
)

include_directories(
  include/swarmnxt_controller_ros2
)

add_library(swarmnxt_controller_class src/swarmnxt_controller.cpp)
add_executable(swarmnxt_controller_node src/swarmnxt_controller_node.cpp)
target_link_libraries(swarmnxt_controller_node swarmnxt_controller_class)

ament_target_dependencies(swarmnxt_controller_class
  ${DEPENDENCIES}
)

ament_target_dependencies(swarmnxt_controller_node
${DEPENDENCIES})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
  swarmnxt_controller_class
  LIBRARY DESTINATION lib
)

install(TARGETS
swarmnxt_controller_node
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
