cmake_minimum_required(VERSION 3.8)
project(safety_checker_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(swarmnxt_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_srvs REQUIRED)
find_package(swarmnxt_controller_ros2 REQUIRED)


set(DEPENDENCIES
  rclcpp
  builtin_interfaces
  geometry_msgs
  std_srvs
  nav_msgs
  mavros_msgs
  swarmnxt_msgs
  nlohmann_json
  swarmnxt_controller_ros2 
)

include_directories(
  include/safety_checker_ros2
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

add_library(safety_checker_class src/safety_checker.cpp)
add_executable(safety_checker_node src/safety_checker_node.cpp)
target_link_libraries(safety_checker_node safety_checker_class)

ament_target_dependencies(safety_checker_class
  ${DEPENDENCIES}
)

ament_target_dependencies(safety_checker_node
  ${DEPENDENCIES}
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
  safety_checker_class
  LIBRARY DESTINATION lib
)

install(TARGETS
safety_checker_node
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
