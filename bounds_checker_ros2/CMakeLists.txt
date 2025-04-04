cmake_minimum_required(VERSION 3.8)
project(bounds_checker_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(DEPENDENCIES
  rclcpp
  builtin_interfaces
  geometry_msgs
  rosidl_default_generators
)

include_directories(
  include/bounds_checker_ros2
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

add_library(bounds_checker_class src/bounds_checker.cpp)

ament_target_dependencies(bounds_checker_class
  ${DEPENDENCIES}
)

target_link_libraries(bounds_checker_class
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

ament_export_dependencies(rosidl_default_runtime)

set(MSG_FILES
  "msg/Plane.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES builtin_interfaces geometry_msgs
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
