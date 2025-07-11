cmake_minimum_required(VERSION 3.8)
project(latency_checker_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(DEPENDENCIES
  rclcpp
  builtin_interfaces
  rosidl_default_generators
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include/latency_checker_ros2
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

add_library(latency_checker_class src/latency_checker.cpp)
add_executable(latency_checker_node src/latency_checker_node.cpp)
target_link_libraries(latency_checker_node latency_checker_class)

ament_target_dependencies(latency_checker_class
  ${DEPENDENCIES}
)

ament_target_dependencies(latency_checker_node
${DEPENDENCIES})

target_link_libraries(latency_checker_class
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

ament_export_dependencies(rosidl_default_runtime)

set(MSG_FILES
"msg/Heartbeat.msg"
"msg/NameLatency.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
${MSG_FILES}
DEPENDENCIES builtin_interfaces
)

install(TARGETS latency_checker_class
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
latency_checker_node
DESTINATION lib/${PROJECT_NAME}
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
