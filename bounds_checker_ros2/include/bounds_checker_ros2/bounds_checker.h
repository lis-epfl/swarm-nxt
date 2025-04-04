#pragma once

// #include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "bounds_checker_ros2/msg/plane.hpp"

namespace bounds_checker
{
class BoundsChecker : public ::rclcpp::Node
{
public:
  BoundsChecker();

  bool provide_convex_hull(const std::vector<bounds_checker_ros2::msg::Plane> planes);
  

private:

  bool is_point_in_hull(const geometry_msgs::msg::Point& point);

  std::vector<bounds_checker_ros2::msg::Plane> planes_;
};
}  // namespace bounds_checker
