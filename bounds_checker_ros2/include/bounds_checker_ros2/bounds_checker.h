#pragma once

// #include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
// #include "geometry_msgs/msg/vec3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "bounds_checker_ros2/msg/plane.hpp"
#include "nlohmann/json.hpp"

namespace bounds_checker
{
class BoundsChecker : public ::rclcpp::Node
{
public:
  BoundsChecker();

  void load_hull_from_file(const std::string& filepath);
  bool is_point_in_hull(const geometry_msgs::msg::Point& point);
  void clear_planes();

private:


  std::vector<bounds_checker_ros2::msg::Plane> planes_;
  bool are_planes_valid_ = false; 
};
}  // namespace bounds_checker
