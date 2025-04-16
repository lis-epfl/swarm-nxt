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

  void loadHullFromFile(const std::string& filepath);
  bool isPointInHull(const geometry_msgs::msg::Point& point);
  void clearPlanes();

private:


  std::vector<bounds_checker_ros2::msg::Plane> planes_;
  bool are_planes_valid_ = false; 
};
}  // namespace bounds_checker
