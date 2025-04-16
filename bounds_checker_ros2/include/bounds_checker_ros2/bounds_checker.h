#pragma once

// #include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
// #include "geometry_msgs/msg/vec3.hpp"
#include "bounds_checker_ros2/msg/plane.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bounds_checker {
class BoundsChecker : public ::rclcpp::Node {
 public:
  BoundsChecker();

  void LoadHullFromFile(const std::filesystem::path& filepath);
  bool IsPointInHull(const geometry_msgs::msg::Point& point);
  void ClearPlanes();

  std::vector<bounds_checker_ros2::msg::Plane> GetPlanes();

 private:
  void DeclareRosParameters();
  void initializeRosParameters();

  std::vector<bounds_checker_ros2::msg::Plane> planes_;
  bool are_planes_valid_ = false;
  std::string topic_prefix_ = "";
};
}  // namespace bounds_checker
