#pragma once

// #include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
// #include "geometry_msgs/msg/vec3.hpp"
#include "bounds_checker_ros2/msg/plane.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"


namespace bounds_checker {
class BoundsChecker : public ::rclcpp::Node {
 public:
  BoundsChecker();

  void LoadHullFromFile(const std::filesystem::path& filepath);
  bool IsPointInHull(const geometry_msgs::msg::Point& point);
  void HandlePoseMessage(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void ClearPlanes();

  std::vector<bounds_checker_ros2::msg::Plane> GetPlanes();

 private:
  void DeclareRosParameters();
  void InitializeRosParameters();

  std::vector<bounds_checker_ros2::msg::Plane> planes_;
  bool are_planes_valid_ = false;
  std::string topic_prefix_ = "";

  // parameters
  std::string position_topic_suffix_;
  std::string trajectory_topic_suffix_;

  // subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr trajectory_sub_;
  
  // publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr safe_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr safe_trajectory_pub_;



};
}  // namespace bounds_checker
