#pragma once

#include "bounds_checker_ros2/msg/plane.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace bounds_checker {
class BoundsChecker : public ::rclcpp::Node {
 public:
  BoundsChecker();

  void LoadHullFromFile(const std::filesystem::path &filepath);
  bool IsPointInHull(const geometry_msgs::msg::Point &point);
  void HandlePoseMessage(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void HandleTrajectoryMessage(const nav_msgs::msg::Path &msg);
  geometry_msgs::msg::Point ProjectPointToClosestPlane(
      const geometry_msgs::msg::Point &point);
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
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;

  // publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr safe_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr safe_trajectory_pub_;

  // clients
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr land_client_;
};
}  // namespace bounds_checker
