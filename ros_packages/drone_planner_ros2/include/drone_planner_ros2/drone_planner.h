#pragma once

#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <deque>

namespace drone_planner {
class DronePlanner : public ::rclcpp::Node {
public:
  DronePlanner();
  void GoalsCallback(const nav_msgs::msg::Goals &msg);
  void DepthImageCallback(const sensor_msgs::msg::PointCloud2 &msg);

private:
  std::deque<geometry_msgs::msg::PoseStamped> current_goals_;

  // subscribers
  rclcpp::Subscription<nav_msgs::msg::Goals>::SharedPtr goals_sub_;
  std::map<std::string, rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr>
      traj_sub_map_; // for the trajectories of others
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      depth_image_sub_;

  // publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr desired_traj_pub_;

  // clients

  // Callbacks

  void TimerCallback();
};
} // namespace drone_planner
