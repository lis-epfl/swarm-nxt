#pragma once

#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace drone_planner {
class DronePlanner : public ::rclcpp::Node {
 public:
  DronePlanner();
  void GoalsCallback(const nav_msgs::msg::Goals &msg);
  void DepthImageCallback(const sensor_msgs::msg::PointCloud2 &msg);

 private:
  geometry_msgs::msg::PoseStamped current_goal_;
  bool new_goal_ = true;
  geometry_msgs::msg::PoseStamped current_position_;

  nav_msgs::msg::Path trajectory_;
  // TODO: make these parameters?
  float drone_speed_m_s_ = 5;         // max linear speed
  float yaw_speed_rad_s_ = 0.175;     // max yaw speed
  float trajectory_density_hz_ = 10;  // the frequency to generate trajectory
                                      // information for

  // subscribers
  rclcpp::Subscription<nav_msgs::msg::Goals>::SharedPtr goals_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      mavros_pose_sub_;
  std::map<std::string, rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr>
      traj_sub_map_;  // for the trajectories of others
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      depth_image_sub_;

  // publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr desired_traj_pub_;

  // clients

  // timers
  rclcpp::TimerBase::SharedPtr traj_pub_timer_;

  // Callbacks

  nav_msgs::msg::Path GenerateTrajectory();
  void PublishTrajectory();
  void MavrosPoseCallback(const geometry_msgs::msg::PoseStamped &msg);
};
}  // namespace drone_planner
