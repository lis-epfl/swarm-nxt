#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mutex>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <swarmnxt_msgs/msg/controller_command.hpp>
#include <swarmnxt_msgs/msg/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace swarmnxt_controller {

class Controller : public rclcpp::Node {
 public:
  Controller();

 private:
  float waypoint_acceptance_radius_;
  std::mutex traj_mutex_;
  std::mutex pos_mutex_;
  nav_msgs::msg::Path traj_;
  unsigned int cur_traj_index_;
  bool reached_dest_ = false;
  bool enabled_ = false;

  tf2::Vector3 cur_target_;
  tf2::Vector3 cur_pos_;

  mavros_msgs::msg::State mavros_state_;

  // service callbacks

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      drone_pos_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr traj_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<swarmnxt_msgs::msg::Trigger>::SharedPtr enable_sub_;

  // Publisher
  rclcpp::Publisher<swarmnxt_msgs::msg::ControllerCommand>::SharedPtr
      command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;

  // helpers
  void Loop();
  void SendTrajectoryMessage();
  nav_msgs::msg::Path GetTrajectoryCopy();
  tf2::Vector3 GetPositionCopy();
  void UpdateTrajectory(const nav_msgs::msg::Path new_traj);

  // Timer
  rclcpp::TimerBase::SharedPtr loop_timer_;

  // Callbacks
  void MavrosPoseCallback(const geometry_msgs::msg::PoseStamped& msg);
  void EnableCallback(const swarmnxt_msgs::msg::Trigger& msg);
  void MavrosStateCallback(const mavros_msgs::msg::State& msg);
  void TrajectoryCallback(const nav_msgs::msg::Path& msg);
};

}  // namespace swarmnxt_controller
