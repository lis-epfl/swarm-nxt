#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <swarmnxt_controller_ros2/msg/controller_command.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mutex>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace swarmnxt_controller {

// Enum for controller states
enum class ControllerState { Idle, TakingOff, FollowingTrajectory, Landing };

class Controller : public rclcpp::Node {
 public:
  Controller();

 private:
  // State
  ControllerState state_{ControllerState::Idle};

  float waypoint_acceptance_radius_;
  std::mutex traj_mutex_;
  std::mutex pos_mutex_;
  nav_msgs::msg::Path traj_;
  unsigned int cur_traj_index_;
  bool reached_dest_ = false;

  tf2::Vector3 cur_target_;
  tf2::Vector3 cur_pos_;
  unsigned int num_traj_messages_sent_ = 0;

  mavros_msgs::msg::State mavros_state_;

  // Service servers
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeoff_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_traj_srv_;

  // service clients
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
  // service callbacks

  void TakeoffService(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void LandService(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void StartTrajectoryService(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      drone_pos_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr safe_traj_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  // Publisher
  rclcpp::Publisher<swarmnxt_controller_ros2::msg::ControllerCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;

  // helpers
  void SendTrajectoryMessage();
  nav_msgs::msg::Path GetTrajectoryCopy();
  tf2::Vector3 GetPositionCopy();
  void UpdateTrajectory(const nav_msgs::msg::Path new_traj);

  // Timer
  rclcpp::TimerBase::SharedPtr loop_timer_;

  // Callbacks
  void Loop();
  void MavrosPoseCallback(const geometry_msgs::msg::PoseStamped& msg);
  void MavrosStateCallback(const mavros_msgs::msg::State& msg);
  void TrajectoryCallback(const nav_msgs::msg::Path& msg);
  bool ChangePX4State(const std::string& mode);
};

}  // namespace swarmnxt_controller
