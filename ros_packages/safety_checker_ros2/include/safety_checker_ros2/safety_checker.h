#pragma once

#include <chrono>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "safety_checker_ros2/msg/plane.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "swarmnxt_controller_ros2/msg/controller_command.hpp"

namespace safety_checker {

enum SafetyStatus : uint8_t {
  SAFE = 0,
  UNSAFE_OUT_OF_BOUNDS = 1 << 0,
  UNSAFE_COMMAND_SEND_RATE = 1 << 1,
  UNSAFE_OTHER = 1 << 2
};

enum class DroneState { Idle, TakingOff, Flying, Landing };

class SafetyChecker : public ::rclcpp::Node {
 public:
  SafetyChecker();

  void LoadHullFromFile(const std::filesystem::path &filepath);
  bool IsPointInHull(const geometry_msgs::msg::Point &point);
  void HandlePoseMessage(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void HandleTrajectoryMessage(const nav_msgs::msg::Path &msg);
  void HandleControllerCommand(
      const swarmnxt_controller_ros2::msg::ControllerCommand &msg);

  void LandNow();
  void LandServer(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                  const std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
  void EStop();  // disarm immediately. not to be used except for emergencies!

  bool RequestStateTransition(const DroneState to_state);
  geometry_msgs::msg::Point ProjectPointToClosestPlane(
      const geometry_msgs::msg::Point &point);
  void ClearPlanes();

  std::vector<safety_checker_ros2::msg::Plane> GetPlanes();

 private:
  std::vector<safety_checker_ros2::msg::Plane> planes_;
  swarmnxt_controller_ros2::msg::ControllerCommand latest_cmd_;

  void Loop();
  bool are_planes_valid_ = false;
  uint8_t safety_flags_ = SafetyStatus::SAFE;
  DroneState drone_state_{DroneState::Idle};
  std::string topic_prefix_ = "";
  float plane_offset_;  // meters, positive

  rclcpp::TimerBase::SharedPtr command_timer_;

  // parameters
  std::string position_topic_suffix_;
  std::string trajectory_topic_suffix_;

  // subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr
      trajectory_sub_;  // deprecated
  rclcpp::Subscription<
      swarmnxt_controller_ros2::msg::ControllerCommand>::SharedPtr command_sub_;

  // publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr safe_trajectory_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      position_cmd_pub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr rate_cmd_pub_;

  // servers
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_service_;
};

}  // namespace safety_checker
