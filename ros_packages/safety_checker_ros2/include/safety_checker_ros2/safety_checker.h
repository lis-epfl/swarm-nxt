#pragma once

#include <chrono>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "swarmnxt_msgs/msg/controller_command.hpp"
#include "swarmnxt_msgs/msg/plane.hpp"

namespace safety_checker {

enum SafetyStatus : uint8_t {
  SAFE = 0,
  UNSAFE_OUT_OF_BOUNDS = 1 << 0,
  UNSAFE_COMMAND_SEND_RATE = 1 << 1,
  UNSAFE_OTHER = 1 << 2
};

enum class DroneState { Idle, TakingOff, Hold, Flying, Landing };

class SafetyChecker : public ::rclcpp::Node {
 public:
  SafetyChecker();

  void LoadHullFromFile(const std::filesystem::path &filepath);
  bool IsPointInHull(const geometry_msgs::msg::Point &point);
  void HandlePoseMessage(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void HandleTrajectoryMessage(const nav_msgs::msg::Path &msg);  // deprecated
  void HandleControllerCommand(
      const swarmnxt_msgs::msg::ControllerCommand &msg);

  void SetModeForwarder(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
  void LandNow();

  geometry_msgs::msg::Point ProjectPointToClosestPlane(
      const geometry_msgs::msg::Point &point);
  void ClearPlanes();
  
  std::vector<swarmnxt_msgs::msg::Plane> GetPlanes();

 private:
  std::vector<swarmnxt_msgs::msg::Plane> planes_;

  bool are_planes_valid_ = false;
  uint8_t safety_flags_ = SafetyStatus::SAFE;
  float plane_offset_;  // meters, positive

  // parameters
  std::string position_topic_suffix_;
  std::string trajectory_topic_suffix_;

  // subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr
      trajectory_sub_;  // deprecated
  rclcpp::Subscription<swarmnxt_msgs::msg::ControllerCommand>::SharedPtr
      command_sub_;

  // publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr safe_trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
      position_cmd_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

  // servers
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_mode_server_;
};

}  // namespace safety_checker
