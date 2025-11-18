#pragma once

#include <chrono>
#include <string>
#include <regex>
#include <cstdint>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "px4_msgs/msg/actuator_motors.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_thrust_setpoint.hpp"
#include "px4_msgs/msg/vehicle_torque_setpoint.hpp"
#include "px4_ros_com/frame_transforms.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "swarmnxt_msgs/msg/controller_command.hpp"
#include "swarmnxt_msgs/msg/plane.hpp"
#include "swarmnxt_msgs/srv/get_planes.hpp"
#include <atomic>

namespace safety_checker {

enum SafetyStatus : uint8_t {
  SAFE = 0,
  UNSAFE_OUT_OF_BOUNDS = 1 << 0,
  UNSAFE_COMMAND_SEND_RATE = 1 << 1,
  UNSAFE_HIGH_COVARIANCE = 1 << 2
};

enum class DroneState { Idle, TakingOff, Hold, Flying, Landing };

class SafetyChecker : public ::rclcpp::Node {
public:
  SafetyChecker();

  bool IsPointInHull(const geometry_msgs::msg::Point &point);
  void
  HandlePoseMessage(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void HandleTrajectoryMessage(const nav_msgs::msg::Path &msg); // deprecated
  void
  HandleControllerCommand(const swarmnxt_msgs::msg::ControllerCommand &msg);
  void HandleVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void GetPlanesCallback(
      const std::shared_ptr<swarmnxt_msgs::srv::GetPlanes::Request> request,
      std::shared_ptr<swarmnxt_msgs::srv::GetPlanes::Response> response);

  void
  SetModeForwarder(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
  void LandNow();

private:
  std::atomic<uint8_t> current_nav_state_;
  uint8_t target_system_ = 1;
  std::vector<std::vector<double>> planes_;
  double position_uncertainty_threshold_;

  uint8_t safety_flags_ = SafetyStatus::SAFE;
  double plane_offset_; // meters, positive

  // parameters
  std::string position_topic_suffix_;
  std::string trajectory_topic_suffix_;

  // subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr
      trajectory_sub_; // deprecated
  rclcpp::Subscription<swarmnxt_msgs::msg::ControllerCommand>::SharedPtr
      command_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
      vehicle_status_sub_;

  // publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr safe_trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr
      rates_cmd_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr
      torque_cmd_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr
      thrust_cmd_pub_;
  rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr motors_cmd_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
      offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
      vehicle_command_pub_;

  // services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_mode_server_;
  rclcpp::Service<swarmnxt_msgs::srv::GetPlanes>::SharedPtr planes_srv_;
};

} // namespace safety_checker
