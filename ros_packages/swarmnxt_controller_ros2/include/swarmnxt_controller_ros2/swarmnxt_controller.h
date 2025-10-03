#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <multi_agent_planner_msgs/msg/state.hpp>
#include <multi_agent_planner_msgs/msg/trajectory.hpp>
#include <mutex>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <swarmnxt_msgs/msg/controller_command.hpp>
#include <swarmnxt_msgs/msg/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace swarmnxt_controller {

typedef enum {
  TRAJ_TYPE_UNKNOWN,
  TRAJ_TYPE_POSE,
  TRAJ_TYPE_HDSM
} TrajectoryType;

class Controller : public rclcpp::Node {
 public:
  Controller();

 private:
  float waypoint_acceptance_radius_;
  std::mutex traj_mutex_;
  std::mutex pos_mutex_;
  nav_msgs::msg::Path traj_;
  multi_agent_planner_msgs::msg::Trajectory hdsm_traj_;
  TrajectoryType traj_type_ = TRAJ_TYPE_UNKNOWN;
  int cur_traj_index_;
  bool reached_dest_ = false;
  bool enabled_ = false;

  tf2::Vector3 cur_target_;
  tf2::Vector3 cur_pos_;
  tf2::Vector3 cur_velocity_;
  tf2::Vector3 cur_acceleration_;

  px4_msgs::msg::VehicleStatus vehicle_status_;

  // service callbacks

  // Subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      drone_pos_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pose_type_traj_sub_;
  rclcpp::Subscription<multi_agent_planner_msgs::msg::Trajectory>::SharedPtr
      hdsm_type_traj_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
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
  void UpdateTrajectoryPoseType(const nav_msgs::msg::Path new_traj);
  void UpdateTrajectoryHDSMType(
      const multi_agent_planner_msgs::msg::Trajectory new_traj);

  // Timer
  rclcpp::TimerBase::SharedPtr loop_timer_;

  // Callbacks
  void VehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition& msg);
  void EnableCallback(const swarmnxt_msgs::msg::Trigger& msg);
  void VehicleStatusCallback(const px4_msgs::msg::VehicleStatus& msg);
  void PoseTypeTrajectoryCallback(const nav_msgs::msg::Path& msg);
  void HDSMTypeTrajectoryCallback(
      const multi_agent_planner_msgs::msg::Trajectory& msg);
};

}  // namespace swarmnxt_controller
