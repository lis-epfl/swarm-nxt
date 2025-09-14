#include "swarmnxt_controller.h"

namespace swarmnxt_controller {

Controller::Controller() : ::rclcpp::Node("swarmnxt_controller") {
  auto logger = this->get_logger();
  RCLCPP_INFO(logger, "Starting the controller node...");

  std::string hdsm_agent_prefix = "";

  this->declare_parameter("waypoint_acceptance_radius", 0.5f);  // meters
  this->declare_parameter("hdsm_agent_prefix", "");
  this->get_parameter("waypoint_acceptance_radius",
                      waypoint_acceptance_radius_);
  this->get_parameter("hdsm_agent_prefix", hdsm_agent_prefix);

  std::string ns = this->get_namespace();

  rclcpp::QoS best_effort_qos =
      rclcpp::QoS(rclcpp::KeepLast(10))
          .reliability(rclcpp::ReliabilityPolicy::BestEffort);

  rclcpp::QoS reliable_qos =
      rclcpp::QoS(rclcpp::KeepLast(10))
          .reliability(rclcpp::ReliabilityPolicy::Reliable);

  drone_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      ns + "/mavros/local_position/pose", best_effort_qos,
      std::bind(&Controller::MavrosPoseCallback, this, std::placeholders::_1));

  enable_sub_ = this->create_subscription<swarmnxt_msgs::msg::Trigger>(
      ns + "/controller/enable", reliable_qos,
      std::bind(&Controller::EnableCallback, this, std::placeholders::_1));

  pose_type_traj_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      ns + "/trajectory", 10,
      std::bind(&Controller::PoseTypeTrajectoryCallback, this,
                std::placeholders::_1));

  hdsm_type_traj_sub_ =
      this->create_subscription<multi_agent_planner_msgs::msg::Trajectory>(
          hdsm_agent_prefix + "/traj_full", 10,
          std::bind(&Controller::HDSMTypeTrajectoryCallback, this,
                    std::placeholders::_1));
  command_pub_ = this->create_publisher<swarmnxt_msgs::msg::ControllerCommand>(
      ns + "/controller/cmd", 10);

  done_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      ns + "/controller/reached_destination", 10);

  loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(5),
                                        std::bind(&Controller::Loop, this));
}
nav_msgs::msg::Path Controller::GetTrajectoryCopy() {
  std::lock_guard<std::mutex> lock(traj_mutex_);
  nav_msgs::msg::Path path = traj_;
  return path;
}

tf2::Vector3 Controller::GetPositionCopy() {
  std::lock_guard<std::mutex> lock(pos_mutex_);
  tf2::Vector3 pos = cur_pos_;
  return pos;
}

void Controller::MavrosPoseCallback(
    const geometry_msgs::msg::PoseStamped& msg) {
  RCLCPP_INFO(this->get_logger(),
              "Got a new position. x: %5.2f, y: %5.2f, z: %5.2f",
              msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  std::lock_guard<std::mutex> lock(pos_mutex_);
  tf2::fromMsg(msg.pose.position, cur_pos_);  // todo: yaw
}

void Controller::EnableCallback(const swarmnxt_msgs::msg::Trigger& msg) {
  RCLCPP_INFO(this->get_logger(), "Got an enable message");
  enabled_ = msg.enable;
}
void Controller::UpdateTrajectoryPoseType(const nav_msgs::msg::Path new_traj) {
  std::lock_guard<std::mutex> lock(traj_mutex_);
  if (traj_type_ == TrajectoryType::TRAJ_TYPE_UNKNOWN) {
    traj_type_ = TrajectoryType::TRAJ_TYPE_POSE;
  }

  if (traj_type_ == TrajectoryType::TRAJ_TYPE_POSE) {
    traj_ = new_traj;
    cur_traj_index_ = 0;
  } else {
    RCLCPP_ERROR(
        this->get_logger(),
        "Got a pose trajectory type unexpectedly. Are two planners on?");
  }
}

void Controller::UpdateTrajectoryHDSMType(
    const multi_agent_planner_msgs::msg::Trajectory new_traj) {
  std::lock_guard<std::mutex> lock(traj_mutex_);
  if (traj_type_ == TrajectoryType::TRAJ_TYPE_UNKNOWN) {
    traj_type_ = TrajectoryType::TRAJ_TYPE_HDSM;
  }

  if (traj_type_ == TrajectoryType::TRAJ_TYPE_HDSM) {
    // Store the original HDSM trajectory for velocity and acceleration access
    hdsm_traj_ = new_traj;

    nav_msgs::msg::Path new_traj_path = nav_msgs::msg::Path();
    new_traj_path.header.stamp =
        rclcpp::Time(static_cast<int64_t>(new_traj.planning_start_time * 1e9));
    new_traj_path.header.frame_id = "world";
    size_t i = 1;
    for (const auto& state : new_traj.states) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = rclcpp::Time(new_traj_path.header.stamp) +
                          rclcpp::Duration::from_nanoseconds(
                              static_cast<int64_t>(i * new_traj.dt * 1e9));
      pose.header.frame_id = "world";
      // Extract position (indices 0-2)
      pose.pose.position.x = state.position[0];
      pose.pose.position.y = state.position[1];
      pose.pose.position.z = state.position[2];

      new_traj_path.poses.push_back(pose);
      i++;
    }

    traj_ = new_traj_path;
    cur_traj_index_ = 0;
  } else {
    RCLCPP_ERROR(
        this->get_logger(),
        "Got a HDSM trajectory type unexpectedly. Are two planners on?");
  }
}

void Controller::SendTrajectoryMessage() {
  auto cur_traj = GetTrajectoryCopy();
  auto cur_pos = GetPositionCopy();
  swarmnxt_msgs::msg::ControllerCommand msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "world";

  // Determine command type based on trajectory type
  if (traj_type_ == TrajectoryType::TRAJ_TYPE_HDSM) {
    msg.command_type_mask = swarmnxt_msgs::msg::ControllerCommand::PVA_SETPOINT;
  } else {
    msg.command_type_mask =
        swarmnxt_msgs::msg::ControllerCommand::POSITION_SETPOINT;
    msg.pose_cmd.header.stamp = this->now();
    msg.pose_cmd.header.frame_id = "world";
  }
  std_msgs::msg::Bool done_msg;
  done_msg.data = false;

  const int num_traj_points = cur_traj.poses.size();

  if (num_traj_points == 0) {
    RCLCPP_WARN(this->get_logger(), "no trajectory!");
    return;
  }

  auto current_time = this->now();

  RCLCPP_INFO(
      this->get_logger(),
      "Before timestamp search - cur_pos: [%5.2f, %5.2f, %5.2f], cur_target_: "
      "[%5.2f, %5.2f, %5.2f], traj_index: %u, num_points: %u",
      cur_pos.x(), cur_pos.y(), cur_pos.z(), cur_target_.x(), cur_target_.y(),
      cur_target_.z(), cur_traj_index_, num_traj_points);

  // Find the first waypoint that's in the future
  // Constrain to only increment index to prevent backward jumps
  bool found_future_waypoint = false;
  for (int i = cur_traj_index_; i < num_traj_points; i++) {
    rclcpp::Time traj_point_time(cur_traj.poses[i].header.stamp);
    auto lookahead_time = rclcpp::Duration::from_nanoseconds(50000000);
    if (traj_point_time > current_time + lookahead_time) {
      // Only allow forward progress or staying at current index
      if (i >= cur_traj_index_) {
        cur_traj_index_ = i;
        found_future_waypoint = true;
        RCLCPP_INFO(this->get_logger(), "Found future waypoint at index %u",
                    cur_traj_index_);
      }
      break;
    }
  }

  if (!found_future_waypoint) {
    // All waypoints are in the past, use the last one or mark as done
    if (num_traj_points > 0) {
      cur_traj_index_ = num_traj_points - 1;
      RCLCPP_INFO(this->get_logger(),
                  "No future waypoints, using last waypoint at index %u",
                  cur_traj_index_);
    } else {
      reached_dest_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "No waypoints available, marking as done");
    }
  }

  // Set the current target from the selected waypoint
  if (cur_traj_index_ < num_traj_points) {
    tf2::fromMsg(cur_traj.poses.at(cur_traj_index_).pose.position, cur_target_);
    RCLCPP_INFO(this->get_logger(),
                "Set target from waypoint %u: [%5.2f, %5.2f, %5.2f]",
                cur_traj_index_, cur_target_.x(), cur_target_.y(),
                cur_target_.z());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Did not set any setpoint!");
  }

  RCLCPP_INFO(this->get_logger(),
              "Setting target: x: %5.2f, y: %5.2f, z: %5.2f", cur_target_.x(),
              cur_target_.y(), cur_target_.z());

  if (msg.command_type_mask ==
      swarmnxt_msgs::msg::ControllerCommand::PVA_SETPOINT) {
    // Use HDSM trajectory data for position, velocity, acceleration
    if (cur_traj_index_ < hdsm_traj_.states.size()) {
      const auto& state = hdsm_traj_.states[cur_traj_index_];

      // Create PositionTarget message
      msg.raw_cmd.header.stamp = this->now();
      msg.raw_cmd.header.frame_id = "world";
      msg.raw_cmd.coordinate_frame =
          mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
      msg.raw_cmd.type_mask =
          0;  // Use all fields (position, velocity, acceleration)

      // Position
      msg.raw_cmd.position.x = state.position[0];
      msg.raw_cmd.position.y = state.position[1];
      msg.raw_cmd.position.z = state.position[2];

      // Velocity
      msg.raw_cmd.velocity.x = state.velocity[0];
      msg.raw_cmd.velocity.y = state.velocity[1];
      msg.raw_cmd.velocity.z = state.velocity[2];

      // Acceleration
      msg.raw_cmd.acceleration_or_force.x = state.acceleration[0];
      msg.raw_cmd.acceleration_or_force.y = state.acceleration[1];
      msg.raw_cmd.acceleration_or_force.z = state.acceleration[2];
    }
  } else {
    // Traditional position-only command
    tf2::toMsg(cur_target_, msg.pose_cmd.pose.position);
  }

  done_pub_->publish(done_msg);
  command_pub_->publish(msg);
}

void Controller::Loop() {
  auto& clk = *this->get_clock();
  auto logger = this->get_logger();
  RCLCPP_INFO_THROTTLE(logger, clk, 1000, "ctrl enabled: %d", enabled_);
  if (enabled_) {  // and the time of the last received pose plus goal are close
    SendTrajectoryMessage();

    auto cur_pos = GetPositionCopy();
    tf2::Vector3 destination;
    tf2::fromMsg(traj_.poses.back().pose.position, destination);
    reached_dest_ =
        (tf2::tf2Distance(cur_pos, destination) < waypoint_acceptance_radius_);
  }
}

void Controller::PoseTypeTrajectoryCallback(const nav_msgs::msg::Path& msg) {
  UpdateTrajectoryPoseType(msg);
}

void Controller::HDSMTypeTrajectoryCallback(
    const multi_agent_planner_msgs::msg::Trajectory& msg) {
  UpdateTrajectoryHDSMType(msg);
}

void Controller::MavrosStateCallback(const mavros_msgs::msg::State& msg) {
  RCLCPP_INFO(this->get_logger(), "Got mavros state");
  mavros_state_ = msg;
}

}  // namespace swarmnxt_controller
