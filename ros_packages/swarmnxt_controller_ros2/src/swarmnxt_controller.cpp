#include "swarmnxt_controller.h"

namespace swarmnxt_controller {

Controller::Controller() : ::rclcpp::Node("swarmnxt_controller") {
  auto logger = this->get_logger();
  RCLCPP_INFO(logger, "Starting the controller node...");
  
  this->declare_parameter("waypoint_acceptance_radius", 0.5f); // meters
  this->get_parameter("waypoint_acceptance_radius", waypoint_acceptance_radius_);

  std::string ns = this->get_namespace();

  takeoff_srv_ = this->create_service<std_srvs::srv::Trigger>(
      ns + "/controller/takeoff",
      std::bind(&Controller::TakeoffService, this, std::placeholders::_1,
                std::placeholders::_2));
  land_srv_ = this->create_service<std_srvs::srv::Trigger>(
      ns + "/controller/land",
      std::bind(&Controller::LandService, this, std::placeholders::_1,
                std::placeholders::_2));
  start_traj_srv_ = this->create_service<std_srvs::srv::Trigger>(
      ns + "/controller/start_trajectory",
      std::bind(&Controller::StartTrajectoryService, this,
                std::placeholders::_1, std::placeholders::_2));

  rclcpp::QoS best_effort_qos =
      rclcpp::QoS(rclcpp::KeepLast(10))
          .reliability(rclcpp::ReliabilityPolicy::BestEffort);

  drone_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      ns + "/mavros/local_position/pose", best_effort_qos,
      std::bind(&Controller::MavrosPoseCallback, this, std::placeholders::_1));

  safe_traj_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      ns + "/trajectory", 10,
      std::bind(&Controller::TrajectoryCallback, this, std::placeholders::_1));

  state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      ns + "/mavros/state", 10,
      std::bind(&Controller::MavrosStateCallback, this, std::placeholders::_1));

  setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      ns + "/mavros/setpoint_position/local", 10);

  done_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      ns + "/controller/reached_destination", 10);

  loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(30),
                                        std::bind(&Controller::Loop, this));

  set_mode_client_ =
      this->create_client<mavros_msgs::srv::SetMode>(ns + "/mavros/set_mode");
  arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
      ns + "/mavros/cmd/arming");
  takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>(
      ns + "/mavros/cmd/takeoff");
  land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>(
      ns + "/mavros/cmd/land");
}

nav_msgs::msg::Path Controller::GetTrajectoryCopy() {
  std::lock_guard<std::mutex> lock(traj_mutex_); 
  nav_msgs::msg::Path path = traj_; 
  return path; 
}

void Controller::UpdateTrajectory(nav_msgs::msg::Path new_traj) { 
  std::lock_guard<std::mutex> lock(traj_mutex_);
  traj_ = new_traj;
}

void Controller::SendTrajectoryMessage() {
  auto cur_traj = GetTrajectoryCopy(); 
  geometry_msgs::msg::PoseStamped msg;
  std_msgs::msg::Bool done_msg;
  done_msg.data = false;

  float distance = tf2::tf2Distance(cur_pos_, cur_target_);
  const unsigned int num_traj_points = cur_traj.poses.size();

  if (num_traj_points == 0) {
    RCLCPP_WARN(this->get_logger(), "no trajectory!");
    return;
  }
  num_traj_messages_sent_ += 1;

  // get distance from current target
  if (reached_dest_) {
    RCLCPP_INFO(this->get_logger(), "Reached destination, so not advancing");
    tf2::toMsg(cur_pos_, msg.pose.position);
    setpoint_pub_->publish(msg);
    done_msg.data = true;
    done_pub_->publish(done_msg);
    return;
  }

  while (distance < waypoint_acceptance_radius_) {
    if (cur_traj_index_ < num_traj_points) {
      tf2::fromMsg(cur_traj.poses.at(cur_traj_index_).pose.position, cur_target_);
      distance = tf2::tf2Distance(cur_pos_, cur_target_);
      cur_traj_index_++;
    } else {
      reached_dest_ = true;
      break;
    }
  }
  tf2::toMsg(cur_target_, msg.pose.position);
  done_pub_->publish(done_msg);
  setpoint_pub_->publish(msg);
}

bool Controller::change_px4_state(const std::string& mode) {
  auto logger = this->get_logger();
  const std::map<std::string, ControllerState> mode_map = {
      {"AUTO.LOITER",
       ControllerState::Idle},  // todo: this needs to be more complicated
      {"AUTO.LAND", ControllerState::Landing},
      {"AUTO.TAKEOFF", ControllerState::TakingOff},
      {"OFFBOARD", ControllerState::FollowingTrajectory}};
  if (mode_map.find(mode) == mode_map.end()) {
    RCLCPP_ERROR(logger, "Invalid mode: %s", mode.c_str());
    return false;
  }
  mavros_msgs::srv::SetMode::Request set_mode_request;
  set_mode_request.custom_mode = mode;

  if (set_mode_client_->wait_for_service(std::chrono::seconds(1))) {
    auto set_mode_future = set_mode_client_->async_send_request(
        std::make_shared<mavros_msgs::srv::SetMode::Request>(set_mode_request),
        [this, logger, mode, mode_map](
            rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
          auto response = future.get();
          if (response->mode_sent) {
            RCLCPP_INFO(logger, "Mode changed to %s", mode.c_str());
            this->state_ = mode_map.at(mode);
          } else {
            RCLCPP_ERROR(logger, "Failed to change mode to %s", mode.c_str());
          }
        });
  } else {
    RCLCPP_ERROR(logger, "SetMode service not available");
    return false;
  }

  return true;
}

void Controller::TakeoffService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (state_ == ControllerState::Idle) {
    change_px4_state("AUTO.TAKEOFF");
    response->success = true;
    response->message = "Taking off!";
  } else {
    response->success = false;
    response->message = "Cannot take off: not in Idle state.";
  }
}

void Controller::LandService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (state_ != ControllerState::Idle && state_ != ControllerState::Landing) {
    RCLCPP_INFO(this->get_logger(), "Sending land command");
    change_px4_state("AUTO.LAND");
    response->success = true;
    response->message = "Landing!";
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to send land command");
    response->success = false;
    response->message = "Cannot land: not in flight.";
  }
}

void Controller::StartTrajectoryService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  RCLCPP_ERROR(this->get_logger(), "Following trajectory state?");
  if (state_ == ControllerState::TakingOff) {
    state_ = ControllerState::FollowingTrajectory;
    response->success = true;
    response->message = "Started following trajectory!";
  } else {
    response->success = false;
    response->message = "Cannot start trajectory: not in TakingOff state.";
  }
}

void Controller::Loop() {
  auto logger = this->get_logger();

  switch (state_) {
    case ControllerState::Idle:
      RCLCPP_INFO(logger, "state: idle");
      break;
    case ControllerState::TakingOff:
      RCLCPP_INFO(logger, "state: taking off");
      SendTrajectoryMessage();
      if (num_traj_messages_sent_ > 100 /* && altitude correct */) {
        // we are done with taking off, move to follow traj
        // dont do this yet!
        change_px4_state("OFFBOARD");
      }
      break;
    case ControllerState::FollowingTrajectory:
      RCLCPP_INFO(logger, "state: following trajectory");
      SendTrajectoryMessage();
      break;
    case ControllerState::Landing:
      RCLCPP_INFO(logger, "state: landing");
      // stop sending messages
      // switch mode
      if (!mavros_state_.armed) {
        // landing done
        RCLCPP_INFO(this->get_logger(),
                    "Landing finished, changing internal state to idle");
        state_ = ControllerState::Idle;
        num_traj_messages_sent_ = 0;
      }

      break;
  }

  if (state_ != ControllerState::Idle && !mavros_state_.armed) {
    if (mavros_state_.mode == "OFFBOARD") {
      // get the system back to idle

      RCLCPP_INFO(logger,
                  "Transitioning to non-offboad since controller was disarmed");
      change_px4_state("AUTO.LOITER");
    }
  }
}

void Controller::MavrosPoseCallback(
    const geometry_msgs::msg::PoseStamped& msg) {
  RCLCPP_INFO(this->get_logger(), "Got a new position");
  tf2::fromMsg(msg.pose.position, cur_pos_);  // todo: yaw
}

void Controller::TrajectoryCallback(const nav_msgs::msg::Path& msg) {
  RCLCPP_INFO(this->get_logger(), "Got a new path");
  // assumes that all trajectories are planned from the current position
  // does not support trajectories that start behind the drone and continue past
  // it
  
  UpdateTrajectory(msg); 
  cur_traj_index_ = 0;
  tf2::Vector3 destination;
  tf2::fromMsg(msg.poses.back().pose.position, destination);
  reached_dest_ = (tf2::tf2Distance(cur_pos_, destination) < waypoint_acceptance_radius_);
}

void Controller::MavrosStateCallback(const mavros_msgs::msg::State& msg) {
  RCLCPP_INFO(this->get_logger(), "Got mavros state");
  mavros_state_ = msg;
}
}  // namespace swarmnxt_controller
