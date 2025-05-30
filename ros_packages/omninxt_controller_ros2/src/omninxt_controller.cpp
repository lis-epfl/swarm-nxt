#include "omninxt_controller.h"

namespace omninxt_controller {

Controller::Controller() : ::rclcpp::Node("omninxt_controller") {
  auto logger = this->get_logger();
  RCLCPP_INFO(logger, "Starting the controller node...");

  std::string ns = this->get_namespace();

  takeoff_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "takeoff", std::bind(&Controller::TakeoffService, this,
                           std::placeholders::_1, std::placeholders::_2));
  land_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "land", std::bind(&Controller::LandService, this, std::placeholders::_1,
                        std::placeholders::_2));
  start_traj_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "start_trajectory",
      std::bind(&Controller::StartTrajectoryService, this,
                std::placeholders::_1, std::placeholders::_2));

  rclcpp::QoS best_effort_qos =
      rclcpp::QoS(rclcpp::KeepLast(10))
          .reliability(rclcpp::ReliabilityPolicy::BestEffort);

  drone_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      ns + "/local_position/pose", best_effort_qos,
      std::bind(&Controller::MavrosPoseCallback, this, std::placeholders::_1));

  safe_traj_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      ns + "/trajectory_safe", 10,
      std::bind(&Controller::TrajectoryCallback, this, std::placeholders::_1));

  // DeclareRosParameters();
  // InitializeRosParameters();

  setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      ns + "/setpoint_position/local", 10);

  loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(30),
                                        std::bind(&Controller::Loop, this));

  set_mode_client_ =
      this->create_client<mavros_msgs::srv::SetMode>(ns + "/set_mode");
  arming_client_ =
      this->create_client<mavros_msgs::srv::CommandBool>(ns + "/cmd/arming");
  takeoff_client_ =
      this->create_client<mavros_msgs::srv::CommandTOL>(ns + "/cmd/takeoff");
  land_client_ =
      this->create_client<mavros_msgs::srv::CommandTOL>(ns + "/cmd/land");
}

void Controller::SendTrajectoryMessage() {
  const float DISTANCE_TOL = 0.5f;  // 50 cm
  std::lock_guard<std::mutex> lock(traj_mutex_);
  geometry_msgs::msg::PoseStamped msg;
  // get distance from current target
  if (reached_dest_) {
    RCLCPP_INFO(this->get_logger(), "Reached destination, so not advancing");
    tf2::toMsg(cur_pos_, msg.pose.position);
    setpoint_pub_->publish(msg);
    return;
  }

  float distance = tf2::tf2Distance(cur_pos_, cur_target_);
  const int num_traj_points = traj_.poses.size();
  while (distance < DISTANCE_TOL) {
    if (++cur_traj_index_ < num_traj_points) {
      tf2::fromMsg(traj_.poses.at(cur_traj_index_), cur_target_);
      distance = tf2::tf2Distance(cur_pos_, cur_target_);
    } else {
      reached_dest_ = true;
    }
  }
  tf2::toMsg(cur_target_, msg.pose.position);
  setpoint_pub_->publish(msg);
}

bool Controller::change_px4_state(const std::string& mode) {
  auto logger = this->get_logger();
  const std::map<std::string, ControllerState> mode_map = {
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

  if (arm) {
    mavros_msgs::srv::CommandBool::Request arm_request;
    arm_request.value = true;

    if (arming_client_->wait_for_service(std::chrono::seconds(1))) {
      auto arm_future = arming_client_->async_send_request(
          std::make_shared<mavros_msgs::srv::CommandBool::Request>(
              arm_request));
      if (arm_future.wait_for(std::chrono::seconds(1)) ==
          std::future_status::ready) {
        auto response = arm_future.get();
        if (response->success) {
          RCLCPP_INFO(logger, "Armed successfully");
        } else {
          RCLCPP_ERROR(logger, "Failed to arm");
          return false;
        }
      } else {
        RCLCPP_ERROR(logger, "Arming service call timed out");
        return false;
      }
    } else {
      RCLCPP_ERROR(logger, "Arming service not available");
      return false;
    }
  }

  return true;
}

void Controller::TakeoffService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (state_ == ControllerState::Idle) {
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
  if (state_ == ControllerState::FollowingTrajectory) {
    mavros_msgs::srv::CommandTOL::Request land_request;
    land_request.altitude = 0.0f;
    land_request.latitude = 0.0f;
    land_request.longitude = 0.0f;
    land_request.yaw = 0.0f;

    land_client_->async_send_request(
        std::make_shared<mavros_msgs::srv::CommandTOL::Request>(land_request),
        [this](
            rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
          auto response = future.get();
          if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Land command sent successfully.");
            state_ = ControllerState::Landing;
          } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to send land command.");
          }
        });
    response->success = true;
    response->message = "Landing!";
  } else {
    response->success = false;
    response->message = "Cannot land: not in flight.";
  }
}

void Controller::StartTrajectoryService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
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
      // do nothing
      break;
    case ControllerState::TakingOff:
      // start sending trajectory messages
      break;
    case ControllerState::FollowingTrajectory:
      // send trajectory message
      // change mode once critical number have been sent
      break;
    case ControllerState::Landing:
      // stop sending messages
      // switch mode

      break;
  }
}

void Controller::MavrosPoseCallback(
    const geometry_msgs::msg::PoseStamped& msg) {
  RCLCPP_INFO(this->get_logger(), "Got a new position");
  tf2::fromMsg(msg, cur_pos_);
}

void Controller::TrajectoryCallback(const nav_msgs::msg::Path& msg) {
  RCLCPP_INFO(this->get_logger(), "Got a new path");
  // assumes that all trajectories are planned from the current position
  // does not support trajectories that start behind the drone and continue past
  // it
  std::lock_guard<std::mutex> lock(traj_mutex_);
  traj_ = msg;
  cur_traj_index_ = 0;
  tf2::Vector3 destination;
  tf2::fromMsg(traj_.poses.back(), destination);
  reached_dest_ = (tf2::tf2Distance(cur_pos_, destination) < DISTANCE_TOL);
}

}  // namespace omninxt_controller
