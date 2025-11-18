#include "safety_checker.h"

#include <fstream>
namespace safety_checker {

SafetyChecker::SafetyChecker()
    : ::rclcpp::Node("safety_checker"), current_nav_state_(0) {
  std::string filepath;

  this->declare_parameter<std::vector<double>>("planes", std::vector<double>{});
  std::vector<double> planes_tmp;
  this->get_parameter("planes", planes_tmp);

  for (int i = 0; i < planes_tmp.size() / 4; i++) {
    std::vector<double> plane = {planes_tmp[4 * i], planes_tmp[4 * i + 1],
                                 planes_tmp[4 * i + 2], planes_tmp[4 * i + 3]};
    planes_.push_back(plane);
  }

  this->declare_parameter("plane_offset", 0.3); // meters
  this->get_parameter("plane_offset", plane_offset_);
  RCLCPP_INFO(this->get_logger(), "Using plane offset: %5.2f", plane_offset_);

  this->declare_parameter("position_uncertainty_threshold",
                          0.2); // Default: 0.2 meters
  this->get_parameter("position_uncertainty_threshold",
                      position_uncertainty_threshold_);
  RCLCPP_INFO(this->get_logger(),
              "Using position uncertainty threshold: %5.2f m",
              position_uncertainty_threshold_);

  // --- NAMESPACE PARSING LOGIC ---
  std::string ns = this->get_namespace();
  if (ns != "/") {
    ns.erase(0, 1); // Remove leading '/' (e.g., "/nxt7" -> "nxt7")
    std::smatch match;
    std::regex re("[0-9]+$"); // Find one or more digits at the end

    if (std::regex_search(ns, match, re) && match.size() > 0) {
      try {
        target_system_ = static_cast<uint8_t>(std::stoi(match.str(0)));
        RCLCPP_INFO(this->get_logger(),
                    "Target system ID set to %u from namespace '%s'",
                    target_system_, this->get_namespace());
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(),
                    "Could not parse number from namespace '%s'. Defaulting to "
                    "1. Error: %s",
                    this->get_namespace(), e.what());
        target_system_ = 1;
      }
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "No number found in namespace '%s'. Defaulting to 1.",
                  this->get_namespace());
      target_system_ = 1;
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "No namespace set. Defaulting to 1.");
    target_system_ = 1;
  }

  rclcpp::QoS best_effort_qos =
      rclcpp::QoS(rclcpp::KeepLast(10))
          .reliability(rclcpp::ReliabilityPolicy::BestEffort);

  pose_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", best_effort_qos,
      std::bind(&SafetyChecker::HandlePoseMessage, this,
                std::placeholders::_1));

  command_sub_ = create_subscription<swarmnxt_msgs::msg::ControllerCommand>(
      "/controller/cmd", 10,
      std::bind(&SafetyChecker::HandleControllerCommand, this,
                std::placeholders::_1));

  vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", best_effort_qos,
      std::bind(&SafetyChecker::HandleVehicleStatus, this,
                std::placeholders::_1));

  planes_srv_ = this->create_service<swarmnxt_msgs::srv::GetPlanes>(
      "/get_planes", std::bind(&SafetyChecker::GetPlanesCallback, this,
                               std::placeholders::_1, std::placeholders::_2));

  // deprecated
  safe_trajectory_pub_ =
      create_publisher<nav_msgs::msg::Path>("/trajectory_safe", 10);

  rates_cmd_pub_ = create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
      "/fmu/in/vehicle_rates_setpoint", 10);

  torque_cmd_pub_ = create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
      "/fmu/in/vehicle_torque_setpoint", 10);

  thrust_cmd_pub_ = create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
      "/fmu/in/vehicle_thrust_setpoint", 10);

  motors_cmd_pub_ = create_publisher<px4_msgs::msg::ActuatorMotors>(
      "/fmu/in/actuator_motors", 10);

  offboard_control_mode_pub_ =
      create_publisher<px4_msgs::msg::OffboardControlMode>(
          "/fmu/in/offboard_control_mode", 10);

  vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", 10);

  set_mode_server_ = create_service<std_srvs::srv::Trigger>(
      "/safety/set_offboard_mode",
      std::bind(&SafetyChecker::SetModeForwarder, this, std::placeholders::_1,
                std::placeholders::_2));
}

void SafetyChecker::HandleVehicleStatus(
    const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
  // Store the current navigation state
  current_nav_state_.store(msg->nav_state);
}

void SafetyChecker::GetPlanesCallback(
    const std::shared_ptr<swarmnxt_msgs::srv::GetPlanes::Request> request,
    std::shared_ptr<swarmnxt_msgs::srv::GetPlanes::Response> response) {
  (void)request;
  // Request is empty, suppress unused warning
  for (const auto &plane : planes_) {
    swarmnxt_msgs::msg::Plane plane_msg;
    plane_msg.normal.x = plane[0];
    plane_msg.normal.y = plane[1];
    plane_msg.normal.z = plane[2];
    plane_msg.d = plane[3];
    response->planes.push_back(plane_msg);
  }
}

void SafetyChecker::HandleControllerCommand(
    const swarmnxt_msgs::msg::ControllerCommand &msg) {
  auto cmd = msg;
  auto cur_time = this->now();
  auto cmd_age = cur_time - cmd.header.stamp;
  // make sure the command isn't stale
  if (cmd_age.nanoseconds() > 20E6) {
    safety_flags_ |= SafetyStatus::UNSAFE_COMMAND_SEND_RATE;
    RCLCPP_ERROR(this->get_logger(),
                 "Command was too old, stopping. Age: %5.2fms",
                 cmd_age.nanoseconds() / 1e6);
    LandNow();
  }

  if (safety_flags_ == SafetyStatus::SAFE) {
    // Set offboard control mode flags based on command type mask
    px4_msgs::msg::OffboardControlMode offboard_mode{};
    offboard_mode.timestamp = this->now().nanoseconds() / 1000;

    // Initialize all flags to false
    offboard_mode.position = false;
    offboard_mode.velocity = false;
    offboard_mode.acceleration = false;
    offboard_mode.attitude = false;
    offboard_mode.body_rate = false;
    offboard_mode.thrust_and_torque = false;
    offboard_mode.direct_actuator = false;

    switch (cmd.command_type_mask) {
    case swarmnxt_msgs::msg::ControllerCommand::RATE_SETPOINT: {
      // Configure offboard mode for body rate control
      offboard_mode.body_rate = true;

      // Publish offboard control mode
      offboard_control_mode_pub_->publish(offboard_mode);

      if (current_nav_state_.load() ==
          px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        // Create and publish vehicle rates setpoint
        px4_msgs::msg::VehicleRatesSetpoint rates_msg{};
        rates_msg.timestamp = this->now().nanoseconds() / 1000;

        // Assuming rate_cmd has [roll_rate, pitch_rate, yaw_rate] in rad/s
        if (cmd.rate_cmd.size() >= 3) {
          rates_msg.roll = cmd.rate_cmd[0];
          rates_msg.pitch = cmd.rate_cmd[1];
          rates_msg.yaw = cmd.rate_cmd[2];
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "Rate command has insufficient elements: %zu",
                      cmd.rate_cmd.size());
        }

        // Use thrust_cmd for thrust (normalized 0-1)
        rates_msg.thrust_body[0] = 0.0; // x thrust
        rates_msg.thrust_body[1] = 0.0; // y thrust
        rates_msg.thrust_body[2] =
            cmd.thrust_cmd; // z thrust (negative for upward)

        rates_cmd_pub_->publish(rates_msg);
      }
      break;
    }

    case swarmnxt_msgs::msg::ControllerCommand::TORQUE_SETPOINT: {
      // Configure offboard mode for thrust and torque control
      offboard_mode.thrust_and_torque = true;

      // Publish offboard control mode
      offboard_control_mode_pub_->publish(offboard_mode);

      if (current_nav_state_.load() ==
          px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        // Create and publish vehicle torque setpoint
        px4_msgs::msg::VehicleTorqueSetpoint torque_msg{};
        torque_msg.timestamp = this->now().nanoseconds() / 1000;

        // Assuming torque_cmd has [roll_torque, pitch_torque, yaw_torque] in Nm
        if (cmd.torque_cmd.size() >= 3) {
          torque_msg.xyz[0] = cmd.torque_cmd[0]; // roll torque
          torque_msg.xyz[1] = cmd.torque_cmd[1]; // pitch torque
          torque_msg.xyz[2] = cmd.torque_cmd[2]; // yaw torque
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "Torque command has insufficient elements: %zu",
                      cmd.torque_cmd.size());
        }

        torque_cmd_pub_->publish(torque_msg);

        // Create and publish separate vehicle thrust setpoint
        px4_msgs::msg::VehicleThrustSetpoint thrust_msg{};
        thrust_msg.timestamp = this->now().nanoseconds() / 1000;

        // Set thrust (normalized 0-1)
        thrust_msg.xyz[0] = 0.0;            // x thrust
        thrust_msg.xyz[1] = 0.0;            // y thrust
        thrust_msg.xyz[2] = cmd.thrust_cmd; // z thrust (negative for upward)

        thrust_cmd_pub_->publish(thrust_msg);
      }
      break;
    }

    case swarmnxt_msgs::msg::ControllerCommand::MOTOR_SETPOINT: {
      // Configure offboard mode for direct actuator control
      offboard_mode.direct_actuator = true;

      // Publish offboard control mode
      offboard_control_mode_pub_->publish(offboard_mode);

      if (current_nav_state_.load() ==
          px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        // Create and publish actuator motors setpoint
        px4_msgs::msg::ActuatorMotors motors_msg{};
        motors_msg.timestamp = this->now().nanoseconds() / 1000;

        // Copy motor commands (up to 12 motors supported by PX4)
        size_t num_motors = std::min(cmd.motor_cmd.size(), size_t(12));
        for (size_t i = 0; i < num_motors; ++i) {
          motors_msg.control[i] = cmd.motor_cmd[i]; // normalized 0-1
        }

        // Set remaining motors to NaN (disarmed)
        for (size_t i = num_motors; i < 12; ++i) {
          motors_msg.control[i] = NAN;
        }

        // publish motor command only when we are already in OFFBOARD mode,
        // otherwise the FC crashes and bricks
        motors_cmd_pub_->publish(motors_msg);
      }
      break;
    }

    default:
      RCLCPP_ERROR(this->get_logger(), "Command had an unexpected type: %d",
                   cmd.command_type_mask);
      return;
    }
  } else {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Not sending offboard messages because of "
                          "unsafety. Code: %d. Landing!",
                          safety_flags_);
    LandNow();
  }
}

void SafetyChecker::SetModeForwarder(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
  // if its safe, then switch to offboard mode. otherwise, send a fail response.
  if (safety_flags_ == SafetyStatus::SAFE) {
    RCLCPP_INFO(get_logger(), "Switching to Offboard mode");

    // Send vehicle command to switch to offboard mode
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp = this->now().nanoseconds() / 1000;
    cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    cmd.param1 = 1; // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    cmd.param2 = 6; // PX4_CUSTOM_MAIN_MODE_OFFBOARD
    cmd.target_system = target_system_;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;

    vehicle_command_pub_->publish(cmd);
    resp->success = true;
    resp->message = "Offboard mode command sent";
  } else {
    RCLCPP_WARN(get_logger(),
                "Did not switch to offboard mode because we are in an unsafe "
                "state. Unsafe code: %d",
                safety_flags_);
    resp->success = false;
    resp->message = "Unsafe state, cannot switch to offboard";
  }
}

void SafetyChecker::LandNow() {
  // Send vehicle command to land
  px4_msgs::msg::VehicleCommand cmd{};
  cmd.timestamp = this->now().nanoseconds() / 1000;
  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
  cmd.param1 = 0;
  cmd.param2 = 0;
  cmd.param3 = 0;
  cmd.param4 = NAN; // Yaw angle
  cmd.param5 = NAN; // Latitude
  cmd.param6 = NAN; // Longitude
  cmd.param7 = NAN; // Altitude
  cmd.target_system = target_system_;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 1;
  cmd.from_external = true;

  vehicle_command_pub_->publish(cmd);
  RCLCPP_INFO(get_logger(), "Sent land command. SafetyFlag Code: %d",
              safety_flags_);
  safety_flags_ &= (~SafetyStatus::UNSAFE_COMMAND_SEND_RATE);
}

bool SafetyChecker::IsPointInHull(const geometry_msgs::msg::Point &point) {
  for (const auto &plane : planes_) {
    double val = plane[0] * point.x + plane[1] * point.y + plane[2] * point.z +
                 plane[3] - plane_offset_;

    if (val > 0) {
      RCLCPP_INFO(this->get_logger(),
                  "Failed on plane [%5.2f, %5.2f, %5.2f, %5.2f]", plane[0],
                  plane[1], plane[2], plane[3]);
      return false;
    }
  }
  return true;
}

void SafetyChecker::HandlePoseMessage(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
  auto logger = this->get_logger();

  if (msg->eph > position_uncertainty_threshold_ ||
      msg->epv > position_uncertainty_threshold_) {
    RCLCPP_WARN_THROTTLE(logger, *this->get_clock(), 1000,
                         "High position uncertainty! EPH: %.2f, EPV: %.2f "
                         "(Threshold: %.2f). Landing...",
                         msg->eph, msg->epv, position_uncertainty_threshold_);
    safety_flags_ |= SafetyStatus::UNSAFE_HIGH_COVARIANCE;
    LandNow(); // Trigger landing
  } else {
    // Pose uncertainty is acceptable, clear the flag
    safety_flags_ &= ~SafetyStatus::UNSAFE_HIGH_COVARIANCE;
  }

  // Convert FRD to FLU for bounds checking using frame_transforms
  geometry_msgs::msg::Point position;
  position.x = msg->x;
  position.y = -msg->y;
  position.z = -msg->z;

  if (IsPointInHull(position)) {
    /* RCLCPP_INFO(logger, "Pose is within bounds."); */
    safety_flags_ &= ~SafetyStatus::UNSAFE_OUT_OF_BOUNDS;
  } else {
    RCLCPP_INFO(logger, "Pose is out of bounds, landing...");
    safety_flags_ |= SafetyStatus::UNSAFE_OUT_OF_BOUNDS;
    LandNow();
  }
}
} // namespace safety_checker
