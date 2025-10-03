#include "safety_checker.h"

#include <fstream>
namespace safety_checker {

SafetyChecker::SafetyChecker() : ::rclcpp::Node("safety_checker") {
  std::string filepath;

  std::string ns = this->get_namespace();

  this->declare_parameter("plane_file", "/var/opt/config/bounds.json");
  this->get_parameter("plane_file", filepath);

  this->declare_parameter("plane_offset", 0.5f);  // meters
  this->get_parameter("plane_offset", plane_offset_);

  if (plane_offset_ < 0) {
    RCLCPP_WARN(this->get_logger(),
                "Parameter plane_offset set to: %.5f, must be positive. "
                "Setting to zero.",
                plane_offset_);
    plane_offset_ = 0.0f;
  }

  RCLCPP_INFO(this->get_logger(), "Using plane offset: %5.2f", plane_offset_);

  LoadHullFromFile(filepath);

  rclcpp::QoS best_effort_qos =
      rclcpp::QoS(rclcpp::KeepLast(10))
          .reliability(rclcpp::ReliabilityPolicy::BestEffort);

  pose_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      ns + "/fmu/out/vehicle_local_position", best_effort_qos,
      std::bind(&SafetyChecker::HandlePoseMessage, this,
                std::placeholders::_1));

  command_sub_ = create_subscription<swarmnxt_msgs::msg::ControllerCommand>(
      ns + "/controller/cmd", 10,
      std::bind(&SafetyChecker::HandleControllerCommand, this,
                std::placeholders::_1));

  // deprecated
  safe_trajectory_pub_ =
      create_publisher<nav_msgs::msg::Path>(ns + "/trajectory_safe", 10);

  position_cmd_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      ns + "/fmu/in/trajectory_setpoint", 10);

  offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      ns + "/fmu/in/offboard_control_mode", 10);

  vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      ns + "/fmu/in/vehicle_command", 10);

  set_mode_server_ = create_service<std_srvs::srv::Trigger>(
      ns + "/safety/set_offboard_mode",
      std::bind(&SafetyChecker::SetModeForwarder, this, std::placeholders::_1,
                std::placeholders::_2));
}

void SafetyChecker::HandleControllerCommand(
    const swarmnxt_msgs::msg::ControllerCommand &msg) {
  auto cmd = msg;
  auto cur_time = this->now();
  auto cmd_age = cur_time - cmd.header.stamp;
  // make sure the command isn't stale
  if (cmd_age.nanoseconds() > 20E6) {
    safety_flags_ |= SafetyStatus::UNSAFE_COMMAND_SEND_RATE;
    RCLCPP_ERROR(this->get_logger(), "Command was too old, stopping. Age: %5.2fms", cmd_age.nanoseconds()/1e6);
    LandNow();
  }

  if (safety_flags_ == SafetyStatus::SAFE) {
    // Set offboard control mode flags based on command type mask
    px4_msgs::msg::OffboardControlMode offboard_mode{};
    offboard_mode.timestamp = this->now().nanoseconds() / 1000;

    switch (cmd.command_type_mask) {
      case swarmnxt_msgs::msg::ControllerCommand::POSITION_SETPOINT:
        offboard_mode.position = true;
        offboard_mode.velocity = false;
        offboard_mode.acceleration = false;
        offboard_mode.attitude = false;
        offboard_mode.body_rate = false;
        break;
      case swarmnxt_msgs::msg::ControllerCommand::RATE_SETPOINT:
        offboard_mode.position = false;
        offboard_mode.velocity = false;
        offboard_mode.acceleration = false;
        offboard_mode.attitude = true;
        offboard_mode.body_rate = false;
        break;
      case swarmnxt_msgs::msg::ControllerCommand::PVA_SETPOINT:
        offboard_mode.position = true;
        offboard_mode.velocity = true;
        offboard_mode.acceleration = true;
        offboard_mode.attitude = false;
        offboard_mode.body_rate = false;
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Command had an unexpected type: %d",
                     cmd.command_type_mask);
        return;
    }

    // Publish offboard control mode and trajectory setpoint
    offboard_control_mode_pub_->publish(offboard_mode);
    position_cmd_pub_->publish(cmd.cmd);
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
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;

    vehicle_command_pub_->publish(cmd);
    resp->success = true;
    resp->message = "Offboard mode command sent";
  } else {
    RCLCPP_WARN(
        get_logger(),
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
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 1;
  cmd.from_external = true;

  vehicle_command_pub_->publish(cmd);
  RCLCPP_INFO(get_logger(),
              "Sent land command. SafetyFlag Code: %d",
              safety_flags_);
  safety_flags_ &= (~SafetyStatus::UNSAFE_COMMAND_SEND_RATE);
}

void SafetyChecker::LoadHullFromFile(const std::filesystem::path &filepath) {
  auto logger = this->get_logger();

  are_planes_valid_ = false;
  using json = nlohmann::json;

  if (!std::filesystem::exists(filepath)) {
    throw std::runtime_error("File " + filepath.string() + " does not exist");
  }

  std::ifstream file(filepath);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open plane definition file...");
  }

  json json_parser;
  file >> json_parser;

  bool valid_parse = true;

  for (const auto &arr : json_parser) {
    if (!arr.is_array() || arr.size() != 4) {
      RCLCPP_ERROR(logger, "Invalid JSON Format...");
      valid_parse = false;
    }
    swarmnxt_msgs::msg::Plane plane;
    geometry_msgs::msg::Vector3 normal;

    double a = arr[0].get<double>();
    double b = arr[1].get<double>();
    double c = arr[2].get<double>();
    double d = arr[3].get<double>();

    RCLCPP_INFO(logger, "Plane elements: %5.2f, %5.2f, %5.2f, %5.2f", a, b, c,
                d);

    if (plane_offset_ < std::abs(d)) {
      if (d < 0) {
        d = d + plane_offset_;
      } else {
        d = d - plane_offset_;
      }
    } else {
      RCLCPP_WARN(logger,
                  "Plane offset was too high, this facet did not get scaled!");
    }

    RCLCPP_INFO(logger, "Plane elements (scaled): %5.2f, %5.2f, %5.2f, %5.2f",
                a, b, c, d);

    normal.set__x(a);
    normal.set__y(b);
    normal.set__z(c);

    plane.set__normal(normal);
    plane.set__offset(d);

    planes_.push_back(plane);
  }

  if (!valid_parse) {
    throw std::runtime_error("The planes could not be parsed...");
  }

  are_planes_valid_ = true;
}

bool SafetyChecker::IsPointInHull(const geometry_msgs::msg::Point &point) {
  if (!are_planes_valid_) {
    return false;
  }
  for (const auto &plane : planes_) {
    double val = plane.normal.x * point.x + plane.normal.y * point.y +
                 plane.normal.z * point.z - plane.offset;

    if (val > 0) {
      RCLCPP_INFO(this->get_logger(),
                  "Failed on plane [%5.2f, %5.2f, %5.2f, %5.2f]",
                  plane.normal.x, plane.normal.y, plane.normal.z, plane.offset);
      return false;
    }
  }
  return true;
}

geometry_msgs::msg::Point SafetyChecker::ProjectPointToClosestPlane(
    const geometry_msgs::msg::Point &point) {
  geometry_msgs::msg::Point projected_point = point;

  for (const auto &plane : planes_) {
    // which plane are we violating?
    // Calculate the distance from the point to the plane
    double numerator = plane.normal.x * projected_point.x +
                       plane.normal.y * projected_point.y +
                       plane.normal.z * projected_point.z - plane.offset;
    double denominator = std::sqrt(plane.normal.x * plane.normal.x +
                                   plane.normal.y * plane.normal.y +
                                   plane.normal.z * plane.normal.z);

    if (numerator > 0) {
      // we are violating this plane, so project onto this plane
      double scale = numerator / (denominator * denominator);
      projected_point.x = projected_point.x - scale * plane.normal.x;
      projected_point.y = projected_point.y - scale * plane.normal.y;
      projected_point.z = projected_point.z - scale * plane.normal.z;
    }
  }

  return projected_point;
}

void SafetyChecker::HandlePoseMessage(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
  auto logger = this->get_logger();

  // Convert NED to ENU for bounds checking
  geometry_msgs::msg::Point position;
  position.x = msg->x;
  position.y = msg->y;
  position.z = -msg->z; // NED to ENU

  if (IsPointInHull(position)) {
    RCLCPP_DEBUG(logger, "Pose is within bounds.");
    safety_flags_ &= ~SafetyStatus::UNSAFE_OUT_OF_BOUNDS;
  } else {
    RCLCPP_WARN(logger, "Pose is out of bounds, landing...");
    safety_flags_ |= SafetyStatus::UNSAFE_OUT_OF_BOUNDS;
    LandNow();
  }
}

// DEPRECATED: We do not project trajectory messages on the safe plane anymore.
// We simply land if the pose ends up outside of of the (shrunk) bounds.
void SafetyChecker::HandleTrajectoryMessage(const nav_msgs::msg::Path &msg) {
  // TODO: What if another drone in the swarm gets the same projected value?
  RCLCPP_WARN(this->get_logger(),
              "Checking trajectory messages are deprecated!");
  auto safe_traj = msg;
  bool safe = true;
  // todo: make the trajectory hull an inset of the true hull.
  for (auto &pose : safe_traj.poses) {
    if (!IsPointInHull(pose.pose.position)) {
      safe = false;
      pose.pose.position = ProjectPointToClosestPlane(pose.pose.position);
    }

    // todo: make this a paramter
    // enforce a min z height
    if (pose.pose.position.z < 0.7f) {
      safe = false;
      pose.pose.position.z = 0.7f;
    }
  }

  if (!safe) {
    RCLCPP_WARN(this->get_logger(), "Trajectory was unsafe");
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Trajectory was safe");
  }

  safe_trajectory_pub_->publish(safe_traj);
}

void SafetyChecker::ClearPlanes() {
  are_planes_valid_ = false;
  planes_.clear();
}

std::vector<swarmnxt_msgs::msg::Plane> SafetyChecker::GetPlanes() {
  return planes_;
}
}  // namespace safety_checker
