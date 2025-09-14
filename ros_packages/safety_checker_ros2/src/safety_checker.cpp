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

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      ns + "/mavros/local_position/pose", best_effort_qos,
      std::bind(&SafetyChecker::HandlePoseMessage, this,
                std::placeholders::_1));

  command_sub_ = create_subscription<swarmnxt_msgs::msg::ControllerCommand>(
      ns + "/controller/cmd", 10,
      std::bind(&SafetyChecker::HandleControllerCommand, this,
                std::placeholders::_1));

  // deprecated
  safe_trajectory_pub_ =
      create_publisher<nav_msgs::msg::Path>(ns + "/trajectory_safe", 10);

  position_cmd_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
      ns + "/mavros/setpoint_raw/local", 10);

  rate_cmd_pub_ = create_publisher<mavros_msgs::msg::AttitudeTarget>(
      ns + "/mavros/setpoint_raw/attitude", 10);

  set_mode_server_ = create_service<mavros_msgs::srv::SetMode>(
      ns + "/safety/mavros/set_mode",
      std::bind(&SafetyChecker::SetModeForwarder, this, std::placeholders::_1,
                std::placeholders::_2));

  set_mode_client_ =
      create_client<mavros_msgs::srv::SetMode>(ns + "/mavros/set_mode");

  land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>(
      ns + "/mavros/cmd/land");
}

void SafetyChecker::HandleControllerCommand(
    const swarmnxt_msgs::msg::ControllerCommand &msg) {
  auto cmd = msg;
  auto cur_time = this->now();
  auto cmd_age = cur_time - cmd.header.stamp;
  // make sure the command isn't stale
  if (cmd_age.nanoseconds() > 20E6) {
    safety_flags_ |= SafetyStatus::UNSAFE_COMMAND_SEND_RATE;
    RCLCPP_ERROR(this->get_logger(), "Command was too old, stopping...");
    LandNow();
  }

  if (safety_flags_ == SafetyStatus::SAFE) {
    switch (cmd.command_type_mask) {
      case swarmnxt_msgs::msg::ControllerCommand::POSITION_SETPOINT: {
        // Convert PoseStamped to PositionTarget for backward compatibility
        mavros_msgs::msg::PositionTarget pos_target;
        pos_target.header = cmd.pose_cmd.header;
        pos_target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        pos_target.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
                               mavros_msgs::msg::PositionTarget::IGNORE_VY |
                               mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                               mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                               mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                               mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                               mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
        pos_target.position.x = cmd.pose_cmd.pose.position.x;
        pos_target.position.y = cmd.pose_cmd.pose.position.y;
        pos_target.position.z = cmd.pose_cmd.pose.position.z;
        position_cmd_pub_->publish(pos_target);
        break;
      }
      case swarmnxt_msgs::msg::ControllerCommand::RATE_SETPOINT:
        rate_cmd_pub_->publish(cmd.rate_cmd);
        break;
      case swarmnxt_msgs::msg::ControllerCommand::PVA_SETPOINT:
        position_cmd_pub_->publish(cmd.raw_cmd);
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Command had an unexpected type: %d",
                     cmd.command_type_mask);
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
    const std::shared_ptr<mavros_msgs::srv::SetMode::Request> req,
    std::shared_ptr<mavros_msgs::srv::SetMode::Response> resp) {
  // if its safe, then forward the setmode request. otherwise, send a fail
  // response.
  if (safety_flags_ == SafetyStatus::SAFE) {
    RCLCPP_INFO(get_logger(), "Forwarding SetMode to Mavros");
    auto set_mode_future = set_mode_client_->async_send_request(
        req,
        [resp](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
          auto response = future.get();
          resp->mode_sent = response->mode_sent;
        });

  } else {
    RCLCPP_WARN(
        get_logger(),
        "Did not forward the mode change request because we are in an unsafe "
        "state. Unsafe code: %d",
        safety_flags_);
    resp->mode_sent = false;
  }
}

void SafetyChecker::LandNow() {
  auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
  // do we have to send any position or something?
  auto land_future = land_client_->async_send_request(
      req,
      [this](
          rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
        auto response = future.get();
        if (!response->success) {
          RCLCPP_ERROR(get_logger(),
                       "Could not send a land now command to MAVROS!");
        } else {
          RCLCPP_INFO(get_logger(), "Successfully changed mode to land");
        }
      });
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

     RCLCPP_INFO(logger, "Plane elements (scaled): %5.2f, %5.2f, %5.2f, %5.2f", a, b, c,
                d);

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
      RCLCPP_INFO(this->get_logger(), "Failed on plane [%5.2f, %5.2f, %5.2f, %5.2f]", plane.normal.x, plane.normal.y, plane.normal.z, plane.offset);
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
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  auto logger = this->get_logger();
  const auto &position = msg->pose.position;

  if (IsPointInHull(position)) {
    RCLCPP_INFO(logger, "Pose is within bounds.");
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
    RCLCPP_INFO(this->get_logger(), "Trajectory was safe");
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
