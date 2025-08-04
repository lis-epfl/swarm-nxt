#include "bounds_checker.h"

#include <fstream>
namespace bounds_checker {

BoundsChecker::BoundsChecker() : ::rclcpp::Node("bounds_checker") {
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

  LoadHullFromFile(filepath);

  rclcpp::QoS best_effort_qos =
      rclcpp::QoS(rclcpp::KeepLast(10))
          .reliability(rclcpp::ReliabilityPolicy::BestEffort);

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      ns + "/mavros/local_position/pose", best_effort_qos,
      std::bind(&BoundsChecker::HandlePoseMessage, this,
                std::placeholders::_1));

  trajectory_sub_ = create_subscription<nav_msgs::msg::Path>(
      ns + "/trajectory", 10,
      std::bind(&BoundsChecker::HandleTrajectoryMessage, this,
                std::placeholders::_1));

  // deprecated
  safe_trajectory_pub_ =
      create_publisher<nav_msgs::msg::Path>(ns + "/trajectory_safe", 10);

  land_client_ = create_client<std_srvs::srv::Trigger>(ns + "/controller/land");

  // Wait for the service to be available
  while (!land_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(),
                "Waiting for ~/controller/land service to be available...");
  }
}

void BoundsChecker::LoadHullFromFile(const std::filesystem::path &filepath) {
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
    bounds_checker_ros2::msg::Plane plane;
    geometry_msgs::msg::Vector3 normal;

    double a = arr[0].get<double>();
    double b = arr[1].get<double>();
    double c = arr[2].get<double>();
    double d = arr[3].get<double>();

    RCLCPP_INFO(logger, "Plane elements: %5.2f, %5.2f, %5.2f, %5.2f", a, b, c,
                d);
    if (plane_offset_ < std::abs(d)) {
      d = std::signbit(d) * (std::abs(d) - plane_offset_);
    } else {
      RCLCPP_WARN(logger,
                  "Plane offset was too high, this facet did not get scaled!");
    }

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

bool BoundsChecker::IsPointInHull(const geometry_msgs::msg::Point &point) {
  if (!are_planes_valid_) {
    return false;
  }
  for (const auto &plane : planes_) {
    double val = plane.normal.x * point.x + plane.normal.y * point.y +
                 plane.normal.z * point.z - plane.offset;

    if (val > 0) {
      return false;
    }
  }
  return true;
}

geometry_msgs::msg::Point BoundsChecker::ProjectPointToClosestPlane(
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

void BoundsChecker::HandlePoseMessage(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  auto logger = this->get_logger();
  const auto &position = msg->pose.position;

  if (IsPointInHull(position)) {
    RCLCPP_INFO(logger, "Pose is within bounds.");
  } else {
    RCLCPP_INFO(logger, "Pose is out of bounds, landing...");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result_future = land_client_->async_send_request(request);

    try {
      // Wait for the result with a timeout, spinning to process callbacks
      auto start = std::chrono::steady_clock::now();
      auto timeout = std::chrono::seconds(2);  // adjust as needed
      while (rclcpp::ok() && result_future.wait_for(std::chrono::milliseconds(
                                 100)) != std::future_status::ready) {
        rclcpp::spin_some(this->get_node_base_interface());
        if (std::chrono::steady_clock::now() - start > timeout) {
          throw std::runtime_error(
              "Timeout waiting for landing service response");
        }
      }
      auto result = result_future.get();
      if (result->success) {
        RCLCPP_INFO(logger, "Landing command sent successfully: %s",
                    result->message.c_str());
      } else {
        RCLCPP_WARN(logger, "Landing command failed: %s",
                    result->message.c_str());
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(logger, "Failed to call landing service: %s", e.what());
    }
  }
}

// DEPRECATED: We do not project trajectory messages on the safe plane anymore.
// We simply land if the pose ends up outside of of the (shrunk) bounds.
void BoundsChecker::HandleTrajectoryMessage(const nav_msgs::msg::Path &msg) {
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

void BoundsChecker::ClearPlanes() {
  are_planes_valid_ = false;
  planes_.clear();
}

std::vector<bounds_checker_ros2::msg::Plane> BoundsChecker::GetPlanes() {
  return planes_;
}
}  // namespace bounds_checker
