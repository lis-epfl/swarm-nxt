#include "bounds_checker.h"

#include <fstream>
namespace bounds_checker {

BoundsChecker::BoundsChecker() : ::rclcpp::Node("bounds_checker") {
  std::string filepath;

  this->declare_parameter("plane_file", "config/bounds.json");
  this->get_parameter("plane_file", filepath);

  DeclareRosParameters();
  InitializeRosParameters();

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/mavros/local_position/pose", 10,
      std::bind(&BoundsChecker::HandlePoseMessage, this,
                std::placeholders::_1));

  trajectory_sub_ = create_subscription<nav_msgs::msg::Path>(
      "~/trajectory_desired", 10,
      std::bind(&BoundsChecker::HandleTrajectoryMessage, this,
                std::placeholders::_1));

  safe_trajectory_pub_ =
      create_publisher<nav_msgs::msg::Path>("~/trajectory_safe", 10);

  safe_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("~/pose_safe", 10);

  land_client_ = create_client<std_srvs::srv::Trigger>("~/controller/land");

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

  // TODO: emit a warning if the plane does not form a closed hull. or just
  // throw an error, talk to charbel about this
}

bool BoundsChecker::IsPointInHull(const geometry_msgs::msg::Point &point) {
  if (!are_planes_valid_) {
    return false;
  }
  for (const auto &plane : planes_) {
    double val = plane.normal.x * point.x + plane.normal.y * point.y +
                 plane.normal.z * point.z + plane.offset;

    RCLCPP_INFO(get_logger(),
                "Point (%5.2f, %5.2f, %5.2f) dot product with plane: [%5.2f, "
                "%5.2f, %5.2f, %5.2f] = %5.2f\r\n",
                point.x, point.y, point.z, plane.normal.x, plane.normal.y,
                plane.normal.z, plane.offset, val);

    if (val < 0) {
      return false;
    }
  }
  return true;
}

geometry_msgs::msg::Point BoundsChecker::ProjectPointToClosestPlane(
    const geometry_msgs::msg::Point &point) {
  double min_distance = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point projected_point;

  for (const auto &plane : planes_) {
    // Calculate the distance from the point to the plane
    double numerator = plane.normal.x * point.x + plane.normal.y * point.y +
                       plane.normal.z * point.z + plane.offset;
    double denominator = std::sqrt(plane.normal.x * plane.normal.x +
                                   plane.normal.y * plane.normal.y +
                                   plane.normal.z * plane.normal.z);

    double distance = std::abs(numerator) / denominator;

    // If this plane is closer, calculate the projection
    if (distance < min_distance) {
      min_distance = distance;

      double scale = numerator / (denominator * denominator);
      projected_point.x = point.x - scale * plane.normal.x;
      projected_point.y = point.y - scale * plane.normal.y;
      projected_point.z = point.z - scale * plane.normal.z;
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
    safe_pose_pub_->publish(*msg);
  } else {
    RCLCPP_INFO(logger, "Pose is out of bounds, landing...");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result_future = land_client_->async_send_request(request);

    try {
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

void BoundsChecker::HandleTrajectoryMessage(const nav_msgs::msg::Path &msg) {
  // TODO: What if another drone in the swarm gets the same projected value?
  auto safe_traj = msg;

  // todo: make the trajectory hull an inset of the true hull.
  for (auto &pose : safe_traj.poses) {
    if (!IsPointInHull(pose.pose.position)) {
      pose.pose.position = ProjectPointToClosestPlane(pose.pose.position);
    }
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

void BoundsChecker::InitializeRosParameters() {}

void BoundsChecker::DeclareRosParameters() {}

} // namespace bounds_checker