#include "bounds_checker.h"

#include <fstream>
namespace bounds_checker {

BoundsChecker::BoundsChecker() : ::rclcpp::Node("bounds_checker") {
  std::string filepath;
  this->declare_parameter("plane_file", "/opt/omninxt/bounds.json");
  this->get_parameter("plane_file", filepath);
  topic_prefix_ = std::getenv("HOSTNAME");
}

void BoundsChecker::LoadHullFromFile(const std::filesystem::path& filepath) {
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

  for (const auto& arr : json_parser) {
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

  are_planes_valid_ = valid_parse;
}

bool BoundsChecker::IsPointInHull(const geometry_msgs::msg::Point& point) {
  if (!are_planes_valid_) {
    return false;
  }
  for (const auto& plane : planes_) {
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

void BoundsChecker::ClearPlanes() {
  are_planes_valid_ = false;
  planes_.clear();
}

std::vector<bounds_checker_ros2::msg::Plane> BoundsChecker::GetPlanes() {
  return planes_;
}

void BoundsChecker::GetTestFilePath() {

}

void BoundsChecker::DeclareRosParameters() {}

}  // namespace bounds_checker