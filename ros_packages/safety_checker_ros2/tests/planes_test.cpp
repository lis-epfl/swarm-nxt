#include <gtest/gtest.h>

#include "safety_checker.h"
#include "rclcpp/rclcpp.hpp"

using namespace safety_checker;

std::filesystem::path GetTestFilePath(const std::string &filename) {
  std::filesystem::path src_file_path = __FILE__;
  auto parent_path = src_file_path.parent_path();

  auto test_file_path = parent_path / "test_plane_files" / filename;

  return test_file_path;
}

TEST(SafetyCheckerTest, GetPlanesReturnsCorrectData) {
  // Arrange
  auto sc = SafetyChecker();
  // Act
  std::filesystem::path invalid_file = GetTestFilePath("invalid1.json");
  sc.LoadHullFromFile(invalid_file);
  auto planes = sc.GetPlanes();

  ASSERT_FALSE(planes.empty()) << "Planes list should not be empty.";
}

TEST(SafetyCheckerTest, NonExistentFileErrorThrown) {
  // Arrange
  auto sc = SafetyChecker();
  // Act
  std::filesystem::path non_existent = GetTestFilePath("non_existent.json");
  ASSERT_THROW(sc.LoadHullFromFile(non_existent), std::runtime_error)
      << "Expected std::runtime_error when loading a non-existent file.";
}

TEST(SafetyCheckerTest, PointInsideTest) {
  auto sc = SafetyChecker();
  sc.LoadHullFromFile(GetTestFilePath("valid_cube_111m.json"));

  geometry_msgs::msg::Point point;
  point.x = 0;
  point.y = 0.5;
  point.z = 0.6;

  ASSERT_TRUE(sc.IsPointInHull(point))
      << "Point [0, 0.5, 0.6] is inside the convex hull of 1x1x1m";

  point.x = 1;
  point.y = 0;
  point.z = 0;

  ASSERT_TRUE(sc.IsPointInHull(point))
      << "Point [1, 0, 0] is on the surface (and therefore inside) a convex "
         "hull of 1x1x1m";
}

TEST(SafetyCheckerTest, PointOutsideTest) {
  auto sc = SafetyChecker();
  sc.LoadHullFromFile(GetTestFilePath("valid_cube_111m.json"));

  geometry_msgs::msg::Point point;
  point.x = 2;
  point.y = 0;
  point.z = 0;

  ASSERT_FALSE(sc.IsPointInHull(point))
      << "Point [2, 0, 0] is outside the convex hull of 1x1x1m";
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  return result;
}