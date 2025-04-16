#include <gtest/gtest.h>
#include "bounds_checker.h"  

#include "rclcpp/rclcpp.hpp"

using namespace bounds_checker;

std::filesystem::path getTestFilePath(const std::string& filename) {
    std::filesystem::path src_file_path = __FILE__;
    auto parent_path = src_file_path.parent_path();

    auto test_file_path = parent_path / "test_plane_files" / filename; 

    return test_file_path;
}

TEST(BoundsCheckerTest, GetPlanesReturnsCorrectData) {
    // Arrange
    auto bc = BoundsChecker(); 
    // Act
    std::filesystem::path invalid_file = getTestFilePath("invalid1.json");
    bc.loadHullFromFile(invalid_file);
    auto planes = bc.getPlanes();

    ASSERT_FALSE(planes.empty()) << "Planes list should not be empty.";
}

TEST(BoundsCheckerTest, NonExistentFileErrorThrown) {
    // Arrange
    auto bc = BoundsChecker(); 
    // Act
    std::filesystem::path non_existent = getTestFilePath("non_existent.json");
    ASSERT_THROW(bc.loadHullFromFile(non_existent), std::runtime_error) << "Expected std::runtime_error when loading a non-existent file.";
}


TEST(BoundsCheckerTest, PointInsideTest) {
    auto bc = BoundsChecker(); 
    bc.loadHullFromFile(getTestFilePath("valid_cube_111m.json"));

    geometry_msgs::msg::Point point;
    point.x = 0;
    point.y = 0.5;
    point.z = 0.6;

    ASSERT_TRUE(bc.isPointInHull(point)) << "Point [0, 0.5, 0.6] is inside the convex hull of 1x1x1m";

    point.x = 1;
    point.y = 0;
    point.z = 0;

    ASSERT_TRUE(bc.isPointInHull(point)) << "Point [1, 0, 0] is on the surface (and therefore inside) a convex hull of 1x1x1m";

}


TEST(BoundsCheckerTest, PointOutsideTest) {
    auto bc = BoundsChecker(); 
    bc.loadHullFromFile(getTestFilePath("valid_cube_111m.json"));

    geometry_msgs::msg::Point point;
    point.x = 2;
    point.y = 0;
    point.z = 0;

    ASSERT_FALSE(bc.isPointInHull(point)) << "Point [2, 0, 0] is outside the convex hull of 1x1x1m";


}



int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(0, nullptr);

    int result = RUN_ALL_TESTS();

    return result;
}