#include <gtest/gtest.h>
#include "bounds_checker.h"  

#include "rclcpp/rclcpp.hpp"

using namespace bounds_checker;

TEST(BoundsCheckerTest, GetPlanesReturnsCorrectData) {
    // Arrange
    auto bc = BoundsChecker(); 
    // Act
    bc.load_hull_from_file("/tmp/test_plane_files/invalid1.json");


    // Assert
    ASSERT_FALSE(planes.empty()) << "Planes list should not be empty.";
    
    // Add more assertions based on the expected behavior of get_planes
    // For example:
    // ASSERT_EQ(planes.size(), expected_size);
    // ASSERT_EQ(planes[0].some_property, expected_value);
}


TEST(BoundsCheckerTest, PointInsideTest) {
    // Arrange
    auto bc = BoundsChecker(); 
    // Act
    bc.load_hull_from_file("/tmp/test_plane_files/valid_cube_111m.json");

    geometry_msgs::msg::Point point;
    point.x = 0;
    point.y = 0.5;
    point.z = 0.6;

    ASSERT_TRUE(bc.is_point_in_hull(point)) << "Point [0, 0.5, 0.6] is inside the convex hull of 1x1x1m";

    point.x = 1;
    point.y = 0;
    point.z = 0;

    ASSERT_TRUE(bc.is_point_in_hull(point)) << "Point [1, 0, 0] is on the surface (and therefore inside) a convex hull of 1x1x1m";

}


TEST(BoundsCheckerTest, PointOutsideTest) {
    // Arrange
    auto bc = BoundsChecker(); 
    // Act
    bc.load_hull_from_file("/tmp/test_plane_files/valid_cube_111m.json");

    geometry_msgs::msg::Point point;
    point.x = 2;
    point.y = 0;
    point.z = 0;

    ASSERT_FALSE(bc.is_point_in_hull(point)) << "Point [2, 0, 0] is outside the convex hull of 1x1x1m";


}



int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(0, nullptr);

    int result = RUN_ALL_TESTS();

    return result;
}