#include "bounds_checker.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bounds_checker::BoundsChecker>());
  rclcpp::shutdown();
}