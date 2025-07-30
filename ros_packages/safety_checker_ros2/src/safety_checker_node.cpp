#include "safety_checker.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<safety_checker::SafetyChecker>());
  rclcpp::shutdown();
}