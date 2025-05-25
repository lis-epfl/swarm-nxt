#include "drone_planner.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_planner::DronePlanner>());
  rclcpp::shutdown();
}