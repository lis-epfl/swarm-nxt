#include "latency_checker.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<latency_checker::LatencyChecker>());
  rclcpp::shutdown();
}
