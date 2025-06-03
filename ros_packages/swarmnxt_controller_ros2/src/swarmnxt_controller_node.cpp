#include "swarmnxt_controller.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<swarmnxt_controller::Controller>());
  rclcpp::shutdown();
}
