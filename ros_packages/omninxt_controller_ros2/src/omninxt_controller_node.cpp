#include "omninxt_controller.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<omninxt_controller::Controller>());
  rclcpp::shutdown();
}