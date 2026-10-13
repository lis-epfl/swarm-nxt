#include "vision_pose.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<vision_pose::VisionPose>());
  } catch (const std::exception &e) {
    // a parameter that cannot work: say so and leave
    RCLCPP_FATAL(rclcpp::get_logger("vision_pose_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
