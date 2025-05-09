#include "latency_checker_ros2/latency_checker.h"

// read a local file provided by ansible that contains list of peers (can just be hostnames, since we can construct topic names)
// publish stochastically (so that they are not synchronized) a ~/latency/heartbeat message that the others will subscribe to (maybe once every ten seconds)
// - includes:
//     + timestamp of when the message was made
//     + the latency calculated for all the peers in ms
// subscribe to all the peers heartbeat messages
// when a heartbeat message comes in, then update the latency in our table of latencies.

namespace latency_checker
{

  LatencyChecker::LatencyChecker() : ::rclcpp::Node("latency_checker")
  {
    this->declare_parameter<std::string>("config_file_path", "/tmp/config");
    config_file_path_ = this->get_parameter("config_file_path").as_string();

    if (config_file_path_.empty() || !std::filesystem::exists(config_file_path_))
    {
      throw std::runtime_error("Invalid or non-existent config file path: " + config_file_path_);
    }

    
  }


}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<latency_checker::LatencyChecker>());
  rclcpp::shutdown();
}