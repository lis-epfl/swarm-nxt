#include "latency_checker.h"

// read a local file provided by ansible that contains list of peers (can just
// be hostnames, since we can construct topic names) publish stochastically (so
// that they are not synchronized) a ~/latency/heartbeat message that the others
// will subscribe to (maybe once every ten seconds)
// - includes:
//     + timestamp of when the message was made
//     + the latency calculated for all the peers in ms
// subscribe to all the peers heartbeat messages
// when a heartbeat message comes in, then update the latency in our table of
// latencies.

namespace latency_checker {

LatencyChecker::LatencyChecker() : ::rclcpp::Node("latency_checker") {
  this->declare_parameter<std::string>("peer_file_path", "config/peer_list");
  config_file_path_ = this->get_parameter("peer_file_path").as_string();

  if (config_file_path_.empty() ||
      !std::filesystem::exists(config_file_path_)) {
    throw std::runtime_error("Invalid or non-existent config file path: " +
                             config_file_path_);
  }

  std::ifstream config_file(config_file_path_);
  if (!config_file.is_open()) {
    throw std::runtime_error("Failed to open config file: " +
                             config_file_path_);
  }

  std::string line;
  while (std::getline(config_file, line)) {
    if (!line.empty()) {
      peers_.push_back(line);
    }
  }
  config_file.close();

  my_name_ = rclcpp::Node::get_namespace();
  RCLCPP_INFO(this->get_logger(), "My name: %s", my_name_.c_str());
  for (const auto &peer : peers_) {
    std::string ns = "/" + peer;
    if (ns == my_name_) {
      continue;
    }

    std::string topic_name = ns + "/" + rclcpp::Node::get_name() + "/heartbeat";
    auto subscription =
        this->create_subscription<latency_checker_ros2::msg::Heartbeat>(
            topic_name, 10,
            std::bind(&LatencyChecker::HandleHeartbeatMessage, this,
                      std::placeholders::_1));

    peer_subs_.insert({ns, subscription});
    heartbeat_map_.insert({ns, rclcpp::Duration::from_seconds(0)});
  }

  heartbeat_publisher_ =
      this->create_publisher<latency_checker_ros2::msg::Heartbeat>(
          "~/heartbeat", 10);
  heartbeat_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&LatencyChecker::PublishHeartbeat, this));
}

void LatencyChecker::HandleHeartbeatMessage(
    const latency_checker_ros2::msg::Heartbeat &msg) {
  std::string name = msg.node_name;
  auto ts = msg.timestamp;

  auto latency = this->get_clock()->now() - rclcpp::Time(ts);
  heartbeat_map_.insert_or_assign(name, latency);
}

void LatencyChecker::PublishHeartbeat() {
  auto logger = this->get_logger();
  auto msg = latency_checker_ros2::msg::Heartbeat();
  msg.node_name = my_name_;
  msg.timestamp = this->get_clock()->now();
  for (const auto &entry : heartbeat_map_) {
    const auto &peer_name = entry.first;
    const auto &latency = entry.second;

    latency_checker_ros2::msg::NameLatency latency_msg;
    latency_msg.name = peer_name;
    latency_msg.latency = rclcpp::Duration::from_nanoseconds(
        latency.to_chrono<std::chrono::nanoseconds>().count());

    msg.latency_list.push_back(latency_msg);
  }

  heartbeat_publisher_->publish(msg);
}

}  // namespace latency_checker
