#include "latency_checker.h"
#include <filesystem> // Make sure this is included for std::filesystem
#include <fstream>
#include <string>
#include <chrono>

namespace latency_checker {

LatencyChecker::LatencyChecker() : ::rclcpp::Node("latency_checker") {
  this->declare_parameter<std::string>("peer_file_path", "config/peer_list");
  config_file_path_ = this->get_parameter("peer_file_path").as_string();

  if (config_file_path_.empty() ||
      !std::filesystem::exists(config_file_path_)) {
    // ⬇️ ADDED LOGGING ⬇️
    RCLCPP_ERROR(this->get_logger(), "Invalid or non-existent config file path: %s",
                 config_file_path_.c_str());
    throw std::runtime_error("Invalid or non-existent config file path: " +
                             config_file_path_);
  }

  std::ifstream config_file(config_file_path_);
  if (!config_file.is_open()) {
    // ⬇️ ADDED LOGGING ⬇️
    RCLCPP_ERROR(this->get_logger(), "Failed to open config file: %s",
                 config_file_path_.c_str());
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
  // ⬇️ ADDED LOGGING ⬇️
  RCLCPP_INFO(this->get_logger(), "Loaded %zu peers from %s", peers_.size(), config_file_path_.c_str());

  my_name_ = rclcpp::Node::get_namespace();
  RCLCPP_INFO(this->get_logger(), "My name: %s", my_name_.c_str());

  for (const auto &peer : peers_) {
    std::string ns = "/" + peer;
    if (ns == my_name_) {
      continue;
    }

    std::string topic_name = ns + "/" + rclcpp::Node::get_name() + "/heartbeat";

    // ⬇️ ADDED LOGGING ⬇️
    RCLCPP_INFO(this->get_logger(), "Subscribing to peer topic: %s", topic_name.c_str());

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
      std::chrono::seconds(10),
      std::bind(&LatencyChecker::PublishHeartbeat, this));

  // ⬇️ ADDED LOGGING ⬇️
  RCLCPP_INFO(this->get_logger(), "LatencyChecker node initialized successfully.");
}

void LatencyChecker::HandleHeartbeatMessage(
    const latency_checker_ros2::msg::Heartbeat &msg) {
  std::string name = msg.node_name;
  auto ts = msg.timestamp;
  auto now = this->get_clock()->now();

  auto latency = now - rclcpp::Time(ts);

  // ⬇️ ADDED LOGGING ⬇️
  // This is the most important log for calculation.
  // We log the raw nanoseconds to confirm the calculation is correct.
  RCLCPP_INFO(this->get_logger(),
              "[SUB] Received from: %s. Calculated latency (ns): %ld",
              name.c_str(),
              latency.nanoseconds());

  heartbeat_map_.insert_or_assign(name, latency);
}

void LatencyChecker::PublishHeartbeat() {
  auto logger = this->get_logger();
  RCLCPP_INFO(logger, "[PUB] Publishing heartbeat timer fired.");

  auto msg = latency_checker_ros2::msg::Heartbeat();
  msg.node_name = my_name_;
  msg.timestamp = this->get_clock()->now();

  for (const auto &entry : heartbeat_map_) {
    const auto &peer_name = entry.first;
    const auto &latency = entry.second; // This is an rclcpp::Duration

    latency_checker_ros2::msg::NameLatency latency_msg;
    latency_msg.name = peer_name;

    // ⬇️ ADDED LOGGING ⬇️
    // Log the high-precision value we have stored in our map.
    RCLCPP_INFO(logger,
                "[PUB] Processing peer: %s. Stored latency (ns): %ld",
                peer_name.c_str(),
                latency.nanoseconds());

    // ---
    // ⬇️ THIS IS THE FIX ⬇️
    // Convert the rclcpp::Duration to a double (in seconds)
    // and then multiply by 1000.0 to get floating-point milliseconds.
    // This assumes your `latency_msg.latency` field is a `float64`.
    double latency_ms = latency.seconds() * 1000.0;
    latency_msg.latency = latency_ms;
    // ---
    // ⬆️ END OF FIX ⬆️
    // ---

    // ⬇️ ADDED LOGGING ⬇️
    // Log the final floating-point value being sent.
    // This is where you would see 0.0 if you were using integer truncation.
    RCLCPP_INFO(logger,
                "[PUB] ...Converted to (ms): %f. Adding to message.",
                latency_ms);

    msg.latency_list.push_back(latency_msg);
  }

  heartbeat_publisher_->publish(msg);
  RCLCPP_INFO(logger, "[PUB] Heartbeat message published.");
}

} // namespace latency_checker
