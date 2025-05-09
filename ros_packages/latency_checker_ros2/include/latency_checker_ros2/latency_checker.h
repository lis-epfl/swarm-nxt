#ifndef LATENCY_CHECKER_H_
#define LATENCY_CHECKER_H_

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <fstream> 
#include <filesystem>
#include "latency_checker_ros2/msg/heartbeat.hpp"
#include "latency_checker_ros2/msg/name_latency.hpp"

namespace latency_checker {
    class LatencyChecker : public ::rclcpp::Node {
        public:
            LatencyChecker();

            void initialize(const std::string &config_file);
            void HandleHeartbeatMessage();

        private:
            std::unordered_map<std::string, rclcpp::Duration> heartbeat_map_;
            std::unordered_map<std::string, rclcpp::Subscription<latency_checker_ros2::msg::Heartbeat>::SharedPtr> peer_subs_;
            std::vector<std::string> peers_;

            std::string config_file_path_; 
    };
}

#endif // LATENCY_CHECKER_H_