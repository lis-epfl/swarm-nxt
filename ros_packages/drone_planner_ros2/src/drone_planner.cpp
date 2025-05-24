#include "drone_planner.h"

namespace drone_planner {

DronePlanner::DronePlanner() : ::rclcpp::Node("drone_planner") {
  std::string peer_file;

  this->declare_parameter("peer_file", "config/peers");
  this->get_parameter("peer_file", peer_file);

  // DeclareRosParameters();
  // InitializeRosParameters();

  goals_sub_ = this->create_subscription<nav_msgs::msg::Goals>(
      "~/goals", 10,
      std::bind(&DronePlanner::GoalsCallback, this, std::placeholders::_1));

  // Example: subscribing to trajectories of other drones (peer IDs to be filled
  // in) for (const auto& peer_id : peer_ids_) {
  //   traj_sub_map_[peer_id] = this->create_subscription<nav_msgs::msg::Path>(
  //       "traj_" + peer_id, 10,
  //       std::bind(&DronePlanner::TrajectoryCallback, this,
  //       std::placeholders::_1, peer_id));
  // }

  depth_image_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "~/depth_image", 10,
      std::bind(&DronePlanner::DepthImageCallback, this,
                std::placeholders::_1));

  desired_traj_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("~/trajectory_desired", 10);

  auto timer_ =
      this->create_wall_timer(std::chrono::milliseconds(100),
                              std::bind(&DronePlanner::TimerCallback, this));
}

void DronePlanner::GoalsCallback(const nav_msgs::msg::Goals &msg) {
  // this could be a service? maybe that's better...

  // find the last element of the current_goals_ in the incoming message.
  auto last_goal = current_goals_.at(current_goals_.size() - 1);

  bool found_end = false;
  for (const auto &goal : msg.goals) {
    if (found_end) {
      current_goals_.push_back(goal);
    }
    if (goal == last_goal) {
      found_end = true;
    }
  }

  // if the incoming goal list does not contain any of our current goals then
  // we've received a fully new list of goals
  if (!found_end) {
    for (const auto &goal : msg.goals) {
      current_goals_.push_back(goal);
    }
  }
}

void DronePlanner::DepthImageCallback(
    const sensor_msgs::msg::PointCloud2 &msg) {
  // placeholder for future
}

void DronePlanner::TimerCallback() {
  // Placeholder for periodic tasks
}

} // namespace drone_planner