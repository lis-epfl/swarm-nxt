#include "drone_planner.h"

namespace drone_planner {

geometry_msgs::msg::Pose operator-(geometry_msgs::msg::Pose& pose1,
                                   geometry_msgs::msg::Pose& pose2) {
  geometry_msgs::msg::Pose result;
  tf2::Quaternion q1;
  tf2::Quaternion q2;

  tf2::fromMsg(pose1.orientation, q1);
  tf2::fromMsg(pose2.orientation, q2);

  auto q2_inv = q2;
  q2_inv[3] = -q2_inv[3];
  auto qr = q1 * q2_inv;

  result.orientation = tf2::toMsg(qr);

  result.position.x = pose1.position.x - pose2.position.x;
  result.position.y = pose1.position.y - pose2.position.y;
  result.position.z = pose1.position.z - pose2.position.z;

  return result;
}

DronePlanner::DronePlanner() : ::rclcpp::Node("drone_planner") {
  std::string peer_file;
  auto logger = this->get_logger();
  RCLCPP_INFO(logger, "Starting the drone planner node...");

  this->declare_parameter("peer_file", "config/peers");
  this->get_parameter("peer_file", peer_file);

  std::string ns = this->get_namespace();

  goals_sub_ = this->create_subscription<nav_msgs::msg::Goals>(
      ns + "/goals", 10,
      std::bind(&DronePlanner::GoalsCallback, this, std::placeholders::_1));

  rclcpp::QoS best_effort_qos =
      rclcpp::QoS(rclcpp::KeepLast(10))
          .reliability(rclcpp::ReliabilityPolicy::BestEffort);

  mavros_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      ns + "/mavros/local_position/pose", best_effort_qos,
      std::bind(&DronePlanner::MavrosPoseCallback, this,
                std::placeholders::_1));
  // for the future: subscribe to all the peers' paths to avoid them.
  // for (const auto& peer_id : peer_ids_) {
  //   traj_sub_map_[peer_id] = this->create_subscription<nav_msgs::msg::Path>(
  //       "traj_" + peer_id, 10,
  //       std::bind(&DronePlanner::TrajectoryCallback, this,
  //       std::placeholders::_1, peer_id));
  // }

  depth_image_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      ns + "/depth_image", 10,
      std::bind(&DronePlanner::DepthImageCallback, this,
                std::placeholders::_1));

  desired_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      ns + "/trajectory", 10);

  // Store the timer as a member variable to keep it alive
  traj_pub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DronePlanner::PublishTrajectory, this));
}

void DronePlanner::GoalsCallback(const nav_msgs::msg::Goals& msg)
{
  // right now this just sets the current goal to the first goal in the vec.
  // Later, this can use the whole list so the drone can be more autonomous
  auto logger = this->get_logger();
  if (msg.goals.size() > 0) {
    auto goal = msg.goals.at(0);
    if (goal.pose != current_goal_.pose) {
      current_goal_ = goal;
      new_goal_ = true;
    }
  } else {
    RCLCPP_WARN(logger, "Got zero goals");
  }
}

void DronePlanner::DepthImageCallback(
    const sensor_msgs::msg::PointCloud2& msg) {
  // placeholder for future
}

nav_msgs::msg::Path DronePlanner::GenerateTrajectory() {
  // new_goal_ = false;
  const float DISTANCE_TOL_M = 0.05;  // if within 5cm of goal, we are there
                                      // and do not generate any more
  const float ANGLE_TOL_RAD =
      1.0 * 2 * M_PI;  // if the angle is within 1 degree, we are there.

  nav_msgs::msg::Path trajectory = nav_msgs::msg::Path();
  trajectory.header.frame_id = "map";
  trajectory.header.stamp = this->now();

  geometry_msgs::msg::Pose current_goal_unstamped; 
  geometry_msgs::msg::Pose current_position_unstamped;

  current_goal_unstamped = current_goal_.pose;
  current_position_unstamped = current_position_.pose;

  geometry_msgs::msg::Pose delta = current_goal_unstamped - current_position_unstamped;

  Eigen::Vector3d delta2;
  delta2 << delta.position.x, delta.position.y, delta.position.z;

  float dist = delta2.norm();
  tf2::Quaternion quat;
  tf2::fromMsg(delta.orientation, quat);

  auto angle = quat.getAngle();

  if (abs(angle) < ANGLE_TOL_RAD && abs(dist) < DISTANCE_TOL_M) {
    // we are within our tolerance, so don't plan anything.

    // todo: do we use the stamp field?
    trajectory.poses.push_back(current_position_);
    return trajectory;
  }

  // otherwise, find the distance and make the trajectory.

  float time_to_target_translation = dist / drone_speed_m_s_;
  float time_to_target_rotation = angle / yaw_speed_rad_s_;
  float traj_time =
      std::max(time_to_target_rotation, time_to_target_translation);
  int num_traj_points =
      static_cast<int>(std::ceil(traj_time * trajectory_density_hz_));

  RCLCPP_INFO(this->get_logger(), "Num points: %d", num_traj_points);

  tf2::Quaternion start_quaternion, goal_quaternion;

  tf2::fromMsg(current_position_.pose.orientation, start_quaternion);
  tf2::fromMsg(current_goal_.pose.orientation, goal_quaternion);

  // so we don't have to keep growing the vec
  trajectory.poses.reserve(num_traj_points);

  auto translation_step_size = delta2 / num_traj_points;

  for (int i = 0; i < num_traj_points; i++) {
    // linearly interpolate both the linear pose and the quaternion
    auto pose = geometry_msgs::msg::PoseStamped();
    pose.pose.position.x =
        current_position_.pose.position.x + translation_step_size.x() * i;
    pose.pose.position.y =
        current_position_.pose.position.y + translation_step_size.y() * i;
    pose.pose.position.z =
        current_position_.pose.position.z + translation_step_size.z() * i;

    double t = static_cast<double>(i) / static_cast<double>(num_traj_points);
    tf2::Quaternion interp_quat = start_quaternion.slerp(goal_quaternion, t);

    pose.pose.orientation = tf2::toMsg(interp_quat);
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    trajectory.poses.push_back(pose);
  }

  return trajectory;
}

void DronePlanner::PublishTrajectory() {
  auto logger = this->get_logger();
  if (new_goal_) {
    trajectory_ = GenerateTrajectory();
  }

  desired_traj_pub_->publish(trajectory_);
}

void DronePlanner::MavrosPoseCallback(
    const geometry_msgs::msg::PoseStamped& msg) {
  RCLCPP_INFO(this->get_logger(), "Got a new position");
  current_position_ = msg;
}

}  // namespace drone_planner
