#include "drone_planner.h"

namespace drone_planner {

geometry_msgs::msg::Pose operator-(geometry_msgs::msg::PoseStamped& pose1,
                                   geometry_msgs::msg::PoseStamped& pose2) {
  geometry_msgs::msg::Pose result;
  tf2::Quaternion q1;
  tf2::Quaternion q2;

  tf2::fromMsg(pose1.pose.orientation, q1);
  tf2::fromMsg(pose2.pose.orientation, q2);

  auto q1_inv = q1;
  q1_inv[3] = -q1_inv[3];
  auto qr = q2 * q1_inv;

  result.orientation = tf2::toMsg(qr);

  result.position.x = pose2.pose.position.x - pose1.pose.position.x;
  result.position.y = pose2.pose.position.y - pose1.pose.position.y;
  result.position.z = pose2.pose.position.z - pose1.pose.position.z;

  return result;
}

DronePlanner::DronePlanner() : ::rclcpp::Node("drone_planner") {
  std::string peer_file;

  this->declare_parameter("peer_file", "config/peers");
  this->get_parameter("peer_file", peer_file);

  // DeclareRosParameters();
  // InitializeRosParameters();

  goals_sub_ = this->create_subscription<nav_msgs::msg::Goals>(
      "~/goals", 10,
      std::bind(&DronePlanner::GoalsCallback, this, std::placeholders::_1));

  mavros_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/mavros/local_position/pose", 10,
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
      "~/depth_image", 10,
      std::bind(&DronePlanner::DepthImageCallback, this,
                std::placeholders::_1));

  desired_traj_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("~/trajectory_desired", 10);

  auto timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DronePlanner::PublishTrajectoryFromGoal, this));
}

void DronePlanner::GoalsCallback(const nav_msgs::msg::Goals& msg) {
  // right now this just sets the current goal to the first goal in the vec.
  // Later, this can use the whole list so the drone can be more autonomous
  auto logger = this->get_logger();
  if (msg.goals.size() > 0) {
    current_goal_ = msg.goals.at(0);
  } else {
    RCLCPP_WARN(logger, "Got zero goals");
  }
}

void DronePlanner::DepthImageCallback(
    const sensor_msgs::msg::PointCloud2& msg) {
  // placeholder for future
}

void DronePlanner::PublishTrajectoryFromGoal() {
  // todo: only do this if the goal has _changed_. otherwise we can take a
  // shortcut and send the previous trajectory.

  const float DISTANCE_TOL_M =
      0.05;  // if within 5cm of goal, we are there and do not generate any more
  const float ANGLE_TOL_RAD =
      1.0 * 2 * M_PI;  // if the angle is within 1 degree, we are there.

  nav_msgs::msg::Path trajectory = nav_msgs::msg::Path();
  trajectory.header.frame_id = "local_drone";
  trajectory.header.stamp = this->now();

  geometry_msgs::msg::Pose delta = current_goal_ - cur_pos_;

  Eigen::Vector3d delta2;
  delta2 << delta.position.x, delta.position.y, delta.position.z;

  float dist = delta2.norm();
  tf2::Quaternion quat;
  tf2::fromMsg(delta.orientation, quat);

  auto angle = quat.getAngle();

  if (abs(angle) < ANGLE_TOL_RAD && abs(dist) < DISTANCE_TOL_M) {
    // we are within our tolerance, so don't plan anything.

    // todo: do we use the stamp field?
    trajectory.poses.push_back(cur_pos_);
    desired_traj_pub_->publish(trajectory);
    return;
  }

  // otherwise, find the distance and make the trajectory.

  float time_to_target_translation = dist / drone_speed_m_s_;
  float time_to_target_rotation = angle / yaw_speed_rad_s_;
  float traj_time =
      std::max(time_to_target_rotation, time_to_target_translation);
  int num_traj_points =
      static_cast<int>(std::ceil(traj_time / trajectory_density_hz_));

  tf2::Quaternion start_quaternion, goal_quaternion;

  tf2::fromMsg(cur_pos_.pose.orientation, start_quaternion);
  tf2::fromMsg(current_goal_.pose.orientation, goal_quaternion);

  // so we don't have to keep growing the vec
  trajectory.poses.reserve(num_traj_points);

  auto translation_step_size = delta2 / traj_time;

  for (int i = 0; i < num_traj_points; i++) {
    // linearly interpolate both the linear pose and the quaternion
    auto pose = geometry_msgs::msg::PoseStamped();
    pose.pose.position.x =
        cur_pos_.pose.position.x + translation_step_size.x() * i;
    pose.pose.position.y =
        cur_pos_.pose.position.y + translation_step_size.y() * i;
    pose.pose.position.z =
        cur_pos_.pose.position.z + translation_step_size.z() * i;

    double t = static_cast<double>(i) / static_cast<double>(num_traj_points);
    tf2::Quaternion interp_quat = start_quaternion.slerp(goal_quaternion, t);

    pose.pose.orientation = tf2::toMsg(interp_quat);
    pose.header.frame_id = "local_drone";
    pose.header.stamp = this->now();
    trajectory.poses.push_back(pose);
  }

  desired_traj_pub_->publish(trajectory);
}

void DronePlanner::MavrosPoseCallback(
    const geometry_msgs::msg::PoseStamped& msg) {
  cur_pos_ = msg;
}

}  // namespace drone_planner
