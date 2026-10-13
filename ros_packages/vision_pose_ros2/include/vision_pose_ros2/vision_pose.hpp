#ifndef VISION_POSE_CLASS_H_
#define VISION_POSE_CLASS_H_

#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "optitrack_multiplexer_ros2_msgs/msg/rigid_body_stamped.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "swarmnxt_msgs/msg/vision_pose_status.hpp"
#include "vision_pose_core.hpp"

namespace vision_pose {

// The node that gives PX4 its pose: the mocap, or the onboard VIO placed in
// the room. This class only wires topics, parameters and timers; what is sent
// is decided in VisionPoseCore.
class VisionPose : public rclcpp::Node {
 public:
  // constructor; throws std::invalid_argument on a parameter that cannot work
  VisionPose();

 private:
  /*-------------- methods ---------------*/
  // declare ros parameters
  void DeclareRosParameters();

  // declare a number, or an array of numbers, that may also be given as
  // integers
  void DeclareDouble(const std::string &name, double value);
  void DeclareDoubleArray(const std::string &name,
                          const std::vector<double> &value);

  // initialize ros parameters; returns the ones the logic uses
  Params InitializeRosParameters();

  // callback for mocap data
  void MocapCallback(
      const optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped::SharedPtr);

  // callback for the estimator pose
  void VioCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);

  // callback for the arming state
  void VehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr);

  // callback for PX4's own position
  void VehicleOdometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr);

  // time-based checks at 100 Hz and status at 10 Hz (without a VIO topic:
  // the status only, at 10 Hz)
  void TimerCallback();

  // send one sample to PX4
  void PublishPx4(const Px4Sample &sample);

  // publish the status message
  void PublishStatus();

  // print what the logic has to say; publish the status on a change of source
  void FlushCore();

  // steady clock in seconds, for the timeouts
  static double SteadyNow();

  /*--------- member variables -----------*/
  // topic on which we get the data from mocap
  std::string mocap_topic_;
  // topic of the estimator pose ("" = no VIO handling at all)
  std::string vio_topic_;
  // PX4 topics we listen to
  std::string vehicle_status_topic_;
  std::string vehicle_odometry_topic_;
  // topics of the placed VIO and of the status
  std::string odometry_topic_;
  std::string status_topic_;

  // source selection, placement and conversions
  std::unique_ptr<VisionPoseCore> core_;

  // subscribers
  rclcpp::Subscription<optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped>::
      SharedPtr mocap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      vio_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
      vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr
      vehicle_odometry_sub_;

  // publishers
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<swarmnxt_msgs::msg::VisionPoseStatus>::SharedPtr
      status_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  // timer calls per status message, and their count
  unsigned int status_every_ = 10;
  unsigned int timer_ticks_ = 0;

  // messages reused at every publication (constant fields set once)
  px4_msgs::msg::VehicleOdometry px4_msg_;
  nav_msgs::msg::Odometry odometry_msg_;

  // clock of the throttled log lines
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

}  // namespace vision_pose

#endif
