#include "vision_pose.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace vision_pose {

namespace {

// version suffix of a PX4 topic name ("" or "_v<N>"), built from the message
// type by the rule of px4_ros2::getMessageNameVersion()
template <typename T, typename = void>
struct HasMessageVersion : std::false_type {};

template <typename T>
struct HasMessageVersion<T, std::void_t<decltype(T::MESSAGE_VERSION)>>
    : std::true_type {};

template <typename T>
std::string MessageNameVersion() {
  if constexpr (HasMessageVersion<T>::value) {
    if (T::MESSAGE_VERSION == 0) {
      return "";
    }
    return "_v" + std::to_string(T::MESSAGE_VERSION);
  } else {
    return "";
  }
}

// stamp of a message in nanoseconds (rclcpp::Time would throw on a negative
// one)
int64_t StampNs(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<int64_t>(stamp.sec) * 1000000000LL +
         static_cast<int64_t>(stamp.nanosec);
}

builtin_interfaces::msg::Time StampMsg(int64_t stamp_ns) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
  return stamp;
}

double Rad(double deg) { return deg * M_PI / 180.0; }

// a parameter that cannot work stops the node at start
void RequirePositive(const std::string &name, double value) {
  if (!(std::isfinite(value) && value > 0.0)) {
    throw std::invalid_argument("parameter " + name +
                                " must be finite and above zero");
  }
}

void RequireNotNegative(const std::string &name, double value) {
  if (!(std::isfinite(value) && value >= 0.0)) {
    throw std::invalid_argument("parameter " + name +
                                " must be finite and not negative");
  }
}

void RequireFinite(const std::string &name, double value) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument("parameter " + name + " must be finite");
  }
}

void RequireTopic(const std::string &name, const std::string &value) {
  if (value.empty()) {
    throw std::invalid_argument("parameter " + name + " must not be empty");
  }
}

}  // namespace

VisionPose::VisionPose() : rclcpp::Node("vision_pose_node") {
  // declare environment parameters
  DeclareRosParameters();

  // initialize parameters
  const Params params = InitializeRosParameters();
  core_ = std::make_unique<VisionPoseCore>(params);

  // constant part of the PX4 message, as the px4_ros2 local position
  // measurement interface sent it (NED frames, no velocity, PX4 sets the
  // timestamp)
  px4_msg_.timestamp = 0;
  px4_msg_.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  px4_msg_.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
  px4_msg_.velocity = {NAN, NAN, NAN};
  px4_msg_.velocity_variance = {NAN, NAN, NAN};
  px4_msg_.angular_velocity = {NAN, NAN, NAN};
  px4_msg_.quality = 0;

  // the placed VIO: IMU body in the room frame
  odometry_msg_.header.frame_id = "world";
  odometry_msg_.child_frame_id = "imu";

  // PX4's pose input, on the topic name and QoS of the px4_ros2 interface
  // the old node used: relative name, version suffix of the message, depth 10
  px4_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
      "fmu/in/vehicle_visual_odometry" +
          MessageNameVersion<px4_msgs::msg::VehicleOdometry>(),
      10);

  status_pub_ =
      create_publisher<swarmnxt_msgs::msg::VisionPoseStatus>(status_topic_, 10);

  rclcpp::QoS best_effort_qos =
      rclcpp::QoS(rclcpp::KeepLast(10))
          .reliability(rclcpp::ReliabilityPolicy::BestEffort);

  // subscription to the mocap topic
  mocap_sub_ = create_subscription<
      optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped>(
      mocap_topic_, 10,
      std::bind(&VisionPose::MocapCallback, this, std::placeholders::_1));

  vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      vehicle_status_topic_, best_effort_qos,
      std::bind(&VisionPose::VehicleStatusCallback, this,
                std::placeholders::_1));

  vehicle_odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      vehicle_odometry_topic_, best_effort_qos,
      std::bind(&VisionPose::VehicleOdometryCallback, this,
                std::placeholders::_1));

  // the estimator, only when a topic is given
  if (!vio_topic_.empty()) {
    odometry_pub_ =
        create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 10);
    vio_sub_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            vio_topic_, best_effort_qos,
            std::bind(&VisionPose::VioCallback, this, std::placeholders::_1));
  }

  // wall timer (steady clock): 100 Hz to see the VIO stop, 10 Hz when there
  // is only the status to publish
  status_every_ = vio_topic_.empty() ? 1 : 10;
  timer_ = create_wall_timer(std::chrono::milliseconds(100 / status_every_),
                             std::bind(&VisionPose::TimerCallback, this));

  RCLCPP_INFO(get_logger(), "Using mocap topic: %s", mocap_topic_.c_str());
  if (vio_topic_.empty()) {
    RCLCPP_INFO(get_logger(), "No VIO topic: PX4 is fed the mocap");
  } else if (!params.use_vio) {
    RCLCPP_INFO(get_logger(),
                "Using VIO topic: %s (placed and logged only, PX4 is fed the "
                "mocap)",
                vio_topic_.c_str());
  } else {
    RCLCPP_INFO(get_logger(),
                "Using VIO topic: %s (PX4 is fed the VIO once it is ready, "
                "fallback to the mocap at %.2f m)",
                vio_topic_.c_str(), params.fallback_dist_m);
  }
  RCLCPP_INFO(get_logger(), "Vision pose node initialized, PX4 input on %s",
              px4_pub_->get_topic_name());
}

void VisionPose::DeclareDouble(const std::string &name, double value) {
  // read once at start: a later change is refused instead of ignored
  rcl_interfaces::msg::ParameterDescriptor fixed;
  fixed.read_only = true;
  // a value given as an integer (3 for 3.0) is taken as that number; any
  // other type is refused by the declaration, which stops the node
  const auto &overrides =
      get_node_parameters_interface()->get_parameter_overrides();
  const auto given = overrides.find(name);
  if (given != overrides.end() &&
      given->second.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    const double number = static_cast<double>(given->second.get<int64_t>());
    declare_parameter(name, rclcpp::ParameterValue(number), fixed, true);
    return;
  }
  declare_parameter<double>(name, value, fixed);
}

void VisionPose::DeclareDoubleArray(const std::string &name,
                                    const std::vector<double> &value) {
  rcl_interfaces::msg::ParameterDescriptor fixed;
  fixed.read_only = true;
  // as DeclareDouble: an array of integers is taken as those numbers
  const auto &overrides =
      get_node_parameters_interface()->get_parameter_overrides();
  const auto given = overrides.find(name);
  if (given != overrides.end() &&
      given->second.get_type() ==
          rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
    const std::vector<int64_t> &integers =
        given->second.get<std::vector<int64_t>>();
    declare_parameter(name,
                      rclcpp::ParameterValue(std::vector<double>(
                          integers.begin(), integers.end())),
                      fixed, true);
    return;
  }
  declare_parameter<std::vector<double>>(name, value, fixed);
}

void VisionPose::DeclareRosParameters() {
  // read once at start: a later change is refused instead of ignored
  rcl_interfaces::msg::ParameterDescriptor fixed;
  fixed.read_only = true;

  declare_parameter<std::string>(
      "mocap_topic", "/optitrack_multiplexer_node/rigid_body/mavros", fixed);
  DeclareDouble("pos_var", 0.000001);
  DeclareDouble("att_var", 0.000001);
  declare_parameter<std::string>("vio_topic", "", fixed);
  declare_parameter<bool>("use_vio", false, fixed);
  declare_parameter<bool>("has_mount", false, fixed);
  DeclareDouble("mount_yaw_deg", 0.0);
  declare_parameter<bool>("world_from_start_pose", false, fixed);
  DeclareDoubleArray("start_position", {0.0, 0.0, 0.0});
  DeclareDouble("start_yaw_deg", 0.0);
  DeclareDouble("fallback_dist_m", 0.15);
  DeclareDouble("fallback_hold_s", 0.10);
  DeclareDouble("vio_settle_s", 7.0);
  DeclareDouble("vio_still_m", 0.03);
  DeclareDouble("vio_timeout_s", 0.33);
  DeclareDoubleArray("vio_lever_arm", {0.0, 0.0, 0.0});
  DeclareDouble("vio_pos_var", 0.000001);
  DeclareDouble("vio_att_var", 0.000001);
  declare_parameter<bool>("correct_mocap_attitude", false, fixed);
  DeclareDouble("heading_trim_rate_dps", 0.0);
  DeclareDouble("heading_trim_start_m", 2.0);
  DeclareDouble("heading_trim_window_s", 20.0);
  DeclareDouble("heading_trim_max_deg", 2.0);
  declare_parameter<std::string>("vehicle_status_topic",
                                 "/fmu/out/vehicle_status_v1", fixed);
  declare_parameter<std::string>("vehicle_odometry_topic",
                                 "/fmu/out/vehicle_odometry", fixed);
  declare_parameter<std::string>("odometry_topic", "vio/odometry", fixed);
  declare_parameter<std::string>("status_topic", "vio/status", fixed);
}

Params VisionPose::InitializeRosParameters() {
  Params params;

  mocap_topic_ = get_parameter("mocap_topic").as_string();
  vio_topic_ = get_parameter("vio_topic").as_string();
  vehicle_status_topic_ = get_parameter("vehicle_status_topic").as_string();
  vehicle_odometry_topic_ = get_parameter("vehicle_odometry_topic").as_string();
  odometry_topic_ = get_parameter("odometry_topic").as_string();
  status_topic_ = get_parameter("status_topic").as_string();
  RequireTopic("mocap_topic", mocap_topic_);
  RequireTopic("vehicle_status_topic", vehicle_status_topic_);
  RequireTopic("vehicle_odometry_topic", vehicle_odometry_topic_);
  RequireTopic("odometry_topic", odometry_topic_);
  RequireTopic("status_topic", status_topic_);

  // variances of the mocap feed (PX4 needs them above zero)
  params.pos_var = get_parameter("pos_var").as_double();
  params.att_var = get_parameter("att_var").as_double();
  RequirePositive("pos_var", params.pos_var);
  RequirePositive("att_var", params.att_var);

  // VIO
  params.vio_enabled = !vio_topic_.empty();
  params.use_vio = get_parameter("use_vio").as_bool();
  if (params.use_vio && !params.vio_enabled) {
    RCLCPP_WARN(get_logger(),
                "use_vio is set without a vio_topic: PX4 is fed the mocap");
    params.use_vio = false;
  }
  params.has_mount = get_parameter("has_mount").as_bool();
  const double mount_yaw_deg = get_parameter("mount_yaw_deg").as_double();
  RequireFinite("mount_yaw_deg", mount_yaw_deg);
  params.mount_yaw = Rad(mount_yaw_deg);

  params.world_from_start_pose =
      get_parameter("world_from_start_pose").as_bool();
  const std::vector<double> start_position =
      get_parameter("start_position").as_double_array();
  if (start_position.size() != 3) {
    throw std::invalid_argument("parameter start_position must hold 3 values");
  }
  for (size_t i = 0; i < 3; ++i) {
    RequireFinite("start_position", start_position[i]);
    params.start_position(static_cast<Eigen::Index>(i)) = start_position[i];
  }
  const double start_yaw_deg = get_parameter("start_yaw_deg").as_double();
  RequireFinite("start_yaw_deg", start_yaw_deg);
  params.start_yaw = Rad(start_yaw_deg);

  params.fallback_dist_m = get_parameter("fallback_dist_m").as_double();
  params.fallback_hold_s = get_parameter("fallback_hold_s").as_double();
  RequirePositive("fallback_dist_m", params.fallback_dist_m);
  RequireNotNegative("fallback_hold_s", params.fallback_hold_s);

  params.vio_settle_s = get_parameter("vio_settle_s").as_double();
  params.vio_still_m = get_parameter("vio_still_m").as_double();
  params.vio_timeout_s = get_parameter("vio_timeout_s").as_double();
  RequirePositive("vio_settle_s", params.vio_settle_s);
  RequirePositive("vio_still_m", params.vio_still_m);
  RequirePositive("vio_timeout_s", params.vio_timeout_s);

  const std::vector<double> lever_arm =
      get_parameter("vio_lever_arm").as_double_array();
  if (lever_arm.size() != 3) {
    throw std::invalid_argument("parameter vio_lever_arm must hold 3 values");
  }
  for (size_t i = 0; i < 3; ++i) {
    RequireFinite("vio_lever_arm", lever_arm[i]);
    params.vio_lever_arm(static_cast<Eigen::Index>(i)) = lever_arm[i];
  }
  params.vio_pos_var = get_parameter("vio_pos_var").as_double();
  params.vio_att_var = get_parameter("vio_att_var").as_double();
  RequirePositive("vio_pos_var", params.vio_pos_var);
  RequirePositive("vio_att_var", params.vio_att_var);

  params.correct_mocap_attitude =
      get_parameter("correct_mocap_attitude").as_bool();

  // heading trim
  const double trim_rate_dps =
      get_parameter("heading_trim_rate_dps").as_double();
  const double trim_max_deg = get_parameter("heading_trim_max_deg").as_double();
  params.heading_trim_start_m =
      get_parameter("heading_trim_start_m").as_double();
  params.heading_trim_window_s =
      get_parameter("heading_trim_window_s").as_double();
  RequireNotNegative("heading_trim_rate_dps", trim_rate_dps);
  RequireNotNegative("heading_trim_max_deg", trim_max_deg);
  RequireNotNegative("heading_trim_start_m", params.heading_trim_start_m);
  RequireNotNegative("heading_trim_window_s", params.heading_trim_window_s);
  params.heading_trim_rate = Rad(trim_rate_dps);
  params.heading_trim_max = Rad(trim_max_deg);

  return params;
}

void VisionPose::MocapCallback(
    const optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped::SharedPtr
        msg) {
  RCLCPP_INFO_THROTTLE(
      this->get_logger(), steady_clock_, 1000,
      "[THROTTLED] Executing mocap callback with position: [%5.2f, %5.2f, "
      "%5.2f]",
      msg->rigid_body.pose.position.x, msg->rigid_body.pose.position.y,
      msg->rigid_body.pose.position.z);

  MocapSample sample;
  sample.stamp_ns = StampNs(msg->stamp);
  sample.tracking_valid = msg->rigid_body.tracking_valid;
  sample.px = msg->rigid_body.pose.position.x;
  sample.py = msg->rigid_body.pose.position.y;
  sample.pz = msg->rigid_body.pose.position.z;
  sample.qx = msg->rigid_body.pose.orientation.q_x;
  sample.qy = msg->rigid_body.pose.orientation.q_y;
  sample.qz = msg->rigid_body.pose.orientation.q_z;
  sample.qw = msg->rigid_body.pose.orientation.q_w;

  // an invalid sample gives no message
  Px4Sample px4;
  if (core_->HandleMocap(sample, SteadyNow(), &px4)) {
    PublishPx4(px4);
  }
  FlushCore();
}

void VisionPose::VioCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  // pose of the IMU body (FLU) in the estimator's world, stamped on the IMU
  // clock; read as a Hamilton quaternion the orientation is body -> world
  VioPose pose;
  pose.stamp_ns = StampNs(msg->header.stamp);
  pose.p = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                           msg->pose.pose.position.z);
  pose.q = Eigen::Quaterniond(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  const double age_s =
      static_cast<double>(this->now().nanoseconds() - pose.stamp_ns) * 1e-9;

  Px4Sample px4;
  PlacedPose placed;
  const VisionPoseCore::VioResult result =
      core_->HandleVio(pose, SteadyNow(), age_s, &px4, &placed);
  if (result.px4) {
    PublishPx4(px4);
  }
  if (result.placed) {
    odometry_msg_.header.stamp = StampMsg(placed.stamp_ns);
    odometry_msg_.pose.pose.position.x = placed.p.x();
    odometry_msg_.pose.pose.position.y = placed.p.y();
    odometry_msg_.pose.pose.position.z = placed.p.z();
    odometry_msg_.pose.pose.orientation.w = placed.q.w();
    odometry_msg_.pose.pose.orientation.x = placed.q.x();
    odometry_msg_.pose.pose.orientation.y = placed.q.y();
    odometry_msg_.pose.pose.orientation.z = placed.q.z();
    odometry_pub_->publish(odometry_msg_);
  }
  if (result.rejected) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), steady_clock_, 1000,
        "[THROTTLED] VIO pose dropped: not finite, not a rotation or out of "
        "order");
  }
  FlushCore();
}

void VisionPose::VehicleStatusCallback(
    const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
  core_->HandleVehicleStatus(
      msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED,
      SteadyNow());
  FlushCore();
}

void VisionPose::VehicleOdometryCallback(
    const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
  // PX4's position in NED at timestamp_sample: the stamp of the IMU sample
  // the estimate belongs to, in microseconds and already on this computer's
  // clock (the uXRCE-DDS client adds its time offset), so it compares with
  // the mocap at equal stamps
  if (msg->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED ||
      msg->timestamp_sample >
          static_cast<uint64_t>(std::numeric_limits<int64_t>::max() / 1000)) {
    return;
  }
  core_->HandleVehicleOdometry(
      static_cast<int64_t>(msg->timestamp_sample) * 1000,
      Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]),
      SteadyNow());
  FlushCore();
}

void VisionPose::TimerCallback() {
  core_->HandleTimer(SteadyNow());
  FlushCore();
  // status at 10 Hz
  if (++timer_ticks_ % status_every_ == 0) {
    PublishStatus();
  }
}

void VisionPose::PublishPx4(const Px4Sample &sample) {
  px4_msg_.timestamp_sample = sample.timestamp_sample;
  for (size_t i = 0; i < 3; ++i) {
    px4_msg_.position[i] = sample.position[i];
    px4_msg_.position_variance[i] = sample.position_variance;
    px4_msg_.orientation_variance[i] = sample.orientation_variance;
  }
  for (size_t i = 0; i < 4; ++i) {
    px4_msg_.q[i] = sample.q[i];
  }
  px4_msg_.reset_counter = sample.reset_counter;
  px4_pub_->publish(px4_msg_);
}

void VisionPose::PublishStatus() {
  const Status status = core_->GetStatus(SteadyNow());
  swarmnxt_msgs::msg::VisionPoseStatus msg;
  msg.header.stamp = this->now();
  msg.source = static_cast<uint8_t>(status.source);
  msg.use_vio = status.use_vio;
  msg.vio_alive = status.vio_alive;
  msg.vio_ready = status.vio_ready;
  msg.mocap_valid = status.mocap_valid;
  msg.armed = status.armed;
  msg.fallback_latched = status.fallback_latched;
  msg.reason = static_cast<uint8_t>(status.reason);
  msg.reset_counter = status.reset_counter;
  msg.heading_deg = status.heading_deg;
  msg.heading_trim_deg = status.heading_trim_deg;
  msg.heading_fit_deg = status.heading_fit_deg;
  msg.body_tilt_deg = status.body_tilt_deg;
  msg.vio_mocap_dist_m = status.vio_mocap_dist_m;
  msg.px4_mocap_dist_m = status.px4_mocap_dist_m;
  msg.vio_age_s = status.vio_age_s;
  msg.vio_poses = status.vio_poses;
  msg.px4_sent = status.px4_sent;
  msg.fallbacks = status.fallbacks;
  msg.dist_exceed_count = status.dist_exceed_count;
  status_pub_->publish(msg);
}

void VisionPose::FlushCore() {
  std::string line;
  while (core_->PopLogLine(&line)) {
    RCLCPP_INFO(get_logger(), "%s", line.c_str());
  }
  if (core_->TakeSourceChanged()) {
    PublishStatus();
  }
}

double VisionPose::SteadyNow() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

}  // namespace vision_pose
