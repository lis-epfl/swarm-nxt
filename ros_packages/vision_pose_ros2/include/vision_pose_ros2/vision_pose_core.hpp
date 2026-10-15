#ifndef VISION_POSE_CORE_H_
#define VISION_POSE_CORE_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <vector>

namespace vision_pose {

// what PX4 is fed (values of swarmnxt_msgs/VisionPoseStatus.source)
enum class Source : uint8_t { kNone = 0, kMocap = 1, kVio = 2 };

// why PX4 is not fed the VIO (values of swarmnxt_msgs/VisionPoseStatus.reason)
enum class Reason : uint8_t {
  kNone = 0,
  kVioNotReadyBeforeArming = 1,
  kVioTimeout = 2,
  kDistanceLimit = 3,
  kNoMount = 4,
  kNoVehicleStatus = 5,
  kOperator = 6,
  kVioNotReady = 7,
  kNoVehicleOdometry = 8
};

const char *SourceName(Source source);
const char *ReasonText(Reason reason);

// timestamp_sample of the PX4 message from a stamp in nanoseconds: the
// operation of px4_ros2::LocalPositionMeasurementInterface::update() (int64
// to double, times 1e-3, truncated). Integer division differs from it by 1 us
// on part of the samples.
inline uint64_t StampToPx4Us(int64_t stamp_ns) {
  return static_cast<uint64_t>(static_cast<double>(stamp_ns) * 1e-3);
}

// node parameters used by the logic (angles in radians)
struct Params {
  // mocap feed
  double pos_var = 1e-6;
  double att_var = 1e-6;
  // a VIO topic is given: place the VIO and report on it
  bool vio_enabled = false;
  // PX4 may be fed the VIO
  bool use_vio = false;
  // heading of the IMU relative to the rigid body, from the calibration
  bool has_mount = false;
  double mount_yaw = 0.0;
  // without any mocap sample, place the VIO at this pose (outdoors)
  bool world_from_start_pose = false;
  Eigen::Vector3d start_position = Eigen::Vector3d::Zero();
  double start_yaw = 0.0;
  // fallback to the mocap: PX4's position against the mocap
  double fallback_dist_m = 0.15;
  double fallback_hold_s = 0.10;
  // VIO readiness and loss
  double vio_settle_s = 7.0;
  double vio_still_m = 0.03;
  double vio_timeout_s = 0.33;
  // VIO feed
  double vio_pos_var = 1e-6;
  double vio_att_var = 1e-6;
  // position of the IMU from the origin of the rigid body, in the IMU frame
  // (metres), from the calibration: the VIO is moved back by it onto the
  // point the mocap reports. Zero: the VIO is taken as it comes.
  Eigen::Vector3d vio_lever_arm = Eigen::Vector3d::Zero();
  // after a fallback, send the attitude of the IMU body instead of the rigid
  // body's
  bool correct_mocap_attitude = false;
  // heading trim (rate 0 = off)
  double heading_trim_rate = 0.0;
  double heading_trim_start_m = 2.0;
  double heading_trim_window_s = 20.0;
  double heading_trim_max = 2.0 * M_PI / 180.0;
};

// one mocap sample as received (room frame, z up; float32 like the message)
struct MocapSample {
  int64_t stamp_ns = 0;
  bool tracking_valid = false;
  float px = 0.0f, py = 0.0f, pz = 0.0f;
  float qx = 0.0f, qy = 0.0f, qz = 0.0f, qw = 1.0f;
};

// one estimator pose: IMU body (FLU) in the VIO world (z up)
struct VioPose {
  int64_t stamp_ns = 0;
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

// the fields of px4_msgs/VehicleOdometry that change from message to message
struct Px4Sample {
  uint64_t timestamp_sample = 0;           // microseconds
  float position[3] = {0.0f, 0.0f, 0.0f};  // NED
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};   // w, x, y, z; FRD body to NED
  float position_variance = 0.0f;
  float orientation_variance = 0.0f;
  uint8_t reset_counter = 0;
};

// the VIO pose placed in the room (z up, IMU body FLU)
struct PlacedPose {
  int64_t stamp_ns = 0;
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

// content of swarmnxt_msgs/VisionPoseStatus (angles in degrees)
struct Status {
  Source source = Source::kNone;
  bool use_vio = false;
  bool vio_alive = false;
  bool vio_ready = false;
  bool mocap_valid = false;
  bool armed = false;
  bool fallback_latched = false;
  Reason reason = Reason::kNone;
  uint8_t reset_counter = 0;
  float heading_deg = std::numeric_limits<float>::quiet_NaN();
  float heading_trim_deg = 0.0f;
  float heading_fit_deg = std::numeric_limits<float>::quiet_NaN();
  float body_tilt_deg = std::numeric_limits<float>::quiet_NaN();
  float vio_mocap_dist_m = std::numeric_limits<float>::quiet_NaN();
  float px4_mocap_dist_m = std::numeric_limits<float>::quiet_NaN();
  float vio_age_s = std::numeric_limits<float>::quiet_NaN();
  uint32_t vio_poses = 0;
  uint32_t px4_sent = 0;
  uint32_t fallbacks = 0;
  uint32_t dist_exceed_count = 0;
};

// ring buffer of fixed capacity, allocated once; index 0 is the oldest element
// and a push on a full buffer overwrites it
template <typename T>
class Ring {
 public:
  explicit Ring(size_t capacity) : data_(capacity) {}

  void Clear() {
    head_ = 0;
    size_ = 0;
  }
  void Push(const T &value) {
    data_[(head_ + size_) % data_.size()] = value;
    if (size_ < data_.size()) {
      ++size_;
    } else {
      head_ = (head_ + 1) % data_.size();
    }
  }
  void PopOldest() {
    head_ = (head_ + 1) % data_.size();
    --size_;
  }
  bool empty() const { return size_ == 0; }
  size_t size() const { return size_; }
  const T &operator[](size_t i) const {
    return data_[(head_ + i) % data_.size()];
  }
  const T &Oldest() const { return (*this)[0]; }
  const T &Newest() const { return (*this)[size_ - 1]; }

 private:
  std::vector<T> data_;
  size_t head_ = 0;
  size_t size_ = 0;
};

// Everything the node decides: which source PX4 is fed, where the VIO sits in
// the room, the conversions to the PX4 message. No ROS dependency: stamps are
// nanoseconds, `now_s` is a steady clock in seconds. Not thread safe (the
// node spins a single-threaded executor).
class VisionPoseCore {
 public:
  explicit VisionPoseCore(const Params &params);

  // what a VIO pose produced
  struct VioResult {
    bool px4 = false;       // *px4 is to be sent to PX4
    bool placed = false;    // *placed holds the pose in the room
    bool rejected = false;  // the pose was not usable (values or order)
  };

  // one mocap sample; true when *px4 is to be sent to PX4
  bool HandleMocap(const MocapSample &sample, double now_s, Px4Sample *px4);

  // one estimator pose; age_s is its age at arrival (for the status only)
  VioResult HandleVio(const VioPose &pose, double now_s, double age_s,
                      Px4Sample *px4, PlacedPose *placed);

  // arming state from the vehicle status; it counts as known for a limited
  // time after the latest message
  void HandleVehicleStatus(bool armed, double now_s);

  // PX4's own position (NED) at its sample stamp, for the distance rule
  void HandleVehicleOdometry(int64_t stamp_ns,
                             const Eigen::Vector3d &position_ned, double now_s);

  // periodic checks: the VIO timeout, the comparison of PX4 with the mocap
  void HandleTimer(double now_s);

  Status GetStatus(double now_s) const;

  // INFO lines for the node to print, oldest first
  bool PopLogLine(std::string *line);

  // true once after every change of source
  bool TakeSourceChanged();

 private:
  /*-------------- types ---------------*/
  struct MocapEntry {
    int64_t stamp_ns = 0;
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  };
  // VIO pose with the mocap interpolated at its stamp
  struct PairEntry {
    int64_t stamp_ns = 0;
    Eigen::Vector3d p_v = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_v = Eigen::Quaterniond::Identity();
    Eigen::Vector3d p_m = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_m = Eigen::Quaterniond::Identity();
  };
  struct OdometryEntry {
    int64_t stamp_ns = 0;
    Eigen::Vector3d p_ned = Eigen::Vector3d::Zero();
  };
  // VIO pose the mocap has not reached, with where it was placed on arrival
  struct WaitingEntry {
    int64_t stamp_ns = 0;
    Eigen::Vector3d p_v = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_v = Eigen::Quaterniond::Identity();
    bool placed = false;
    Eigen::Vector3d p_w = Eigen::Vector3d::Zero();
  };

  /*-------------- methods ---------------*/
  // mocap at a stamp, from two valid samples around it; q may be null
  bool InterpolateMocap(int64_t stamp_ns, Eigen::Vector3d *p,
                        Eigen::Quaterniond *q) const;

  // the VIO has stood still for vio_settle_s without a gap in its poses
  bool IsSettled() const;

  // place the VIO in the room from the last second of poses; true when a
  // placement against the mocap was made at this pose
  bool Place();

  // pair the waiting VIO poses whose stamp the mocap has passed
  void PairWaiting(double now_s);

  // in flight, from one pair: heading fit and start of the trim
  void UpdateHeadingFit(int64_t stamp_ns, const Eigen::Vector3d &p_v,
                        const Eigen::Vector3d &p_m);

  // in flight, at every pose: slew of the trim toward the fit
  void SlewTrim(int64_t stamp_ns, int64_t prev_stamp_ns);

  // from one pair: distance of the placed VIO to the mocap and its crossings
  // of the limit
  void EvaluateVioDistance(const Eigen::Vector3d &p_w,
                           const Eigen::Vector3d &p_m, double now_s);

  // MOCAP/NONE -> VIO when everything allows it
  void MaybeSwitchToVio(double now_s);

  // compare the waiting PX4 positions with the mocap (distance rule)
  void EvaluatePx4Distance(double now_s);

  // VIO -> MOCAP (or NONE), latched
  void Fallback(Reason reason, const std::string &detail, double now_s);

  void SetSource(Source source, const std::string &reason);

  // conversions to the PX4 message
  void FillMocap(const MocapSample &sample, Px4Sample *px4) const;
  void FillVio(const PlacedPose &placed, Px4Sample *px4) const;

  bool MocapValid(double now_s) const;
  // a vehicle status arrived recently: the arming state is known
  bool ArmingKnown(double now_s) const;
  // PX4's position was compared with the mocap recently: the distance rule
  // is being evaluated
  bool Px4DistanceKnown(double now_s) const;
  // the placement in use, or the one that would be made, has what it needs
  bool MountOk() const;
  Reason CurrentReason(double now_s) const;

  void Log(const char *format, ...) __attribute__((format(printf, 2, 3)));

  /*--------- member variables -----------*/
  const Params params_;
  // parameters as the PX4 message carries them
  const float pos_var_f_;
  const float att_var_f_;
  const float vio_pos_var_f_;
  const float vio_att_var_f_;
  // durations in nanoseconds of stamps
  const int64_t settle_ns_;
  const int64_t hold_ns_;
  const int64_t trim_window_ns_;

  // state machine
  Source source_ = Source::kNone;
  bool source_changed_ = false;
  uint8_t reset_counter_ = 0;
  bool fallback_latched_ = false;
  Reason fallback_reason_ = Reason::kNone;
  bool have_status_ = false;
  bool armed_ = false;
  double status_rx_s_ = 0.0;  // arrival of the latest vehicle status

  // mocap
  Ring<MocapEntry> mocap_ring_;
  bool mocap_seen_ = false;        // a valid sample has arrived once
  bool mocap_last_valid_ = false;  // the latest sample was valid
  double mocap_rx_s_ = 0.0;        // arrival of the latest valid sample
  double mocap_run_s_ = 0.0;       // arrival of the first sample after a hole
  double mocap_kept_s_ = 0.0;      // arrival of the latest one in mocap_ring_
  int mocap_out_of_order_ = 0;     // samples in a row left out of the buffer

  // VIO
  Ring<MocapEntry> vio_ring_;        // poses of the settle window (VIO world)
  Ring<PairEntry> pair_ring_;        // pairs of the placement window
  Ring<WaitingEntry> waiting_ring_;  // poses the mocap has not reached
  int64_t vio_stamp_ns_ = 0;         // stamp of the latest pose (0: none)
  int64_t vio_run_start_ns_ = 0;     // first stamp of the gap-free run
  double vio_rx_s_ = 0.0;
  double vio_age_s_ = std::numeric_limits<double>::quiet_NaN();
  bool vio_ready_ = false;  // settled and placed (kept while armed)
  bool placing_ = false;    // the pose before was placed against the mocap

  // timer
  double timer_s_ = -1.0;        // previous call of HandleTimer
  int timer_late_calls_ = 0;     // late calls since the timer was last calm
  double timer_late_s_ = -1e9;   // the latest late call
  double timer_grace_s_ = -1.0;  // the time-based checks are skipped until then
  double vio_held_s_ = 0.0;      // hold-ups of the node since the latest pose
  Reason reason_logged_ = Reason::kNone;  // of the latest "VIO not used" line

  // placement: p_W = Rz(psi0 + trim) (p_v - p_v0) + p_0
  bool placed_ = false;
  bool placement_from_mocap_ = false;
  int placement_pairs_ = 0;
  double psi0_ = 0.0;
  Eigen::Vector3d p_v0_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_0_ = Eigen::Vector3d::Zero();
  // rigid body -> IMU body, identified with the placement
  Eigen::Quaterniond q_c_ = Eigen::Quaterniond::Identity();
  double body_tilt_ = std::numeric_limits<double>::quiet_NaN();
  // the one PX4 had on the VIO when the VIO was given up: it stays in the
  // mocap feed whatever is identified afterwards
  bool have_feed_c_ = false;
  Eigen::Quaterniond q_c_feed_ = Eigen::Quaterniond::Identity();

  // heading fit and trim
  double fit_cross_ = 0.0;
  double fit_dot_ = 0.0;
  double heading_fit_ = std::numeric_limits<double>::quiet_NaN();
  double trim_ = 0.0;
  bool trim_started_ = false;
  int64_t trim_start_ns_ = 0;

  // distances
  Ring<OdometryEntry>
      odometry_pending_;           // PX4 positions the mocap has not reached
  int64_t odometry_stamp_ns_ = 0;  // stamp of the latest one stored
  int odometry_out_of_order_ = 0;  // positions in a row left out
  double px4_mocap_dist_ = std::numeric_limits<double>::quiet_NaN();
  double px4_dist_rx_s_ = 0.0;
  bool px4_above_ = false;  // above the limit at the last comparison
  int64_t px4_above_since_ns_ = 0;
  int64_t px4_eval_stamp_ns_ = 0;  // stamp of the last comparison
  double vio_mocap_dist_ = std::numeric_limits<double>::quiet_NaN();
  double vio_dist_rx_s_ = 0.0;  // when the pair it comes from was formed
  bool vio_above_ = false;

  // counters
  uint32_t vio_poses_ = 0;
  uint32_t px4_sent_ = 0;
  uint32_t fallbacks_ = 0;
  uint32_t dist_exceed_count_ = 0;

  std::deque<std::string> log_;
};

}  // namespace vision_pose

#endif
