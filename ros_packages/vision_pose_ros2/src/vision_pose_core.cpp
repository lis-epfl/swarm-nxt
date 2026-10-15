#include "vision_pose_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>

namespace vision_pose {

namespace {

// estimator poses further apart than this (stamps) break the standstill
constexpr int64_t kVioMaxGapNs = 200'000'000;
// the placement is computed from the poses of this last stretch
constexpr int64_t kPlacementWindowNs = 1'000'000'000;
// poses needed in it
constexpr int kMinPlacementPoses = 3;
// two mocap samples further apart than this do not bracket a stamp
constexpr int64_t kMocapMaxGapNs = 50'000'000;
// a valid mocap sample that arrived longer ago than this is not available
constexpr double kMocapValidS = 0.1;
// a comparison of PX4 with the mocap older than this is not reported, and
// the distance rule does not count as being evaluated
constexpr double kPx4DistValidS = 0.5;
// the hold of the distance rule goes on over a stretch without comparison
// (PX4 positions further apart, no mocap around them) up to this long
constexpr int64_t kPx4EvalMaxGapNs = 500'000'000;
// on the VIO, placed against the mocap: after this long without a comparison
// of PX4 with the mocap, while the mocap has been arriving without a hole for
// as long, the distance rule is not running and the VIO is given up
constexpr double kNoComparisonS = 1.0;
// the distance of the placed VIO to the mocap is reported for this long after
// the pair it comes from was formed
constexpr double kVioDistValidS = 0.5;
// the arming state is known for this long after a vehicle status (PX4 sends
// one every 0.5 s; the longest gap of the recorded flight is 1.01 s)
constexpr double kStatusValidS = 2.0;
// the heading fit is published from this much summed |d_vio| |d_mocap| (m^2)
constexpr double kHeadingFitMinM2 = 1.0;
// stamps that go back by more than this start a new stream (replay, clock
// step); smaller steps back are samples out of order and are left out
constexpr int64_t kStreamRestartNs = 5'000'000'000;
// VIO stamps that jump forward by more than this, beyond the time the node
// itself was held up: the estimator has started again, in another world
constexpr double kVioRestartJumpS = 1.0;
// this many mocap samples or PX4 positions in a row behind the newest stamp
// held: that stamp was the wrong one (a single stamp far ahead), start again
constexpr int kOutOfOrderRestart = 20;
// calls of HandleTimer further apart than this: the node itself was held up
constexpr double kTimerHeldUpS = 0.05;
// after a hold-up the time-based checks are skipped for this long: what
// arrived meanwhile is read first
constexpr double kTimerGraceS = 0.2;
// only this many late calls start that period again, until the timer has
// been on time for kTimerCalmS: hold-ups that keep coming, with calls on time
// in between or without, must not switch the checks off
constexpr int kTimerLateCallsMax = 2;
constexpr double kTimerCalmS = 1.0;
// buffer sizes
constexpr size_t kMocapRingSize = 512;    // 2.1 s at 240 Hz
constexpr size_t kPairRingSize = 256;     // the placement window
constexpr size_t kWaitingRingSize = 64;   // VIO poses ahead of the mocap
constexpr size_t kOdometryRingSize = 64;  // 0.64 s at 100 Hz
constexpr double kVioMaxRateHz = 250.0;   // the settle window holds this rate

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

double Wrap(double angle) { return std::remainder(angle, 2.0 * M_PI); }

double Deg(double rad) { return rad * 180.0 / M_PI; }

double Yaw(const Eigen::Matrix3d &r) { return std::atan2(r(1, 0), r(0, 0)); }

Eigen::Quaterniond Rz(double angle) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
}

size_t VioRingSize(double settle_s) {
  const double poses = std::ceil(settle_s * kVioMaxRateHz) + 8.0;
  return static_cast<size_t>(std::min(std::max(poses, 64.0), 65536.0));
}

// Attitude for PX4 (FRD body to NED) from the quaternion (w, x, y, z) of the
// FLU body in the room: the rotation is (w, x, -y, -z) and it is sent with
// the opposite sign, normalised in float32, exactly as the old node did.
void SetPx4Attitude(float w, float x, float y, float z, Px4Sample *px4) {
  const Eigen::Quaternionf q = Eigen::Quaternionf(-w, -x, y, z).normalized();
  px4->q[0] = q.w();
  px4->q[1] = q.x();
  px4->q[2] = q.y();
  px4->q[3] = q.z();
}

}  // namespace

const char *SourceName(Source source) {
  switch (source) {
    case Source::kMocap:
      return "MOCAP";
    case Source::kVio:
      return "VIO";
    default:
      return "NONE";
  }
}

const char *ReasonText(Reason reason) {
  switch (reason) {
    case Reason::kVioNotReadyBeforeArming:
      return "vio not ready before arming";
    case Reason::kVioTimeout:
      return "vio timeout";
    case Reason::kDistanceLimit:
      return "distance limit";
    case Reason::kNoMount:
      return "no mount";
    case Reason::kNoVehicleStatus:
      return "no vehicle status";
    case Reason::kOperator:
      return "operator (use_vio false)";
    case Reason::kVioNotReady:
      return "vio not ready";
    case Reason::kNoVehicleOdometry:
      return "no vehicle odometry";
    default:
      return "none";
  }
}

VisionPoseCore::VisionPoseCore(const Params &params)
    : params_(params),
      pos_var_f_(static_cast<float>(params.pos_var)),
      att_var_f_(static_cast<float>(params.att_var)),
      vio_pos_var_f_(static_cast<float>(params.vio_pos_var)),
      vio_att_var_f_(static_cast<float>(params.vio_att_var)),
      settle_ns_(static_cast<int64_t>(params.vio_settle_s * 1e9)),
      hold_ns_(static_cast<int64_t>(params.fallback_hold_s * 1e9)),
      trim_window_ns_(static_cast<int64_t>(params.heading_trim_window_s * 1e9)),
      mocap_ring_(kMocapRingSize),
      vio_ring_(VioRingSize(params.vio_settle_s)),
      pair_ring_(kPairRingSize),
      waiting_ring_(kWaitingRingSize),
      odometry_pending_(kOdometryRingSize) {}

bool VisionPoseCore::HandleMocap(const MocapSample &sample, double now_s,
                                 Px4Sample *px4) {
  // an invalid or non-finite sample is dropped
  const bool valid = sample.tracking_valid && sample.stamp_ns > 0 &&
                     std::isfinite(sample.px) && std::isfinite(sample.py) &&
                     std::isfinite(sample.pz) && std::isfinite(sample.qx) &&
                     std::isfinite(sample.qy) && std::isfinite(sample.qz) &&
                     std::isfinite(sample.qw);
  mocap_last_valid_ = valid;
  if (!valid) {
    return false;
  }
  mocap_seen_ = true;
  mocap_rx_s_ = now_s;

  // keep the sample for the comparisons at equal stamps (stamps increasing,
  // quaternion close to unit)
  const Eigen::Quaterniond q(sample.qw, sample.qx, sample.qy, sample.qz);
  const double norm2 = q.squaredNorm();
  if (norm2 > 0.81 && norm2 < 1.21) {
    if (!mocap_ring_.empty() &&
        sample.stamp_ns <= mocap_ring_.Newest().stamp_ns &&
        (mocap_ring_.Newest().stamp_ns - sample.stamp_ns > kStreamRestartNs ||
         ++mocap_out_of_order_ >= kOutOfOrderRestart)) {
      mocap_ring_.Clear();
    }
    if (mocap_ring_.empty() ||
        sample.stamp_ns > mocap_ring_.Newest().stamp_ns) {
      // the samples bracket every stamp from here on, until the next hole
      if (mocap_ring_.empty() ||
          sample.stamp_ns - mocap_ring_.Newest().stamp_ns > kMocapMaxGapNs) {
        mocap_run_s_ = now_s;
      }
      mocap_kept_s_ = now_s;
      mocap_ring_.Push({sample.stamp_ns,
                        Eigen::Vector3d(sample.px, sample.py, sample.pz),
                        q.normalized()});
      mocap_out_of_order_ = 0;
    }
  }

  // the VIO poses that waited for this sample, and the distance rules: they
  // may give the VIO up, then this sample is the first one PX4 gets from the
  // mocap
  PairWaiting(now_s);
  EvaluatePx4Distance(now_s);

  if (source_ == Source::kNone) {
    SetSource(Source::kMocap, fallback_latched_ ? "mocap valid again"
                                                : "first valid mocap sample");
  }
  if (source_ != Source::kMocap) {
    return false;
  }
  FillMocap(sample, px4);
  ++px4_sent_;
  return true;
}

VisionPoseCore::VioResult VisionPoseCore::HandleVio(const VioPose &pose,
                                                    double now_s, double age_s,
                                                    Px4Sample *px4,
                                                    PlacedPose *placed) {
  VioResult result;
  if (!params_.vio_enabled) {
    return result;
  }
  // the poses that wait for the mocap come first: pairs are formed in the
  // order of their stamps
  PairWaiting(now_s);
  // a pose that is not finite, not a rotation or out of order is dropped
  const double norm2 = pose.q.squaredNorm();
  if (pose.stamp_ns <= 0 || !pose.p.allFinite() ||
      !pose.q.coeffs().allFinite() || !(norm2 > 0.81 && norm2 < 1.21)) {
    result.rejected = true;
    return result;
  }
  if (vio_stamp_ns_ > 0 && pose.stamp_ns <= vio_stamp_ns_) {
    // A new stream is taken only on the ground while PX4 is not fed the VIO:
    // the placement in use belongs to the old one. Otherwise its poses are
    // dropped like any pose out of order, and PX4 on the VIO goes back to the
    // mocap by the timeout.
    if (vio_stamp_ns_ - pose.stamp_ns <= kStreamRestartNs || armed_ ||
        source_ == Source::kVio) {
      result.rejected = true;
      return result;
    }
    vio_ring_.Clear();
    pair_ring_.Clear();
    waiting_ring_.Clear();
    vio_stamp_ns_ = 0;
  }
  // a hold-up of the node itself makes the stamps jump as well: the poses of
  // that time were not read
  const double held_s =
      vio_held_s_ + (timer_s_ >= 0.0 && now_s - timer_s_ > kTimerHeldUpS
                         ? now_s - timer_s_
                         : 0.0);
  const double jump_s =
      static_cast<double>(pose.stamp_ns - vio_stamp_ns_) * 1e-9;
  // The stamps that pass use the hold-up time up. (Poses that waited through
  // a hold-up are read first, with stamps from its beginning: the jump to
  // the present comes after them.)
  vio_held_s_ = std::max(0.0, vio_held_s_ - std::max(jump_s, 0.0));
  if (vio_stamp_ns_ > 0 && jump_s - held_s > kVioRestartJumpS) {
    // The estimator has started again: its poses are in another world and
    // the placement does not hold for them. PX4 on the VIO goes back to the
    // mocap first (it keeps the body rotation it had); on the ground the VIO
    // is placed again once it has stood, in flight it stays unplaced.
    if (source_ == Source::kVio) {
      char detail[96];
      std::snprintf(detail, sizeof(detail), "vio timeout: stamps jumped %.2f s",
                    jump_s);
      Fallback(Reason::kVioTimeout, detail, now_s);
    }
    if (placed_) {
      Log("VIO stream restarted (stamps jumped %.2f s): placement dropped",
          jump_s);
    }
    placed_ = false;
    vio_ready_ = false;
    body_tilt_ = kNaN;
    heading_fit_ = kNaN;
    trim_started_ = false;
    vio_mocap_dist_ = kNaN;
    vio_above_ = false;
    vio_ring_.Clear();
    pair_ring_.Clear();
    waiting_ring_.Clear();
  }
  const Eigen::Quaterniond q_v = pose.q.normalized();
  // The estimator gives the position of the IMU, the mocap that of the rigid
  // body's origin. Moved back by the lever arm of the calibration (the IMU's
  // position from that origin, in the IMU frame), the VIO describes the same
  // point as the mocap at any attitude: for the placement, the distance to the
  // mocap and PX4.
  const Eigen::Vector3d p_v = pose.p - q_v * params_.vio_lever_arm;
  const int64_t prev_stamp_ns = vio_stamp_ns_;
  if (prev_stamp_ns == 0 || pose.stamp_ns - prev_stamp_ns > kVioMaxGapNs) {
    vio_run_start_ns_ = pose.stamp_ns;
  }
  vio_stamp_ns_ = pose.stamp_ns;
  vio_rx_s_ = now_s;
  vio_age_s_ = age_s;
  ++vio_poses_;
  vio_ring_.Push({pose.stamp_ns, p_v, q_v});

  // the mocap at the stamp of the pose; a pose the mocap has not reached
  // waits for it (PairWaiting)
  Eigen::Vector3d p_m = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_m = Eigen::Quaterniond::Identity();
  const bool paired = InterpolateMocap(pose.stamp_ns, &p_m, &q_m);
  const bool waits = !paired && (mocap_ring_.empty() ||
                                 pose.stamp_ns > mocap_ring_.Newest().stamp_ns);
  if (paired) {
    pair_ring_.Push({pose.stamp_ns, p_v, q_v, p_m, q_m});
  }

  // place the VIO in the room, on the ground only: the placement is frozen
  // from the arming edge on, and while the arming state is not known
  bool placed_now = false;  // against the mocap, at this pose
  if (ArmingKnown(now_s) && !armed_) {
    const bool was_ready = vio_ready_;
    const bool settled = IsSettled();
    // where the placement in use so far puts this pose
    const double psi_old = psi0_ + trim_;
    const Eigen::Vector3d p_old = Rz(psi_old) * (p_v - p_v0_) + p_0_;
    if (settled) {
      placed_now = Place();
    }
    vio_ready_ = settled && placed_;
    if (vio_ready_ && !was_ready) {
      if (placement_from_mocap_) {
        Log("VIO ready: heading %.2f deg (mount yaw %.2f deg), pin vio [%.3f, "
            "%.3f, %.3f] room [%.3f, %.3f, %.3f], body tilt %.2f deg, %d pairs",
            Deg(Wrap(psi0_)), Deg(params_.has_mount ? params_.mount_yaw : 0.0),
            p_v0_.x(), p_v0_.y(), p_v0_.z(), p_0_.x(), p_0_.y(), p_0_.z(),
            Deg(body_tilt_), placement_pairs_);
      } else {
        Log("VIO ready: placed at the start pose without mocap, heading %.2f "
            "deg, pin vio [%.3f, %.3f, %.3f] room [%.3f, %.3f, %.3f]",
            Deg(Wrap(psi0_)), p_v0_.x(), p_v0_.y(), p_v0_.z(), p_0_.x(),
            p_0_.y(), p_0_.z());
      }
    } else if (!vio_ready_ && was_ready) {
      Log("VIO not ready any more: its poses moved or paused");
    }
    if (placed_now && !placing_ && source_ == Source::kVio) {
      // The first placement after a stretch without one (the flight, the
      // drone was moved, the mocap was away) while PX4 is fed the VIO: the
      // pose sent steps, and PX4 is told so. The placement at every pose that
      // follows while it stands is not a step.
      ++reset_counter_;
      // (rounded here, so that a turn of nothing is not printed as -0.00)
      const double turn_deg =
          std::round(Deg(Wrap(psi0_ + trim_ - psi_old)) * 100.0) / 100.0;
      Log("VIO placed again: step %.3f m, heading %+.2f deg, reset_counter %u",
          (Rz(psi0_ + trim_) * (p_v - p_v0_) + p_0_ - p_old).norm(),
          turn_deg + 0.0, static_cast<unsigned>(reset_counter_));
    }
  }
  placing_ = placed_now;

  if (placed_) {
    if (armed_) {
      if (paired) {
        UpdateHeadingFit(pose.stamp_ns, p_v, p_m);
      }
      SlewTrim(pose.stamp_ns, prev_stamp_ns);
    }
    const Eigen::Quaterniond q_psi = Rz(psi0_ + trim_);
    placed->stamp_ns = pose.stamp_ns;
    placed->p = q_psi * (p_v - p_v0_) + p_0_;
    placed->q = q_psi * q_v;
    result.placed = true;

    // distance to the mocap
    if (paired) {
      EvaluateVioDistance(placed->p, p_m, now_s);
    } else if (source_ == Source::kVio) {
      vio_above_ = false;
    }
  }
  if (waits) {
    waiting_ring_.Push({pose.stamp_ns, p_v, q_v, result.placed, placed->p});
  }

  MaybeSwitchToVio(now_s);

  if (source_ == Source::kVio && placed_) {
    FillVio(*placed, px4);
    ++px4_sent_;
    result.px4 = true;
  }
  return result;
}

void VisionPoseCore::HandleVehicleStatus(bool armed, double now_s) {
  const bool arming_edge = have_status_ && !armed_ && armed;
  have_status_ = true;
  armed_ = armed;
  status_rx_s_ = now_s;
  if (!arming_edge) {
    return;
  }
  // a new flight: the heading fit starts again; the trim stays with the
  // placement it belongs to
  fit_cross_ = 0.0;
  fit_dot_ = 0.0;
  heading_fit_ = kNaN;
  trim_started_ = false;
  if (placed_) {
    Log("armed: placement frozen, heading %.2f deg, pin room [%.3f, %.3f, "
        "%.3f]",
        Deg(Wrap(psi0_ + trim_)), p_0_.x(), p_0_.y(), p_0_.z());
  }
  if (params_.vio_enabled && params_.use_vio && source_ != Source::kVio) {
    Log("armed: PX4 stays on %s for this flight (%s)", SourceName(source_),
        ReasonText(CurrentReason(now_s)));
  }
}

void VisionPoseCore::HandleVehicleOdometry(int64_t stamp_ns,
                                           const Eigen::Vector3d &position_ned,
                                           double now_s) {
  if (stamp_ns <= 0 || !position_ned.allFinite()) {
    return;
  }
  if (stamp_ns <= odometry_stamp_ns_) {
    if (odometry_stamp_ns_ - stamp_ns <= kStreamRestartNs &&
        ++odometry_out_of_order_ < kOutOfOrderRestart) {
      return;
    }
    odometry_pending_.Clear();
    px4_above_ = false;
  }
  odometry_out_of_order_ = 0;
  odometry_stamp_ns_ = stamp_ns;
  // it waits until mocap samples bracket its stamp
  odometry_pending_.Push({stamp_ns, position_ned});
  EvaluatePx4Distance(now_s);
}

void VisionPoseCore::HandleTimer(double now_s) {
  if (!params_.vio_enabled) {
    return;  // the mocap alone: nothing to watch
  }
  // say why PX4 is not fed the VIO, once per reason
  if (params_.use_vio) {
    const Reason reason = CurrentReason(now_s);
    if (reason != reason_logged_ && reason != Reason::kNone) {
      Log("VIO not used: %s", ReasonText(reason));
    }
    reason_logged_ = reason;
  }

  // After a hold-up of the node itself what arrived meanwhile is still
  // waiting to be read: the silence is not the estimator's, nor PX4's. The
  // time-based checks are skipped for a while after it.
  if (timer_s_ >= 0.0 && now_s - timer_s_ > kTimerHeldUpS) {
    vio_held_s_ += now_s - timer_s_;
    // a late call long after the one before starts the count again
    if (now_s - timer_late_s_ > kTimerCalmS) {
      timer_late_calls_ = 0;
    }
    timer_late_s_ = now_s;
    if (++timer_late_calls_ <= kTimerLateCallsMax) {
      timer_grace_s_ = now_s + kTimerGraceS;
    }
  }
  timer_s_ = now_s;
  if (now_s < timer_grace_s_ || source_ != Source::kVio) {
    return;
  }
  const double silence_s = now_s - vio_rx_s_;
  if (silence_s > params_.vio_timeout_s) {
    char detail[96];
    std::snprintf(detail, sizeof(detail), "vio timeout: no pose for %.2f s",
                  silence_s);
    Fallback(Reason::kVioTimeout, detail, now_s);
    return;
  }
  // Placed against the mocap, PX4 stays on the VIO only while its position
  // is being compared with the mocap. A mocap that is away, has just come
  // back or gives no sample to compare with (none kept in mocap_ring_) does
  // not count: the VIO is what PX4 has then.
  const double blind_s = now_s - px4_dist_rx_s_;
  if (placement_from_mocap_ && MocapValid(now_s) &&
      now_s - mocap_kept_s_ <= kMocapValidS &&
      now_s - mocap_run_s_ >= kNoComparisonS && blind_s > kNoComparisonS) {
    char detail[128];
    std::snprintf(detail, sizeof(detail),
                  "no comparison of PX4 with the mocap for %.2f s: the "
                  "distance rule cannot run",
                  blind_s);
    Fallback(Reason::kNoVehicleOdometry, detail, now_s);
  }
}

Status VisionPoseCore::GetStatus(double now_s) const {
  Status status;
  status.source = source_;
  status.use_vio = params_.vio_enabled && params_.use_vio;
  status.vio_alive =
      vio_stamp_ns_ > 0 && now_s - vio_rx_s_ <= params_.vio_timeout_s;
  status.vio_ready = vio_ready_ && status.vio_alive;
  status.mocap_valid = MocapValid(now_s);
  status.armed = armed_;
  status.fallback_latched = fallback_latched_;
  status.reason = CurrentReason(now_s);
  status.reset_counter = reset_counter_;
  if (placed_) {
    status.heading_deg = static_cast<float>(Deg(Wrap(psi0_ + trim_)));
    status.heading_trim_deg = static_cast<float>(Deg(trim_));
  }
  status.heading_fit_deg = static_cast<float>(Deg(heading_fit_));
  status.body_tilt_deg = static_cast<float>(Deg(body_tilt_));
  if (now_s - vio_dist_rx_s_ <= kVioDistValidS) {
    status.vio_mocap_dist_m = static_cast<float>(vio_mocap_dist_);
  }
  if (Px4DistanceKnown(now_s)) {
    status.px4_mocap_dist_m = static_cast<float>(px4_mocap_dist_);
  }
  status.vio_age_s = static_cast<float>(vio_age_s_);
  status.vio_poses = vio_poses_;
  status.px4_sent = px4_sent_;
  status.fallbacks = fallbacks_;
  status.dist_exceed_count = dist_exceed_count_;
  return status;
}

bool VisionPoseCore::PopLogLine(std::string *line) {
  if (log_.empty()) {
    return false;
  }
  *line = std::move(log_.front());
  log_.pop_front();
  return true;
}

bool VisionPoseCore::TakeSourceChanged() {
  const bool changed = source_changed_;
  source_changed_ = false;
  return changed;
}

bool VisionPoseCore::InterpolateMocap(int64_t stamp_ns, Eigen::Vector3d *p,
                                      Eigen::Quaterniond *q) const {
  // from the newest pair of samples back: the stamp asked for is recent
  for (size_t i = mocap_ring_.size(); i-- > 1;) {
    const MocapEntry &after = mocap_ring_[i];
    const MocapEntry &before = mocap_ring_[i - 1];
    if (stamp_ns > after.stamp_ns) {
      return false;  // the mocap has not reached this stamp
    }
    if (stamp_ns < before.stamp_ns) {
      continue;
    }
    const int64_t gap_ns = after.stamp_ns - before.stamp_ns;
    if (gap_ns > kMocapMaxGapNs) {
      return false;  // the mocap was lost around this stamp
    }
    const double u = static_cast<double>(stamp_ns - before.stamp_ns) /
                     static_cast<double>(gap_ns);
    *p = before.p + u * (after.p - before.p);
    if (q != nullptr) {
      *q = before.q.slerp(u, after.q);
    }
    return true;
  }
  return false;
}

bool VisionPoseCore::IsSettled() const {
  // the poses without a gap must cover vio_settle_s
  const int64_t start_ns = vio_stamp_ns_ - settle_ns_;
  if (vio_run_start_ns_ > start_ns) {
    return false;
  }
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  size_t first = vio_ring_.size();
  for (size_t i = vio_ring_.size(); i-- > 0;) {
    if (vio_ring_[i].stamp_ns < start_ns) {
      break;
    }
    mean += vio_ring_[i].p;
    first = i;
  }
  // the buffer must hold the whole window (it does up to kVioMaxRateHz)
  if (first == vio_ring_.size() ||
      vio_ring_[first].stamp_ns - start_ns > kVioMaxGapNs) {
    return false;
  }
  mean /= static_cast<double>(vio_ring_.size() - first);
  // and all of them lie within vio_still_m of their mean
  for (size_t i = first; i < vio_ring_.size(); ++i) {
    if ((vio_ring_[i].p - mean).norm() > params_.vio_still_m) {
      return false;
    }
  }
  return true;
}

bool VisionPoseCore::Place() {
  // placed at the start pose: once, there is nothing to place it against
  if (placed_ && !placement_from_mocap_) {
    return false;
  }
  const int64_t start_ns = vio_stamp_ns_ - kPlacementWindowNs;

  // pairs of the last second: heading from the two attitudes, pin from the
  // two positions
  int pairs = 0;
  double sum_sin = 0.0;
  double sum_cos = 0.0;
  Eigen::Vector3d sum_v = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_m = Eigen::Vector3d::Zero();
  for (size_t i = pair_ring_.size(); i-- > 0;) {
    const PairEntry &pair = pair_ring_[i];
    if (pair.stamp_ns < start_ns) {
      break;
    }
    const double yaw =
        Yaw((pair.q_m * pair.q_v.conjugate()).toRotationMatrix());
    sum_sin += std::sin(yaw);
    sum_cos += std::cos(yaw);
    sum_v += pair.p_v;
    sum_m += pair.p_m;
    ++pairs;
  }
  if (pairs >= kMinPlacementPoses) {
    const double psi0 = std::atan2(sum_sin, sum_cos) +
                        (params_.has_mount ? params_.mount_yaw : 0.0);
    // rigid body -> IMU body: mean of R_v^T Rz(psi0)^T R_m over the pairs,
    // projected onto a rotation
    const Eigen::Quaterniond q_psi_inv = Rz(-psi0);
    Eigen::Matrix3d sum_c = Eigen::Matrix3d::Zero();
    for (size_t i = pair_ring_.size(); i-- > 0;) {
      const PairEntry &pair = pair_ring_[i];
      if (pair.stamp_ns < start_ns) {
        break;
      }
      sum_c += (pair.q_v.conjugate() * q_psi_inv * pair.q_m).toRotationMatrix();
    }
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        sum_c, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d u = svd.matrixU();
    if ((u * svd.matrixV().transpose()).determinant() < 0.0) {
      u.col(2) = -u.col(2);
    }
    const Eigen::Matrix3d c = u * svd.matrixV().transpose();

    psi0_ = psi0;
    p_v0_ = sum_v / static_cast<double>(pairs);
    p_0_ = sum_m / static_cast<double>(pairs);
    q_c_ = Eigen::Quaterniond(c);
    body_tilt_ = std::acos(std::min(std::max(c(2, 2), -1.0), 1.0));
    trim_ = 0.0;
    placement_pairs_ = pairs;
    placement_from_mocap_ = true;
    placed_ = true;
    return true;
  }

  // Without any mocap sample the VIO may only be placed at the given start
  // pose (outdoors); indoors it stays unplaced and PX4 is never fed it.
  if (placed_ || mocap_seen_ || !params_.world_from_start_pose) {
    return false;
  }
  int poses = 0;
  sum_sin = 0.0;
  sum_cos = 0.0;
  sum_v.setZero();
  for (size_t i = vio_ring_.size(); i-- > 0;) {
    const MocapEntry &entry = vio_ring_[i];
    if (entry.stamp_ns < start_ns) {
      break;
    }
    const double yaw = Yaw(entry.q.toRotationMatrix());
    sum_sin += std::sin(yaw);
    sum_cos += std::cos(yaw);
    sum_v += entry.p;
    ++poses;
  }
  if (poses < kMinPlacementPoses) {
    return false;
  }
  psi0_ = params_.start_yaw - std::atan2(sum_sin, sum_cos);
  p_v0_ = sum_v / static_cast<double>(poses);
  p_0_ = params_.start_position;
  q_c_ = Eigen::Quaterniond::Identity();
  body_tilt_ = kNaN;
  trim_ = 0.0;
  placement_pairs_ = 0;
  placement_from_mocap_ = false;
  placed_ = true;
  return false;
}

void VisionPoseCore::PairWaiting(double now_s) {
  while (!waiting_ring_.empty() && !mocap_ring_.empty() &&
         waiting_ring_.Oldest().stamp_ns <= mocap_ring_.Newest().stamp_ns) {
    const WaitingEntry entry = waiting_ring_.Oldest();
    waiting_ring_.PopOldest();
    Eigen::Vector3d p_m = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_m = Eigen::Quaterniond::Identity();
    if (!InterpolateMocap(entry.stamp_ns, &p_m, &q_m)) {
      continue;  // no valid mocap around this stamp: never paired
    }
    // what a pair is used for, as for a pose paired when it arrives; the
    // distance is that of the pose as it was placed (and sent) then
    pair_ring_.Push({entry.stamp_ns, entry.p_v, entry.q_v, p_m, q_m});
    if (placed_ && armed_) {
      UpdateHeadingFit(entry.stamp_ns, entry.p_v, p_m);
    }
    if (placed_ && entry.placed) {
      EvaluateVioDistance(entry.p_w, p_m, now_s);
    }
  }
}

void VisionPoseCore::UpdateHeadingFit(int64_t stamp_ns,
                                      const Eigen::Vector3d &p_v,
                                      const Eigen::Vector3d &p_m) {
  // horizontal displacement from the pin: the VIO placed without trim, and
  // the mocap. The rotation that maps the first onto the second, summed
  // over the flight, is atan2(sum of cross products, sum of dot products).
  const Eigen::Vector3d d_v = Rz(psi0_) * (p_v - p_v0_);
  const Eigen::Vector3d d_m = p_m - p_0_;
  fit_cross_ += d_v.x() * d_m.y() - d_v.y() * d_m.x();
  fit_dot_ += d_v.x() * d_m.x() + d_v.y() * d_m.y();
  if (std::hypot(fit_cross_, fit_dot_) >= kHeadingFitMinM2) {
    heading_fit_ = std::atan2(fit_cross_, fit_dot_);
  }
  // the trim starts the first time the drone is far enough from the pin
  if (!trim_started_ && params_.heading_trim_rate > 0.0 &&
      std::hypot(d_m.x(), d_m.y()) >= params_.heading_trim_start_m) {
    trim_started_ = true;
    trim_start_ns_ = stamp_ns;
  }
}

void VisionPoseCore::SlewTrim(int64_t stamp_ns, int64_t prev_stamp_ns) {
  // slew toward the fit for the trim window, then stay
  if (trim_started_ && stamp_ns - trim_start_ns_ <= trim_window_ns_ &&
      std::isfinite(heading_fit_) && prev_stamp_ns > 0) {
    const double limit = params_.heading_trim_rate *
                         static_cast<double>(stamp_ns - prev_stamp_ns) * 1e-9;
    const double step = Wrap(heading_fit_ - trim_);
    trim_ += std::min(std::max(step, -limit), limit);
    trim_ = std::min(std::max(trim_, -params_.heading_trim_max),
                     params_.heading_trim_max);
  }
}

void VisionPoseCore::EvaluateVioDistance(const Eigen::Vector3d &p_w,
                                         const Eigen::Vector3d &p_m,
                                         double now_s) {
  const double dist = (p_w - p_m).norm();
  vio_mocap_dist_ = dist;
  vio_dist_rx_s_ = now_s;
  // its crossings of the limit are counted while PX4 is not on the VIO (on
  // the VIO the PX4 position is what counts)
  const bool above = source_ != Source::kVio && dist > params_.fallback_dist_m;
  if (above && !vio_above_) {
    ++dist_exceed_count_;
  }
  vio_above_ = above;
}

void VisionPoseCore::MaybeSwitchToVio(double now_s) {
  // only on the ground, never in the air; never again after a fallback
  if (source_ == Source::kVio || !params_.use_vio || fallback_latched_ ||
      !ArmingKnown(now_s) || armed_ || !vio_ready_ || !MountOk()) {
    return;
  }
  // placed against the mocap, the VIO is fed only while the distance rule
  // is being evaluated: PX4's own position must be arriving
  if (placement_from_mocap_ && !Px4DistanceKnown(now_s)) {
    return;
  }
  ++reset_counter_;
  SetSource(Source::kVio, "vio ready");
}

void VisionPoseCore::EvaluatePx4Distance(double now_s) {
  while (!odometry_pending_.empty() && !mocap_ring_.empty() &&
         odometry_pending_.Oldest().stamp_ns <= mocap_ring_.Newest().stamp_ns) {
    const OdometryEntry entry = odometry_pending_.Oldest();
    odometry_pending_.PopOldest();
    Eigen::Vector3d p_m = Eigen::Vector3d::Zero();
    if (!InterpolateMocap(entry.stamp_ns, &p_m, nullptr)) {
      continue;  // no valid mocap around this stamp: not evaluated
    }
    const Eigen::Vector3d p_m_ned(p_m.x(), -p_m.y(), -p_m.z());
    const double dist = (entry.p_ned - p_m_ned).norm();
    px4_mocap_dist_ = dist;
    px4_dist_rx_s_ = now_s;
    if (source_ != Source::kVio) {
      px4_above_ = false;
      continue;
    }
    const bool above = dist > params_.fallback_dist_m;
    if (above) {
      // the hold starts at the crossing, and again after a long stretch
      // without comparison
      if (!px4_above_) {
        ++dist_exceed_count_;
        px4_above_since_ns_ = entry.stamp_ns;
      } else if (entry.stamp_ns - px4_eval_stamp_ns_ > kPx4EvalMaxGapNs) {
        px4_above_since_ns_ = entry.stamp_ns;
      }
    }
    px4_above_ = above;
    px4_eval_stamp_ns_ = entry.stamp_ns;
    if (above && entry.stamp_ns - px4_above_since_ns_ >= hold_ns_) {
      char detail[128];
      std::snprintf(
          detail, sizeof(detail),
          "distance limit: PX4 %.3f m from the mocap, above %.2f m "
          "for %.2f s",
          dist, params_.fallback_dist_m,
          static_cast<double>(entry.stamp_ns - px4_above_since_ns_) * 1e-9);
      Fallback(Reason::kDistanceLimit, detail, now_s);
    }
  }
}

void VisionPoseCore::Fallback(Reason reason, const std::string &detail,
                              double now_s) {
  fallback_latched_ = true;
  fallback_reason_ = reason;
  ++fallbacks_;
  ++reset_counter_;
  px4_above_ = false;
  // the body rotation PX4 had on the VIO stays in the mocap feed
  have_feed_c_ = placed_ && placement_from_mocap_;
  q_c_feed_ = q_c_;
  // without a valid mocap PX4 gets nothing until one arrives
  SetSource(MocapValid(now_s) ? Source::kMocap : Source::kNone, detail);
}

void VisionPoseCore::SetSource(Source source, const std::string &reason) {
  Log("pose source: %s -> %s (%s)", SourceName(source_), SourceName(source),
      reason.c_str());
  source_ = source;
  source_changed_ = true;
}

void VisionPoseCore::FillMocap(const MocapSample &sample,
                               Px4Sample *px4) const {
  px4->timestamp_sample = StampToPx4Us(sample.stamp_ns);
  // negative values for FLU -> FRD
  px4->position[0] = sample.px;
  px4->position[1] = -sample.py;
  px4->position[2] = -sample.pz;
  if (fallback_latched_ && params_.correct_mocap_attitude && have_feed_c_) {
    // after a fallback PX4 keeps the attitude of the IMU body it had on the
    // VIO: R_m C^T instead of the rigid body's R_m
    const Eigen::Quaterniond q_i =
        Eigen::Quaterniond(sample.qw, sample.qx, sample.qy, sample.qz) *
        q_c_feed_.conjugate();
    SetPx4Attitude(static_cast<float>(q_i.w()), static_cast<float>(q_i.x()),
                   static_cast<float>(q_i.y()), static_cast<float>(q_i.z()),
                   px4);
  } else {
    SetPx4Attitude(sample.qw, sample.qx, sample.qy, sample.qz, px4);
  }
  px4->position_variance = pos_var_f_;
  px4->orientation_variance = att_var_f_;
  px4->reset_counter = reset_counter_;
}

void VisionPoseCore::FillVio(const PlacedPose &placed, Px4Sample *px4) const {
  px4->timestamp_sample = StampToPx4Us(placed.stamp_ns);
  px4->position[0] = static_cast<float>(placed.p.x());
  px4->position[1] = static_cast<float>(-placed.p.y());
  px4->position[2] = static_cast<float>(-placed.p.z());
  SetPx4Attitude(
      static_cast<float>(placed.q.w()), static_cast<float>(placed.q.x()),
      static_cast<float>(placed.q.y()), static_cast<float>(placed.q.z()), px4);
  px4->position_variance = vio_pos_var_f_;
  px4->orientation_variance = vio_att_var_f_;
  px4->reset_counter = reset_counter_;
}

bool VisionPoseCore::MocapValid(double now_s) const {
  return mocap_seen_ && mocap_last_valid_ &&
         now_s - mocap_rx_s_ <= kMocapValidS;
}

bool VisionPoseCore::ArmingKnown(double now_s) const {
  return have_status_ && now_s - status_rx_s_ <= kStatusValidS;
}

bool VisionPoseCore::Px4DistanceKnown(double now_s) const {
  return std::isfinite(px4_mocap_dist_) &&
         now_s - px4_dist_rx_s_ <= kPx4DistValidS;
}

bool VisionPoseCore::MountOk() const {
  // placed against the mocap the VIO needs the mount yaw of the calibration;
  // placed at the start pose it does not
  const bool from_mocap = placed_
                              ? placement_from_mocap_
                              : (mocap_seen_ || !params_.world_from_start_pose);
  return !from_mocap || params_.has_mount;
}

Reason VisionPoseCore::CurrentReason(double now_s) const {
  if (source_ == Source::kVio) {
    return Reason::kNone;
  }
  if (!params_.vio_enabled || !params_.use_vio) {
    return Reason::kOperator;
  }
  if (fallback_latched_) {
    return fallback_reason_;
  }
  if (!ArmingKnown(now_s)) {
    return Reason::kNoVehicleStatus;
  }
  if (!MountOk()) {
    return Reason::kNoMount;
  }
  if (vio_ready_ && placement_from_mocap_ && !Px4DistanceKnown(now_s)) {
    return Reason::kNoVehicleOdometry;
  }
  return armed_ ? Reason::kVioNotReadyBeforeArming : Reason::kVioNotReady;
}

void VisionPoseCore::Log(const char *format, ...) {
  char line[384];
  va_list args;
  va_start(args, format);
  std::vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  // lines nobody reads are not kept for ever
  if (log_.size() >= 64) {
    log_.pop_front();
  }
  log_.emplace_back(line);
}

}  // namespace vision_pose
