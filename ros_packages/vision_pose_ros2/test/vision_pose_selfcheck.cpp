// Offline check of VisionPoseCore on synthetic data: no ROS graph, no
// recording. A rigid body moves in the room; the mocap sees it at 240 Hz, the
// estimator sees its IMU body at 20 Hz in a world rotated and shifted against
// the room, PX4 reports a position at 100 Hz and the arming state at 2 Hz.
// Every message reaches the logic with a delay, in arrival order.

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "vision_pose_core.hpp"

using vision_pose::Params;
using vision_pose::PlacedPose;
using vision_pose::Px4Sample;
using vision_pose::Reason;
using vision_pose::Source;
using vision_pose::Status;
using vision_pose::VisionPoseCore;

namespace {

const double kInf = std::numeric_limits<double>::infinity();
const double kDeg = M_PI / 180.0;
const int64_t kEpochNs =
    1780600000LL * 1000000000LL;  // stamps like the fleet's
const double kSteady0 = 5000.0;   // steady clock at t = 0
const double kMocapDelay = 0.007, kVioDelay = 0.058, kOdometryDelay = 0.005;

int g_checks = 0;
int g_failed = 0;

void Check(bool ok, const char *what, const std::string &detail = "") {
  ++g_checks;
  if (!ok) {
    ++g_failed;
  }
  std::printf("  [%s] %s%s%s\n", ok ? " ok " : "FAIL", what,
              detail.empty() ? "" : " : ", detail.c_str());
}

std::string Fmt(const char *format, ...) __attribute__((format(printf, 1, 2)));
std::string Fmt(const char *format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return buffer;
}

Eigen::Quaterniond Rz(double a) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()));
}
Eigen::Quaterniond Rx(double a) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()));
}
Eigen::Quaterniond Ry(double a) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(a, Eigen::Vector3d::UnitY()));
}
double Yaw(const Eigen::Quaterniond &q) {
  const Eigen::Matrix3d r = q.toRotationMatrix();
  return std::atan2(r(1, 0), r(0, 0));
}
double Wrap(double a) { return std::remainder(a, 2.0 * M_PI); }
int64_t Stamp(double t) {
  return kEpochNs + static_cast<int64_t>(std::llround(t * 1e9));
}
const Eigen::Vector3d kStart(1.2, -0.8, 0.05);
const double kStartYaw = 40.0 * kDeg;

// pose of the rigid body in the room
struct Truth {
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
};

// on the ground (tilted by `slope`) until `takeoff`, then a figure of 3 m
// with attitude changes until `land`, then level and still where it is
Truth Flight(double t, double takeoff, double land, double slope = 0.0) {
  const double s = t < takeoff ? 0.0 : std::min(t, land) - takeoff;
  Truth truth;
  truth.p = kStart + Eigen::Vector3d(3.0 * (1.0 - std::cos(0.7 * s)),
                                     1.5 * std::sin(1.4 * s),
                                     1.0 - std::cos(0.9 * s));
  const Eigen::Quaterniond yaw = Rz(kStartYaw + 0.5 * std::sin(0.4 * s));
  if (t < takeoff) {
    truth.q = yaw * Ry(slope);
  } else if (t >= land) {
    truth.q = yaw;
  } else {
    truth.q = yaw * Ry(0.25 * std::sin(1.1 * s)) * Rx(0.2 * std::sin(1.7 * s));
  }
  return truth;
}

struct Scenario {
  Params params;
  double duration = 20.0;
  double takeoff = kInf;  // never: stands still
  double land = kInf;
  double slope = 0.0;                     // tilt of the ground it starts from
  double psi = 0.6;                       // VIO world -> room heading
  Eigen::Vector3d shift{4.0, -7.0, 0.3};  // p_room = Rz(psi) p_vio + shift
  Eigen::Quaterniond c = Eigen::Quaterniond::Identity();  // rigid body -> IMU
  double status_from = 0.0;  // first vehicle status (kInf: none)
  double status_gap_from = kInf, status_gap_until = kInf;  // none in between
  double arm = kInf;
  double disarm = kInf;
  double mocap_from = 0.0, mocap_until = kInf;  // mocap present in between
  double mocap_delay = kMocapDelay;             // stamp to arrival
  double mocap_gap_from = kInf, mocap_gap_until = kInf;
  // from mocap_holes_from on, the first mocap_hole_s of every
  // mocap_hole_period_s are missing
  double mocap_holes_from = kInf, mocap_hole_period_s = 0.0, mocap_hole_s = 0.0;
  double mocap_bad_at = kInf;        // one sample 1 m off
  // in between its samples are marked valid with a quaternion of zeros
  double mocap_zero_q_from = kInf, mocap_zero_q_until = kInf;
  double mocap_bad_stamp_at = kInf;  // one sample stamped 3 s ahead
  double vio_until = kInf;
  double vio_gap_from = kInf, vio_gap_until = kInf;  // no pose in between
  // from here on the estimator is another run: stamps shifted (8 s back
  // unless said otherwise), its world 5 m away
  double vio_restart_at = kInf;
  double vio_restart_stamp_s = -8.0;
  double vio_back_once_at = kInf;  // one extra pose stamped 10 s back
  // from here on the stamps of the mocap and of PX4 are 8 s back as well (a
  // recording played again)
  double stamps_back_from = kInf;
  // PX4's own position: rate, presence, one sample stamped 3 s ahead
  double odometry_hz = 100.0;
  double odometry_from = 0.0;
  double odometry_gap_from = kInf, odometry_gap_until = kInf;
  double odometry_bad_stamp_at = kInf;
  // error of the VIO attitude (applied in the body frame)
  std::function<Eigen::Quaterniond(double)> vio_attitude_error = [](double) {
    return Eigen::Quaterniond::Identity();
  };
  // error of PX4's position and of the VIO position (room frame, metres)
  std::function<Eigen::Vector3d(double)> px4_error = [](double) {
    return Eigen::Vector3d(0.0, 0.0, 0.0);
  };
  std::function<Eigen::Vector3d(double)> vio_error = [](double) {
    return Eigen::Vector3d(0.0, 0.0, 0.0);
  };
};

struct Sent {
  double t = 0.0;  // arrival time of the input that produced it
  bool from_vio = false;
  Px4Sample sample;
  Source source = Source::kNone;
};

struct Record {
  std::vector<Sent> sent;
  std::vector<std::pair<double, PlacedPose>> placed;  // (stamp time, pose)
  std::vector<std::pair<double, std::string>> log;
  std::vector<std::pair<double, Status>> status;  // every 0.1 s
  Status last;

  double LogTime(const std::string &text) const {
    for (const auto &line : log) {
      if (line.second.find(text) != std::string::npos) {
        return line.first;
      }
    }
    return kInf;
  }
  std::string LogLine(const std::string &text) const {
    for (const auto &line : log) {
      if (line.second.find(text) != std::string::npos) {
        return line.second;
      }
    }
    return "(no such line)";
  }
  int LogCount(const std::string &text) const {
    int n = 0;
    for (const auto &line : log) {
      n += line.second.find(text) != std::string::npos ? 1 : 0;
    }
    return n;
  }
  const Status &StatusAt(double t) const {
    size_t i = 0;
    while (i + 1 < status.size() && status[i + 1].first <= t) {
      ++i;
    }
    return status[i].second;
  }
};

Record Run(const Scenario &sc) {
  struct Event {
    double arrival;
    int kind;  // 0 mocap, 1 vio, 2 odometry, 3 status, 4 timer
    double t;  // stamp
    long index;
  };
  std::vector<Event> events;
  for (long k = 0; k / 240.0 < sc.duration; ++k) {
    const double t = k / 240.0;
    const bool gap = (t >= sc.mocap_gap_from && t < sc.mocap_gap_until) ||
                     (t >= sc.mocap_holes_from &&
                      std::fmod(t - sc.mocap_holes_from,
                                sc.mocap_hole_period_s) < sc.mocap_hole_s);
    if (t >= sc.mocap_from && t < sc.mocap_until && !gap) {
      events.push_back({t + sc.mocap_delay, 0, t, k});
    }
  }
  for (long k = 0; 0.0013 + k / 20.0 < std::min(sc.duration, sc.vio_until);
       ++k) {
    const double t = 0.0013 + k / 20.0;
    if (!(t >= sc.vio_gap_from && t < sc.vio_gap_until)) {
      events.push_back({t + kVioDelay, 1, t, k});
    }
  }
  if (std::isfinite(sc.vio_back_once_at)) {
    events.push_back(
        {sc.vio_back_once_at + kVioDelay, 5, sc.vio_back_once_at, 0});
  }
  for (long k = 0; 0.0004 + k / sc.odometry_hz < sc.duration; ++k) {
    const double t = 0.0004 + k / sc.odometry_hz;
    const bool gap = t >= sc.odometry_gap_from && t < sc.odometry_gap_until;
    if (t >= sc.odometry_from && !gap) {
      events.push_back({t + kOdometryDelay, 2, t, k});
    }
  }
  for (long k = 0; sc.status_from + k / 2.0 < sc.duration; ++k) {
    const double t = sc.status_from + k / 2.0;
    if (!(t >= sc.status_gap_from && t < sc.status_gap_until)) {
      events.push_back({t, 3, t, k});
    }
  }
  for (long k = 0; k / 100.0 < sc.duration; ++k) {
    events.push_back({k / 100.0 + 0.0001, 4, k / 100.0, k});
  }
  std::stable_sort(
      events.begin(), events.end(),
      [](const Event &a, const Event &b) { return a.arrival < b.arrival; });

  VisionPoseCore core(sc.params);
  Record record;
  long bad_index = std::isfinite(sc.mocap_bad_at)
                       ? std::lround(sc.mocap_bad_at * 240.0)
                       : -1;
  const long bad_stamp_index = std::isfinite(sc.mocap_bad_stamp_at)
                                   ? std::lround(sc.mocap_bad_stamp_at * 240.0)
                                   : -1;
  const long bad_odometry_index =
      std::isfinite(sc.odometry_bad_stamp_at)
          ? std::lround(sc.odometry_bad_stamp_at * sc.odometry_hz)
          : -1;
  for (const Event &event : events) {
    const double now = kSteady0 + event.arrival;
    const Truth truth = Flight(event.t, sc.takeoff, sc.land, sc.slope);
    if (event.kind == 0) {
      vision_pose::MocapSample sample;
      sample.stamp_ns =
          Stamp(event.t + (event.index == bad_stamp_index ? 3.0 : 0.0) -
                (event.t >= sc.stamps_back_from ? 8.0 : 0.0));
      sample.tracking_valid = true;
      Eigen::Vector3d p = truth.p;
      if (event.index == bad_index) {
        p.x() += 1.0;
      }
      sample.px = static_cast<float>(p.x());
      sample.py = static_cast<float>(p.y());
      sample.pz = static_cast<float>(p.z());
      sample.qx = static_cast<float>(truth.q.x());
      sample.qy = static_cast<float>(truth.q.y());
      sample.qz = static_cast<float>(truth.q.z());
      sample.qw = static_cast<float>(truth.q.w());
      if (event.t >= sc.mocap_zero_q_from && event.t < sc.mocap_zero_q_until) {
        sample.qx = sample.qy = sample.qz = sample.qw = 0.0f;
      }
      Px4Sample px4;
      if (core.HandleMocap(sample, now, &px4)) {
        record.sent.push_back(
            {event.arrival, false, px4, core.GetStatus(now).source});
      }
    } else if (event.kind == 1 || event.kind == 5) {
      // IMU body in the VIO world: R_v = Rz(psi)^T R_m C^T
      const bool restarted = event.t >= sc.vio_restart_at;
      vision_pose::VioPose pose;
      pose.stamp_ns = Stamp(event.t - (event.kind == 5 ? 10.0 : 0.0) +
                            (restarted ? sc.vio_restart_stamp_s : 0.0));
      pose.p = Rz(-sc.psi) * (truth.p + sc.vio_error(event.t) - sc.shift -
                              (restarted ? Eigen::Vector3d(5.0, 0.0, 0.0)
                                         : Eigen::Vector3d(0.0, 0.0, 0.0)));
      pose.q = Rz(-sc.psi) * truth.q * sc.c.conjugate() *
               sc.vio_attitude_error(event.t);
      Px4Sample px4;
      PlacedPose placed;
      const auto result = core.HandleVio(pose, now, kVioDelay, &px4, &placed);
      if (result.px4) {
        record.sent.push_back(
            {event.arrival, true, px4, core.GetStatus(now).source});
      }
      if (result.placed) {
        record.placed.push_back({event.t, placed});
      }
    } else if (event.kind == 2) {
      const Eigen::Vector3d p = truth.p + sc.px4_error(event.t);
      core.HandleVehicleOdometry(
          Stamp(event.t + (event.index == bad_odometry_index ? 3.0 : 0.0) -
                (event.t >= sc.stamps_back_from ? 8.0 : 0.0)),
          Eigen::Vector3d(p.x(), -p.y(), -p.z()), now);
    } else if (event.kind == 3) {
      core.HandleVehicleStatus(event.t >= sc.arm && event.t < sc.disarm, now);
    } else {
      core.HandleTimer(now);
      if (event.index % 10 == 0) {
        record.status.push_back({event.arrival, core.GetStatus(now)});
      }
    }
    std::string line;
    while (core.PopLogLine(&line)) {
      record.log.push_back({event.arrival, line});
    }
    record.last = core.GetStatus(now);
  }
  return record;
}

// largest distance of the placed VIO from the true position, and largest
// angle from the true attitude of the IMU body R_m C^T
void PlacedError(const Scenario &sc, const Record &record, double from,
                 double *position, double *angle) {
  *position = 0.0;
  *angle = 0.0;
  for (const auto &entry : record.placed) {
    if (entry.first < from) {
      continue;
    }
    const Truth truth = Flight(entry.first, sc.takeoff, sc.land, sc.slope);
    *position = std::max(*position, (entry.second.p - truth.p).norm());
    const Eigen::Quaterniond q_i = truth.q * sc.c.conjugate();
    *angle = std::max(*angle, q_i.angularDistance(entry.second.q));
  }
}

Params VioNav() {
  Params params;
  params.vio_enabled = true;
  params.use_vio = true;
  params.has_mount = true;
  return params;
}

int CountSent(const Record &record, bool from_vio, double from, double until) {
  int n = 0;
  for (const Sent &sent : record.sent) {
    n +=
        (sent.from_vio == from_vio && sent.t >= from && sent.t < until) ? 1 : 0;
  }
  return n;
}

// VIO messages sent to PX4 in [from, until) whose position is further than
// `limit` from the true one
int CountBadVio(const Scenario &sc, const Record &record, double from,
                double until, double limit) {
  int n = 0;
  for (const Sent &sent : record.sent) {
    if (!sent.from_vio || sent.t < from || sent.t >= until) {
      continue;
    }
    const Truth truth =
        Flight(sent.t - kVioDelay, sc.takeoff, sc.land, sc.slope);
    const Eigen::Vector3d p(sent.sample.position[0], -sent.sample.position[1],
                            -sent.sample.position[2]);
    n += (p - truth.p).norm() > limit ? 1 : 0;
  }
  return n;
}

/*------------------------------ checks -------------------------------*/

void CheckMocapConversion() {
  std::printf("mocap path: the conversion of the old node\n");
  Params params;
  VisionPoseCore core(params);
  vision_pose::MocapSample sample;
  sample.tracking_valid = true;
  sample.px = 1.25f;
  sample.py = -2.5f;
  sample.pz = 0.75f;
  sample.qx = 0.1f;
  sample.qy = -0.2f;
  sample.qz = 0.3f;
  sample.qw = 0.9f;
  // timestamp_sample as px4_ros2 computes it, against integer division
  int differ = 0;
  bool same = true;
  for (int k = 0; k < 2400; ++k) {
    const int64_t ns = kEpochNs + 324794450123LL + k * 4166667LL;
    const uint64_t reference = ns * 1e-3;  // the expression of px4_ros2
    same = same && vision_pose::StampToPx4Us(ns) == reference;
    differ += reference != static_cast<uint64_t>(ns / 1000) ? 1 : 0;
  }
  Check(same, "timestamp_sample equals nanoseconds * 1e-3 truncated",
        Fmt("differs from integer division on %d of 2400 stamps", differ));

  sample.stamp_ns = kEpochNs + 123456789;
  Px4Sample px4;
  const bool sent = core.HandleMocap(sample, kSteady0, &px4);
  const Eigen::Quaternionf q =
      Eigen::Quaternionf(-sample.qw, -sample.qx, sample.qy, sample.qz)
          .normalized();
  Check(sent && core.GetStatus(kSteady0).source == Source::kMocap,
        "first valid sample: source MOCAP and one message");
  Check(px4.position[0] == sample.px && px4.position[1] == -sample.py &&
            px4.position[2] == -sample.pz,
        "position = (x, -y, -z), exact");
  Check(px4.q[0] == q.w() && px4.q[1] == q.x() && px4.q[2] == q.y() &&
            px4.q[3] == q.z(),
        "q = normalised (-qw, -qx, qy, qz), bit for bit");
  Check(px4.position_variance == static_cast<float>(1e-6) &&
            px4.orientation_variance == static_cast<float>(1e-6) &&
            px4.reset_counter == 0,
        "variances float32(1e-6), reset_counter 0");

  // many samples: position and attitude bit for bit, one message each
  uint32_t seed = 12345u;
  auto next = [&seed]() {
    seed = seed * 1664525u + 1013904223u;
    return static_cast<float>(seed >> 8) / 8388608.0f - 1.0f;  // [-1, 1)
  };
  int mismatches = 0;
  const int many = 100000;
  for (int k = 0; k < many; ++k) {
    vision_pose::MocapSample random = sample;
    random.stamp_ns = sample.stamp_ns + (k + 1) * 4166667LL;
    random.px = 5.0f * next();
    random.py = 5.0f * next();
    random.pz = 2.0f * next();
    // as the mocap sends them: unit to float32 rounding
    const Eigen::Quaternionf unit =
        Eigen::Quaternionf(next(), next(), next(), next()).normalized();
    random.qw = unit.w();
    random.qx = unit.x();
    random.qy = unit.y();
    random.qz = unit.z();
    Px4Sample out;
    const bool one = core.HandleMocap(random, kSteady0, &out);
    const Eigen::Quaternionf expected =
        Eigen::Quaternionf(-random.qw, -random.qx, random.qy, random.qz)
            .normalized();
    const bool equal =
        one &&
        out.timestamp_sample == vision_pose::StampToPx4Us(random.stamp_ns) &&
        out.position[0] == random.px && out.position[1] == -random.py &&
        out.position[2] == -random.pz && out.q[0] == expected.w() &&
        out.q[1] == expected.x() && out.q[2] == expected.y() &&
        out.q[3] == expected.z();
    mismatches += equal ? 0 : 1;
  }
  Check(mismatches == 0, "100000 random samples: one message each, bit for bit",
        Fmt("%d mismatches", mismatches));

  // samples that must give nothing, and must not stop the node
  const float nan = std::numeric_limits<float>::quiet_NaN();
  int wrongly_sent = 0;
  vision_pose::MocapSample bad = sample;
  bad.stamp_ns += 1000000;
  bad.tracking_valid = false;
  wrongly_sent += core.HandleMocap(bad, kSteady0, &px4) ? 1 : 0;
  bad = sample;
  bad.stamp_ns = 0;
  wrongly_sent += core.HandleMocap(bad, kSteady0, &px4) ? 1 : 0;
  bad.stamp_ns = -5;
  wrongly_sent += core.HandleMocap(bad, kSteady0, &px4) ? 1 : 0;
  bad = sample;
  bad.stamp_ns += 2000000;
  bad.py = nan;
  wrongly_sent += core.HandleMocap(bad, kSteady0, &px4) ? 1 : 0;
  bad.py = 0.0f;
  bad.qw = std::numeric_limits<float>::infinity();
  wrongly_sent += core.HandleMocap(bad, kSteady0, &px4) ? 1 : 0;
  Check(wrongly_sent == 0,
        "invalid, zero or negative stamp, NaN, infinity: no message");
  Check(!core.GetStatus(kSteady0).mocap_valid,
        "mocap_valid false after an invalid sample");
  // an older stamp is still forwarded (the old node did)
  bad = sample;
  bad.stamp_ns -= 1000;
  Check(core.HandleMocap(bad, kSteady0, &px4),
        "valid sample out of order: "
        "forwarded");
  // so is a quaternion of zeros marked valid by the mocap (the old node did;
  // PX4 leaves the attitude of such a message out)
  bad = sample;
  bad.stamp_ns += 3000000;
  bad.qx = bad.qy = bad.qz = bad.qw = 0.0f;
  const bool zero_sent = core.HandleMocap(bad, kSteady0, &px4);
  Check(zero_sent && px4.q[0] == 0.0f && px4.q[1] == 0.0f && px4.q[2] == 0.0f &&
            px4.q[3] == 0.0f && px4.position[0] == bad.px,
        "valid sample with a quaternion of zeros: forwarded with q = 0, as "
        "the old node did");
}

void CheckPlacementExact() {
  std::printf("placement: VIO world rotated and shifted, mount = pure yaw\n");
  Scenario sc;
  sc.params.vio_enabled = true;  // shadow: place and log only
  sc.params.has_mount = true;
  sc.params.mount_yaw = 2.72 * kDeg;
  sc.c = Rz(-2.72 * kDeg);  // psi0 = yaw(R_m R_v^T) + mount yaw
  sc.psi = 2.1;
  sc.arm = 9.5;
  sc.takeoff = 10.5;
  sc.duration = 26.0;
  const Record record = Run(sc);
  if (record.placed.empty()) {
    Check(false, "the VIO is placed");
    return;
  }
  double position, angle;
  PlacedError(sc, record, 0.0, &position, &angle);
  Check(record.placed.front().first > 7.0 && record.placed.front().first < 7.1,
        "first placed pose after 7 s of poses",
        Fmt("stamp %.3f s, %zu placed poses", record.placed.front().first,
            record.placed.size()));
  Check(position < 1e-6, "placed VIO on the true position within 1e-6 m",
        Fmt("max %.3g m over standstill and flight", position));
  Check(angle < 1e-6, "placed attitude = R_m C^T within 1e-6 rad",
        Fmt("max %.3g rad", angle));
  Check(std::abs(Wrap(record.last.heading_deg * kDeg - sc.psi)) < 1e-5,
        "heading in use = the true one",
        Fmt("%.4f deg vs %.4f deg", record.last.heading_deg,
            Wrap(sc.psi) / kDeg));
  Check(record.last.body_tilt_deg < 1e-3, "body tilt 0 for a pure yaw mount",
        Fmt("%.2g deg", record.last.body_tilt_deg));
  Check(record.last.source == Source::kMocap &&
            record.last.reason == Reason::kOperator &&
            CountSent(record, true, 0.0, kInf) == 0,
        "use_vio false: PX4 on the mocap throughout, reason operator");
  Check(std::abs(record.last.heading_fit_deg) < 1e-3 &&
            record.last.heading_trim_deg == 0.0f,
        "heading fit 0 in flight, trim off",
        Fmt("fit %.2g deg", record.last.heading_fit_deg));
  Check(record.LogCount("VIO ready:") == 1 &&
            record.LogCount("armed: placement frozen") == 1,
        "one 'VIO ready' line, one freeze at arming");
}

void CheckTilt() {
  std::printf("placement: rigid body tilted against the IMU body\n");
  Scenario sc;
  sc.params.vio_enabled = true;
  sc.params.has_mount = true;
  sc.params.mount_yaw = 2.72 * kDeg;
  sc.c = Rz(-2.72 * kDeg) *
         Eigen::Quaterniond(Eigen::AngleAxisd(
             2.5 * kDeg, Eigen::Vector3d(0.6, -0.8, 0.0)));  // pure tilt
  sc.arm = 9.5;
  sc.takeoff = 10.5;
  sc.duration = 26.0;
  const Record record = Run(sc);
  double position, angle;
  PlacedError(sc, record, 0.0, &position, &angle);
  const double tilt_true =
      std::acos((sc.c * Eigen::Vector3d::UnitZ()).z()) / kDeg;
  Check(std::abs(record.last.body_tilt_deg - tilt_true) < 0.01,
        "body tilt identified",
        Fmt("%.4f deg, true %.4f deg", record.last.body_tilt_deg, tilt_true));
  // the heading read from the two attitudes is exact only without tilt
  Check(std::abs(Wrap(record.last.heading_deg * kDeg - sc.psi)) / kDeg < 0.05,
        "heading within 0.05 deg with a tilt of 2.5 deg",
        Fmt("error %.4f deg",
            Wrap(record.last.heading_deg * kDeg - sc.psi) / kDeg));
  Check(position < 0.005 && angle < 0.001,
        "placed VIO within 5 mm and 0.06 deg of the truth",
        Fmt("max %.2f mm, %.4f deg", 1e3 * position, angle / kDeg));

  // not level at the start: the heading read from the attitudes is off by
  // about (tilt between the bodies) x sin(slope)
  sc.slope = 5.0 * kDeg;
  const Record sloped = Run(sc);
  PlacedError(sc, sloped, 0.0, &position, &angle);
  Check(std::abs(Wrap(sloped.last.heading_deg * kDeg - sc.psi)) / kDeg < 0.3,
        "standing on a slope of 5 deg: heading within 0.3 deg",
        Fmt("error %.4f deg, placed VIO up to %.1f mm off",
            Wrap(sloped.last.heading_deg * kDeg - sc.psi) / kDeg,
            1e3 * position));
}

void CheckVioNav() {
  std::printf("state machine: VIO from the ground\n");
  Scenario sc;
  sc.params = VioNav();
  sc.params.mount_yaw = 2.72 * kDeg;
  sc.c = Rz(-2.72 * kDeg);
  sc.arm = 9.5;
  sc.takeoff = 10.5;
  sc.duration = 24.0;
  const Record record = Run(sc);
  const double t_mocap = record.LogTime("pose source: NONE -> MOCAP");
  const double t_vio = record.LogTime("pose source: MOCAP -> VIO (vio ready)");
  Check(t_mocap < 0.01, "NONE -> MOCAP at the first valid sample",
        Fmt("at %.3f s", t_mocap));
  Check(t_vio > 7.0 && t_vio < 7.2, "MOCAP -> VIO once 7 s of poses stand",
        Fmt("at %.3f s", t_vio));
  Check(record.LogTime("VIO ready:") <= t_vio, "'VIO ready' line before it");
  Check(CountSent(record, false, t_vio, kInf) == 0 &&
            CountSent(record, true, 0.0, t_vio) == 0,
        "on the VIO no mocap sample is forwarded, before it no VIO pose");
  Check(CountSent(record, true, t_vio, kInf) ==
            static_cast<int>(record.placed.size()),
        "one PX4 message per VIO pose",
        Fmt("%d", CountSent(record, true, t_vio, kInf)));
  Check(record.last.source == Source::kVio && record.last.reset_counter == 1 &&
            record.last.reason == Reason::kNone && record.last.fallbacks == 0,
        "still on the VIO at the end of the flight, reset_counter 1");
  // content of the VIO messages against the truth
  double position = 0.0, angle = 0.0;
  bool stamps = true, counters = true, sign = true;
  for (size_t i = 0, k = 0; i < record.sent.size(); ++i) {
    const Sent &sent = record.sent[i];
    if (!sent.from_vio) {
      counters = counters && sent.sample.reset_counter == 0;
      continue;
    }
    const double t = record.placed[k].first;
    const PlacedPose &placed = record.placed[k++].second;
    const Truth truth = Flight(t, sc.takeoff, sc.land);
    const Eigen::Vector3d ned(sent.sample.position[0], sent.sample.position[1],
                              sent.sample.position[2]);
    position = std::max(
        position,
        (ned - Eigen::Vector3d(truth.p.x(), -truth.p.y(), -truth.p.z()))
            .norm());
    // quat(D R D) of the IMU body attitude R = R_m C^T
    const Eigen::Matrix3d d = Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();
    const Eigen::Quaterniond expected(
        d * (truth.q * sc.c.conjugate()).toRotationMatrix() * d);
    const Eigen::Quaterniond got(sent.sample.q[0], sent.sample.q[1],
                                 sent.sample.q[2], sent.sample.q[3]);
    angle = std::max(angle, expected.angularDistance(got));
    stamps = stamps && sent.sample.timestamp_sample ==
                           vision_pose::StampToPx4Us(Stamp(t));
    counters = counters && sent.sample.reset_counter == 1;
    sign = sign && sent.sample.q[0] * static_cast<float>(placed.q.w()) <= 0.0f;
  }
  Check(position < 2e-6, "VIO message position = D p within float32",
        Fmt("max %.3g m", position));
  Check(angle < 2e-6, "VIO message attitude = quat(D R D)",
        Fmt("max %.3g rad", angle));
  Check(stamps, "timestamp_sample = VIO stamp in microseconds");
  Check(counters, "reset_counter 0 on the mocap, 1 on the VIO");
  Check(sign, "quaternion sign as on the mocap path (w negated)");
}

void CheckArmedBeforeReady() {
  std::printf("state machine: armed before the VIO is ready\n");
  Scenario sc;
  sc.params = VioNav();
  sc.arm = 3.0;
  sc.takeoff = 4.0;
  sc.land = 14.0;
  sc.disarm = 15.0;
  sc.duration = 26.0;
  const Record record = Run(sc);
  const double t_vio = record.LogTime("pose source: MOCAP -> VIO");
  Check(record.StatusAt(10.0).source == Source::kMocap &&
            record.StatusAt(10.0).reason == Reason::kVioNotReadyBeforeArming &&
            !record.StatusAt(10.0).vio_ready && !record.placed.empty() &&
            record.placed.front().first > 15.0,
        "in flight: MOCAP, not placed, reason 'vio not ready before arming'");
  Check(record.LogCount("armed: PX4 stays on MOCAP for this flight") == 1,
        "one line says so at arming");
  Check(t_vio > 21.0 && t_vio < 21.2,
        "MOCAP -> VIO on the ground, 7 s after it landed",
        Fmt("at %.3f s", t_vio));
  double position, angle;
  PlacedError(sc, record, 0.0, &position, &angle);
  Check(position < 1e-6, "placed where it landed", Fmt("max %.3g m", position));
}

void CheckDistanceFallback() {
  std::printf("fallback: PX4 further than fallback_dist_m from the mocap\n");
  Scenario sc;
  sc.params = VioNav();
  sc.params.mount_yaw = 2.72 * kDeg;
  sc.params.correct_mocap_attitude = true;
  sc.c = Rz(-2.72 * kDeg) * Ry(1.5 * kDeg);
  sc.arm = 9.5;
  sc.takeoff = 10.5;
  sc.land = 22.0;
  sc.disarm = 23.0;
  sc.duration = 34.0;
  sc.mocap_bad_at = 12.0;
  sc.px4_error = [](double t) {
    const bool blip = t >= 13.0 && t < 13.05;
    return Eigen::Vector3d(blip || t >= 15.0 ? 0.2 : 0.0, 0.0, 0.0);
  };
  const Record record = Run(sc);
  const double t_back = record.LogTime("pose source: VIO -> MOCAP");
  Check(record.StatusAt(14.9).source == Source::kVio &&
            record.StatusAt(14.9).dist_exceed_count == 2,
        "one mocap sample 1 m off and 50 ms at 20 cm: two crossings, no "
        "fallback");
  Check(t_back > 15.10 && t_back < 15.13,
        "fallback 0.10 s after PX4 went 20 cm off",
        Fmt("at %.3f s: %s", t_back,
            record.LogLine("pose source: VIO -> MOCAP").c_str()));
  Check(record.LogCount("pose source: VIO -> MOCAP (distance limit") == 1 &&
            record.last.reason == Reason::kDistanceLimit &&
            record.last.fallback_latched && record.last.fallbacks == 1 &&
            record.last.reset_counter == 2,
        "reason distance limit, latched, reset_counter 2");
  Check(CountSent(record, true, t_back, kInf) == 0 &&
            CountSent(record, false, t_back, kInf) > 4000,
        "the mocap is forwarded from then on, the VIO never again",
        Fmt("%d mocap messages", CountSent(record, false, t_back, kInf)));
  Check(record.last.source == Source::kMocap && record.last.vio_ready,
        "back on the ground and ready again: stays on the mocap");
  // attitude of the mocap messages after the fallback: quat(D R_m C^T D)
  double angle = 0.0, uncorrected = 0.0;
  for (const Sent &sent : record.sent) {
    if (sent.from_vio || sent.t < t_back) {
      continue;
    }
    const Truth truth = Flight(sent.t - kMocapDelay, sc.takeoff, sc.land);
    const Eigen::Matrix3d d = Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();
    const Eigen::Quaterniond got(sent.sample.q[0], sent.sample.q[1],
                                 sent.sample.q[2], sent.sample.q[3]);
    angle = std::max(
        angle, Eigen::Quaterniond(
                   d * (truth.q * sc.c.conjugate()).toRotationMatrix() * d)
                   .angularDistance(got));
    uncorrected = std::max(
        uncorrected, Eigen::Quaterniond(d * truth.q.toRotationMatrix() * d)
                         .angularDistance(got));
  }
  Check(angle < 5e-4, "after it the attitude sent is the IMU body's, R_m C^T",
        Fmt("max %.4f deg from it, %.2f deg from the rigid body's",
            angle / kDeg, uncorrected / kDeg));
}

void CheckVioTimeout() {
  std::printf("fallback: the VIO stops\n");
  Scenario sc;
  sc.params = VioNav();
  sc.arm = 9.5;
  sc.takeoff = 10.5;
  sc.vio_until = 14.0;
  sc.duration = 18.0;
  Record record = Run(sc);
  if (record.placed.empty()) {
    Check(false, "the VIO is placed");
    return;
  }
  const double last_rx = record.placed.back().first + kVioDelay;
  double t_back = record.LogTime("pose source: VIO -> MOCAP (vio timeout");
  Check(t_back > last_rx + 0.33 && t_back < last_rx + 0.35,
        "VIO -> MOCAP 0.33 s after the last pose arrived",
        Fmt("%.3f s after it", t_back - last_rx));
  Check(record.last.reason == Reason::kVioTimeout &&
            record.last.fallback_latched && record.last.reset_counter == 2 &&
            !record.last.vio_alive,
        "reason vio timeout, latched, reset_counter 2");
  const Sent *first = nullptr;
  for (const Sent &sent : record.sent) {
    if (!sent.from_vio && sent.t > t_back && first == nullptr) {
      first = &sent;
    }
  }
  Check(first != nullptr && first->t - t_back < 0.005 &&
            first->sample.reset_counter == 2,
        "next mocap sample forwarded within 5 ms, with the new counter");

  // the same without a mocap: nothing is sent until one is back
  sc.mocap_gap_from = 13.5;
  sc.mocap_gap_until = 16.0;
  record = Run(sc);
  t_back = record.LogTime("pose source: VIO -> NONE (vio timeout");
  const double t_mocap =
      record.LogTime("pose source: NONE -> MOCAP (mocap valid again)");
  Check(t_back > last_rx + 0.33 && t_back < last_rx + 0.35 &&
            record.StatusAt(15.5).source == Source::kNone &&
            !record.StatusAt(15.5).mocap_valid &&
            CountSent(record, false, t_back, t_mocap) == 0,
        "without a valid mocap: VIO -> NONE, nothing sent");
  Check(
      t_mocap > 16.0 && t_mocap < 16.02 && record.last.source == Source::kMocap,
      "NONE -> MOCAP when the mocap is back", Fmt("at %.3f s", t_mocap));
}

void CheckNeverVio() {
  std::printf("state machine: cases in which PX4 must not get the VIO\n");
  Scenario sc;
  sc.params = VioNav();
  sc.params.has_mount = false;
  sc.duration = 12.0;
  Record record = Run(sc);
  Check(record.last.source == Source::kMocap && record.last.vio_ready &&
            record.last.reason == Reason::kNoMount && !record.placed.empty() &&
            CountSent(record, true, 0.0, kInf) == 0,
        "no mount: placed and logged, PX4 on the mocap, reason no mount");

  sc.params = VioNav();
  sc.status_from = kInf;
  record = Run(sc);
  Check(record.last.source == Source::kMocap && !record.last.vio_ready &&
            record.last.reason == Reason::kNoVehicleStatus &&
            record.placed.empty(),
        "no vehicle status: not placed, PX4 on the mocap, reason says so");

  sc.params = VioNav();
  sc.status_from = 0.0;
  sc.mocap_until = 0.0;
  record = Run(sc);
  Check(record.last.source == Source::kNone && record.sent.empty() &&
            record.placed.empty() && !record.last.vio_ready,
        "no mocap, indoor configuration: source NONE, nothing sent, VIO not "
        "placed");

  // restarted in flight: armed from the first status, hovering still
  sc.params = VioNav();
  sc.mocap_until = kInf;
  sc.arm = -1.0;
  record = Run(sc);
  Check(record.last.source == Source::kMocap && record.placed.empty() &&
            record.last.reason == Reason::kVioNotReadyBeforeArming &&
            record.last.armed,
        "started while armed: stays on the mocap, VIO not placed");
}

void CheckStartPose() {
  std::printf("placement without mocap: world_from_start_pose\n");
  Scenario sc;
  sc.params = VioNav();
  sc.params.has_mount = false;
  sc.params.world_from_start_pose = true;
  sc.params.start_position = Eigen::Vector3d(10.0, 20.0, 0.5);
  sc.params.start_yaw = 30.0 * kDeg;
  sc.mocap_until = 0.0;
  sc.arm = 9.5;
  sc.takeoff = 10.5;
  sc.duration = 20.0;
  const Record record = Run(sc);
  const double t_vio = record.LogTime("pose source: NONE -> VIO (vio ready)");
  Check(t_vio > 7.0 && t_vio < 7.2 && record.last.source == Source::kVio &&
            record.last.reset_counter == 1,
        "NONE -> VIO once it stands", Fmt("at %.3f s", t_vio));
  if (record.placed.empty()) {
    return;
  }
  const PlacedPose &first = record.placed.front().second;
  Check((first.p - sc.params.start_position).norm() < 1e-9 &&
            std::abs(Wrap(Yaw(first.q) - sc.params.start_yaw)) < 1e-9,
        "placed at start_position with start_yaw");
  // the motion that follows is the true one, in the start frame
  const Truth t0 = Flight(0.0, sc.takeoff, sc.land);
  const Eigen::Quaterniond to_start = Rz(sc.params.start_yaw - Yaw(t0.q));
  double position = 0.0;
  for (const auto &entry : record.placed) {
    const Truth truth = Flight(entry.first, sc.takeoff, sc.land);
    position = std::max(
        position, (entry.second.p -
                   (to_start * (truth.p - t0.p) + sc.params.start_position))
                      .norm());
  }
  Check(position < 1e-6, "flight reproduced in the start frame within 1e-6 m",
        Fmt("max %.3g m", position));
  Check(std::isnan(record.last.body_tilt_deg) &&
            std::isnan(record.last.heading_fit_deg),
        "no body tilt and no heading fit without mocap");
}

void CheckHeadingTrim() {
  std::printf("heading trim: placement heading 1 deg off\n");
  Scenario sc;
  sc.params = VioNav();
  sc.params.mount_yaw = 1.0 * kDeg;  // the true mount yaw is 0
  sc.arm = 9.5;
  sc.takeoff = 10.5;
  sc.duration = 34.0;
  Record record = Run(sc);
  double position, angle;
  PlacedError(sc, record, 30.0, &position, &angle);
  Check(std::abs(record.last.heading_fit_deg + 1.0) < 0.01 &&
            record.last.heading_trim_deg == 0.0f && position > 0.02,
        "trim off: the fit reads -1 deg, nothing is corrected",
        Fmt("fit %.4f deg, placed VIO up to %.1f cm off",
            record.last.heading_fit_deg, 100.0 * position));

  sc.params.heading_trim_rate = 0.2 * kDeg;
  record = Run(sc);
  PlacedError(sc, record, 30.0, &position, &angle);
  Check(std::abs(record.last.heading_trim_deg + 1.0) < 0.01 && position < 1e-3,
        "0.2 deg/s: trim reaches -1 deg, placed VIO back on the truth",
        Fmt("trim %.4f deg, max %.2f mm after 30 s",
            record.last.heading_trim_deg, 1e3 * position));
  // rate: change of the trim between consecutive status samples (0.1 s)
  double max_rate = 0.0;
  for (size_t i = 1; i < record.status.size(); ++i) {
    max_rate = std::max(max_rate,
                        std::abs(record.status[i].second.heading_trim_deg -
                                 record.status[i - 1].second.heading_trim_deg) /
                            0.1);
  }
  Check(max_rate < 0.2 * 1.01 && max_rate > 0.19,
        "it never moves faster than the rate", Fmt("max %.3f deg/s", max_rate));
  Check(record.last.reset_counter == 1, "no change of reset_counter");

  sc.params.heading_trim_rate = 0.05 * kDeg;
  sc.params.heading_trim_window_s = 4.0;
  record = Run(sc);
  Check(std::abs(record.last.heading_trim_deg + 0.2) < 0.005,
        "0.05 deg/s for a 4 s window: stops at -0.2 deg",
        Fmt("trim %.4f deg", record.last.heading_trim_deg));

  sc.params.mount_yaw = 3.0 * kDeg;
  sc.params.heading_trim_rate = 0.5 * kDeg;
  sc.params.heading_trim_window_s = 20.0;
  record = Run(sc);
  Check(std::abs(record.last.heading_trim_deg + 2.0) < 1e-4,
        "3 deg off: the trim stops at heading_trim_max_deg",
        Fmt("trim %.4f deg, fit %.4f deg", record.last.heading_trim_deg,
            record.last.heading_fit_deg));
}

void CheckShadowCount() {
  std::printf("shadow: crossings of the limit by the placed VIO\n");
  Scenario sc;
  sc.params.vio_enabled = true;
  sc.params.has_mount = true;
  sc.arm = 9.5;
  sc.takeoff = 10.5;
  sc.duration = 20.0;
  sc.vio_error = [](double t) {
    return Eigen::Vector3d(0.0, t >= 14.0 && t < 16.0 ? 0.25 : 0.0, 0.0);
  };
  const Record record = Run(sc);
  Check(
      record.last.dist_exceed_count == 1 && record.last.fallbacks == 0 &&
          record.last.source == Source::kMocap &&
          std::abs(record.StatusAt(15.0).vio_mocap_dist_m - 0.25) < 1e-3,
      "VIO 25 cm off for 2 s: one crossing counted, PX4 untouched",
      Fmt("distance reported %.3f m", record.StatusAt(15.0).vio_mocap_dist_m));
}

void CheckBadVio() {
  std::printf("VIO poses that must be dropped\n");
  VisionPoseCore core(VioNav());
  vision_pose::VioPose pose;
  Px4Sample px4;
  PlacedPose placed;
  pose.stamp_ns = Stamp(1.0);
  int rejected = 0;
  int accepted =
      core.HandleVio(pose, kSteady0, 0.05, &px4, &placed).rejected ? 0 : 1;
  vision_pose::VioPose bad = pose;
  bad.stamp_ns = Stamp(0.9);  // out of order
  rejected += core.HandleVio(bad, kSteady0, 0.05, &px4, &placed).rejected;
  bad.stamp_ns = Stamp(1.1);
  bad.p.x() = std::numeric_limits<double>::quiet_NaN();
  rejected += core.HandleVio(bad, kSteady0, 0.05, &px4, &placed).rejected;
  bad = pose;
  bad.stamp_ns = Stamp(1.2);
  bad.q = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
  rejected += core.HandleVio(bad, kSteady0, 0.05, &px4, &placed).rejected;
  bad = pose;
  bad.stamp_ns = 0;
  rejected += core.HandleVio(bad, kSteady0, 0.05, &px4, &placed).rejected;
  Check(
      accepted == 1 && rejected == 4 && core.GetStatus(kSteady0).vio_poses == 1,
      "out of order, NaN, zero quaternion, zero stamp: dropped and counted "
      "out");
}

void CheckHeldUp() {
  std::printf("VIO timeout when the node itself was held up\n");
  // on the VIO after 8 s of standstill; returns the time reached
  auto to_vio = [](VisionPoseCore *core) {
    Px4Sample px4;
    PlacedPose placed;
    double t = 0.0;
    for (long ms = 0; ms < 8000; ++ms) {
      t = ms * 1e-3;
      const double now = kSteady0 + t;
      if (ms % 500 == 0) {
        core->HandleVehicleStatus(false, now);
      }
      if (ms % 10 == 5) {
        core->HandleVehicleOdometry(Stamp(t - 0.005),
                                    Eigen::Vector3d(0.0, 0.0, 0.0), now);
      }
      if (ms % 4 == 0) {
        vision_pose::MocapSample sample;
        sample.stamp_ns = Stamp(t - 0.007);
        sample.tracking_valid = true;
        core->HandleMocap(sample, now, &px4);
      }
      if (ms % 50 == 0 && ms >= 100) {
        vision_pose::VioPose pose;
        pose.stamp_ns = Stamp(t - 0.058);
        core->HandleVio(pose, now, 0.058, &px4, &placed);
      }
      if (ms % 10 == 0) {
        core->HandleTimer(now);
      }
    }
    return t + 1e-3;
  };
  Px4Sample px4;
  PlacedPose placed;
  vision_pose::VioPose pose;

  // 0.5 s without a single callback, the pose that arrived meanwhile is
  // read after the timer
  VisionPoseCore held(VioNav());
  double t = to_vio(&held);
  const bool on_vio = held.GetStatus(kSteady0 + t).source == Source::kVio;
  t += 0.5;
  held.HandleTimer(kSteady0 + t);
  const bool kept = held.GetStatus(kSteady0 + t).source == Source::kVio;
  pose.stamp_ns = Stamp(t - 0.3);
  held.HandleVio(pose, kSteady0 + t, 0.3, &px4, &placed);
  held.HandleTimer(kSteady0 + t + 0.01);
  Check(on_vio && kept &&
            held.GetStatus(kSteady0 + t + 0.01).source == Source::kVio,
        "node held up for 0.5 s, a pose was waiting: no fallback");

  // the same, but no pose was waiting: fallback 0.2 s after the hold-up
  VisionPoseCore silent(VioNav());
  t = to_vio(&silent);
  t += 0.5;
  double t_fallback = kInf;
  for (int call = 0; call < 40 && !std::isfinite(t_fallback); ++call) {
    silent.HandleTimer(kSteady0 + t + 0.01 * call);
    if (silent.GetStatus(kSteady0 + t + 0.01 * call).source != Source::kVio) {
      t_fallback = 0.01 * call;
    }
  }
  Check(t_fallback > 0.195 && t_fallback < 0.215 &&
            silent.GetStatus(kSteady0 + t + 1.0).reason == Reason::kVioTimeout,
        "node held up and no pose waiting: fallback 0.2 s after it continues",
        Fmt("%.2f s after it", t_fallback));

  // a timer that is always late does not hide the silence
  VisionPoseCore slow(VioNav());
  t = to_vio(&slow);
  int calls = 0;
  while (slow.GetStatus(kSteady0 + t).source == Source::kVio && calls < 100) {
    t += 0.06;
    slow.HandleTimer(kSteady0 + t);
    ++calls;
  }
  Check(calls <= 7 && slow.GetStatus(kSteady0 + t).fallback_latched,
        "timer every 60 ms and no pose: fallback all the same",
        Fmt("after %d calls, %.2f s", calls, 0.06 * calls));
}

void CheckStaleStatus() {
  std::printf("state machine: the vehicle status stops arriving\n");
  // two status messages say disarmed, then none; takeoff at 3 s, and from
  // 6 s the drone hovers still in the air
  Scenario sc;
  sc.params = VioNav();
  sc.status_gap_from = 0.9;
  sc.takeoff = 3.0;
  sc.land = 6.0;
  sc.duration = 20.0;
  Record record = Run(sc);
  const double height = Flight(19.0, sc.takeoff, sc.land).p.z() - kStart.z();
  Check(height > 1.5 && record.LogCount("pose source: MOCAP -> VIO") == 0 &&
            record.last.source == Source::kMocap && record.placed.empty() &&
            CountSent(record, true, 0.0, kInf) == 0,
        "status lost on the ground, then hovering still: no switch in the "
        "air, VIO not placed",
        Fmt("hover %.2f m above the start for 14 s", height));
  Check(record.last.reason == Reason::kNoVehicleStatus && !record.last.armed,
        "reason 'no vehicle status' although the last one said disarmed");

  // on the ground the status pauses for 4 s and comes back: no latch
  sc = Scenario();
  sc.params = VioNav();
  sc.status_gap_from = 5.1;
  sc.status_gap_until = 9.4;
  sc.duration = 12.0;
  record = Run(sc);
  const double t_vio = record.LogTime("pose source: MOCAP -> VIO (vio ready)");
  Check(record.StatusAt(9.0).source == Source::kMocap &&
            record.StatusAt(9.0).reason == Reason::kNoVehicleStatus &&
            t_vio > 9.49 && t_vio < 9.6 && record.last.source == Source::kVio,
        "status silent from 5.0 to 9.5 s on the ground: MOCAP -> VIO only "
        "once it is back",
        Fmt("at %.3f s (7 s of poses stand at 7.06 s)", t_vio));
  // one message lost (1 s between two) does not stop anything
  sc.status_gap_from = 6.9;
  sc.status_gap_until = 7.4;
  record = Run(sc);
  Check(record.LogTime("pose source: MOCAP -> VIO (vio ready)") < 7.2,
        "one status message lost: MOCAP -> VIO as usual",
        Fmt("at %.3f s",
            record.LogTime("pose source: MOCAP -> VIO (vio ready)")));
}

void CheckOdometryNeeded() {
  std::printf("state machine: PX4's own position is not arriving\n");
  Scenario sc;
  sc.params = VioNav();
  sc.odometry_from = kInf;
  sc.arm = 12.0;
  sc.duration = 14.0;
  Record record = Run(sc);
  Check(record.LogCount("pose source: MOCAP -> VIO") == 0 &&
            record.StatusAt(11.0).source == Source::kMocap &&
            record.StatusAt(11.0).vio_ready && !record.placed.empty() &&
            record.StatusAt(11.0).reason == Reason::kNoVehicleOdometry &&
            std::isnan(record.StatusAt(11.0).px4_mocap_dist_m),
        "no vehicle odometry: VIO placed and ready, PX4 stays on the mocap, "
        "reason says so");
  Check(record.LogCount("armed: PX4 stays on MOCAP for this flight (no "
                        "vehicle odometry)") == 1,
        "one line says so at arming",
        record.LogLine("armed: PX4 stays").c_str());

  sc.odometry_from = 9.0;
  sc.arm = kInf;
  record = Run(sc);
  const double t_vio = record.LogTime("pose source: MOCAP -> VIO (vio ready)");
  Check(t_vio > 9.0 && t_vio < 9.1 && record.last.source == Source::kVio,
        "the odometry starts at 9.0 s on the ground: MOCAP -> VIO then",
        Fmt("at %.3f s", t_vio));
}

void CheckDistanceRuleRates() {
  std::printf(
      "fallback: the distance rule at other odometry rates and "
      "with a mocap that comes and goes\n");
  Scenario base;
  base.params = VioNav();
  base.arm = 9.5;
  base.takeoff = 10.5;
  base.duration = 18.0;
  base.px4_error = [](double t) {
    return Eigen::Vector3d(t >= 15.0 ? 0.2 : 0.0, 0.0, 0.0);
  };
  for (const double hz : {50.0, 20.0, 15.0, 10.0, 5.0}) {
    Scenario sc = base;
    sc.odometry_hz = hz;
    const Record record = Run(sc);
    const double t_back = record.LogTime("pose source: VIO -> MOCAP");
    // the second sample above the limit that is 0.10 s after the first
    Check(record.StatusAt(14.9).source == Source::kVio && t_back > 15.10 &&
              t_back < 15.12 + 1.0 / hz &&
              record.last.reason == Reason::kDistanceLimit,
          Fmt("PX4 odometry at %.0f Hz, 20 cm off from 15.0 s: fallback", hz)
              .c_str(),
          Fmt("at %.3f s", t_back));
  }
  // mocap present for 60 ms out of every 120 ms
  Scenario sc = base;
  sc.mocap_holes_from = 14.0;
  sc.mocap_hole_period_s = 0.12;
  sc.mocap_hole_s = 0.06;
  Record record = Run(sc);
  double t_back = record.LogTime("pose source: VIO -> ");
  Check(t_back > 15.10 && t_back < 15.30 &&
            record.last.reason == Reason::kDistanceLimit,
        "mocap missing 60 ms out of every 120 ms: the hold goes on over the "
        "holes, fallback",
        Fmt("at %.3f s", t_back));
  // 0.3 s without PX4 position inside the excursion: the hold goes on
  sc = base;
  sc.odometry_gap_from = 15.015;
  sc.odometry_gap_until = 15.315;
  record = Run(sc);
  t_back = record.LogTime("pose source: VIO -> MOCAP");
  Check(t_back > 15.31 && t_back < 15.35,
        "two samples above, 0.3 s without odometry, one above: fallback on "
        "that one",
        Fmt("at %.3f s", t_back));
  // 0.8 s without: the hold starts again
  sc.odometry_gap_until = 15.815;
  record = Run(sc);
  t_back = record.LogTime("pose source: VIO -> MOCAP");
  Check(t_back > 15.91 && t_back < 15.95,
        "0.8 s without odometry: the hold starts again, fallback 0.10 s "
        "after it is back",
        Fmt("at %.3f s", t_back));
}

void CheckBadStamp() {
  std::printf(
      "fallback: one stamp 3 s ahead does not stop the distance "
      "rule\n");
  Scenario base;
  base.params = VioNav();
  base.arm = 9.5;
  base.takeoff = 10.5;
  base.duration = 19.0;
  base.px4_error = [](double t) {
    return Eigen::Vector3d(t >= 15.0 ? 0.2 : 0.0, 0.0, 0.0);
  };
  Scenario sc = base;
  sc.mocap_bad_stamp_at = 14.95;
  Record record = Run(sc);
  double t_back = record.LogTime("pose source: VIO -> MOCAP");
  Check(t_back > 15.10 && t_back < 15.25 &&
            record.last.reason == Reason::kDistanceLimit,
        "one mocap sample stamped 3 s ahead at 14.95 s, PX4 20 cm off from "
        "15.0 s: fallback",
        Fmt("at %.3f s", t_back));
  sc = base;
  sc.odometry_bad_stamp_at = 14.95;
  record = Run(sc);
  t_back = record.LogTime("pose source: VIO -> MOCAP");
  Check(t_back > 15.10 && t_back < 15.40 &&
            record.last.reason == Reason::kDistanceLimit,
        "one PX4 position stamped 3 s ahead at 14.95 s: fallback",
        Fmt("at %.3f s", t_back));
}

void CheckVioRestart() {
  std::printf("the estimator starts again: stamps 8 s back, another world\n");
  // in flight on the VIO
  Scenario sc;
  sc.params = VioNav();
  sc.arm = 9.5;
  sc.takeoff = 10.5;
  sc.vio_restart_at = 15.0;
  sc.duration = 18.0;
  Record record = Run(sc);
  const double t_back = record.LogTime("pose source: VIO -> MOCAP");
  double position, angle;
  PlacedError(sc, record, 0.0, &position, &angle);
  Check(record.StatusAt(14.9).source == Source::kVio &&
            CountSent(record, true, 15.0 + kVioDelay, kInf) == 0 &&
            position < 1e-6,
        "on the VIO in flight: no pose of the new stream is sent to PX4 or "
        "placed",
        Fmt("%d VIO messages after it, placed poses within %.3g m of the truth",
            CountSent(record, true, 15.0 + kVioDelay, kInf), position));
  Check(t_back > 15.33 && t_back < 15.39 &&
            record.last.reason == Reason::kVioTimeout &&
            record.last.source == Source::kMocap,
        "VIO -> MOCAP by the timeout, 0.33 s after the last pose of the old "
        "stream",
        Fmt("at %.3f s", t_back));

  // one pose stamped 10 s back between good ones: dropped, nothing else
  sc = Scenario();
  sc.params = VioNav();
  sc.arm = 9.5;
  sc.takeoff = 10.5;
  sc.vio_back_once_at = 14.02;
  sc.duration = 18.0;
  record = Run(sc);
  bool increasing = true;
  uint64_t previous = 0;
  for (const Sent &sent : record.sent) {
    if (sent.from_vio) {
      increasing = increasing && sent.sample.timestamp_sample > previous;
      previous = sent.sample.timestamp_sample;
    }
  }
  Check(record.last.source == Source::kVio && record.last.fallbacks == 0 &&
            increasing &&
            CountSent(record, true, 0.0, kInf) ==
                static_cast<int>(record.placed.size()),
        "one pose stamped 10 s back while on the VIO: dropped, still on the "
        "VIO, stamps sent increasing");

  // on the ground in shadow, a recording played again (every stamp 8 s
  // back): the new stream is taken and placed again
  sc = Scenario();
  sc.params.vio_enabled = true;
  sc.params.has_mount = true;
  sc.vio_restart_at = 10.0;
  sc.stamps_back_from = 10.0;
  sc.duration = 20.0;
  record = Run(sc);
  double late = 0.0;
  for (const auto &entry : record.placed) {
    if (entry.first > 17.2) {
      const Truth truth = Flight(entry.first, sc.takeoff, sc.land);
      late = std::max(late, (entry.second.p - truth.p).norm());
    }
  }
  Check(record.LogCount("VIO ready:") == 2 && late < 1e-6 &&
            record.last.vio_ready,
        "disarmed, not on the VIO: the new stream is placed again after 7 s",
        Fmt("placed within %.3g m of the truth from 17.2 s", late));
}

void CheckFallbackKeepsBodyRotation() {
  std::printf(
      "fallback: the body rotation of the mocap feed stays the one "
      "of the start\n");
  // the VIO goes wrong in flight (position 60 cm, attitude 5 deg of tilt);
  // after the landing the drone stands disarmed and the bad VIO is placed
  // again
  Scenario sc;
  sc.params = VioNav();
  sc.params.mount_yaw = 2.72 * kDeg;
  sc.params.correct_mocap_attitude = true;
  sc.c = Rz(-2.72 * kDeg) * Ry(1.5 * kDeg);
  sc.arm = 9.5;
  sc.takeoff = 10.5;
  sc.land = 22.0;
  sc.disarm = 23.0;
  sc.duration = 36.0;
  sc.px4_error = [](double t) {
    return Eigen::Vector3d(t >= 15.0 ? 0.6 : 0.0, 0.0, 0.0);
  };
  sc.vio_attitude_error = [](double t) {
    return t >= 15.0 ? Rx(5.0 * kDeg) : Eigen::Quaterniond::Identity();
  };
  const Record record = Run(sc);
  const double t_back = record.LogTime("pose source: VIO -> MOCAP");
  double in_flight = 0.0, standing = 0.0;
  for (const Sent &sent : record.sent) {
    if (sent.from_vio || sent.t < t_back) {
      continue;
    }
    const Truth truth = Flight(sent.t - kMocapDelay, sc.takeoff, sc.land);
    const Eigen::Matrix3d d = Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();
    const Eigen::Quaterniond got(sent.sample.q[0], sent.sample.q[1],
                                 sent.sample.q[2], sent.sample.q[3]);
    const double angle =
        Eigen::Quaterniond(d * (truth.q * sc.c.conjugate()).toRotationMatrix() *
                           d)
            .angularDistance(got);
    (sent.t < 23.0 ? in_flight : standing) =
        std::max(sent.t < 23.0 ? in_flight : standing, angle);
  }
  Check(t_back > 15.10 && t_back < 15.13 &&
            record.LogCount("VIO ready:") == 2 &&
            std::abs(record.last.body_tilt_deg - 1.5) > 2.0,
        "fallback in flight; standing again the bad VIO is placed again",
        Fmt("body tilt in the status %.2f deg (1.50 deg at the start)",
            record.last.body_tilt_deg));
  Check(in_flight < 5e-4 && standing < 5e-4,
        "attitude sent from the mocap = R_m C^T with the C of the start, "
        "before and after that",
        Fmt("max %.4f deg in flight, %.4f deg standing afterwards",
            in_flight / kDeg, standing / kDeg));
}

void CheckLateMocap() {
  std::printf("pairing: the mocap arrives 100 ms later than the VIO poses\n");
  // placement heading 1 deg off and the trim on: the fit, the start of the
  // trim and the distance all need the pairs
  Scenario sc;
  sc.params = VioNav();
  sc.params.mount_yaw = 1.0 * kDeg;  // the true mount yaw is 0
  sc.params.heading_trim_rate = 0.2 * kDeg;
  sc.arm = 9.5;
  sc.takeoff = 10.5;
  sc.duration = 34.0;
  const Record lead = Run(sc);  // the mocap leads, as on the drones today
  sc.mocap_delay = kVioDelay + 0.100;
  Record record = Run(sc);
  const double t_vio = record.LogTime("pose source: MOCAP -> VIO (vio ready)");
  Check(t_vio > 7.0 && t_vio < 7.2 && record.last.source == Source::kVio &&
            record.last.fallbacks == 0,
        "the VIO is placed, ready and handed over on the ground, and PX4 "
        "stays on it",
        Fmt("at %.3f s; %s", t_vio, record.LogLine("VIO ready:").c_str()));
  Check(record.LogLine("armed: placement frozen") ==
                lead.LogLine("armed: placement frozen") &&
            record.LogCount("armed: placement frozen") == 1 &&
            std::abs(record.StatusAt(10.0).heading_deg -
                     lead.StatusAt(10.0).heading_deg) < 1e-4,
        "heading and pin frozen at arming as with a mocap that leads",
        record.LogLine("armed: placement frozen").c_str());
  double position, angle;
  PlacedError(sc, record, 30.0, &position, &angle);
  Check(std::abs(record.last.heading_fit_deg + 1.0) < 0.01 &&
            std::abs(record.last.heading_trim_deg + 1.0) < 0.01 &&
            position < 1e-3,
        "in flight the fit reads -1 deg and the trim takes it out",
        Fmt("fit %.4f deg, trim %.4f deg, placed VIO within %.2f mm after 30 s",
            record.last.heading_fit_deg, record.last.heading_trim_deg,
            1e3 * position));

  // shadow: the distance of the placed VIO and its crossings
  Scenario shadow;
  shadow.params.vio_enabled = true;
  shadow.params.has_mount = true;
  shadow.mocap_delay = kVioDelay + 0.100;
  shadow.arm = 9.5;
  shadow.takeoff = 10.5;
  shadow.duration = 20.0;
  shadow.vio_error = [](double t) {
    return Eigen::Vector3d(0.0, t >= 14.0 && t < 16.0 ? 0.25 : 0.0, 0.0);
  };
  record = Run(shadow);
  Check(record.last.dist_exceed_count == 1 &&
            std::abs(record.StatusAt(15.0).vio_mocap_dist_m - 0.25) < 1e-3 &&
            record.StatusAt(19.0).vio_mocap_dist_m < 1e-5,
        "shadow, VIO 25 cm off for 2 s: one crossing counted, distance "
        "reported",
        Fmt("%.3f m at 15 s, %.1e m at 19 s",
            record.StatusAt(15.0).vio_mocap_dist_m,
            record.StatusAt(19.0).vio_mocap_dist_m));

  // the mocap is away for 2.5 s in flight (it leads otherwise): the poses of
  // that time wait for it and are dropped when it is back, not paired across
  // the hole; the status keeps the distance of the latest pair for 0.5 s
  Scenario hole;
  hole.params = VioNav();
  hole.arm = 9.5;
  hole.takeoff = 10.5;
  hole.duration = 20.0;
  hole.mocap_gap_from = 13.5;
  hole.mocap_gap_until = 16.0;
  record = Run(hole);
  Check(std::isfinite(record.StatusAt(13.8).vio_mocap_dist_m) &&
            std::isnan(record.StatusAt(14.3).vio_mocap_dist_m) &&
            record.StatusAt(16.3).vio_mocap_dist_m < 1e-5,
        "mocap away 13.5 to 16.0 s: vio_mocap_dist_m is the latest pair's for "
        "0.5 s, then NaN, and back with the mocap",
        Fmt("%.1e m at 13.8 s, %f at 14.3 s, %.1e m at 16.3 s",
            record.StatusAt(13.8).vio_mocap_dist_m,
            record.StatusAt(14.3).vio_mocap_dist_m,
            record.StatusAt(16.3).vio_mocap_dist_m));
  Check(
      record.last.source == Source::kVio && record.last.fallbacks == 0 &&
          record.last.dist_exceed_count == 0 &&
          CountSent(record, true, 13.5, 16.0) == 50,
      "no pose is paired across the hole, PX4 stays on the VIO through it",
      Fmt("%d VIO messages in the 2.5 s", CountSent(record, true, 13.5, 16.0)));
}

void CheckPlacedAgain() {
  std::printf(
      "placed again after the landing while PX4 is on the VIO: "
      "reset_counter\n");
  // the VIO drifts by 8 cm during the flight
  Scenario sc;
  sc.params = VioNav();
  sc.arm = 9.5;
  sc.takeoff = 10.5;
  sc.land = 22.0;
  sc.disarm = 23.0;
  sc.duration = 36.0;
  sc.vio_error = [](double t) {
    return Eigen::Vector3d(
        0.08 * std::min(std::max((t - 12.0) / 8.0, 0.0), 1.0), 0.0, 0.0);
  };
  const Record record = Run(sc);
  const double t_again = record.LogTime("VIO placed again:");
  Check(record.LogCount("VIO placed again:") == 1 && t_again > 29.0 &&
            t_again < 29.2 &&
            record.LogLine("VIO placed again:") ==
                "VIO placed again: step 0.080 m, heading +0.00 deg, "
                "reset_counter 2",
        "one 'VIO placed again' line, 7 s after it landed",
        Fmt("at %.3f s: %s", t_again,
            record.LogLine("VIO placed again:").c_str()));
  // the messages around it: the counter changes on the message that steps
  double step_at_change = 0.0, other_steps = 0.0;
  int changes = 0;
  bool order = true;
  const Sent *previous = nullptr;
  for (const Sent &sent : record.sent) {
    if (!sent.from_vio || sent.t < 22.2) {
      continue;
    }
    if (previous != nullptr) {
      const double step = (Eigen::Vector3f(sent.sample.position) -
                           Eigen::Vector3f(previous->sample.position))
                              .norm();
      if (sent.sample.reset_counter != previous->sample.reset_counter) {
        ++changes;
        step_at_change = step;
        order = order && previous->sample.reset_counter == 1 &&
                sent.sample.reset_counter == 2;
      } else {
        other_steps = std::max(other_steps, step);
      }
    }
    previous = &sent;
  }
  Check(changes == 1 && order && std::abs(step_at_change - 0.08) < 1e-4 &&
            other_steps < 1e-5,
        "the pose sent steps by 8 cm once, on the message whose "
        "reset_counter goes 1 -> 2",
        Fmt("step %.4f m with the change, %.1e m at most between the others",
            step_at_change, other_steps));
  Check(record.last.reset_counter == 2 && record.last.source == Source::kVio &&
            record.last.fallbacks == 0 && record.last.vio_ready,
        "the placement at every pose of the 7 s that follow does not touch "
        "the counter; still on the VIO");
}

void CheckPlacedAgainWithoutReadyEdge() {
  std::printf(
      "placed again while PX4 is on the VIO: placements that are not at a "
      "'ready' edge\n");
  // The vehicle stands armed for 9 s after the landing (longer than
  // vio_settle_s) before it is disarmed: the VIO is ready all along, and the
  // first pose after the disarming is placed anew. The VIO drifted by 8 cm
  // and by 2 deg of heading in the flight.
  Scenario sc;
  sc.params = VioNav();
  sc.arm = 9.5;
  sc.takeoff = 10.5;
  sc.land = 22.0;
  sc.disarm = 31.0;
  sc.duration = 40.0;
  sc.vio_error = [](double t) {
    return Eigen::Vector3d(
        0.08 * std::min(std::max((t - 12.0) / 8.0, 0.0), 1.0), 0.0, 0.0);
  };
  sc.vio_attitude_error = [](double t) {
    return Rz(2.0 * kDeg * std::min(std::max((t - 12.0) / 8.0, 0.0), 1.0));
  };
  // steps of the VIO messages to PX4 from `from` on: with a change of the
  // counter, and the largest one without
  auto steps = [](const Record &record, double from, int *changes,
                  double *step_at_change, double *other_steps, bool *order) {
    *changes = 0;
    *step_at_change = 0.0;
    *other_steps = 0.0;
    *order = true;
    const Sent *previous = nullptr;
    for (const Sent &sent : record.sent) {
      if (!sent.from_vio || sent.t < from) {
        continue;
      }
      if (previous != nullptr) {
        const double step = (Eigen::Vector3f(sent.sample.position) -
                             Eigen::Vector3f(previous->sample.position))
                                .norm();
        if (sent.sample.reset_counter != previous->sample.reset_counter) {
          ++*changes;
          *step_at_change = step;
          *order = *order && previous->sample.reset_counter == 1 &&
                   sent.sample.reset_counter == 2;
        } else {
          *other_steps = std::max(*other_steps, step);
        }
      }
      previous = &sent;
    }
  };
  Record record = Run(sc);
  double t_again = record.LogTime("VIO placed again:");
  Check(record.LogCount("VIO placed again:") == 1 && t_again > 31.0 &&
            t_again < 31.1 &&
            record.LogLine("VIO placed again:") ==
                "VIO placed again: step 0.080 m, heading -2.00 deg, "
                "reset_counter 2",
        "disarmed 9 s after the landing: one 'VIO placed again' line at the "
        "first pose after it, with the step and the turn of the heading",
        Fmt("at %.3f s: %s", t_again,
            record.LogLine("VIO placed again:").c_str()));
  int changes;
  double step_at_change, other_steps;
  bool order;
  steps(record, 22.2, &changes, &step_at_change, &other_steps, &order);
  Check(changes == 1 && order && std::abs(step_at_change - 0.08) < 1e-4 &&
            other_steps < 1e-5 && record.last.reset_counter == 2 &&
            record.last.source == Source::kVio && record.last.fallbacks == 0,
        "the pose sent steps by 8 cm once, on the message whose "
        "reset_counter goes 1 -> 2",
        Fmt("%d change(s), step %.4f m with it, %.1e m at most between the "
            "others",
            changes, step_at_change, other_steps));

  // Disarmed 1 s after the landing, but the mocap is away from 27 s to 36 s:
  // when the VIO has stood for 7 s there is nothing to place it against. No
  // step is counted then; the step comes, with the counter, when the mocap
  // is back.
  sc.disarm = 23.0;
  sc.duration = 44.0;
  sc.vio_attitude_error = [](double) { return Eigen::Quaterniond::Identity(); };
  sc.mocap_gap_from = 27.0;
  sc.mocap_gap_until = 36.0;
  record = Run(sc);
  t_again = record.LogTime("VIO placed again:");
  Check(record.LogCount("VIO placed again:") == 1 && t_again > 36.0 &&
            t_again < 36.2 && record.StatusAt(35.9).reset_counter == 1 &&
            record.LogLine("VIO placed again:") ==
                "VIO placed again: step 0.080 m, heading +0.00 deg, "
                "reset_counter 2",
        "no mocap when the VIO has stood for 7 s: nothing is counted; one "
        "'VIO placed again' line when the mocap is back",
        Fmt("%d line(s), the first at %.3f s: %s; reset_counter %u at 35.9 s",
            record.LogCount("VIO placed again:"), t_again,
            record.LogLine("VIO placed again:").c_str(),
            static_cast<unsigned>(record.StatusAt(35.9).reset_counter)));
  steps(record, 23.2, &changes, &step_at_change, &other_steps, &order);
  Check(changes == 1 && order && std::abs(step_at_change - 0.08) < 1e-4 &&
            other_steps < 1e-5 && record.last.reset_counter == 2 &&
            record.last.source == Source::kVio && record.last.fallbacks == 0,
        "the pose sent steps by 8 cm once, when the mocap is back, on the "
        "message whose reset_counter goes 1 -> 2",
        Fmt("%d change(s), step %.4f m with it, %.1e m at most between the "
            "others",
            changes, step_at_change, other_steps));
}

void CheckVioJump() {
  std::printf(
      "a VIO that jumps: its poses are sent as they come; the fallback comes "
      "from the 15 cm rule once PX4's own position has followed\n");
  // the VIO jumps by 5 m at 15 s; PX4's own position stays where it is for
  // 1 s (it refuses the jump at first) and then follows it
  Scenario sc;
  sc.params = VioNav();
  sc.arm = 9.5;
  sc.takeoff = 10.5;
  sc.duration = 19.0;
  sc.vio_error = [](double t) {
    return Eigen::Vector3d(t >= 15.0 ? 5.0 : 0.0, 0.0, 0.0);
  };
  sc.px4_error = [](double t) {
    return Eigen::Vector3d(t >= 16.0 ? 5.0 : 0.0, 0.0, 0.0);
  };
  const Record record = Run(sc);
  const double t_back = record.LogTime("pose source: VIO -> MOCAP");
  Check(record.StatusAt(15.95).source == Source::kVio &&
            record.StatusAt(15.95).fallbacks == 0 && t_back > 16.0,
        "VIO 5 m off from 15.0 s, PX4's position unchanged until 16.0 s: no "
        "fallback meanwhile",
        Fmt("at 15.95 s: source %s, %u fallback(s)",
            SourceName(record.StatusAt(15.95).source),
            record.StatusAt(15.95).fallbacks));
  // poses stamped 15.0013 to 15.9013 arrive before 16.0 s, those stamped
  // 15.9513 to 16.0513 before the fallback
  Check(CountBadVio(sc, record, 15.0, 16.0, 1.0) == 19 &&
            CountBadVio(sc, record, 0.0, kInf, 1.0) == 22,
        "the poses 5 m off are sent meanwhile, and until the fallback",
        Fmt("%d poses 5 m off reached PX4, %d of them before 16.0 s",
            CountBadVio(sc, record, 0.0, kInf, 1.0),
            CountBadVio(sc, record, 15.0, 16.0, 1.0)));
  Check(t_back > 16.10 && t_back < 16.13 &&
            record.last.reason == Reason::kDistanceLimit &&
            record.last.fallback_latched && record.last.fallbacks == 1 &&
            record.last.source == Source::kMocap &&
            CountSent(record, true, t_back, kInf) == 0,
        "fallback 0.10 s after PX4's position went beyond 15 cm: reason "
        "distance limit, latched, no VIO pose after it",
        Fmt("at %.3f s: %s", t_back,
            record.LogLine("pose source: VIO -> MOCAP").c_str()));
}

void CheckNoComparison() {
  std::printf(
      "fallback: PX4's position stops being compared with the mocap in "
      "flight\n");
  Scenario base;
  base.params = VioNav();
  base.arm = 9.5;
  base.takeoff = 10.5;
  base.duration = 20.0;
  base.odometry_gap_from = 15.0;
  Record record = Run(base);
  double t_back = record.LogTime("pose source: VIO -> MOCAP");
  Check(t_back > 15.99 && t_back < 16.03 &&
            record.last.reason == Reason::kNoVehicleOdometry &&
            record.last.fallback_latched && record.last.reset_counter == 2,
        "PX4's odometry stops at 15.0 s on the VIO: VIO -> MOCAP 1.0 s later, "
        "reason no vehicle odometry",
        Fmt("at %.3f s: %s", t_back,
            record.LogLine("pose source: VIO -> MOCAP").c_str()));
  Check(record.LogLine("pose source: VIO -> MOCAP") ==
            "pose source: VIO -> MOCAP (no comparison of PX4 with the mocap "
            "for 1.00 s: the distance rule cannot run)",
        "the log line says what is missing");

  // the mocap is missing as well: the VIO is all PX4 has
  Scenario sc = base;
  sc.mocap_gap_from = 15.0;
  record = Run(sc);
  Check(record.last.source == Source::kVio && record.last.fallbacks == 0 &&
            CountSent(record, true, 16.0, 20.0) == 80,
        "the mocap stops at 15.0 s too: no fallback, PX4 stays on the VIO",
        Fmt("%d VIO messages from 16 to 20 s",
            CountSent(record, true, 16.0, 20.0)));
  // and comes back at 17.0 s: the 1.0 s counts from its return
  sc.mocap_gap_until = 17.0;
  record = Run(sc);
  t_back = record.LogTime("pose source: VIO -> MOCAP");
  Check(record.StatusAt(17.9).source == Source::kVio && t_back > 18.0 &&
            t_back < 18.03 && record.last.reason == Reason::kNoVehicleOdometry,
        "the mocap is back at 17.0 s, the odometry is not: fallback 1.0 s "
        "after its return",
        Fmt("at %.3f s", t_back));
  // 0.8 s without odometry: nothing
  sc = base;
  sc.odometry_gap_until = 15.8;
  record = Run(sc);
  Check(record.last.source == Source::kVio && record.last.fallbacks == 0,
        "0.8 s without odometry: no fallback");
  // PX4's odometry goes on, and from 15.0 to 18.0 s the mocap samples are
  // marked valid with a quaternion of zeros: there is nothing to compare
  // with, and nothing PX4 could be given instead of the VIO
  sc = base;
  sc.odometry_gap_from = kInf;
  sc.mocap_zero_q_from = 15.0;
  sc.mocap_zero_q_until = 18.0;
  record = Run(sc);
  Check(record.last.source == Source::kVio && record.last.fallbacks == 0 &&
            CountSent(record, true, 15.0, 18.0) == 60 &&
            CountSent(record, false, 15.0, kInf) == 0,
        "mocap samples marked valid with a quaternion of zeros for 3 s, the "
        "odometry running: no fallback, PX4 stays on the VIO",
        Fmt("%d VIO and %d mocap messages to PX4 from 15 to 18 s; %s",
            CountSent(record, true, 15.0, 18.0),
            CountSent(record, false, 15.0, 18.0),
            record.LogLine("pose source: VIO ->").c_str()));
}

void CheckHeldUpPeriod() {
  std::printf("hold-ups of the node: the checks are skipped for 0.2 s\n");
  // on the VIO and armed after 8 s of standstill; returns the time reached
  auto to_vio = [](VisionPoseCore *core) {
    Px4Sample px4;
    PlacedPose placed;
    double t = 0.0;
    for (long ms = 0; ms < 8000; ++ms) {
      t = ms * 1e-3;
      const double now = kSteady0 + t;
      if (ms % 500 == 0) {
        core->HandleVehicleStatus(ms >= 7500, now);
      }
      if (ms % 10 == 5) {
        core->HandleVehicleOdometry(Stamp(t - 0.005),
                                    Eigen::Vector3d(0.0, 0.0, 0.0), now);
      }
      if (ms % 4 == 0) {
        vision_pose::MocapSample sample;
        sample.stamp_ns = Stamp(t - 0.007);
        sample.tracking_valid = true;
        core->HandleMocap(sample, now, &px4);
      }
      if (ms % 50 == 0 && ms >= 100) {
        vision_pose::VioPose pose;
        pose.stamp_ns = Stamp(t - 0.058);
        core->HandleVio(pose, now, 0.058, &px4, &placed);
      }
      if (ms % 10 == 0) {
        core->HandleTimer(now);
      }
    }
    return t + 1e-3;
  };
  Px4Sample px4;
  PlacedPose placed;
  vision_pose::VioPose pose;

  // 3 s without a single callback; then the timer runs twice, 1 ms apart,
  // before the pose that waited is read (what the live node did)
  VisionPoseCore twice(VioNav());
  double t = to_vio(&twice);
  const bool on_vio = twice.GetStatus(kSteady0 + t).source == Source::kVio;
  t += 3.0;
  twice.HandleTimer(kSteady0 + t);
  twice.HandleTimer(kSteady0 + t + 0.001);
  const bool kept = twice.GetStatus(kSteady0 + t).source == Source::kVio;
  pose.stamp_ns = Stamp(t - 0.3);
  const bool sent =
      twice.HandleVio(pose, kSteady0 + t + 0.003, 0.3, &px4, &placed).px4;
  twice.HandleTimer(kSteady0 + t + 0.3);
  Check(on_vio && kept && sent &&
            twice.GetStatus(kSteady0 + t + 0.3).source == Source::kVio &&
            twice.GetStatus(kSteady0 + t + 0.3).fallbacks == 0,
        "held up for 3 s, the timer runs twice before the waiting pose is "
        "read: no fallback, the pose is sent");

  // two hold-ups back to back (2.0 s, one timer call, 0.4 s)
  VisionPoseCore pair(VioNav());
  t = to_vio(&pair);
  t += 2.0;
  pair.HandleTimer(kSteady0 + t);
  t += 0.4;
  pair.HandleTimer(kSteady0 + t);
  pair.HandleTimer(kSteady0 + t + 0.001);
  const bool kept2 = pair.GetStatus(kSteady0 + t).source == Source::kVio;
  pose.stamp_ns = Stamp(t - 0.3);
  const bool sent2 =
      pair.HandleVio(pose, kSteady0 + t + 0.003, 0.3, &px4, &placed).px4;
  Check(kept2 && sent2 &&
            pair.GetStatus(kSteady0 + t + 0.01).source == Source::kVio,
        "two hold-ups back to back (2.0 s and 0.4 s): no fallback");

  // the comparison of PX4 with the mocap after a hold-up: the mocap counts
  // from its return, so nothing fires while the inputs are read again
  VisionPoseCore compare(VioNav());
  t = to_vio(&compare);
  t += 1.5;
  bool on = true;
  for (int ms = 0; ms < 400; ++ms) {
    const double now = kSteady0 + t + ms * 1e-3;
    if (ms % 10 == 0) {
      compare.HandleTimer(now);
    }
    if (ms % 4 == 1) {
      vision_pose::MocapSample sample;
      sample.stamp_ns = Stamp(t + ms * 1e-3 - 0.007);
      sample.tracking_valid = true;
      compare.HandleMocap(sample, now, &px4);
    }
    if (ms % 50 == 2) {
      pose.stamp_ns = Stamp(t + ms * 1e-3 - 0.058);
      compare.HandleVio(pose, now, 0.058, &px4, &placed);
    }
    if (ms % 10 == 5 && ms >= 100) {
      // PX4's odometry is back 0.1 s after the rest
      compare.HandleVehicleOdometry(Stamp(t + ms * 1e-3 - 0.005),
                                    Eigen::Vector3d(0.0, 0.0, 0.0), now);
    }
    on = on && compare.GetStatus(now).source == Source::kVio;
  }
  Check(on && compare.GetStatus(kSteady0 + t + 0.4).fallbacks == 0,
        "held up for 1.5 s, the odometry is read 0.1 s after the rest: no "
        "fallback");
}

// milliseconds of the checks below as seconds
double Sec(long ms) { return static_cast<double>(ms) * 1e-3; }

// For the hold-up checks below: the inputs of a drone that stands (as in
// CheckHeldUpPeriod), read by a node that runs at now_ms after not having
// run since from_ms. It reads what arrived meanwhile, in arrival order, the
// newest 10 messages of each topic (the depth of its subscriptions) or, for
// the VIO with oldest_poses, the 10 that arrived first. No pose arrives from
// vio_until_ms on. Returns the number of VIO poses sent to PX4.
int ReadWaiting(VisionPoseCore *core, long from_ms, long now_ms, bool armed,
                long vio_until_ms, bool oldest_poses = false) {
  const double now = kSteady0 + Sec(now_ms);
  Px4Sample px4;
  PlacedPose placed;
  int sent = 0;
  for (long ms = from_ms + 1; ms <= now_ms; ++ms) {
    const double t = Sec(ms);
    if (ms % 500 == 0) {
      core->HandleVehicleStatus(armed, now);
    }
    if (ms % 10 == 5 && ms > now_ms - 100) {
      core->HandleVehicleOdometry(Stamp(t - 0.005),
                                  Eigen::Vector3d(0.0, 0.0, 0.0), now);
    }
    if (ms % 4 == 0 && ms > now_ms - 40) {
      vision_pose::MocapSample sample;
      sample.stamp_ns = Stamp(t - 0.007);
      sample.tracking_valid = true;
      core->HandleMocap(sample, now, &px4);
    }
    if (ms % 50 == 0 && ms < vio_until_ms &&
        (oldest_poses ? ms <= from_ms + 500 : ms > now_ms - 500)) {
      vision_pose::VioPose pose;
      pose.stamp_ns = Stamp(t - 0.058);
      sent +=
          core->HandleVio(pose, now, now - kSteady0 - t + 0.058, &px4, &placed)
                  .px4
              ? 1
              : 0;
    }
  }
  return sent;
}

// on the VIO and armed after 8 s of standstill, every input read on time;
// returns the time reached in ms
long ToVioArmed(VisionPoseCore *core) {
  for (long ms = 0; ms < 8000; ++ms) {
    ReadWaiting(core, ms - 1, ms, ms >= 7500, ms >= 100 ? 8000 : 0);
    if (ms % 10 == 0) {
      core->HandleTimer(kSteady0 + Sec(ms));
    }
  }
  return 7999;
}

void CheckHeldUpAgainAndAgain() {
  std::printf(
      "hold-ups of the node that keep coming: the checks are not switched "
      "off\n");
  // From 8.0 s the node is stopped for 60 ms and runs for 20 ms, again and
  // again for 4 s. Each time it runs the timer is called twice (once late,
  // once on time), before what arrived meanwhile is read.
  auto starved = [](VisionPoseCore *core, long vio_until_ms, long *last_ms) {
    long ms = ToVioArmed(core);
    long t_fallback_ms = -1;
    for (long slot = 0; slot < 50; ++slot) {
      for (long call = 0; call < 2; ++call) {
        const long now_ms = 8000 + 80 * slot + 60 + 10 * call;
        core->HandleTimer(kSteady0 + Sec(now_ms));
        ReadWaiting(core, ms, now_ms, true, vio_until_ms);
        ms = now_ms;
        if (t_fallback_ms < 0 &&
            core->GetStatus(kSteady0 + Sec(ms)).source != Source::kVio) {
          t_fallback_ms = ms;
        }
      }
    }
    *last_ms = ms;
    return t_fallback_ms;
  };
  // the estimator stops at 8.0 s: its last pose arrived at 7.950 s
  VisionPoseCore stopped(VioNav());
  long ms = 0;
  const long t_fallback_ms = starved(&stopped, 8000, &ms);
  const Status after = stopped.GetStatus(kSteady0 + Sec(ms));
  Check(
      t_fallback_ms > 0 && t_fallback_ms - 7950 < 800 &&
          after.reason == Reason::kVioTimeout && after.fallback_latched &&
          after.source == Source::kMocap,
      "held up 60 ms out of every 80 ms and the VIO has stopped: VIO -> "
      "MOCAP by the timeout all the same",
      t_fallback_ms > 0
          ? Fmt("%.2f s after the last pose arrived", Sec(t_fallback_ms - 7950))
          : Fmt("no fallback in 4 s, source %s", SourceName(after.source)));

  // the same hold-ups with the VIO healthy: no fallback
  VisionPoseCore healthy(VioNav());
  const long t_false_ms = starved(&healthy, 1000000, &ms);
  Check(t_false_ms < 0 &&
            healthy.GetStatus(kSteady0 + Sec(ms)).source == Source::kVio &&
            healthy.GetStatus(kSteady0 + Sec(ms)).fallbacks == 0,
        "the same hold-ups with the VIO healthy: no fallback");

  // then the node runs on time for 1.5 s, and is held up once for 0.5 s with
  // poses waiting: that hold-up is passed over again
  for (long end = ms + 1500; ms < end;) {
    ++ms;
    ReadWaiting(&healthy, ms - 1, ms, true, 1000000);
    if (ms % 10 == 0) {
      healthy.HandleTimer(kSteady0 + Sec(ms));
    }
  }
  const long from_ms = ms;
  ms += 500;
  healthy.HandleTimer(kSteady0 + Sec(ms));
  healthy.HandleTimer(kSteady0 + Sec(ms) + 0.001);
  const bool kept =
      healthy.GetStatus(kSteady0 + Sec(ms)).source == Source::kVio;
  const int sent = ReadWaiting(&healthy, from_ms, ms, true, 1000000);
  healthy.HandleTimer(kSteady0 + Sec(ms) + 0.3);
  Check(
      kept && sent == 10 &&
          healthy.GetStatus(kSteady0 + Sec(ms) + 0.3).source == Source::kVio &&
          healthy.GetStatus(kSteady0 + Sec(ms) + 0.3).fallbacks == 0,
      "after 1.5 s on time, one hold-up of 0.5 s with poses waiting: no "
      "fallback",
      Fmt("%d waiting poses sent", sent));
}

void CheckHeldUpThenOldPoses() {
  std::printf(
      "hold-up of the node: the poses that waited are read first, then the "
      "stamps jump to the present\n");
  // Held up for 2.5 s in flight. When it runs again the timer is called,
  // then the node reads the 10 poses that arrived first (stamps from the
  // beginning of the hold-up), then the stream goes on live.
  VisionPoseCore core(VioNav());
  long ms = ToVioArmed(&core);
  const long from_ms = ms;
  ms += 2501;  // 10.500 s
  core.HandleTimer(kSteady0 + Sec(ms));
  const int old_sent = ReadWaiting(&core, from_ms, ms - 1, true, 1000000, true);
  core.HandleTimer(kSteady0 + Sec(ms) + 0.001);
  // the pose arriving at 10.500 s is stamped 2.05 s after the last one read
  const int live_sent = ReadWaiting(&core, ms - 1, ms, true, 1000000);
  int more_sent = 0;
  for (long end = ms + 1000; ms < end;) {
    ++ms;
    more_sent += ReadWaiting(&core, ms - 1, ms, true, 1000000);
    if (ms % 10 == 0) {
      core.HandleTimer(kSteady0 + Sec(ms));
    }
  }
  const Status status = core.GetStatus(kSteady0 + Sec(ms));
  std::string line, restarted = "no 'VIO stream restarted' line";
  while (core.PopLogLine(&line)) {
    if (line.find("VIO stream restarted") != std::string::npos ||
        line.find("pose source: VIO ->") != std::string::npos) {
      restarted = line;
    }
  }
  Check(old_sent == 10 && live_sent == 1 && more_sent == 20 &&
            status.source == Source::kVio && status.fallbacks == 0 &&
            status.vio_ready,
        "held up for 2.5 s, 10 old poses read before the live ones: not "
        "taken for a restart of the estimator, no fallback",
        Fmt("%d + %d + %d poses sent, source %s; %s", old_sent, live_sent,
            more_sent, SourceName(status.source), restarted.c_str()));

  // the allowance is used up by then: a second later, stamps that jump by
  // 1.5 s are a restart
  Px4Sample px4;
  PlacedPose placed;
  vision_pose::VioPose pose;
  pose.stamp_ns = Stamp(Sec(ms) - 0.058 + 1.5);
  const bool late_sent =
      core.HandleVio(pose, kSteady0 + Sec(ms), 0.058, &px4, &placed).px4;
  const Status late = core.GetStatus(kSteady0 + Sec(ms));
  Check(!late_sent && late.source != Source::kVio &&
            late.reason == Reason::kVioTimeout && late.fallback_latched,
        "1 s after that hold-up, stamps that jump 1.5 s: taken for a "
        "restart, VIO given up");

  // a real restart is still seen: the same, but the estimator's stamps jump
  // by another 3 s when the stream goes on
  VisionPoseCore real(VioNav());
  ms = ToVioArmed(&real);
  ms += 2501;
  real.HandleTimer(kSteady0 + Sec(ms));
  ReadWaiting(&real, from_ms, ms - 1, true, 1000000, true);
  pose.stamp_ns = Stamp(Sec(ms) - 0.058 + 3.0);
  const bool sent =
      real.HandleVio(pose, kSteady0 + Sec(ms), 0.058, &px4, &placed).px4;
  const Status jumped = real.GetStatus(kSteady0 + Sec(ms));
  Check(!sent && jumped.source != Source::kVio &&
            jumped.reason == Reason::kVioTimeout && jumped.fallback_latched,
        "the same with stamps that jump 3 s further than the hold-up: taken "
        "for a restart, VIO given up, the pose is not sent");
}

void CheckVioRestartForward() {
  std::printf(
      "the estimator starts again: its stamps jump forward, another "
      "world\n");
  // shadow, disarmed: 3 s without poses, then another world
  Scenario sc;
  sc.params.vio_enabled = true;
  sc.params.has_mount = true;
  sc.vio_gap_from = 10.0;
  sc.vio_gap_until = 13.0;
  sc.vio_restart_at = 13.0;
  sc.vio_restart_stamp_s = 0.0;
  sc.duration = 24.0;
  Record record = Run(sc);
  double position, angle;
  PlacedError(sc, record, 0.0, &position, &angle);
  int placed_early = 0;
  for (const auto &entry : record.placed) {
    placed_early += entry.first >= 13.0 && entry.first < 19.9 ? 1 : 0;
  }
  Check(record.LogCount("VIO stream restarted") == 1 &&
            record.LogLine("VIO stream restarted") ==
                "VIO stream restarted (stamps jumped 3.05 s): placement "
                "dropped",
        "one line says the placement is dropped",
        record.LogLine("VIO stream restarted").c_str());
  Check(placed_early == 0 && !record.StatusAt(15.0).vio_ready &&
            std::isnan(record.StatusAt(15.0).heading_deg) &&
            std::isnan(record.StatusAt(15.0).body_tilt_deg),
        "disarmed: no placed pose of the new stream until it has stood for "
        "7 s",
        Fmt("%d placed poses from 13.0 to 19.9 s", placed_early));
  double t_second = kInf;
  for (const auto &line : record.log) {
    if (line.second.find("VIO ready:") != std::string::npos) {
      t_second = line.first;
    }
  }
  Check(record.LogCount("VIO ready:") == 2 && position < 1e-6 &&
            record.last.vio_ready && t_second > 20.0 && t_second < 20.2 &&
            !record.placed.empty() && record.placed.back().first > 23.9,
        "then it is placed again, on the truth",
        Fmt("every placed pose within %.3g m of the truth, the second "
            "'VIO ready' at %.3f s",
            position, t_second));

  // shadow, armed and flying: no placed VIO for the rest of the flight
  sc.arm = 8.0;
  sc.takeoff = 8.5;
  record = Run(sc);
  PlacedError(sc, record, 0.0, &position, &angle);
  Check(record.LogCount("VIO stream restarted") == 1 && position < 1e-6 &&
            !record.placed.empty() && record.placed.back().first < 10.0 &&
            !record.last.vio_ready && std::isnan(record.last.heading_fit_deg),
        "armed: no placed pose after the restart",
        Fmt("last placed pose stamped %.3f s",
            record.placed.empty() ? 0.0 : record.placed.back().first));

  // on the VIO in flight, the poses keep arriving but their stamps jump by
  // 3 s (the timeout cannot see it)
  sc = Scenario();
  sc.params = VioNav();
  sc.arm = 9.5;
  sc.takeoff = 10.5;
  sc.vio_restart_at = 15.0;
  sc.vio_restart_stamp_s = 3.0;
  sc.duration = 18.0;
  record = Run(sc);
  const double t_back = record.LogTime("pose source: VIO -> MOCAP");
  Check(t_back > 15.05 && t_back < 15.07 &&
            record.last.reason == Reason::kVioTimeout &&
            record.last.reset_counter == 2 &&
            CountSent(record, true, 15.05, kInf) == 0 &&
            CountBadVio(sc, record, 0.0, kInf, 0.3) == 0,
        "on the VIO, stamps jump 3 s without a pause: VIO -> MOCAP at once, "
        "reason vio timeout, no pose of the new stream sent",
        Fmt("at %.3f s: %s", t_back,
            record.LogLine("pose source: VIO -> MOCAP").c_str()));
  int placed_late = 0;
  for (const auto &entry : record.placed) {
    placed_late += entry.first >= 15.0 ? 1 : 0;
  }
  Check(record.LogCount("VIO stream restarted") == 1 && placed_late == 0,
        "and none placed for the rest of the flight",
        Fmt("%d placed poses of the new stream", placed_late));
}

void CheckReasonLines() {
  std::printf("log: why PX4 is not fed the VIO\n");
  auto lines = [](const Record &record) {
    std::string all;
    for (const auto &line : record.log) {
      if (line.second.find("VIO not used") != std::string::npos) {
        all += Fmt("[%.2f s] ", line.first) + line.second + "  ";
      }
    }
    return all;
  };
  // PX4's own position never arrives
  Scenario sc;
  sc.params = VioNav();
  sc.odometry_from = kInf;
  sc.status_from = 1.0;
  sc.arm = 12.0;
  sc.duration = 14.0;
  Record record = Run(sc);
  bool no_source_words = true;
  for (const auto &line : record.log) {
    if (line.second.find("VIO not used") != std::string::npos) {
      no_source_words = no_source_words &&
                        line.second.find("pose source:") == std::string::npos;
    }
  }
  Check(record.LogCount("VIO not used: no vehicle status") == 1 &&
            record.LogCount("VIO not used: vio not ready") == 1 &&
            record.LogCount("VIO not used: no vehicle odometry") == 1 &&
            record.LogCount("VIO not used") == 3 &&
            record.LogTime("VIO not used: no vehicle odometry") > 7.0 &&
            record.LogTime("VIO not used: no vehicle odometry") < 7.2 &&
            no_source_words,
        "no vehicle odometry: one line per reason as it changes, none with "
        "the words 'pose source:'",
        lines(record).c_str());

  // nominal: the reasons before the handover, nothing while on the VIO, the
  // reason of the fallback after it
  sc = Scenario();
  sc.params = VioNav();
  sc.arm = 9.5;
  sc.takeoff = 10.5;
  sc.duration = 18.0;
  sc.px4_error = [](double t) {
    return Eigen::Vector3d(t >= 15.0 ? 0.2 : 0.0, 0.0, 0.0);
  };
  record = Run(sc);
  const double t_vio = record.LogTime("pose source: MOCAP -> VIO");
  const double t_back = record.LogTime("pose source: VIO -> MOCAP");
  int on_vio = 0;
  for (const auto &line : record.log) {
    on_vio += line.second.find("VIO not used") != std::string::npos &&
                      line.first > t_vio && line.first < t_back
                  ? 1
                  : 0;
  }
  Check(record.LogCount("VIO not used: vio not ready") == 1 &&
            record.LogTime("VIO not used: vio not ready") < t_vio &&
            on_vio == 0 &&
            record.LogCount("VIO not used: distance limit") == 1 &&
            record.LogTime("VIO not used: distance limit") >= t_back &&
            record.LogTime("VIO not used: distance limit") < t_back + 0.011 &&
            record.LogCount("VIO not used") == 2,
        "nominal flight with a fallback: 'vio not ready' before the "
        "handover, nothing on the VIO, 'distance limit' after the fallback",
        lines(record).c_str());

  // use_vio false: no such line
  sc = Scenario();
  sc.params.vio_enabled = true;
  sc.params.has_mount = true;
  sc.duration = 9.0;
  record = Run(sc);
  Check(record.LogCount("VIO not used") == 0 &&
            record.last.reason == Reason::kOperator,
        "use_vio false (reason operator): no line");
}

void CheckDefaults() {
  std::printf("defaults\n");
  const Params params;
  Check(params.vio_timeout_s == 0.33 &&
            params.fallback_dist_m == 0.15 && params.fallback_hold_s == 0.10 &&
            params.vio_settle_s == 7.0,
        "vio_timeout_s 0.33; fallback_dist_m 0.15, "
        "fallback_hold_s 0.10, vio_settle_s 7.0 as before");
  // poses 162 ms apart (the longest step of the recorded flight is 173 ms):
  // one lost (324 ms between its neighbours) is ridden through, two (486 ms)
  // are not
  for (const int lost : {1, 2}) {
    VisionPoseCore core(VioNav());
    Px4Sample px4;
    PlacedPose placed;
    for (long ms = 0; ms < 12000; ++ms) {
      const double t = ms * 1e-3;
      const double now = kSteady0 + t;
      if (ms % 500 == 0) {
        core.HandleVehicleStatus(ms >= 9000, now);
      }
      if (ms % 10 == 5) {
        core.HandleVehicleOdometry(Stamp(t - 0.005),
                                   Eigen::Vector3d(0.0, 0.0, 0.0), now);
      }
      if (ms % 4 == 0) {
        vision_pose::MocapSample sample;
        sample.stamp_ns = Stamp(t - 0.007);
        sample.tracking_valid = true;
        core.HandleMocap(sample, now, &px4);
      }
      // every 162 ms from 9 s on, 20 Hz before
      const bool pose_due =
          ms < 9000 ? ms % 50 == 0 && ms >= 100 : (ms - 9000) % 162 == 0;
      const long index = (ms - 9000) / 162;
      if (pose_due && !(ms >= 9000 && index >= 10 && index < 10 + lost)) {
        vision_pose::VioPose pose;
        pose.stamp_ns = Stamp(t - 0.058);
        core.HandleVio(pose, now, 0.058, &px4, &placed);
      }
      if (ms % 10 == 0) {
        core.HandleTimer(now);
      }
    }
    const Status status = core.GetStatus(kSteady0 + 12.0);
    Check(lost == 1 ? status.source == Source::kVio && status.fallbacks == 0
                    : status.source == Source::kMocap &&
                          status.reason == Reason::kVioTimeout,
          lost == 1 ? "poses 162 ms apart, one lost: PX4 stays on the VIO"
                    : "poses 162 ms apart, two lost in a row: VIO timeout");
  }
}

}  // namespace

int main() {
  CheckMocapConversion();
  CheckPlacementExact();
  CheckTilt();
  CheckVioNav();
  CheckArmedBeforeReady();
  CheckDistanceFallback();
  CheckVioTimeout();
  CheckNeverVio();
  CheckStartPose();
  CheckHeadingTrim();
  CheckShadowCount();
  CheckBadVio();
  CheckHeldUp();
  CheckStaleStatus();
  CheckOdometryNeeded();
  CheckDistanceRuleRates();
  CheckBadStamp();
  CheckVioRestart();
  CheckFallbackKeepsBodyRotation();
  CheckLateMocap();
  CheckPlacedAgain();
  CheckPlacedAgainWithoutReadyEdge();
  CheckVioJump();
  CheckNoComparison();
  CheckHeldUpPeriod();
  CheckHeldUpAgainAndAgain();
  CheckHeldUpThenOldPoses();
  CheckVioRestartForward();
  CheckReasonLines();
  CheckDefaults();
  std::printf("%d checks, %d failed\n", g_checks, g_failed);
  return g_failed == 0 ? 0 : 1;
}
