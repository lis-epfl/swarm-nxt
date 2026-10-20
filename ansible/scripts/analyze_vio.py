#!/usr/bin/env python3
"""Onboard VIO tables of the post-flight analysis.

For every drone of a consolidated log folder that ran the estimator (enable_vio or vio_nav):
how the estimator kept up, how accurate it was against the mocap, what vision_pose_node did
with it, and, with two or more such drones, how well their VIOs agree on where they are
relative to each other. Called by analyze_logs.py; also runs alone:

    python3 analyze_vio.py --log-dir ~/data/consolidated_logs/latest

It reads, per drone: bag/*.mcap (the estimator's poses, the stream placed in the room by
vision_pose_node, its status, the mocap, PX4's land detector and arming state),
vio/run.json (flags and deployed mount, written by the launch file), vio/latency.txt
(written by the estimator while it runs) and the launch log (also for an estimator that died:
when, and the assertion or crash signal it printed). Nothing is printed when no
drone has VIO data. All numbers are over the flight, takeoff to landing by PX4's land
detector; a recording without a flight gives the compute columns over the whole recording.
"""
import argparse
import glob
import json
import os
import re
import sys
import time

import numpy as np

MOCAP_TOPIC = "/optitrack_multiplexer_node/rigid_body/{ns}"
MOCAP_EVERY = 4          # 240 Hz at the source; every 4th sample is read
REASONS = {1: "not ready at arming", 2: "no pose", 3: "distance", 4: "no mount",
           5: "no vehicle status", 6: "flag off", 7: "not ready", 8: "no PX4 position"}


# ---------------------------------------------------------------- reading
def _stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def read_bag(drone_dir, ns):
    """The topics the tables need, as arrays. Times in seconds (epoch)."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    mocap = MOCAP_TOPIC.format(ns=ns)
    odom, status = f"/{ns}/vio/odometry", f"/{ns}/vio/status"
    topics = ["/vio/poseimu", odom, status, mocap, "/fmu/out/vehicle_land_detected",
              "/fmu/out/vehicle_status_v1", "/fmu/out/vehicle_status"]
    out = {k: [] for k in ("vio", "odom", "status", "mocap", "land", "arm")}
    factory, decoders, n_mocap = DecoderFactory(), {}, 0
    for path in sorted(glob.glob(os.path.join(drone_dir, "bag", "*.mcap"))):
        with open(path, "rb") as f:
            for schema, channel, message in make_reader(f).iter_messages(topics=topics):
                topic, t = channel.topic, message.log_time * 1e-9
                if topic == mocap:
                    n_mocap += 1
                    if n_mocap % MOCAP_EVERY:
                        continue
                dec = decoders.get(channel.id)
                if dec is None:
                    dec = decoders[channel.id] = factory.decoder_for(channel.message_encoding, schema)
                m = dec(message.data)
                if topic == "/vio/poseimu":
                    p, q = m.pose.pose.position, m.pose.pose.orientation
                    out["vio"].append((_stamp(m.header), t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                elif topic == odom:
                    p = m.pose.pose.position
                    out["odom"].append((_stamp(m.header), p.x, p.y, p.z))
                elif topic == status:
                    out["status"].append((t, m.source, m.vio_ready, m.reason, m.fallbacks,
                                          m.dist_exceed_count, m.px4_mocap_dist_m, m.fallback_latched))
                elif topic == mocap:
                    s = m.stamp.sec + m.stamp.nanosec * 1e-9
                    # a stamp that goes back is a sample out of order, left out as vision_pose_node
                    # does (nxt6 20 Oct 12:16: one stamped 0.1 s early read as a 14.5 cm error)
                    if not m.rigid_body.tracking_valid or (out["mocap"] and s <= out["mocap"][-1][0]):
                        continue
                    p, q = m.rigid_body.pose.position, m.rigid_body.pose.orientation
                    out["mocap"].append((s, p.x, p.y, p.z, q.q_x, q.q_y, q.q_z, q.q_w))
                elif topic == "/fmu/out/vehicle_land_detected":
                    out["land"].append((t, float(m.landed)))
                else:
                    out["arm"].append((t, float(m.arming_state == 2)))
    # Fixed widths: a topic without a message (the recorder can miss a PX4 topic, see
    # drone_launch.py.j2) gives an empty table instead of a ValueError that loses the drone.
    widths = {"vio": 9, "odom": 4, "status": 8, "mocap": 8, "land": 2, "arm": 2}
    return {k: np.array(v, float).reshape(len(v), widths[k]) for k, v in out.items()}


SIGNALS = {-6: "SIGABRT", -7: "SIGBUS", -9: "SIGKILL", -11: "SIGSEGV", -15: "SIGTERM"}


def read_latency(path):
    """vio/latency.txt as an array, None when it holds no complete line. An estimator that did
    not exit cleanly (crash, kill, power cut) leaves its buffered last line half written: dropped."""
    with open(path, errors="replace") as f:
        lines = f.read().splitlines(keepends=True)
    if lines and not lines[-1].endswith("\n"):
        lines.pop()
    return np.atleast_2d(np.loadtxt(lines)) if lines else None


def read_launch_log(drone_dir):
    """What the estimator itself logged: its settings and its counters, and its death when it died."""
    info = {}
    logs = sorted(glob.glob(os.path.join(drone_dir, "*", "launch.log")))
    if not logs:
        return info
    live = None
    # The process tag of the estimator ([env-15]) is known only from the launch's death line, which
    # comes last: keep the last assertion and crash signal of every tag until then.
    asserted, crashed, stop_t = {}, {}, None
    with open(logs[-1], errors="replace") as f:
        for line in f:
            m = re.match(r"(\d+\.\d+) \[([\w.-]+)\] ", line)
            if m and "Assertion `" in line:
                a = re.search(r"([\w.]+:\d+): .*(Assertion `.*' failed)", line)
                asserted[m.group(2)] = (float(m.group(1)), "%s: %s" % a.groups() if a else line.split("] ", 1)[1].strip())
            elif m and "[crash]: signal" in line:
                crashed[m.group(2)] = (float(m.group(1)), re.search(r"\[crash\]: signal \d+", line).group(0))
            if "[launch]: user interrupted" in line and stop_t is None:
                stop_t = float(line.split(" ", 1)[0])
            d = re.match(r"(\d+\.\d+) \[ERROR\] \[([\w.-]+)\]: process has died \[pid \d+, exit code (-?\d+), "
                         r"cmd '[^']*run_online_msckf", line)
            if d:
                tag, code = d.group(2), int(d.group(3))
                t, why = asserted.get(tag) or crashed.get(tag) or (float(d.group(1)), "")
                info["died"] = {"t": t, "code": code, "why": why, "stopping": stop_t is not None and stop_t <= t}
            if "[xcam-cfg]" in line:
                m = re.search(r"\[xcam-cfg\] on=(\d)", line)
                if m:
                    info["matching"] = "on" if m.group(1) == "1" else "off"
            elif "[nonradial]" in line and "lens" not in info:
                info["lens"] = "fixed"
                if "CALIBRATION STATES" in line:
                    info["lens"] = "states"
            elif "[nonradial]" in line and "CALIBRATION STATES" in line:
                info["lens"] = "states"
            elif "[online-live]" in line:
                live = line
            elif "[online]: sets_closed=" in line:
                m = re.search(r"partial=(\d+).*late_drop=(\d+)", line)
                if m:
                    info["partial"], info["late"] = int(m.group(1)), int(m.group(2))
            elif "[online]: sets_fed=" in line:
                info["fed"] = int(re.search(r"sets_fed=(\d+)", line).group(1))
    if "fed" not in info and live:      # estimator did not exit cleanly: its last 5 s report
        m = re.search(r"fed=(\d+)", live)
        if m:
            info["fed"] = int(m.group(1))
    return info


# ---------------------------------------------------------------- geometry
def quat_to_R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rigid_fit(src, dst, with_scale=False):
    """Rotation, translation (and scale) that map src onto dst in the least-squares sense."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    sc, dc = src - mu_s, dst - mu_d
    U, S, Vt = np.linalg.svd(dc.T @ sc / len(src))
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    s = float(np.trace(np.diag(S) @ D) / (sc ** 2).sum() * len(src)) if with_scale else 1.0
    return R, mu_d - s * R @ mu_s, s


def rms(v):
    return float(np.sqrt(np.mean(np.sum(np.atleast_2d(v) ** 2, axis=1)))) if len(v) else float("nan")


def interp(t, tt, xyz):
    return np.column_stack([np.interp(t, tt, xyz[:, c]) for c in range(xyz.shape[1])])


def flight_window(bag):
    """(takeoff, landing) in seconds, or None. PX4's land detector; without it, the mocap."""
    land = bag["land"]
    if len(land) > 1:
        up = np.where((land[:-1, 1] > 0.5) & (land[1:, 1] < 0.5))[0]
        down = np.where((land[:-1, 1] < 0.5) & (land[1:, 1] > 0.5))[0]
        if len(up):
            t0 = land[up[0] + 1, 0]
            later = down[down > up[0]]
            return t0, (land[later[-1] + 1, 0] if len(later) else land[-1, 0])
        return None
    mc = bag["mocap"]
    if len(mc) > 10:
        high = np.where(mc[:, 3] > mc[0, 3] + 0.10)[0]
        if len(high):
            return mc[high[0], 0], mc[high[-1], 0]
    return None


# ---------------------------------------------------------------- one drone
def analyze_drone(drone_dir):
    """-> dict of results for one drone, or None when it did not run the VIO."""
    ns = os.path.basename(os.path.normpath(drone_dir))
    run_path = os.path.join(drone_dir, "vio", "run.json")
    if not os.path.isfile(run_path):
        return None
    with open(run_path) as f:
        run = json.load(f)
    if not (run.get("enable_vio") or run.get("vio_nav")):
        return None
    r = {"drone": ns, "mode": "nav" if run.get("vio_nav") else "shadow", "note": ""}
    if not run.get("started", False):
        r["note"] = "estimator not started: " + (run.get("problem") or "no reason given")
        return r
    r.update(read_launch_log(drone_dir))
    r.setdefault("matching", "on" if run.get("vio_matching") else "off")
    bag = read_bag(drone_dir, ns)
    vio, mc = bag["vio"], bag["mocap"]
    # The estimator gives the IMU's position, the mocap that of the rigid body's origin. With the
    # calibration's lever arm (mount.json, lever_arm_imu_m: the IMU's position from that origin, IMU
    # frame) the estimate is moved back onto the origin, as vision_pose_node does since it has one.
    lever = (run.get("mount") or {}).get("lever_arm_imu_m")
    if lever is not None and len(vio):
        vio = vio.copy()
        vio[:, 2:5] -= np.einsum("nij,j->ni", np.array([quat_to_R(q) for q in vio[:, 5:9]]),
                                 np.asarray(lever, float))
    if len(vio) < 20:
        r["note"] = "the bag has %d estimator poses" % len(vio)
        return r
    win = flight_window(bag)
    r["flight"] = win is not None
    t0, t1 = win if win else (vio[0, 0], vio[-1, 0])
    r["t0"], r["t1"] = t0, t1

    # --- compute: how the estimator kept up
    lat_path = os.path.join(drone_dir, "vio", "latency.txt")
    lat = None
    if os.path.isfile(lat_path) and os.path.getsize(lat_path):
        try:
            lat = read_latency(lat_path)
            if lat is None:
                r["lat_problem"] = "vio/latency.txt has no complete line"
        except ValueError as e:             # the feed and queue columns only, not the whole drone
            r["lat_problem"] = "vio/latency.txt not read (%s)" % str(e)[:80]
    if lat is not None:
        lat = lat[(lat[:, 0] >= t0) & (lat[:, 0] <= t1)]
        if len(lat):
            r["feed"] = (lat[:, 2].mean(), np.percentile(lat[:, 2], 99), lat[:, 2].max())
            r["queue"] = int(lat[:, 3].max())
    fl = (vio[:, 0] >= t0) & (vio[:, 0] <= t1)
    if fl.sum() > 2:
        age = (vio[fl, 1] - vio[fl, 0]) * 1e3
        r["age"] = tuple(np.percentile(age, [50, 90, 100]))
        r["rate"] = fl.sum() / (t1 - t0)
        r["gap"] = float(np.diff(vio[fl, 1]).max())

    # --- navigation: what vision_pose_node did
    st = bag["status"]
    r["first_pose"] = vio[0, 0] - t0
    if len(st):
        ready = st[st[:, 2] > 0.5]
        if len(ready):
            r["placed"] = ready[0, 0] - t0
        sf = st[(st[:, 0] >= t0) & (st[:, 0] <= t1)]
        if len(sf):
            r["on_vio"] = 100.0 * float(np.mean(sf[:, 1] == 2))
            r["fallbacks"] = int(sf[-1, 4])
            r["latched_reason"] = REASONS.get(int(sf[-1, 3]), "") if sf[-1, 7] > 0.5 else ""
            r["exceed"] = int(sf[-1, 5] - sf[0, 5])
            d = sf[np.isfinite(sf[:, 6]), 6]
            if len(d):
                r["px4"] = (float(np.sqrt(np.mean(d ** 2))) * 100, d.max() * 100)
    r["limit_cm"] = 100.0 * float(run.get("vio_fallback_dist_m") or float("nan"))

    # --- accuracy against the mocap
    if not win or len(mc) < 50 or fl.sum() < 20:
        if win:
            r["note"] = "no mocap in the bag" if len(mc) < 50 else "fewer than 20 poses in flight"
        return r
    tv, pv = vio[fl, 0], vio[fl, 2:5]
    pg = interp(tv, mc[:, 0], mc[:, 1:4])
    R_fit, t_fit, _ = rigid_fit(pv, pg)
    r["ate"] = rms(pv @ R_fit.T + t_fit - pg) * 100
    r["scale"] = (rigid_fit(pv, pg, with_scale=True)[2] - 1.0) * 100

    # pinned on the ground: the last pose before arming (before the takeoff if PX4's arming
    # state is not in the bag), rotation from the mocap attitude there
    arm = bag["arm"]
    armed = arm[arm[:, 1] > 0.5, 0] if len(arm) else []
    t_pin = armed[0] if len(armed) and armed[0] < t0 else t0 - 1.0
    before = np.where(vio[:, 0] <= t_pin)[0]
    a = before[-1] if len(before) else 0
    g = int(np.clip(np.searchsorted(mc[:, 0], vio[a, 0]), 0, len(mc) - 1))
    R_anchor = quat_to_R(mc[g, 4:8]) @ quat_to_R(vio[a, 5:9]).T
    pin_vio, pin_room = vio[a, 2:5], interp(vio[a:a + 1, 0], mc[:, 0], mc[:, 1:4])[0]

    def pinned(R):
        return rms((pv - pin_vio) @ R.T + pin_room - pg) * 100

    r["refit"] = pinned(R_fit)
    M_here = R_fit @ R_anchor.T
    mount = run.get("mount") or {}
    if mount.get("M") is not None and not run.get("mount_problem"):
        r["noumcal"] = pinned(np.array(mount["M"], float) @ R_anchor)
        yaw_here = np.degrees(np.arctan2(M_here[1, 0], M_here[0, 0]))
        r["dyaw"] = (yaw_here - float(mount.get("mount_yaw_deg", 0.0)) + 180.0) % 360.0 - 180.0

    # the stream vision_pose_node placed in the room: what PX4 flies on with vio_nav
    od = bag["odom"]
    od = od[(od[:, 0] >= t0) & (od[:, 0] <= t1)] if len(od) else od
    if len(od) > 10:
        e = np.linalg.norm(od[:, 1:4] - interp(od[:, 0], mc[:, 0], mc[:, 1:4]), axis=1)
        r["flown"] = (float(np.sqrt(np.mean(e ** 2))) * 100, e.max() * 100)
        r["placed_xyz"], r["mocap_xyz"] = od, mc[:, :4]
    return r


# ---------------------------------------------------------------- printing
def _f(v, fmt="%.1f"):
    return "-" if v is None or (isinstance(v, float) and not np.isfinite(v)) else fmt % v


def _trio(v, fmt="%.0f"):
    return "-" if v is None else "/".join(_f(float(x), fmt) for x in v)


def death_note(r):
    """One line on an estimator that died, placed against the flight; None when it did not."""
    d = r.get("died")
    if not d:
        return None
    t0, t1 = r.get("t0"), r.get("t1")
    when = time.strftime("%H:%M:%S", time.localtime(d["t"])) + ("%.1f" % (d["t"] % 1.0))[1:]
    if not r.get("flight") or t0 is None:
        where = "at " + when
    elif d["t"] < t0:
        where = "%.1f s before the takeoff (%s)" % (t0 - d["t"], when)
    elif d["t"] <= t1:
        where = "IN FLIGHT, %.1f s after the takeoff (%s)" % (d["t"] - t0, when)
    else:
        where = "%.1f s after the landing (%s)" % (d["t"] - t1, when)
    code = "exit code %d%s" % (d["code"], " " + SIGNALS[d["code"]] if d["code"] in SIGNALS else "")
    in_flight = bool(r.get("flight")) and t0 is not None and t0 <= d["t"] <= t1
    return "the estimator died %s%s, %s%s%s" % (where, " while the stack was stopping" if d["stopping"] else "",
                                               code, ": " + d["why"] if d["why"] else "",
                                               "; the numbers above stop there" if in_flight else "")


def table(title, header, rows, notes=()):
    width = [max(len(str(c)) for c in col) for col in zip(*([header] + rows))]
    line = " | ".join("%-*s" % (w, h) if i == 0 else "%*s" % (w, h)
                      for i, (w, h) in enumerate(zip(width, header)))
    print("\n" + title)
    print("-" * len(line))
    print(line)
    print("-" * len(line))
    for row in rows:
        print(" | ".join("%-*s" % (w, c) if i == 0 else "%*s" % (w, c)
                         for i, (w, c) in enumerate(zip(width, row))))
    for n in notes:
        print("  * " + n)


def print_tables(log_dir):
    """Print the VIO tables of one consolidated log folder. Returns the number of VIO drones."""
    res = []
    for d in sorted(glob.glob(os.path.join(str(log_dir), "*", ""))):
        if os.path.basename(os.path.normpath(d)) == "host":
            continue
        try:
            one = analyze_drone(d)
        except Exception as e:                     # one drone's files must not hide the others
            one = {"drone": os.path.basename(os.path.normpath(d)), "mode": "?",
                   "note": "not analysed (%s: %s)" % (type(e).__name__, e)}
        if one:
            res.append(one)
    if not res:
        return 0

    g = lambda r, k: r.get(k)
    where = "in flight" if all(r.get("flight") for r in res if "t0" in r) else "whole recording where there was no flight"
    table("VIO COMPUTE (%s)" % where,
          ["Drone", "mode", "matching", "lens", "feed ms mean/p99/max", "queue max",
           "pose age ms p50/p90/max", "rate Hz", "gap s", "sets fed/partial/late"],
          [[r["drone"], r["mode"], r.get("matching", "-"), r.get("lens", "-"), _trio(g(r, "feed")),
            _f(g(r, "queue"), "%d"), _trio(g(r, "age")), _f(g(r, "rate")), _f(g(r, "gap"), "%.2f"),
            "/".join(_f(g(r, k), "%d") for k in ("fed", "partial", "late"))] for r in res],
          ["feed = time the estimator spends on one image set (a new set comes every 33 ms); queue = most sets waiting",
           "pose age = from the instant a pose describes to its arrival in the bag; PX4 drops a pose older than about 190 ms",
           "rate, gap = poses per second and the longest wait between two poses; sets = image sets over the whole recording"])

    table("VIO ACCURACY vs mocap (cm, takeoff to landing)",
          ["Drone", "ATE", "scale %", "NOUMCAL", "refit", "flown rms/max", "dyaw deg"],
          [[r["drone"], _f(g(r, "ate")), _f(g(r, "scale"), "%+.1f"), _f(g(r, "noumcal")), _f(g(r, "refit")),
            _trio(g(r, "flown"), "%.1f"), _f(g(r, "dyaw"), "%+.2f")] for r in res],
          ["ATE = error after the best rigid fit; scale = what a fit with scale would change",
           "NOUMCAL = pinned at the pose at arming, with the deployed mount; refit = with the mount solved on this flight",
           "ATE, NOUMCAL, refit: the estimate is moved onto the rigid body's origin by the lever arm of mount.json, when it has one",
           "flown = the stream vision_pose_node placed in the room (what PX4 flies on with vio_nav), nothing fitted",
           "dyaw = this flight's heading offset minus the deployed one"])

    table("VIO NAVIGATION",
          ["Drone", "mode", "first pose s", "placed s", "on VIO %", "back to mocap", "over limit", "PX4-mocap cm rms/max"],
          [[r["drone"], r["mode"], _f(g(r, "first_pose"), "%+.1f"), _f(g(r, "placed"), "%+.1f"), _f(g(r, "on_vio"), "%.0f"),
            ("-" if g(r, "fallbacks") is None else "%d%s" % (r["fallbacks"], " (%s)" % r["latched_reason"] if r.get("latched_reason") else "")),
            ("-" if g(r, "exceed") is None else "%d (%s cm)" % (r["exceed"], _f(g(r, "limit_cm"), "%.0f"))),
            _trio(g(r, "px4"), "%.1f")] for r in res],
          ["first pose, placed = seconds from the takeoff (negative: before it) to the estimator's first pose and to its placement in the room",
           "on VIO = share of the flight PX4 was fed the VIO; back to mocap = fallbacks in the flight",
           "over limit = times the fallback distance was crossed (PX4 to mocap on the VIO, placed VIO to mocap otherwise)"])

    pairs = [r for r in res if "placed_xyz" in r]
    rows = []
    for i in range(len(pairs)):
        for j in range(i + 1, len(pairs)):
            a, b = pairs[i], pairs[j]
            lo, hi = max(a["t0"], b["t0"]), min(a["t1"], b["t1"])
            if hi - lo < 2.0:
                continue
            t = np.arange(lo, hi, 0.1)
            rel_v = interp(t, a["placed_xyz"][:, 0], a["placed_xyz"][:, 1:4]) - interp(t, b["placed_xyz"][:, 0], b["placed_xyz"][:, 1:4])
            rel_m = interp(t, a["mocap_xyz"][:, 0], a["mocap_xyz"][:, 1:4]) - interp(t, b["mocap_xyz"][:, 0], b["mocap_xyz"][:, 1:4])
            ev = np.linalg.norm(rel_v - rel_m, axis=1) * 100
            dv, dm = np.linalg.norm(rel_v, axis=1), np.linalg.norm(rel_m, axis=1)
            ed = np.abs(dv - dm) * 100
            rows.append(["%s-%s" % (a["drone"], b["drone"]),
                         "%.1f/%.1f" % (np.sqrt(np.mean(ev ** 2)), ev.max()),
                         "%.1f/%.1f" % (np.sqrt(np.mean(ed ** 2)), ed.max()),
                         "%.2f / %.2f" % (dm.min(), dv.min())])
    if rows:
        table("VIO INTER-DRONE (while both fly)",
              ["Pair", "relative position cm rms/max", "distance cm rms/max", "min distance m mocap / VIO"], rows,
              ["error of the vector between two drones as their placed VIOs see it, and of its length"])

    for r in res:
        for n in (death_note(r), r.get("note"), r.get("lat_problem")):
            if n:
                print("  ! %s: %s" % (r["drone"], n))
    return len(res)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Onboard VIO tables of a consolidated log folder")
    ap.add_argument("--log-dir", required=True, help="Path to consolidated logs")
    if print_tables(ap.parse_args().log_dir) == 0:
        print("No drone in this folder ran the onboard VIO.")
        sys.exit(0)
