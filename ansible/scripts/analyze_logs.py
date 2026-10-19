#!/usr/bin/env python3
import argparse
import sys
import time
import pandas as pd
import numpy as np
from pathlib import Path
from scipy.interpolate import interp1d

# --- HELPER: Parse single-line comma logs ---
def load_csv_array(filepath):
    try:
        with open(filepath, 'r') as f:
            line = f.readline()
            if not line:
                return np.array([])
            values = [float(x) for x in line.strip().strip(',').split(',')]
            return np.array(values)
    except Exception as e:
        return np.array([])

# --- CORE: Analyze individual drone stats ---
def analyze_drone(drone_path, offboard=None):
    """offboard: [(enter, exit)] of PX4's offboard mode from the bag, or None (no vehicle status)"""
    stats = {}
    drone_name = drone_path.name

    # 1. MPC Logs (Tracking)
    mpc_states_file = drone_path / "mpc_logs" / "states.csv"
    mpc_comp_file = drone_path / "mpc_logs" / "computation_times.csv"
    trajectory_df = None

    if mpc_states_file.exists():
        try:
            df = pd.read_csv(mpc_states_file)

            # Safety Check: Use FULL trajectory (including takeoff)
            trajectory_df = df[['timestamp', 'x', 'y', 'z']].sort_values('timestamp').copy()

            # Stats Filter: Wait for error < 10cm (ignore takeoff spike)
            stable_indices = df.index[df['error_norm'] < 0.1].tolist()
            if stable_indices:
                df_stats = df.iloc[stable_indices[0]:]
            else:
                df_stats = df
            # PX4 follows the MPC only in offboard mode: after a failsafe, a land command or a
            # pilot taking over, the MPC keeps logging against a reference nobody flies
            if offboard is not None:
                t = df_stats['timestamp'].to_numpy()
                inside = np.zeros(len(t), dtype=bool)
                for a, b in offboard:
                    inside |= (t >= a) & (t < b)
                df_stats = df_stats[inside]
            if df_stats.empty:
                stats['tracking_note'] = 'no MPC state while PX4 was in offboard mode'
            else:
                stats['error_x_mean'] = df_stats['error_x'].abs().mean()
                stats['error_x_max'] = df_stats['error_x'].abs().max()
                stats['error_y_mean'] = df_stats['error_y'].abs().mean()
                stats['error_y_max'] = df_stats['error_y'].abs().max()
                stats['error_z_mean'] = df_stats['error_z'].abs().mean()
                stats['error_z_max'] = df_stats['error_z'].abs().max()
                stats['error_norm_mean'] = df_stats['error_norm'].mean()
                stats['error_norm_max'] = df_stats['error_norm'].max()
        except Exception:
            pass

    if mpc_comp_file.exists():
        try:
            df = pd.read_csv(mpc_comp_file)
            stats['mpc_time_mean'] = df['computation_time_ms'].mean()
            stats['mpc_time_max'] = df['computation_time_ms'].max()
        except Exception:
            pass

    # 2. Depth Logs
    depth_log_file = drone_path / "depth_logs" / "depth_computation_times.csv"
    if depth_log_file.exists():
        try:
            df = pd.read_csv(depth_log_file)
            rect_cols = [c for c in df.columns if '_rect_ms' in c]
            infer_cols = [c for c in df.columns if '_infer_ms' in c]
            cloud_cols = [c for c in df.columns if '_cloud_ms' in c and c != 'max_cloud_ms']

            # For PARALLEL execution, take MAX across pairs (not sum)
            # This correctly represents wall-clock time when pairs run concurrently
            df['max_rect'] = df[rect_cols].max(axis=1)
            df['max_infer'] = df[infer_cols].max(axis=1)

            stats['depth_rect_mean'] = df['max_rect'].mean()
            stats['depth_rect_max'] = df['max_rect'].max()

            # Use inference_wall_ms which captures actual wall time for the
            # inference phase (includes preprocess + parallel rect + parallel infer)
            if 'inference_wall_ms' in df.columns:
                stats['depth_infer_mean'] = df['inference_wall_ms'].mean()
                stats['depth_infer_max'] = df['inference_wall_ms'].max()
            else:
                stats['depth_infer_mean'] = df['max_infer'].mean()
                stats['depth_infer_max'] = df['max_infer'].max()

            # NEW: Use max_cloud_ms if available (pre-computed in C++)
            if 'max_cloud_ms' in df.columns:
                stats['depth_cloud_mean'] = df['max_cloud_ms'].mean()
                stats['depth_cloud_max'] = df['max_cloud_ms'].max()
            else:
                df['max_cloud'] = df[cloud_cols].max(axis=1)
                stats['depth_cloud_mean'] = df['max_cloud'].mean()
                stats['depth_cloud_max'] = df['max_cloud'].max()

            # NEW: Combined pointcloud publish time
            if 'combined_publish_ms' in df.columns:
                stats['depth_comb_mean'] = df['combined_publish_ms'].mean()
                stats['depth_comb_max'] = df['combined_publish_ms'].max()

            # NEW: Total wall time (end-to-end latency)
            if 'total_wall_ms' in df.columns:
                stats['depth_total_mean'] = df['total_wall_ms'].mean()
                stats['depth_total_max'] = df['total_wall_ms'].max()
            elif 'inference_wall_ms' in df.columns:
                # Fallback: estimate total as inference_wall + max_cloud
                if 'max_cloud_ms' in df.columns:
                    df['total_approx'] = df['inference_wall_ms'] + df['max_cloud_ms']
                else:
                    df['total_approx'] = df['inference_wall_ms'] + df[cloud_cols].max(axis=1)
                stats['depth_total_mean'] = df['total_approx'].mean()
                stats['depth_total_max'] = df['total_approx'].max()

        except Exception as e:
            pass

    # 3. Planner & Mapping Logs
    planner_dir = drone_path / "planner_logs"
    if planner_dir.exists():
        # A. Planning Pipeline
        plan_files = {
            'plan_path': 'comp_time_path_*.csv',
            'plan_sc': 'comp_time_sc_*.csv',
            'plan_tasc': 'comp_time_tasc_*.csv',
            'plan_opt': 'comp_time_opt_*.csv',
            'plan_total': 'comp_time_tot_wall_*.csv'
        }
        for metric, pattern in plan_files.items():
            files = list(planner_dir.glob(pattern))
            if files:
                data = load_csv_array(files[0])
                if data.size > 0:
                    stats[f'{metric}_mean'] = np.mean(data)
                    stats[f'{metric}_max'] = np.max(data)

        # B. Mapping Pipeline (with NEW detailed timing)
        map_files = {
            # NEW: Detailed vision pipeline timing
            'map_pcl': 'comp_time_pcl_transform_*.csv',
            'map_count': 'comp_time_point_counting_*.csv',
            'map_thresh': 'comp_time_threshold_*.csv',
            # Existing timing
            'map_raycast': 'comp_time_raycast_*.csv',
            'map_merge': 'comp_time_merge_*.csv',
            'map_uncertain': 'comp_time_uncertain_*.csv',
            'map_inflate': 'comp_time_inflate_*.csv',
            'map_pot': 'comp_time_potential_field_*.csv',
            'map_dyn': 'comp_time_dyn_obst_*.csv',
            'map_total': 'comp_time_total_*.csv'
        }
        for metric, pattern in map_files.items():
            files = list(planner_dir.glob(pattern))
            if files:
                data = load_csv_array(files[0])
                if data.size > 0:
                    stats[f'{metric}_mean'] = np.mean(data)
                    stats[f'{metric}_max'] = np.max(data)

        # Calculate "Other" time (should now be minimal with full instrumentation)
        timed_keys = ['map_pcl', 'map_count', 'map_thresh',
                      'map_raycast', 'map_merge', 'map_uncertain',
                      'map_inflate', 'map_pot', 'map_dyn']
        timed_sum_mean = sum(stats.get(f'{k}_mean', 0) for k in timed_keys)
        timed_sum_max = sum(stats.get(f'{k}_max', 0) for k in timed_keys)

        if 'map_total_mean' in stats:
            stats['map_other_mean'] = max(0, stats['map_total_mean'] - timed_sum_mean)
            stats['map_other_max'] = max(0, stats['map_total_max'] - timed_sum_max)

    return stats, trajectory_df

# --- FLIGHT TIME AND HOW EACH FLIGHT ENDED: PX4's topics in the drone's bag ---
OFFBOARD = 14
NAV_NAMES = {0: "Manual", 1: "Altitude", 2: "Position", 3: "Mission", 4: "Hold", 5: "Return",
             6: "Position slow", 10: "Acro", 12: "Descend", 13: "Termination", 14: "Offboard",
             15: "Stabilized", 17: "Takeoff", 18: "Land", 20: "Precision land", 21: "Orbit"}
PILOT_MODES = {0, 1, 2, 6, 10, 15}
SET_NAV_STATE = 100001      # VehicleCommand VEHICLE_CMD_SET_NAV_STATE: what drone_state_manager sends


def read_flight_bag(drone_path):
    """PX4's land detector, mode and failsafe flag, battery warning and the mode commands sent to it,
    as {name: [(epoch s, ...)]}; None without a bag. A message that does not decode is skipped."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    paths = sorted((drone_path / "bag").glob("*.mcap"))
    if not paths:
        return None
    out = {"land": [], "status": [], "battery": [], "command": []}
    topics = ["/fmu/out/vehicle_land_detected", "/fmu/out/vehicle_status_v1", "/fmu/out/vehicle_status",
              "/fmu/out/battery_status", "/fmu/in/vehicle_command"]
    factory, decoders = DecoderFactory(), {}
    for path in paths:
        with open(path, "rb") as f:
            for schema, channel, message in make_reader(f).iter_messages(topics=topics):
                dec = decoders.get(channel.id)
                if dec is None:
                    dec = decoders[channel.id] = factory.decoder_for(channel.message_encoding, schema)
                try:
                    m = dec(message.data)
                except Exception:
                    continue
                t, topic = message.log_time * 1e-9, channel.topic
                if topic.endswith("land_detected"):
                    out["land"].append((t, bool(m.landed)))
                elif "vehicle_status" in topic:
                    out["status"].append((t, int(m.nav_state), bool(m.failsafe)))
                elif topic.endswith("battery_status"):
                    out["battery"].append((t, int(m.warning)))
                else:
                    out["command"].append((t, int(m.command), float(m.param1)))
    return out


def flight_segments(bag):
    """[(takeoff, landing)] in epoch seconds, one per stretch in the air; [] without a takeoff, None
    without a land detector. A recording that ends in the air ends the last stretch."""
    if not bag or not bag["land"]:
        return None
    segments, up = [], None
    for t, landed in bag["land"]:
        if not landed and up is None:
            up = t
        elif landed and up is not None:
            segments.append((up, t))
            up = None
    if up is not None:
        segments.append((up, bag["land"][-1][0]))
    return segments


def offboard_intervals(bag):
    """[(enter, exit)] of PX4's offboard mode; None without a vehicle status in the bag."""
    if not bag or not bag["status"]:
        return None
    out, start = [], None
    for t, nav, _ in bag["status"]:
        if nav == OFFBOARD and start is None:
            start = t
        elif nav != OFFBOARD and start is not None:
            out.append((start, t))
            start = None
    if start is not None:
        out.append((start, float("inf")))
    return out


def how_it_ended(bag, takeoff, landing):
    """How PX4 left offboard mode in this flight: a failsafe, a mode commanded by the stack, a pilot."""
    st = [r for r in bag["status"] if r[0] <= landing]
    if not st:
        return "no vehicle status in the bag"
    for (t0, nav0, _), (t, nav, failsafe) in zip(st, st[1:]):
        if t < takeoff or nav0 != OFFBOARD or nav == OFFBOARD:
            continue
        name, when = NAV_NAMES.get(nav, "nav_state %d" % nav), "at +%.1f s" % (t - takeoff)
        if failsafe or any(f for tt, _, f in bag["status"] if t <= tt <= t + 0.5):
            low = any(w > 0 for tt, w in bag["battery"] if t - 3.0 <= tt <= t + 1.0)
            return "FAILSAFE: %s %s%s" % (name, when, ", battery warning" if low else "")
        if any(c == SET_NAV_STATE and int(p) == nav for tt, c, p in bag["command"] if t - 1.0 <= tt <= t + 0.1):
            return "%s commanded %s" % (name, when)
        if nav in PILOT_MODES:
            return "pilot took over (%s) %s" % (name, when)
        return "switched to %s %s" % (name, when)
    nav = st[-1][1]
    return "landed in offboard" if nav == OFFBOARD else "flown in %s" % NAV_NAMES.get(nav, "nav_state %d" % nav)


def print_flight_times(flights):
    """flights: {drone: the dict of read_flight_bag, None without a bag, or the exception reading it}"""
    print("\nFLIGHT TIME (PX4's land detector) AND HOW EACH FLIGHT ENDED")
    print("-" * 90)
    print(f"{'Drone':<10} | {'takeoff':^9} | {'landing':^9} | {'in the air':>18} | how it ended")
    print("-" * 90)
    hms = lambda t: time.strftime("%H:%M:%S", time.localtime(t))
    for name, bag in flights.items():
        if isinstance(bag, Exception):
            print(f"{name:<10} | {'-':^9} | {'-':^9} | {'-':>18} | (bag not read: {type(bag).__name__})")
            continue
        seg = flight_segments(bag)
        if not seg:
            note = "no bag" if bag is None else "no land detector in the bag" if seg is None else "no takeoff"
            print(f"{name:<10} | {'-':^9} | {'-':^9} | {'-':>18} | ({note})")
            continue
        air = sum(b - a for a, b in seg)
        cell = f"{air:.1f} s" + (f" ({len(seg)} flights)" if len(seg) > 1 else "")
        ends = "; ".join(how_it_ended(bag, a, b) for a, b in seg)
        print(f"{name:<10} | {hms(seg[0][0]):^9} | {hms(seg[-1][1]):^9} | {cell:>18} | {ends}")
    print("  * in the air = from PX4's takeoff to its landing, summed over the flights in the recording")
    print("  * how it ended = how PX4 left offboard mode (+s from the takeoff): FAILSAFE, a mode commanded by")
    print("    the stack (drone_state_manager), a pilot taking over, or landed still in offboard")


# --- SAFETY CHECK: Calculate Distances ---
def calculate_min_distances(trajectories):
    if len(trajectories) < 2:
        return None, None

    # 1. Common Time Window
    start_times = [df['timestamp'].min() for df in trajectories.values()]
    end_times = [df['timestamp'].max() for df in trajectories.values()]
    t_start = max(start_times)
    t_end = min(end_times)

    if t_end <= t_start:
        return None, None

    # 2. Interpolate to 1000Hz
    common_time = np.arange(t_start, t_end, 0.001) # 1ms steps
    interpolated_pos = {}

    for name, df in trajectories.items():
        df_clean = df.drop_duplicates(subset='timestamp')
        f = interp1d(
            df_clean['timestamp'],
            df_clean[['x', 'y', 'z']],
            axis=0,
            kind='linear',
            fill_value="extrapolate"
        )
        interpolated_pos[name] = f(common_time)

    # 3. Pairwise Distances
    min_dist_global = float('inf')
    min_dist_pair = None
    drone_names = list(trajectories.keys())

    print("\n" + "="*60)
    print(f"INTER-DRONE DISTANCES (Common Window: {t_end - t_start:.2f}s)")
    print("="*60)

    for i in range(len(drone_names)):
        for j in range(i + 1, len(drone_names)):
            name_a = drone_names[i]
            name_b = drone_names[j]

            pos_a = interpolated_pos[name_a]
            pos_b = interpolated_pos[name_b]

            diff = pos_a - pos_b
            dists = np.linalg.norm(diff, axis=1)
            local_min = np.min(dists)

            print(f"{name_a} <-> {name_b}: Min Dist = {local_min:.4f} m")

            if local_min < min_dist_global:
                min_dist_global = local_min
                min_dist_pair = (name_a, name_b)

    return min_dist_global, min_dist_pair

# --- HELPER: Print formatted table ---
def print_table(title, headers, data_keys, all_stats, col_width=13):
    print(f"\n{title}")
    header_str = f"{'Drone':<10} |" + "|".join([f"{h:^{col_width}}" for h in headers]) + "|"
    sep_str = "-" * len(header_str)

    print(sep_str)
    print(header_str)
    print(sep_str)

    def fmt(val):
        if val == 0.0: return "-"
        return f"{val:.1f}"

    for name, s in all_stats.items():
        row_str = f"{name:<10} |"
        for key in data_keys:
            mean = s.get(f"{key}_mean", 0.0)
            max_val = s.get(f"{key}_max", 0.0)
            if mean == 0.0 and max_val == 0.0:
                cell = "-"
            else:
                cell = f"{fmt(mean)}/{fmt(max_val)}"
            row_str += f"{cell:^{col_width}}|"
        print(row_str)

# --- MAIN ---
def main():
    parser = argparse.ArgumentParser(description="Analyze SwarmNXT Flight Logs")
    parser.add_argument('--log-dir', required=True, help="Path to consolidated logs")
    args = parser.parse_args()

    root_dir = Path(args.log_dir)
    if not root_dir.exists():
        print(f"Error: {root_dir} not found.")
        sys.exit(1)

    drone_dirs = [d for d in root_dir.iterdir() if d.is_dir() and d.name != 'host']
    all_stats = {}
    trajectories = {}

    print(f"--- Processing {len(drone_dirs)} Drones in {root_dir.name} ---")

    flights = {}
    for drone_dir in sorted(drone_dirs):
        try:
            flights[drone_dir.name] = read_flight_bag(drone_dir)
        except Exception as e:                 # one drone's bag must not hide the others
            flights[drone_dir.name] = e
        bag = flights[drone_dir.name]
        stats, traj = analyze_drone(drone_dir, offboard_intervals(bag) if isinstance(bag, dict) else None)
        all_stats[drone_dir.name] = stats
        if traj is not None and not traj.empty:
            trajectories[drone_dir.name] = traj

    # 0. FLIGHT TIME
    print_flight_times(flights)

    # 1. TRACKING ERRORS
    print("\nTRACKING ERRORS (Mean / Max in Meters)")
    print("-" * 105)
    print(f"{'Drone':<10} | {'X':^15} | {'Y':^15} | {'Z':^15} | {'Norm':^15} | {'MPC Time (ms)':^15}")
    print("-" * 105)
    for name, s in all_stats.items():
        def fmt_err(k):
            return f"{s.get(k+'_mean', 0):.3f} / {s.get(k+'_max', 0):.3f}"

        mpc_t = f"{s.get('mpc_time_mean', 0):.1f} / {s.get('mpc_time_max', 0):.1f}"
        if 'error_x_mean' in s:
            print(f"{name:<10} | {fmt_err('error_x'):^15} | {fmt_err('error_y'):^15} | {fmt_err('error_z'):^15} | {fmt_err('error_norm'):^15} | {mpc_t:^15}")
        elif 'tracking_note' in s:
            print(f"{name:<10} | ({s['tracking_note']})")
    print("  * from the first error under 10 cm on, while PX4 was in offboard mode (it flies the MPC only then);")
    print("    a flight that left offboard is cut there, see FLIGHT TIME. Without PX4's mode in the bag: the whole log")

    # 2. PLANNING PIPELINE
    print_table(
        "PLANNING PIPELINE (Mean / Max in ms)",
        ["Path Find", "Safe Corr", "Time-Safe", "Optim", "TOTAL"],
        ["plan_path", "plan_sc", "plan_tasc", "plan_opt", "plan_total"],
        all_stats
    )

    # 3. MAPPING PIPELINE (MapBuilder) - DETAILED BREAKDOWN
    # First show the detailed vision pipeline timing
    has_vision_timing = any('map_pcl_mean' in s for s in all_stats.values())

    if has_vision_timing:
        print_table(
            "MAPPING PIPELINE - DETAILED (Mean / Max in ms)",
            ["PCL Xform", "Pt Count", "Raycast", "Thresh", "Uncert", "Inflate", "Potent", "TOTAL"],
            ["map_pcl", "map_count", "map_raycast", "map_thresh", "map_uncertain", "map_inflate", "map_pot", "map_total"],
            all_stats,
            col_width=10
        )
        print("  * PCL Xform = fromROSMsg + transformPointCloud | Pt Count = swarm filter + point accumulation")
    else:
        # Fallback to original format if detailed timing not available
        print_table(
            "MAPPING PIPELINE (Mean / Max in ms)",
            ["Raycast", "Uncertain", "Inflate", "Potential", "Other*", "TOTAL"],
            ["map_raycast", "map_uncertain", "map_inflate", "map_pot", "map_other", "map_total"],
            all_stats
        )
        print("  * Other = PCL transform, point counting, thresholding, etc.")

    # 4. DEPTH PIPELINE - Shows proper parallel timing
    print_table(
        "DEPTH PIPELINE (Mean / Max in ms) [Parallel: showing wall time]",
        ["Infer Wall", "Cloud (max)", "Combine", "TOTAL"],
        ["depth_infer", "depth_cloud", "depth_comb", "depth_total"],
        all_stats
    )
    print("  * Infer Wall = preprocess + rectify + inference (all parallel pairs)")
    print("  * Cloud (max) = slowest pointcloud generation across pairs")
    print("  * Combine = time to assemble and publish combined pointcloud")
    print("  * TOTAL = end-to-end wall time from callback to publish")

    # 5. SAFETY CHECK
    if len(trajectories) > 1:
        min_dist, pair = calculate_min_distances(trajectories)
        if min_dist is not None:
            print(f"\n>>> GLOBAL MINIMUM DISTANCE: {min_dist:.4f} meters")
            print(f">>> CRITICAL PAIR: {pair[0]} and {pair[1]}")

            if min_dist < 0.5:
                print("\n[!!!] WARNING: COLLISION RISK DETECTED (< 0.5m) [!!!]")
            else:
                print("\n[OK] Safe separation maintained.")
    else:
        if len(drone_dirs) > 1:
             print("\n[WARNING] Could not calculate distances (Flight times did not overlap).")

    # 6. ONBOARD VIO (drones flown with enable_vio or vio_nav; prints nothing for the others)
    try:
        import analyze_vio
        analyze_vio.print_tables(root_dir)
    except Exception as e:
        print(f"\n[WARNING] VIO tables skipped ({type(e).__name__}: {e})")

if __name__ == "__main__":
    main()
