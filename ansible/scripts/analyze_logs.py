#!/usr/bin/env python3
import argparse
import sys
import pandas as pd
import numpy as np
from pathlib import Path
from scipy.interpolate import interp1d
from itertools import combinations

# --- HELPER: Parse single-line comma logs ---
def load_planner_log(filepath):
    try:
        with open(filepath, 'r') as f:
            line = f.readline()
            if not line:
                return np.array([])
            values = [float(x) for x in line.strip().strip(',').split(',')]
            return np.array(values)
    except Exception as e:
        # Silent fail or debug print depending on preference
        return np.array([])

# --- CORE: Analyze individual drone stats ---
def analyze_drone(drone_path):
    stats = {}
    drone_name = drone_path.name

    # 1. MPC Logs
    mpc_states_file = drone_path / "mpc_logs" / "states.csv"
    mpc_comp_file = drone_path / "mpc_logs" / "computation_times.csv"
    trajectory_df = None

    if mpc_states_file.exists():
        try:
            df = pd.read_csv(mpc_states_file)

            # --- FILTERING LOGIC START ---
            # Wait for error to drop below 10cm (0.1m) before calculating stats
            # This ignores the large errors during takeoff
            stable_indices = df.index[df['error_norm'] < 0.1].tolist()

            if stable_indices:
                # Slice the dataframe starting from the first time it was stable
                start_idx = stable_indices[0]
                df_stats = df.iloc[start_idx:]
            else:
                # Fallback: if it never stabilized, report on the whole log (and warn)
                print(f"[{drone_name}] WARNING: Tracking error never dropped below 10cm!")
                df_stats = df
            # --- FILTERING LOGIC END ---

            # Basic Error Stats (Calculated on the filtered data)
            stats['error_x_mean'] = df_stats['error_x'].abs().mean()
            stats['error_x_max'] = df_stats['error_x'].abs().max()
            stats['error_y_mean'] = df_stats['error_y'].abs().mean()
            stats['error_y_max'] = df_stats['error_y'].abs().max()
            stats['error_z_mean'] = df_stats['error_z'].abs().mean()
            stats['error_z_max'] = df_stats['error_z'].abs().max()
            stats['error_norm_mean'] = df_stats['error_norm'].mean()
            stats['error_norm_max'] = df_stats['error_norm'].max()

            # Save trajectory for collision check later
            # (We keep the full trajectory for collision checks to be safe,
            # or you can change df to df_stats here too if you prefer)
            trajectory_df = df[['timestamp', 'x', 'y', 'z']].sort_values('timestamp').copy()
        except Exception as e:
            print(f"[{drone_name}] MPC States Error: {e}")

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
            # Summing stereo pairs if columns exist
            rect_cols = [c for c in df.columns if '_rect_ms' in c]
            infer_cols = [c for c in df.columns if '_infer_ms' in c]
            cloud_cols = [c for c in df.columns if '_cloud_ms' in c]

            df['total_rect'] = df[rect_cols].sum(axis=1)
            df['total_infer'] = df[infer_cols].sum(axis=1)
            df['total_cloud'] = df[cloud_cols].sum(axis=1)

            stats['depth_total_mean'] = df['total_wall_ms'].mean()
            stats['depth_total_max'] = df['total_wall_ms'].max()
            stats['depth_rect_mean'] = df['total_rect'].mean()
            stats['depth_infer_mean'] = df['total_infer'].mean()
            stats['depth_cloud_mean'] = df['total_cloud'].mean()

            if 'combined_publish_ms' in df.columns:
                stats['depth_combine_mean'] = df['combined_publish_ms'].mean()
        except Exception:
            pass

    # 3. Planner Logs
    planner_dir = drone_path / "planner_logs"
    if planner_dir.exists():
        planner_files = {
            'planner_safe_corridor': 'comp_time_sc_*.csv',
            'planner_time_aware_sc': 'comp_time_tasc_*.csv',
            'planner_optimization': 'comp_time_opt_*.csv',
            'planner_path_finding': 'comp_time_path_*.csv',
            'planner_total': 'comp_time_tot_wall_*.csv'
        }
        for metric, pattern in planner_files.items():
            files = list(planner_dir.glob(pattern))
            if files:
                data = load_planner_log(files[0])
                if data.size > 0:
                    stats[f'{metric}_mean'] = np.mean(data)
                    stats[f'{metric}_max'] = np.max(data)

    return stats, trajectory_df

# --- CORE: Interpolation & Distance Check ---
def calculate_min_distances(trajectories):
    """
    1. Finds common time window (Intersection of all flight times).
    2. Resamples all drones to 100Hz within that window.
    3. Computes vector distance.
    """
    if len(trajectories) < 2:
        return None, None

    # 1. Determine common time range (Intersection)
    # We can only compare drones when they are ALL in the air/logging.
    start_times = [df['timestamp'].min() for df in trajectories.values()]
    end_times = [df['timestamp'].max() for df in trajectories.values()]

    t_start = max(start_times)
    t_end = min(end_times)

    if t_end <= t_start:
        print("\n[WARNING] No overlapping time interval found between drone logs.")
        print("          Drones may have flown at completely different times.")
        return None, None

    # 2. Interpolate
    common_time = np.arange(t_start, t_end, 0.001)
    interpolated_pos = {}

    for name, df in trajectories.items():
        # Drop duplicates to prevent interpolation errors
        df_clean = df.drop_duplicates(subset='timestamp')

        f = interp1d(
            df_clean['timestamp'],
            df_clean[['x', 'y', 'z']],
            axis=0,
            kind='linear',
            fill_value="extrapolate"
        )
        interpolated_pos[name] = f(common_time)

    # 3. Pairwise Distance Calculation
    min_dist_global = float('inf')
    min_dist_pair = None
    drone_names = list(trajectories.keys())

    print("\n" + "="*80)
    print(f"INTER-DRONE DISTANCES (Common Window: {t_end - t_start:.2f}s)")
    print("="*80)

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

# --- MAIN ---
def main():
    parser = argparse.ArgumentParser(description="Analyze SwarmNXT Flight Logs")
    parser.add_argument('--log-dir', required=True, help="Path to consolidated logs")
    args = parser.parse_args()

    root_dir = Path(args.log_dir)
    if not root_dir.exists():
        print(f"Error: {root_dir} not found.")
        sys.exit(1)

    # Exclude 'host' or non-folders
    drone_dirs = [d for d in root_dir.iterdir() if d.is_dir() and d.name != 'host']

    all_stats = {}
    trajectories = {}

    print(f"--- Processing {len(drone_dirs)} Drones in {root_dir.name} ---")

    for drone_dir in sorted(drone_dirs):
        # Gather Stats and Trajectory
        stats, traj = analyze_drone(drone_dir)
        all_stats[drone_dir.name] = stats
        if traj is not None and not traj.empty:
            trajectories[drone_dir.name] = traj

    # --- DISPLAY TABLES ---

    # 1. Tracking Errors
    print("\nTRACKING ERRORS (Meters)")
    print("-" * 90)
    print(f"{'Drone':<10} | {'X Mean':<7} {'X Max':<7} | {'Y Mean':<7} {'Y Max':<7} | {'Z Mean':<7} {'Z Max':<7} | {'Norm Avg':<8} {'Norm Max':<8}")
    for name, s in all_stats.items():
        if 'error_x_mean' in s:
            print(f"{name:<10} | {s['error_x_mean']:.3f}   {s['error_x_max']:.3f}   | {s['error_y_mean']:.3f}   {s['error_y_max']:.3f}   | {s['error_z_mean']:.3f}   {s['error_z_max']:.3f}   | {s['error_norm_mean']:.3f}    {s['error_norm_max']:.3f}")

    # 2. Compute Times
    print("\nCOMPUTE TIMES (Mean / Max in ms)")
    print("-" * 90)
    def p(d, k): return f"{d.get(k+'_mean', 0):.1f}/{d.get(k+'_max', 0):.1f}"

    print(f"{'Drone':<10} | {'MPC':<12} | {'Plan Tot':<12} | {'Plan Opt':<12} | {'Depth Tot':<12} | {'Depth Inf':<12}")
    for name, s in all_stats.items():
        print(f"{name:<10} | {p(s,'mpc_time'):<12} | {p(s,'planner_total'):<12} | {p(s,'planner_optimization'):<12} | {p(s,'depth_total'):<12} | {p(s,'depth_infer'):<12}")

    # 3. Safety Check (Interpolated)
    if len(trajectories) > 1:
        min_dist, pair = calculate_min_distances(trajectories)
        if min_dist is not None:
            print(f"\n>>> GLOBAL MINIMUM DISTANCE: {min_dist:.4f} meters")
            print(f">>> CRITICAL PAIR: {pair[0]} and {pair[1]}")
    else:
        print("\n[INFO] Need at least 2 drone logs to calculate safety distances.")

if __name__ == "__main__":
    main()
