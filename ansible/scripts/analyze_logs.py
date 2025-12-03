#!/usr/bin/env python3
import argparse
import sys
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
def analyze_drone(drone_path):
    stats = {}
    drone_name = drone_path.name

    # 1. MPC Logs (Tracking)
    mpc_states_file = drone_path / "mpc_logs" / "states.csv"
    mpc_comp_file = drone_path / "mpc_logs" / "computation_times.csv"
    trajectory_df = None

    if mpc_states_file.exists():
        try:
            df = pd.read_csv(mpc_states_file)

            # Stats Filter: Wait for error < 10cm (ignore takeoff spike)
            stable_indices = df.index[df['error_norm'] < 0.1].tolist()
            if stable_indices:
                df_stats = df.iloc[stable_indices[0]:]
            else:
                df_stats = df

            stats['error_x_mean'] = df_stats['error_x'].abs().mean()
            stats['error_x_max'] = df_stats['error_x'].abs().max()
            stats['error_y_mean'] = df_stats['error_y'].abs().mean()
            stats['error_y_max'] = df_stats['error_y'].abs().max()
            stats['error_z_mean'] = df_stats['error_z'].abs().mean()
            stats['error_z_max'] = df_stats['error_z'].abs().max()
            stats['error_norm_mean'] = df_stats['error_norm'].mean()
            stats['error_norm_max'] = df_stats['error_norm'].max()

            # Safety Check: Use FULL trajectory (including takeoff)
            trajectory_df = df[['timestamp', 'x', 'y', 'z']].sort_values('timestamp').copy()
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
            cloud_cols = [c for c in df.columns if '_cloud_ms' in c]

            df['total_rect'] = df[rect_cols].sum(axis=1)
            df['total_infer'] = df[infer_cols].sum(axis=1)
            df['total_cloud'] = df[cloud_cols].sum(axis=1)

            stats['depth_rect_mean'] = df['total_rect'].mean()
            stats['depth_rect_max'] = df['total_rect'].max()
            stats['depth_infer_mean'] = df['total_infer'].mean()
            stats['depth_infer_max'] = df['total_infer'].max()
            stats['depth_cloud_mean'] = df['total_cloud'].mean()
            stats['depth_cloud_max'] = df['total_cloud'].max()

            if 'combined_publish_ms' in df.columns:
                stats['depth_comb_mean'] = df['combined_publish_ms'].mean()
                stats['depth_comb_max'] = df['combined_publish_ms'].max()

            stats['depth_total_mean'] = df['total_wall_ms'].mean()
            stats['depth_total_max'] = df['total_wall_ms'].max()
        except Exception:
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

        # B. Mapping Pipeline
        map_files = {
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

    return stats, trajectory_df

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
    common_time = np.arange(t_start, t_end, 0.001) # 10ms steps
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

    for drone_dir in sorted(drone_dirs):
        stats, traj = analyze_drone(drone_dir)
        all_stats[drone_dir.name] = stats
        if traj is not None and not traj.empty:
            trajectories[drone_dir.name] = traj

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

    # 2. PLANNING PIPELINE
    print_table(
        "PLANNING PIPELINE (Mean / Max in ms)",
        ["Path Find", "Safe Corr", "Time-Safe", "Optim", "TOTAL"],
        ["plan_path", "plan_sc", "plan_tasc", "plan_opt", "plan_total"],
        all_stats
    )

    # 3. MAPPING PIPELINE (MapBuilder)
    print_table(
        "MAPPING PIPELINE (Mean / Max in ms)",
        ["Raycast", "Uncertain", "Inflate", "Potential", "TOTAL"],
        ["map_raycast", "map_uncertain", "map_inflate", "map_pot", "map_total"],
        all_stats
    )

    # 4. DEPTH PIPELINE
    print_table(
        "DEPTH PIPELINE (Mean / Max in ms)",
        ["Rectify", "Inference", "Cloud", "Combine", "TOTAL"],
        ["depth_rect", "depth_infer", "depth_cloud", "depth_comb", "depth_total"],
        all_stats
    )

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

if __name__ == "__main__":
    main()
