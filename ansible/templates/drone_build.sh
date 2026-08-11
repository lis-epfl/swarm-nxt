#!/bin/bash
set -e

# Source user environment and ROS
source /home/lis/.bashrc
source /opt/ros/humble/setup.sh

# Cap build parallelism by AVAILABLE MEMORY, not core count. colcon otherwise fans out
# to one job per core, and an OpenVINS translation unit (ov_core/ov_init/ov_msckf pull
# in Eigen + Ceres templates) peaks around 1.1 GB in cc1plus. Measured failure on a
# 20-core/15 GiB machine: the build asked for ~22 GB and the kernel OOM-killed the
# compiler mid-run. An Orin NX is the same trap in miniature — 8 cores against 16 GB
# shared with the GPU. Override with BUILD_JOBS if you know better.
if [ -z "${BUILD_JOBS:-}" ]; then
    _cores=$(nproc 2>/dev/null || echo 4)
    # MemAvailable is what we can use without swapping; budget 1.5 GB per job.
    _avail_mb=$(awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo 2>/dev/null || echo 4096)
    _mem_jobs=$(( _avail_mb / 1500 ))
    [ "$_mem_jobs" -lt 1 ] && _mem_jobs=1
    BUILD_JOBS=$(( _cores < _mem_jobs ? _cores : _mem_jobs ))
fi
echo "drone_build.sh: building with $BUILD_JOBS parallel compile jobs"
export MAKEFLAGS="-j${BUILD_JOBS}"

# Build message/interface packages first. px4_msgs is in this list because ov_msckf
# does find_package(px4_msgs QUIET): if it is not built yet, ov_msckf silently compiles
# its raw-CDR fallback for SensorCombined instead of the typed path. Both work, but
# which one you get should not depend on build ordering.
colcon build --symlink-install --parallel-workers "$BUILD_JOBS" --packages-select \
    px4_msgs multi_agent_planner_msgs jps3d decomp_util convex_decomp_util \
    path_finding_util voxel_grid_util decomp_ros_msgs decomp_ros_utils

# Source the built packages
source install/setup.sh

# Build main packages
colcon build --symlink-install --parallel-workers "$BUILD_JOBS"
