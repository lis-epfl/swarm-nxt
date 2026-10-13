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
# NOTE: colcon's --parallel-workers and MAKEFLAGS MULTIPLY -- N packages in flight,
# each running make -jN, is N*N concurrent compilers. So the jobs are shared out between
# the packages in flight, and the ceiling stays at BUILD_JOBS compilers.
# Two packages at a time, because most of a build is single long source files and
# packages with one or two files: with one package at a time the other cores sit idle.
# Measured on an Orin NX (nxt4, 13 Oct 2026), a clean build of the nine packages that
# take the time (the estimator, the planner, the MPC and the nodes of this repo):
#   1 package  x 8 jobs: 13 min  7 s; 0.9 GB of memory left at the peak; one compiler or
#                        none running for 67 % of the time
#   2 packages x 4 jobs:  9 min  7 s; 5.6 GB left; 20 %
#   3 packages x 3 jobs: 10 min 18 s; 6.2 GB left
#   3 packages x 2 jobs: 12 min 47 s; 8.1 GB left
# Three at a time is slower than two: the estimator's three packages have to build one
# after the other and set the total time, and with more packages in flight each gets
# fewer jobs.
# Override with BUILD_WORKERS (1 = one package at a time, as before).
BUILD_WORKERS=${BUILD_WORKERS:-2}
[ "$BUILD_WORKERS" -gt "$BUILD_JOBS" ] && BUILD_WORKERS=$BUILD_JOBS
[ "$BUILD_WORKERS" -lt 1 ] && BUILD_WORKERS=1
_jobs_each=$(( BUILD_JOBS / BUILD_WORKERS ))
echo "drone_build.sh: building $BUILD_WORKERS packages at a time with $_jobs_each compile jobs each"
export MAKEFLAGS="-j${_jobs_each}"

# Build message/interface packages first. px4_msgs is in this list because ov_msckf
# does find_package(px4_msgs QUIET): if it is not built yet, ov_msckf silently compiles
# its raw-CDR fallback for SensorCombined instead of the typed path. Both work, but
# which one you get should not depend on build ordering.
colcon build --symlink-install --parallel-workers $BUILD_WORKERS --packages-select \
    px4_msgs multi_agent_planner_msgs jps3d decomp_util convex_decomp_util \
    path_finding_util voxel_grid_util decomp_ros_msgs decomp_ros_utils

# Source the built packages
source install/setup.sh

# Build main packages
colcon build --symlink-install --parallel-workers $BUILD_WORKERS
