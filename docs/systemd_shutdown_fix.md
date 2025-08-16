# Systemd Service Shutdown Fix

## Problem
ROS nodes were not cleanly shutting down when `systemctl stop ros_packages` was called, requiring force kill which took excessive time.

## Root Cause
The systemd service ran a bash script that launched ROS2, but signals (SIGTERM) were not properly propagated through the process chain: systemd → bash → ros2 launch → individual nodes.

## Solution
Two key changes were made to ensure proper signal handling:

### 1. Modified `ansible/scripts/ros_packages_start.sh`
- Changed `ros2 launch {{ base_path }}/launch.py` to `exec ros2 launch {{ base_path }}/launch.py`
- The `exec` command replaces the shell process with the ros2 launch process
- This ensures SIGTERM from systemd reaches ros2 launch directly instead of the intermediate bash process

### 2. Enhanced `ansible/templates/ros_packages_service.j2`
Added systemd configuration for better shutdown behavior:
```ini
# Ensure proper signal handling for clean shutdown
KillMode=mixed
KillSignal=SIGTERM
# Give nodes time to clean up, but don't wait too long
TimeoutStopSec=30
# Send SIGKILL if SIGTERM doesn't work within timeout
SendSIGKILL=yes
```

## Impact
- `systemctl stop ros_packages` now properly signals all ROS nodes to shutdown
- Nodes have 30 seconds to clean up gracefully 
- If any nodes don't respond to SIGTERM, SIGKILL is used as fallback
- No more need for manual force killing

## Testing
Use `test_service_shutdown.sh` to verify the fix works correctly after deployment.

## Technical Details
- ROS2 nodes use `rclcpp::spin()` which properly handles SIGINT/SIGTERM signals
- The issue was in the signal propagation chain, not the nodes themselves
- `exec` ensures the bash script doesn't intercept the signals
- `KillMode=mixed` tells systemd to signal both the main process and all child processes