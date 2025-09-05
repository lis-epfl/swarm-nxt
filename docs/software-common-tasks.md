# Common Software Tasks

The various subheadings of this document are linked in other pages. This page is meant as a reference for commonly performed tasks.

## Password-Free Access

For password free access to the Orin, follow these steps: 

1. Ensure that you have an SSH key in ~/.ssh
    1. If not, run `ssh-keygen -t ed25519 -C <yourCommentHere>`
2. With the USB connected, run `ssh-copy-id lis@192.168.55.1`
3. Make sure that the ssh-agent is on: ``eval `ssh-agent` `` 
4. Add your key to the agent: `ssh-add`
5. Ensure that you can access the device without a password: `ssh lis@192.168.55.1`

## Wi-Fi Setup

SSH into the Orin. The following is a reference for commands that you can use with `nmcli`: 

To see available Wi-Fi networks: `nmcli device wifi list`

To connect to a network with simple auth (open or WPA2/WPA3 password): `sudo nmtui` -> Activate Connection, or: 

```shell
sudo nmcli connection add type wifi \ 
con-name "NetworkName" \
ssid "SSID" \
wifi-sec.psk "password" \
wifi-sec.key-mgmt "wpa-psk"
```

To connect to a more complicated network (eduroam/epfl): 
```bash
sudo nmcli connection add type wifi \
con-name "epfl" \
ssid "epfl" \
wifi-sec.key-mgmt wpa-eap \
802-1x.eap ttls \
802-1x.phase2-auth mschapv2 \
802-1x.identity "YourUsername" \
802-1x.password "YourPassword" \
802-11-wireless.bssid "AA:BB:CC:DD:EE:FF"
```

Replace YourUsername, YourPassword with your GASPAR credentials. You can find the bssid of your local router by `nmcli device wifi list`. Use that instead of `AA:BB:CC:DD:EE:FF`.

To activate a connection, you can either use `sudo nmtui`, or: 

`nmcli con up <Name>
## Wireless Connection to QGroundControl

This is done with mavros. This should be done with the preflight Ansible. You can inspect the script in `ansible/scripts/mavros_start.sh` and the lines related to Mavros in `ansible/drones_preflight.yml`. 

## Get IP of Computer

You can get the internet protocol address of a Linux computer by running `ifconfig` and looking for the four numbers after `inet` under the interface that you care about. Wireless interfaces typically begin with `w`, and ethernet interfaces typically begin with `e`. 

On most networks, you can usually use `hostname.local` in place of the IP address if the two computers are on the same network. Replace `hostname` with the hostname of the computer. You can get the hostname by running: `hostname` or `hostnamectl` on the computer. 

### Rescue Unreachable Device

If you do not know the IP address of a drone, you can try these steps to find it's IP: 

1. Try restarting everything first. 
2. Try running `avahi-resolve -n hostname.local` while on the same network. 
3. Try running `avahi-resolve -n hostname-2.local`. Sometimes, the original hostname gets reserved and this becomes a fallback. If this works, you can run `avahi-publish` in the background with the original hostname to force it to work. 
4. Connect to the recovery port using a USB-C cable. You should be able to SSH into the device at `192.168.55.1` and use `ifconfig` to find the IP.
	1. Then, you can use `avahi-resolve -a <ip>` to find the mDNS hostname. 

## VNC/Remote Desktop Setup

On the host computer, go to terminal and type `gvncviewer lis@nxt1.local` where nxt1 is the hostname of the Orin on which is on the same network as the host computer. Then input the password 'orin'. This will allow you do remote desktop into the Orin. You can also replace `nxt1.local` with the full IP address of the orin e.g. `gvncviewer lis@192.168.55.1`.

!!! important
    If you want to connect the Orin to an external monitor, you have to first modify `/etc/X11/xorg.conf` and comment all the the paragraph that starts with `Section "Device"` and contains `Identifier "Dummy0"`, as well as all the lines after it.

## ROS Package Management

The ansible setup automatically installs convenient bash aliases for managing the ROS packages systemd service on both host computers and drones. These aliases simplify common systemctl operations:

### Available Aliases

- `ros_status` - Check the status of the ros_packages service (`systemctl --user status ros_packages`)
- `ros_start` - Start the ros_packages service (`systemctl --user start ros_packages`)
- `ros_stop` - Stop the ros_packages service (`systemctl --user stop ros_packages`)
- `ros_restart` - Restart the ros_packages service (`systemctl --user restart ros_packages`)

### Log Viewing Function

- `ros_logs [arguments]` - View logs from the ros_packages service with optional journalctl arguments

Examples:
```bash
ros_logs                    # View all logs
ros_logs -f                 # Follow logs in real-time
ros_logs --this-boot        # Show logs from current boot only
ros_logs -n 50              # Show last 50 lines
ros_logs --since "1 hour ago"  # Show logs from last hour
```

### Updating Drone Software

Use the `drones_update.yml` playbook to update and rebuild the ROS packages on drones:

```bash
# Regular update (incremental build)
ansible-playbook ansible/drones_update.yml

# Clean build (removes build/ and install/ directories first)
ansible-playbook ansible/drones_update.yml -e clean_build=true
```

The clean build option is useful when:
- Switching between branches with significant changes
- Resolving build cache issues
- After updating message definitions or package dependencies
- Troubleshooting package discovery problems

## Data Logging with ROS Bag

The SwarmNXT system automatically records flight data using the standard ROS 2 bag recording functionality. This section covers how to configure and use the logging system.

### Logging Overview

The system uses `ros2 bag record` integrated into the drone launch files to automatically capture:
- MAVROS topics (position, state, IMU data)
- Drone planner commands
- Safety checker outputs
- Custom application topics

All data is stored in MCAP format for efficient storage and playback.

### Configuring Logged Topics

Topics to log are configured in the Ansible inventory file `ansible/group_vars/all`:

```yaml
# Configure which topics to record
drone_rosbag_topics: 
  - "/{{ ns }}/*"                    # All topics in drone namespace
  # Or specify individual topics:
  # - "/{{ ns }}/mavros/local_position/pose"
  # - "/{{ ns }}/mavros/state"
  # - "/{{ ns }}/mavros/imu/data"
```

### Common Topic Configurations

#### Basic Flight Logging
```yaml
drone_rosbag_topics:
  - "/{{ ns }}/mavros/local_position/pose"
  - "/{{ ns }}/mavros/state"
  - "/{{ ns }}/mavros/setpoint_position/local"
```

#### Complete Mission Logging
```yaml
drone_rosbag_topics:
  - "/{{ ns }}/mavros/*"
  - "/{{ ns }}/drone_planner/*"
  - "/{{ ns }}/swarmnxt_controller/*"
  - "/{{ ns }}/latency_checker/*"
```

#### Debug Logging
```yaml
drone_rosbag_topics:
  - "/{{ ns }}/*"                    # All drone topics
  - "/optitrack_multiplexer_node/*"  # OptiTrack data
```

### Recorded Data Location

Bag files are automatically saved to:
- **Location**: `$ROS_LOG_DIR/bag/` on each drone
- **Format**: MCAP (`.mcap` files)
- **Size limit**: 1 GiB per file (automatically splits)

### Viewing Recorded Data

```bash
# List available bag files
ls $ROS_LOG_DIR/bag/

# Get information about a bag file
ros2 bag info /path/to/bag_file

# Play back recorded data
ros2 bag play /path/to/bag_file

# Play specific topics only
ros2 bag play /path/to/bag_file --topics /nxt1/mavros/local_position/pose /nxt1/mavros/state
```

### Data Analysis

```bash
# Convert MCAP to other formats
ros2 bag convert -i input.mcap -o output_folder --output-options "format=rosbag2_storage_sqlite"

# Extract data for analysis
ros2 topic echo --bag /path/to/bag_file /topic_name > data.txt

# Use with plotjuggler for visualization
ros2 run plotjuggler plotjuggler --buffer-size 100000
# Then: File -> Load Data -> Load ROS2 bag
```

### Postflight Log Collection

The SwarmNXT system includes automated postflight log collection that gathers all relevant flight data from drones into a centralized location. This includes both ROS bag data and MAVLink flight controller logs.

#### MAVLink Log Collection

In addition to ROS bag data, the system automatically downloads MAVLink logs directly from each drone's flight controller using pymavlink. These logs contain:
- Low-level flight controller data
- Parameter changes
- System status and error messages
- Detailed flight performance metrics

MAVLink logs are saved to:
- **Location**: `${drone_base_path}/logs/latest/mavlink/` on each drone
- **Format**: Binary `.bin` files (standard ArduPilot format)
- **Naming**: `mavlink_log_{id}_{timestamp}.bin`

#### Running Postflight Collection

```bash
# Collect logs from all configured drones
cd /path/to/swarmnxt/ansible
ansible-playbook -i inventory.ini drones_postflight.yml -K
```

This will:
1. Download MAVLink logs from each drone's flight controller
2. Collect all log files from each drone
3. Consolidate them into timestamped directories on the host computer
4. Create symlinks for easy access to latest logs

Collected logs are stored in:
- **Host location**: `${host_base_path}/consolidated_logs/latest/`
- **Structure**: `{hostname}/` subdirectories for each drone
- **Contents**: ROS bags, MAVLink logs, and system logs

## Check EKF Tracking

To check EKF tracking, perform the following steps:

1. On the host computer, make sure ros is sourced: `source /opt/ros/humble/setup.bash`
2. Ensure the `ROS_DOMAIN_ID` is set to the same value as the drones. By default, this is 1. You can check in `ansible/group_vars/all`: `export ROS_DOMAIN_ID=<domain id>`
3. Run plotjuggler: `ros2 run plotjuggler plotjuggler`
4. In plotjuggler, add:
	1. `/mavros/local_position/pose`  xyz, orientation quaternions (EKF Output)
	2. `/mavros/vision_pose/pose_cov` xyz, orientation quaternions (optitrack)
5. Move the drone around in all directions and ensure there are no discontinuities, and the values are tracking each other. 
6. Rotate the drone in different directions and ensure the values are tracking each other

# Common Issues

## Ansible Stuck 

There are a few reasons that ansible can be stuck. Sometimes, the wrong BECOME password is provided. This may result in a long time without any feedback. 

If the password is correct, it is possible that the host or target went to sleep or shutdown in the middle of a play. This can result in Ansible trying to use a dead ssh session. In this case, try deleting the `~/.ansible/cp` folder and restarting the play.
