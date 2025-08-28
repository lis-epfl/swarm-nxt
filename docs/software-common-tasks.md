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

## Logger Configuration and Usage

The SwarmNXT system includes a dedicated `logger_ros2` package for recording ROS 2 topics to bag files. This section covers how to configure and use the logger through YAML configuration files.

### Logger Package Overview

The `logger_ros2` package provides:
- Dynamic topic selection for logging
- Start/stop logging through service calls
- Integration with ROS 2 bag Python library
- MCAP format support for efficient storage

### YAML Configuration Examples

The following examples show different logger configurations. Complete example files are available in `docs/examples/`.

#### Basic Logger Launch Configuration

Create a YAML configuration file (see `docs/examples/logger_config_basic.yaml`):

```yaml
# Basic logger configuration
logger_node:
  ros__parameters:
    topics_to_log:
      - "/mavros/local_position/pose"
      - "/mavros/state"
      - "/mavros/imu/data"
    output_directory: "/home/lis/logs"
    bag_name_prefix: "swarm_flight"
    max_bag_duration: 300  # seconds
    storage_format: "mcap"
```

#### Multi-Drone Logging Configuration

For swarm operations, configure logging for multiple drones (see `docs/examples/logger_config_swarm.yaml`):

```yaml
# Multi-drone logger configuration
multi_drone_logger:
  ros__parameters:
    drone_namespaces: ["nxt1", "nxt2", "nxt3"]
    common_topics:
      - "/mavros/local_position/pose"
      - "/mavros/state"
      - "/mavros/setpoint_position/local"
      - "/latency_checker/heartbeat"
    drone_specific_topics:
      - "/mavros/imu/data"
      - "/mavros/global_position/global"
    sync_logging: true
    output_directory: "/shared/logs"
```

#### Advanced Logger Configuration with QoS Settings

For debugging and advanced use cases (see `docs/examples/logger_config_debug.yaml`):

```yaml
# Advanced logger configuration
advanced_logger:
  ros__parameters:
    logging_profiles:
      high_frequency:
        topics:
          - "/mavros/imu/data"
          - "/mavros/local_position/velocity_local"
        qos_profile:
          depth: 1000
          reliability: "best_effort"
        sample_rate: 100  # Hz
      
      low_frequency:
        topics:
          - "/mavros/state"
          - "/mavros/battery"
        qos_profile:
          depth: 10
          reliability: "reliable"
        sample_rate: 1  # Hz
    
    storage_options:
      max_bag_size_mb: 500
      compression: "zstd"
      split_duration: 60  # seconds
```

### Using the Logger

#### Starting the Logger Node

```bash
# Launch with default configuration
ros2 launch logger_ros2 logger.launch.py

# Launch with custom configuration
ros2 launch logger_ros2 logger.launch.py config_file:=/path/to/logger_config.yaml
```

#### Service Calls for Runtime Control

```bash
# Start logging specific topics
ros2 service call /logger_node/start_logging logger_ros2/srv/StartLogging "topics: ['/mavros/local_position/pose', '/mavros/state']"

# Stop logging
ros2 service call /logger_node/stop_logging logger_ros2/srv/StopLogging

# Get logging status
ros2 service call /logger_node/get_status logger_ros2/srv/GetStatus
```

#### Integration with Launch Files

Include logger in your launch file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # Include logger with configuration
        Node(
            package='logger_ros2',
            executable='logger_node',
            name='logger_node',
            parameters=['/path/to/logger_config.yaml'],
            output='screen'
        ),
        
        # Your other nodes...
    ])
```

### Ansible Integration

The logger can be configured through Ansible templates. Create a template file `logger_config.yaml.j2`:

```yaml
# Ansible template for logger configuration
logger_node:
  ros__parameters:
    topics_to_log:
{% for topic in logger_topics %}
      - "{{ topic }}"
{% endfor %}
    output_directory: "{{ drone_log_path }}"
    bag_name_prefix: "{{ drone_hostname }}_{{ ansible_date_time.date }}"
    drone_namespace: "{{ ns }}"
```

### Viewing Logged Data

```bash
# Play back logged data
ros2 bag play /path/to/your_bag_file

# Get bag info
ros2 bag info /path/to/your_bag_file

# Convert to other formats if needed
ros2 bag convert -i /path/to/mcap_file -o /path/to/output_folder --output-options "format=rosbag2_storage_sqlite"
```

### Best Practices

1. **Topic Selection**: Only log topics you need to reduce storage requirements
2. **QoS Configuration**: Match the QoS settings of the original publishers
3. **Storage Management**: Use automatic splitting and compression for long flights
4. **Synchronization**: For multi-drone operations, ensure time synchronization across drones
5. **Monitoring**: Check disk space regularly when logging high-frequency topics

### Common Logger YAML Configurations

#### Mission Logging
```yaml
mission_logger:
  ros__parameters:
    topics_to_log:
      - "/mavros/local_position/pose"
      - "/mavros/setpoint_position/local"
      - "/drone_planner/trajectory"
      - "/swarmnxt_controller/command"
    auto_start: true
    mission_name: "waypoint_mission"
```

#### Debug Logging
```yaml
debug_logger:
  ros__parameters:
    topics_to_log:
      - "/mavros/imu/data"
      - "/mavros/local_position/velocity_local"
      - "/safety_checker/violations"
      - "/bounds_checker/status"
    log_level: "DEBUG"
    include_system_topics: true
```

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
