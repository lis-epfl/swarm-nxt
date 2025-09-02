# Drone GUI ROS2 Package

A web-based GUI for monitoring and controlling multiple drones in the omni-nxt system.

## Features

- **Real-time drone monitoring**: Shows connection status, armed state, flight mode, and custom drone state
- **Global commands**: Arm/Disarm All, Takeoff All, Land All buttons
- **Individual drone control**: Per-drone Arm/Disarm, Takeoff, Land buttons
- **Responsive web interface**: Works on desktop, tablet, and mobile devices
- **WebSocket integration**: Real-time updates without page refresh

## Installation

1. Install Python dependencies:
```bash
pip install -r requirements.txt
```

2. Build the ROS2 package:
```bash
cd ~/ros_packages
colcon build --packages-select drone_gui_ros2
source install/setup.bash
```

## Usage

### Standalone Launch
```bash
ros2 launch drone_gui_ros2 drone_gui.launch.py
```

### With Custom Parameters
```bash
ros2 launch drone_gui_ros2 drone_gui.launch.py port:=8080 peers_file:=~/ros/config/peers.yaml
```

### Integrated with Host System
The drone GUI is automatically launched with the host system when using the updated `host_launch.py` template.

## Web Interface

Once running, access the web interface at:
- **Local access**: http://localhost:8080
- **Network access**: http://[host-ip]:8080

## Configuration

The system reads the drone list from the `peers.yaml` file specified in the launch parameters. If the file is not found, it defaults to monitoring `nxt1`, `nxt2`, `nxt3`, and `nxt4`.

### ROS Topics

**Subscribed:**
- `/nxtN/mavros/state` - MAVROS state for each drone
- `/nxtN/manager/state` - Custom drone state manager state

**Published:**
- `/global/takeoff` - Global takeoff command
- `/global/land` - Global land command  
- `/global/arm` - Global arm/disarm command

**Services Called:**
- `/nxtN/mavros/cmd/arming` - Individual drone arming
- `/nxtN/mavros/cmd/takeoff` - Individual drone takeoff
- `/nxtN/mavros/cmd/land` - Individual drone landing

## Architecture

- **Backend**: ROS2 Python node with Flask web server
- **Frontend**: HTML/CSS/JavaScript with Socket.IO for real-time updates
- **Communication**: WebSocket for bidirectional real-time communication

## Security Note

The web server runs on all interfaces (0.0.0.0) for network accessibility. In production environments, consider adding authentication and restricting network access as needed.