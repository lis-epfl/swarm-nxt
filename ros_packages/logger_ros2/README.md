# logger_ros2 Package

## Overview
The `logger_ros2` package provides a ROS 2 node that interfaces with the ROS 2 bag Python library to dynamically log topics. This allows users to specify which topics they want to log at runtime, making it a flexible tool for data collection in ROS 2 applications.

## Installation
To install the `logger_ros2` package, clone the repository and build it using the following commands:

```bash
git clone <repository-url>
cd logger_ros2
colcon build
```

Make sure to source your workspace after building:

```bash
source install/setup.bash
```

## Usage
To run the logger node, use the following command:

```bash
ros2 launch logger_ros2 logger.launch.py
```

You can specify the topics to log by modifying the launch file or by passing parameters at runtime.

## Features
- Dynamically load topics for logging.
- Start and stop logging through service calls.
- Interface with the ROS 2 bag Python library for efficient data storage.

## Dependencies
This package requires the following ROS 2 packages:
- `rclpy`
- `rosbag2`

## Maintainers
- Your Name <your.email@example.com>

## License
This project is licensed under the MIT License. See the LICENSE file for details.