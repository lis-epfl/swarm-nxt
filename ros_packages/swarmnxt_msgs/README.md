# swarmnxt_msgs package

## Overview
The `swarmnxt_msgs` package is a message-only ROS 2 package that defines custom message types for use in swarm robotics applications.

## Message Definitions
This package includes the following message definitions:
- `ExampleMessage`: A sample message type that can be used for communication between nodes.

## Building the Package
To build the `swarmnxt_msgs` package, follow these steps:

1. Ensure you have ROS 2 installed and sourced in your terminal.
2. Navigate to the root of your ROS 2 workspace:
   ```bash
   cd ~/your_ros2_workspace
   ```
3. Clone the `swarmnxt_msgs` package into the `src` directory:
   ```bash
   git clone <repository_url>
   ```
4. Build the package using `colcon`:
   ```bash
   colcon build --packages-select swarmnxt_msgs
   ```
5. Source the setup file:
   ```bash
   source install/setup.bash
   ```

## Usage
After building the package, you can use the defined message types in your ROS 2 nodes. Make sure to include the package in your `package.xml` and use the appropriate message types in your code.

## Maintainers
- [Your Name] - [Your Email]

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.