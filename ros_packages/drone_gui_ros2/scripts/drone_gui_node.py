#!/usr/bin/env python3

import json
import os
import yaml
import threading
from typing import Dict, List

try:
    from flask import Flask, send_from_directory
    from flask_socketio import SocketIO, emit
except ImportError:
    print(
        "Flask and Flask-SocketIO are required. Install with: pip install flask flask-socketio"
    )
    exit(1)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from px4_msgs.msg import VehicleStatus, VehicleCommand, VehicleLocalPosition
from swarmnxt_msgs.msg import Trigger, DroneState
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from ament_index_python.packages import get_package_share_directory

from multi_agent_planner_msgs.msg import StartPlanning, StopPlanning


class DroneGUINode(Node):
    def __init__(self, socketio):
        super().__init__("drone_gui_node")
        self.socketio = socketio
        self.drone_states = {}
        self.drone_vehicle_statuses = {}
        self.drone_positions = {}
        self.drone_list = []
        self.hdsm_mapping = {}

        # Declare parameters
        self.declare_parameter("port", 8080)
        self.declare_parameter("peers_file", "~/ros/config/peers.yaml")
        self.declare_parameter("hdsm_mapping_file", "~/ros/config/hdsm_planner_map.yaml")

        # Get parameter values
        self.port = self.get_parameter("port").get_parameter_value().integer_value
        self.peers_file = (
            self.get_parameter("peers_file").get_parameter_value().string_value
        )
        self.hdsm_mapping_file = (
            self.get_parameter("hdsm_mapping_file").get_parameter_value().string_value
        )

        # Load drone list from peers.yaml
        self.load_drone_list()
        
        # Load HDSM mapping
        self.load_hdsm_mapping()

        # Set up QoS
        reliable_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=10)

        # Global command publishers
        self.global_takeoff_pub = self.create_publisher(
            Trigger, "/global/takeoff", reliable_qos
        )
        self.global_land_pub = self.create_publisher(
            Trigger, "/global/land", reliable_qos
        )
        self.global_arm_pub = self.create_publisher(
            Trigger, "/global/arm", reliable_qos
        )
        
        # Controller enable/disable publishers for each drone
        self.controller_enable_pubs = {}
        
        # Planning topic publishers
        self.planning_start_publishers = {}
        self.planning_stop_publishers = {}

        # Goal setpoint publishers for position swapping
        self.goal_publishers = {}

        # Subscribe to drone states
        self.setup_drone_subscriptions()

        # Individual drone service clients
        self.setup_drone_services()
        
        # Set up controller enable/disable publishers
        self.setup_controller_publishers()
        
        # Set up planning topic publishers
        self.setup_planning_publishers()

        # Set up goal publishers for position swapping
        self.setup_goal_publishers()

        # Timer for periodic updates
        self.timer = self.create_timer(0.5, self.update_gui)

        self.get_logger().info(
            f"Drone GUI Node started, monitoring {len(self.drone_list)} drones"
        )

    def load_drone_list(self):
        # Use the parameter-specified peers file path
        peers_path = os.path.expanduser(self.peers_file)
        if not os.path.exists(peers_path):
            # Fallback to a default list if file doesn't exist
            raise RuntimeError(f"peers.yaml not found at {peers_path}")

        with open(peers_path, "r") as f:
            content = f.read()
            # Parse the peers file - it might just be a list of drone names
            lines = [
                line.strip()
                for line in content.split("\n")
                if line.strip() and not line.startswith("#")
            ]

            # Extract drone names (assume format like 'nxtX' or similar)
            self.drone_list = []
            for line in lines:
                if line.startswith("nxt") or "nxt" in line:
                    # Extract drone name
                    parts = line.split()
                    for part in parts:
                        if "nxt" in part:
                            drone_name = part.replace(".local", "").replace(":", "")
                            if drone_name not in self.drone_list:
                                self.drone_list.append(drone_name)

            if not self.drone_list:
                # If we couldn't parse, use default
                raise RuntimeError("Could not load drone list")
    
    def load_hdsm_mapping(self):
        """Load the HDSM planner mapping from YAML file"""
        mapping_path = os.path.expanduser(self.hdsm_mapping_file)
        try:
            if os.path.exists(mapping_path):
                with open(mapping_path, 'r') as f:
                    self.hdsm_mapping = yaml.safe_load(f)
                self.get_logger().info(f"Loaded HDSM mapping: {self.hdsm_mapping}")
            else:
                raise RuntimeError(f"HDSM mapping file not found at {mapping_path}!")
        except Exception as e:
            self.get_logger().error(f"Error loading HDSM mapping: {e}")
            raise

    def setup_drone_subscriptions(self):
        # Set up QoS profile for MAVROS position (uses best effort)
        best_effort_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        
        for drone in self.drone_list:
            # Subscribe to PX4 vehicle status
            self.create_subscription(
                VehicleStatus,
                f"/{drone}/fmu/out/vehicle_status_v1",
                lambda msg, d=drone: self.vehicle_status_callback(msg, d),
                best_effort_qos,
            )

            # Subscribe to custom drone state
            self.create_subscription(
                DroneState,
                f"/{drone}/manager/state",
                lambda msg, d=drone: self.drone_state_callback(msg, d),
                10,
            )

            # Subscribe to position for planning initial state
            self.create_subscription(
                VehicleLocalPosition,
                f"/{drone}/fmu/out/vehicle_local_position",
                lambda msg, d=drone: self.vehicle_local_position_callback(msg, d),
                best_effort_qos,
            )

            # Initialize states
            self.drone_states[drone] = {"state": "UNKNOWN", "timestamp": 0}
            self.drone_vehicle_statuses[drone] = {
                "armed": False,
                "connected": False,
                "nav_state": 0,
            }
            self.drone_positions[drone] = Point(x=0.0, y=0.0, z=0.0)

    def setup_drone_services(self):
        # Replace service clients with publishers for PX4
        self.vehicle_command_pubs = {}

        for drone in self.drone_list:
            self.vehicle_command_pubs[drone] = self.create_publisher(
                VehicleCommand, f"/{drone}/fmu/in/vehicle_command", 10
            )

    def setup_controller_publishers(self):
        reliable_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=10)
        
        for drone in self.drone_list:
            self.controller_enable_pubs[drone] = self.create_publisher(Bool, f"/{drone}/mpc_controller/enable", reliable_qos
            )
    
    def setup_planning_publishers(self):
        """Set up planning topic publishers for each agent node"""
        for drone in self.drone_list:
            # Extract drone number to map to agent node
            drone_num = int(''.join(filter(str.isdigit, drone)))
            if drone_num in self.hdsm_mapping:
                agent_idx = self.hdsm_mapping[drone_num]
                
                # Create topic publishers for the corresponding agent node
                self.planning_start_publishers[drone] = self.create_publisher(
                    StartPlanning, f"/agent_{agent_idx}/start_planning", 10
                )
                self.planning_stop_publishers[drone] = self.create_publisher(
                    StopPlanning, f"/agent_{agent_idx}/stop_planning", 10
                )
                
                self.get_logger().info(f"Set up planning publishers for {drone} -> agent_{agent_idx}")
            else:
                self.get_logger().warn(f"No HDSM mapping found for drone {drone} (num: {drone_num})")

    def setup_goal_publishers(self):
        """Set up goal publishers for position swapping"""
        for drone in self.drone_list:
            # Extract drone number to map to agent node
            drone_num = int(''.join(filter(str.isdigit, drone)))
            if drone_num in self.hdsm_mapping:
                agent_idx = self.hdsm_mapping[drone_num]

                # Create goal publisher for the corresponding agent node
                self.goal_publishers[drone] = self.create_publisher(
                    PointStamped, f"/agent_{agent_idx}/goal", 10
                )

                self.get_logger().info(f"Set up goal publisher for {drone} -> agent_{agent_idx}")
            else:
                self.get_logger().warn(f"No HDSM mapping found for drone {drone} (num: {drone_num}) for goal publisher")

    def vehicle_status_callback(self, msg: VehicleStatus, drone_name: str):
        self.drone_vehicle_statuses[drone_name] = {
            "armed": msg.arming_state == VehicleStatus.ARMING_STATE_ARMED,
            "connected": True,  # If we're getting messages, we're connected
            "nav_state": msg.nav_state,
        }

    def drone_state_callback(self, msg: DroneState, drone_name: str):
        state_map = {
            DroneState.IDLE: "IDLE",
            DroneState.TAKING_OFF: "TAKING_OFF",
            DroneState.HOVERING: "HOVERING",
            DroneState.OFFBOARD: "OFFBOARD",
            DroneState.LANDING: "LANDING",
        }

        self.drone_states[drone_name] = {
            "state": state_map.get(msg.state, "UNKNOWN"),
            "timestamp": self.get_clock().now().nanoseconds,
        }

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition, drone_name: str):
        """Update drone position for planning initial state"""
        # Convert NED to ENU
        self.drone_positions[drone_name].x = msg.x
        self.drone_positions[drone_name].y = msg.y
        self.drone_positions[drone_name].z = -msg.z

    def update_gui(self):
        # Prepare data for web interface
        gui_data = {}
        for drone in self.drone_list:
            vehicle_status = self.drone_vehicle_statuses.get(drone, {})
            drone_state = self.drone_states.get(drone, {})

            # Get agent ID from HDSM mapping
            drone_num = int(''.join(filter(str.isdigit, drone)))
            agent_id = self.hdsm_mapping.get(drone_num, "N/A") if drone_num else "N/A"

            # Map nav_state to mode string
            nav_state = vehicle_status.get("nav_state", 0)
            mode_map = {
                0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "MISSION",
                4: "LOITER", 5: "RTL", 14: "OFFBOARD", 17: "TAKEOFF", 18: "LAND"
            }
            mode = mode_map.get(nav_state, f"NAV_{nav_state}")

            gui_data[drone] = {
                "name": drone,
                "agent_id": agent_id,
                "armed": vehicle_status.get("armed", False),
                "connected": vehicle_status.get("connected", False),
                "mode": mode,
                "state": drone_state.get("state", "UNKNOWN"),
                "last_update": drone_state.get("timestamp", 0),
            }

        # Emit to all connected web clients
        self.socketio.emit("drone_update", gui_data)

    def publish_global_command(self, command: str, enable: bool = True):
        msg = Trigger()
        msg.stamp = self.get_clock().now().to_msg()
        msg.enable = enable

        if command == "takeoff":
            self.global_takeoff_pub.publish(msg)
        elif command == "land":
            self.global_land_pub.publish(msg)
        elif command == "arm":
            self.global_arm_pub.publish(msg)
        elif command == "disarm":
            msg.enable = False
            self.global_arm_pub.publish(msg)
        elif command == "controller_enable":
            self.publish_controller_command(True)
        elif command == "controller_disable":
            self.publish_controller_command(False)
        elif command == "planning_start":
            self.start_planning_all()
            self.publish_controller_command(True)
        elif command == "planning_stop":
            self.stop_planning_all()
        elif command == "swap_positions":
            self.swap_positions()

        self.get_logger().info(f"Published global {command} command")

    def publish_controller_command(self, enable: bool):
        msg = Bool()
        msg.data = enable

        for drone in self.drone_list:
            if drone in self.controller_enable_pubs:
                self.controller_enable_pubs[drone].publish(msg)

        action = "enabled" if enable else "disabled"
        self.get_logger().info(f"Controllers {action} for all drones")
    
    def start_planning_all(self):
        """Start planning for all drones"""
        for drone in self.drone_list:
            self.call_planning_service(drone, "start")
        self.get_logger().info("Started planning for all drones")
    
    def stop_planning_all(self):
        """Stop planning for all drones"""
        for drone in self.drone_list:
            self.call_planning_service(drone, "stop")
        self.get_logger().info("Stopped planning for all drones")
    
    def call_planning_service(self, drone: str, command: str):
        """Publish planning messages for a specific drone"""
        try:
            if command == "start":
                publisher = self.planning_start_publishers.get(drone)
                if publisher:
                    msg = StartPlanning()
                    
                    # Get current position for initial state
                    pos = self.drone_positions.get(drone, Point(x=0.0, y=0.0, z=0.0))
                    
                    # Fill initial_state: [x, y, z, vx, vy, vz, ax, ay, az]
                    msg.initial_state = [
                        float(pos.x), float(pos.y), float(pos.z),  # position
                        0.0, 0.0, 0.0,  # velocity (zeros as requested)
                        0.0, 0.0, 0.0   # acceleration (zeros as requested)
                    ]
                    
                    publisher.publish(msg)
                    self.get_logger().info(f"Published start planning for {drone} with initial state: {msg.initial_state}")
                else:
                    self.get_logger().error(f"Start planning publisher not available for {drone}")
                    
            elif command == "stop":
                publisher = self.planning_stop_publishers.get(drone)
                if publisher:
                    msg = StopPlanning()
                    publisher.publish(msg)
                    self.get_logger().info(f"Published stop planning for {drone}")
                else:
                    self.get_logger().error(f"Stop planning publisher not available for {drone}")
                    
        except Exception as e:
            self.get_logger().error(f"Error publishing planning message for {drone}: {e}")

    def swap_positions(self):
        """Swap positions between exactly 2 drones"""
        import time

        # Check if we have exactly 2 drones
        if len(self.drone_list) != 2:
            self.get_logger().error(f"Swap positions requires exactly 2 drones, but found {len(self.drone_list)} drones")
            return

        drone1, drone2 = self.drone_list[0], self.drone_list[1]

        # Get current positions
        pos1 = self.drone_positions.get(drone1)
        pos2 = self.drone_positions.get(drone2)

        if pos1 is None or pos2 is None:
            self.get_logger().error("Could not get current positions for both drones")
            return

        # Get publishers for both drones
        pub1 = self.goal_publishers.get(drone1)
        pub2 = self.goal_publishers.get(drone2)

        if pub1 is None or pub2 is None:
            self.get_logger().error("Goal publishers not available for both drones")
            return

        # Create PointStamped messages for swapped positions
        # Drone 1 gets drone 2's xy position (keeping its own z)
        goal1 = PointStamped()
        goal1.header.stamp = self.get_clock().now().to_msg()
        goal1.header.frame_id = "world"  # or whatever frame you're using
        goal1.point.x = pos2.x
        goal1.point.y = pos2.y
        goal1.point.z = pos1.z  # Keep original z

        # Drone 2 gets drone 1's xy position (keeping its own z)
        goal2 = PointStamped()
        goal2.header.stamp = self.get_clock().now().to_msg()
        goal2.header.frame_id = "world"
        goal2.point.x = pos1.x
        goal2.point.y = pos1.y
        goal2.point.z = pos2.z  # Keep original z

        # Publish 5 times with 0.1s delay between messages
        for i in range(5):
            pub1.publish(goal1)
            pub2.publish(goal2)
            self.get_logger().info(f"Published swap goals (iteration {i+1}/5): {drone1} -> ({goal1.point.x:.2f}, {goal1.point.y:.2f}, {goal1.point.z:.2f}), {drone2} -> ({goal2.point.x:.2f}, {goal2.point.y:.2f}, {goal2.point.z:.2f})")
            if i < 4:  # Don't sleep after the last iteration
                time.sleep(0.1)

        self.get_logger().info(f"Successfully swapped positions between {drone1} and {drone2}")

    def publish_vehicle_command(self, drone: str, command: int, param1=0.0, param2=0.0):
        """Helper to publish VehicleCommand for a specific drone"""
        publisher = self.vehicle_command_pubs.get(drone)
        if not publisher:
            self.get_logger().error(f"No vehicle command publisher for {drone}")
            return

        cmd = VehicleCommand()
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        cmd.command = command
        cmd.param1 = float(param1)
        cmd.param2 = float(param2)
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        publisher.publish(cmd)

    def call_individual_service(self, drone: str, command: str):
        try:
            if command == "arm":
                self.publish_vehicle_command(
                    drone,
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=1.0
                )

            elif command == "disarm":
                self.publish_vehicle_command(
                    drone,
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=0.0
                )

            elif command == "takeoff":
                self.publish_vehicle_command(
                    drone,
                    VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
                    param7=2.0  # altitude
                )

            elif command == "land":
                self.publish_vehicle_command(
                    drone,
                    VehicleCommand.VEHICLE_CMD_NAV_LAND
                )

            elif command == "planning_start":
                self.call_planning_service(drone, "start")
                return  # Return early to avoid duplicate logging

            elif command == "planning_stop":
                self.call_planning_service(drone, "stop")
                return  # Return early to avoid duplicate logging

            self.get_logger().info(f"Sent {command} command for {drone}")

        except Exception as e:
            self.get_logger().error(f"Error sending {command} command for {drone}: {e}")


# Flask web server setup
app = Flask(__name__)
app.config["SECRET_KEY"] = "drone_gui_secret"
socketio = SocketIO(app, cors_allowed_origins="*")

# Global reference to ROS node
ros_node = None


@app.route("/")
def index():
    try:
        web_dir = get_package_share_directory("drone_gui_ros2") + "/web"
        return send_from_directory(web_dir, "index.html")
    except:
        # Fallback if package not found
        return send_from_directory(
            "/home/niel/Documents/repos/lis/omni-nxt/ros_packages/drone_gui_ros2/web",
            "index.html",
        )


@app.route("/<path:filename>")
def serve_static(filename):
    try:
        web_dir = get_package_share_directory("drone_gui_ros2") + "/web"
        return send_from_directory(web_dir, filename)
    except:
        # Fallback if package not found
        return send_from_directory(
            "/home/niel/Documents/repos/lis/omni-nxt/ros_packages/drone_gui_ros2/web",
            filename,
        )


@socketio.on("global_command")
def handle_global_command(data):
    if ros_node:
        command = data.get("command")
        ros_node.publish_global_command(command)


@socketio.on("individual_command")
def handle_individual_command(data):
    if ros_node:
        drone = data.get("drone")
        command = data.get("command")
        ros_node.call_individual_service(drone, command)


def run_flask_app(port=8080):
    socketio.run(
        app, host="0.0.0.0", port=port, debug=False, allow_unsafe_werkzeug=True
    )


def main(args=None):
    global ros_node

    rclpy.init(args=args)
    ros_node = DroneGUINode(socketio)

    # Start Flask in a separate thread with the configured port
    flask_thread = threading.Thread(
        target=lambda: run_flask_app(ros_node.port), daemon=True
    )
    flask_thread.start()

    ros_node.get_logger().info(f"Web GUI available at http://0.0.0.0:{ros_node.port}")

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
