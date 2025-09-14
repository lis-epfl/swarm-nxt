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
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL, CommandBool
from swarmnxt_msgs.msg import Trigger, DroneState
from geometry_msgs.msg import PoseStamped, Point
from ament_index_python.packages import get_package_share_directory

from multi_agent_planner_msgs.msg import StartPlanning, StopPlanning


class DroneGUINode(Node):
    def __init__(self, socketio):
        super().__init__("drone_gui_node")
        self.socketio = socketio
        self.drone_states = {}
        self.drone_mavros_states = {}
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

        # Subscribe to drone states
        self.setup_drone_subscriptions()

        # Individual drone service clients
        self.setup_drone_services()
        
        # Set up controller enable/disable publishers
        self.setup_controller_publishers()
        
        # Set up planning topic publishers
        self.setup_planning_publishers()

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
        for drone in self.drone_list:
            # Subscribe to MAVROS state
            self.create_subscription(
                State,
                f"/{drone}/mavros/state",
                lambda msg, d=drone: self.mavros_state_callback(msg, d),
                10,
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
                PoseStamped,
                f"/{drone}/mavros/local_position/pose",
                lambda msg, d=drone: self.position_callback(msg, d),
                10,
            )

            # Initialize states
            self.drone_states[drone] = {"state": "UNKNOWN", "timestamp": 0}
            self.drone_mavros_states[drone] = {
                "armed": False,
                "connected": False,
                "mode": "UNKNOWN",
            }
            self.drone_positions[drone] = Point(x=0.0, y=0.0, z=0.0)

    def setup_drone_services(self):
        self.arm_services = {}
        self.takeoff_services = {}
        self.land_services = {}

        for drone in self.drone_list:
            self.arm_services[drone] = self.create_client(
                CommandBool, f"/{drone}/mavros/cmd/arming"
            )
            self.takeoff_services[drone] = self.create_client(
                CommandTOL, f"/{drone}/mavros/cmd/takeoff"
            )
            self.land_services[drone] = self.create_client(
                CommandTOL, f"/{drone}/mavros/cmd/land"
            )

    def setup_controller_publishers(self):
        reliable_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=10)
        
        for drone in self.drone_list:
            self.controller_enable_pubs[drone] = self.create_publisher(
                Trigger, f"/{drone}/controller/enable", reliable_qos
            )
    
    def setup_planning_publishers(self):
        """Set up planning topic publishers for each agent node"""
        for drone in self.drone_list:
            # Extract drone number to map to agent node
            drone_num = ''.join(filter(str.isdigit, drone))
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

    def mavros_state_callback(self, msg: State, drone_name: str):
        self.drone_mavros_states[drone_name] = {
            "armed": msg.armed,
            "connected": msg.connected,
            "mode": msg.mode,
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
    
    def position_callback(self, msg: PoseStamped, drone_name: str):
        """Update drone position for planning initial state"""
        self.drone_positions[drone_name] = msg.pose.position

    def update_gui(self):
        # Prepare data for web interface
        gui_data = {}
        for drone in self.drone_list:
            mavros_state = self.drone_mavros_states.get(drone, {})
            drone_state = self.drone_states.get(drone, {})
            
            # Get agent ID from HDSM mapping
            drone_num = ''.join(filter(str.isdigit, drone))
            agent_id = self.hdsm_mapping.get(drone_num, "N/A") if drone_num else "N/A"

            gui_data[drone] = {
                "name": drone,
                "agent_id": agent_id,
                "armed": mavros_state.get("armed", False),
                "connected": mavros_state.get("connected", False),
                "mode": mavros_state.get("mode", "UNKNOWN"),
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
        elif command == "planning_stop":
            self.stop_planning_all()

        self.get_logger().info(f"Published global {command} command")

    def publish_controller_command(self, enable: bool):
        msg = Trigger()
        msg.stamp = self.get_clock().now().to_msg()
        msg.enable = enable
        
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

    def call_individual_service(self, drone: str, command: str):
        try:
            if command == "arm":
                service = self.arm_services.get(drone)
                if service and service.wait_for_service(timeout_sec=1.0):
                    req = CommandBool.Request()
                    req.value = True
                    service.call_async(req)

            elif command == "disarm":
                service = self.arm_services.get(drone)
                if service and service.wait_for_service(timeout_sec=1.0):
                    req = CommandBool.Request()
                    req.value = False
                    service.call_async(req)

            elif command == "takeoff":
                service = self.takeoff_services.get(drone)
                if service and service.wait_for_service(timeout_sec=1.0):
                    req = CommandTOL.Request()
                    req.altitude = 2.0  # Default takeoff altitude
                    service.call_async(req)

            elif command == "land":
                service = self.land_services.get(drone)
                if service and service.wait_for_service(timeout_sec=1.0):
                    req = CommandTOL.Request()
                    service.call_async(req)
            
            elif command == "planning_start":
                self.call_planning_service(drone, "start")
                return  # Return early to avoid duplicate logging
            
            elif command == "planning_stop":
                self.call_planning_service(drone, "stop")
                return  # Return early to avoid duplicate logging

            self.get_logger().info(f"Called {command} service for {drone}")

        except Exception as e:
            self.get_logger().error(f"Error calling {command} service for {drone}: {e}")


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
