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
from ament_index_python.packages import get_package_share_directory


class DroneGUINode(Node):
    def __init__(self, socketio):
        super().__init__("drone_gui_node")
        self.socketio = socketio
        self.drone_states = {}
        self.drone_mavros_states = {}
        self.drone_list = []

        # Declare parameters
        self.declare_parameter("port", 8080)
        self.declare_parameter("peers_file", "~/ros/config/peers.yaml")

        # Get parameter values
        self.port = self.get_parameter("port").get_parameter_value().integer_value
        self.peers_file = (
            self.get_parameter("peers_file").get_parameter_value().string_value
        )

        # Load drone list from peers.yaml
        self.load_drone_list()

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

        # Subscribe to drone states
        self.setup_drone_subscriptions()

        # Individual drone service clients
        self.setup_drone_services()

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

            # Initialize states
            self.drone_states[drone] = {"state": "UNKNOWN", "timestamp": 0}
            self.drone_mavros_states[drone] = {
                "armed": False,
                "connected": False,
                "mode": "UNKNOWN",
            }

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

    def update_gui(self):
        # Prepare data for web interface
        gui_data = {}
        for drone in self.drone_list:
            mavros_state = self.drone_mavros_states.get(drone, {})
            drone_state = self.drone_states.get(drone, {})

            gui_data[drone] = {
                "name": drone,
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

        self.get_logger().info(f"Published global {command} command")

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
