#!/usr/bin/env python3

import json
import os
import yaml
import threading
from typing import Dict, List
import math
import random
import time

try:
    from flask import Flask, send_from_directory, request
    from flask_socketio import SocketIO, emit
except ImportError:
    print(
        "Flask and Flask-SocketIO are required. Install with: pip install flask flask-socketio"
    )
    exit(1)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, BatteryStatus
from swarmnxt_msgs.msg import Trigger, DroneState
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from ament_index_python.packages import get_package_share_directory

from multi_agent_planner_msgs.msg import StartPlanning, StopPlanning

try:
    from latency_checker_ros2.msg import Heartbeat
except ImportError:
    print(
        "WARNING: latency_checker_ros2.msg.Heartbeat not found."
        " Latency features will be disabled."
    )
    Heartbeat = None


class DroneGUINode(Node):
    def __init__(self, socketio):
        super().__init__("drone_gui_node")
        self.socketio = socketio
        self.drone_states = {}
        self.drone_vehicle_statuses = {}
        self.drone_positions = {}
        self.drone_battery_statuses = {}
        self.drone_latencies = {}
        self.drone_list = []
        self.hdsm_mapping = {}

        # --- Roaming State ---
        self.roaming_active = False
        self.roaming_params = {'radius': 2.0, 'center_height': -2.5, 'cylinder_height': 1.0}
        self.roaming_goals = {} # Stores the current goal for each drone
        self.roaming_loop_timer = None

        # Declare parameters
        self.declare_parameter("port", 8080)
        self.declare_parameter("peers_file", "~/ros/config/peers.yaml")
        self.declare_parameter("hdsm_mapping_file", "~/ros/config/hdsm_planner_map.yaml")
        # --- Using 'hostname' as per your provided file ---
        self.declare_parameter("hostname", "host")
        self.declare_parameter("roaming_threshold", 0.5)

        # Get parameter values
        self.port = self.get_parameter("port").get_parameter_value().integer_value
        self.peers_file = (
            self.get_parameter("peers_file").get_parameter_value().string_value
        )
        self.hdsm_mapping_file = (
            self.get_parameter("hdsm_mapping_file").get_parameter_value().string_value
        )
        self.hostname = self.get_parameter("hostname").get_parameter_value().string_value
        self.roaming_threshold = self.get_parameter("roaming_threshold").get_parameter_value().double_value

        # Load drone list from peers.yaml
        self.load_drone_list()

        # Load HDSM mapping
        self.load_hdsm_mapping()

        # Set up QoS
        reliable_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=10)

        # Per-drone command publishers
        self.arm_pubs = {}
        self.land_pubs = {}
        self.takeoff_pubs = {}
        self.kill_pubs = {}

        # Controller enable/disable publishers for each drone
        self.controller_enable_pubs = {}

        # Planning topic publishers
        self.planning_start_publishers = {}
        self.planning_stop_publishers = {}

        # Goal setpoint publishers for position swapping
        self.goal_publishers = {}

        # Subscribe to drone states
        self.setup_drone_subscriptions()

        # Set up namespaced command publishers
        self.setup_drone_command_publishers()

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

        self.socketio.on_event('connect', self.handle_gui_connect)

    def handle_gui_connect(self):
        """Send current roaming state to a newly connected client."""
        try:
            sid = request.sid
            self.socketio.emit('roaming_status', {'active': self.roaming_active}, to=sid)
        except Exception as e:
            self.get_logger().warn(f"Error handling GUI connect: {e}")

    def load_drone_list(self):
        peers_path = os.path.expanduser(self.peers_file)
        if not os.path.exists(peers_path):
            raise RuntimeError(f"peers.yaml not found at {peers_path}")

        with open(peers_path, "r") as f:
            content = f.read()
            lines = [
                line.strip()
                for line in content.split("\n")
                if line.strip() and not line.startswith("#")
            ]
            self.drone_list = []
            for line in lines:
                if line.startswith("nxt") or "nxt" in line:
                    parts = line.split()
                    for part in parts:
                        if "nxt" in part:
                            drone_name = part.replace(".local", "").replace(":", "")
                            if drone_name not in self.drone_list:
                                self.drone_list.append(drone_name)

            if not self.drone_list:
                raise RuntimeError("Could not load drone list")

    def load_hdsm_mapping(self):
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
        best_effort_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)

        for drone in self.drone_list:
            self.create_subscription(
                VehicleStatus, f"/{drone}/fmu/out/vehicle_status_v1",
                lambda msg, d=drone: self.vehicle_status_callback(msg, d), best_effort_qos,
            )
            self.create_subscription(
                DroneState, f"/{drone}/manager/state",
                lambda msg, d=drone: self.drone_state_callback(msg, d), 10,
            )
            self.create_subscription(
                VehicleLocalPosition, f"/{drone}/fmu/out/vehicle_local_position",
                lambda msg, d=drone: self.vehicle_local_position_callback(msg, d), best_effort_qos,
            )
            self.create_subscription(
                BatteryStatus, f"/{drone}/fmu/out/battery_status",
                lambda msg, d=drone: self.battery_status_callback(msg, d), best_effort_qos,
            )

            # Initialize states
            self.drone_states[drone] = {"state": "UNKNOWN", "timestamp": 0}
            self.drone_vehicle_statuses[drone] = {
                "armed": False, "connected": False, "nav_state": 0,
            }
            self.drone_positions[drone] = Point(x=0.0, y=0.0, z=0.0)

            # --- MODIFIED: Initialized new battery keys ---
            self.drone_battery_statuses[drone] = {
                "soc_estimate": -1.0, # Use -1.0 to indicate "unknown"
                "voltage_v": 0.0,
                "warning": BatteryStatus.WARNING_NONE
            }
            self.drone_latencies[drone] = -1.0

        if Heartbeat is not None:
            # Use self.hostname (which you've defined from parameters)
            topic_name = f"/{self.hostname}/latency_checker/heartbeat"
            self.latency_sub = self.create_subscription(
                Heartbeat, topic_name, self.latency_heartbeat_callback, 10
            )
            self.get_logger().info(f"Subscribing to latency updates on: {topic_name}")
        else:
            self.get_logger().warn("Latency checking is disabled (Heartbeat message not found).")

    def latency_heartbeat_callback(self, msg: Heartbeat):
        for item in msg.latency_list:
            drone_key = item.name.lstrip('/')
            if drone_key in self.drone_list:
                latency_ms = (item.latency.sec * 1000) + (item.latency.nanosec / 1_000_000)
                self.drone_latencies[drone_key] = latency_ms

    def setup_drone_command_publishers(self):
        reliable_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=10)
        for drone in self.drone_list:
            self.arm_pubs[drone] = self.create_publisher(Trigger, f"/{drone}/arm", reliable_qos)
            self.land_pubs[drone] = self.create_publisher(Trigger, f"/{drone}/land", reliable_qos)
            self.takeoff_pubs[drone] = self.create_publisher(Trigger, f"/{drone}/takeoff", reliable_qos)
            self.kill_pubs[drone] = self.create_publisher(Trigger, f"/{drone}/kill", reliable_qos)
        self.get_logger().info(f"Created namespaced arm/land/takeoff/kill publishers for {len(self.drone_list)} drones.")

    def setup_controller_publishers(self):
        reliable_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=10)
        for drone in self.drone_list:
            self.controller_enable_pubs[drone] = self.create_publisher(Bool, f"/{drone}/mpc_controller/enable", reliable_qos)

    def setup_planning_publishers(self):
        for drone in self.drone_list:
            # Safely get drone number
            drone_num_str = ''.join(filter(str.isdigit, drone))
            if not drone_num_str:
                self.get_logger().warn(f"Could not extract number from drone name: {drone}")
                continue
            drone_num = int(drone_num_str)

            if drone_num in self.hdsm_mapping:
                agent_idx = self.hdsm_mapping[drone_num]
                self.planning_start_publishers[drone] = self.create_publisher(StartPlanning, f"/agent_{agent_idx}/start_planning", 10)
                self.planning_stop_publishers[drone] = self.create_publisher(StopPlanning, f"/agent_{agent_idx}/stop_planning", 10)
                self.get_logger().info(f"Set up planning publishers for {drone} -> agent_{agent_idx}")
            else:
                self.get_logger().warn(f"No HDSM mapping found for drone {drone} (num: {drone_num})")

    def setup_goal_publishers(self):
        for drone in self.drone_list:
            # Safely get drone number
            drone_num_str = ''.join(filter(str.isdigit, drone))
            if not drone_num_str:
                self.get_logger().warn(f"Could not extract number from drone name: {drone}")
                continue
            drone_num = int(drone_num_str)

            if drone_num in self.hdsm_mapping:
                agent_idx = self.hdsm_mapping[drone_num]
                self.goal_publishers[drone] = self.create_publisher(PointStamped, f"/agent_{agent_idx}/goal", 10)
                self.get_logger().info(f"Set up goal publisher for {drone} -> agent_{agent_idx}")
            else:
                self.get_logger().warn(f"No HDSM mapping found for drone {drone} (num: {drone_num}) for goal publisher")

    def vehicle_status_callback(self, msg: VehicleStatus, drone_name: str):
        self.drone_vehicle_statuses[drone_name] = {
            "armed": msg.arming_state == VehicleStatus.ARMING_STATE_ARMED,
            "connected": True, "nav_state": msg.nav_state,
        }

    def drone_state_callback(self, msg: DroneState, drone_name: str):
        state_map = {
            DroneState.IDLE: "IDLE", DroneState.TAKING_OFF: "TAKING_OFF",
            DroneState.HOVERING: "HOVERING", DroneState.OFFBOARD: "OFFBOARD",
            DroneState.LANDING: "LANDING",
        }
        self.drone_states[drone_name] = {
            "state": state_map.get(msg.state, "UNKNOWN"),
            "timestamp": self.get_clock().now().nanoseconds,
        }

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition, drone_name: str):
        self.drone_positions[drone_name].x = msg.x
        self.drone_positions[drone_name].y = -msg.y
        self.drone_positions[drone_name].z = -msg.z

    def battery_status_callback(self, msg: BatteryStatus, drone_name: str):
        self.drone_battery_statuses[drone_name] = {
            "soc_estimate": msg.volt_based_soc_estimate,
            "voltage_v": msg.ocv_estimate_filtered,
            "warning": msg.warning
        }

    def update_gui(self):
        gui_data = {}
        for drone in self.drone_list:
            vehicle_status = self.drone_vehicle_statuses.get(drone, {})
            drone_state = self.drone_states.get(drone, {})
            battery_status = self.drone_battery_statuses.get(drone, {})
            latency_ms = self.drone_latencies.get(drone, -1.0)

            # Safely get drone number
            drone_num_str = ''.join(filter(str.isdigit, drone))
            agent_id = "N/A"
            if drone_num_str:
                drone_num = int(drone_num_str)
                agent_id = self.hdsm_mapping.get(drone_num, "N/A")

            nav_state = vehicle_status.get("nav_state", 0)
            mode_map = {
                0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "MISSION",
                4: "LOITER", 5: "RTL", 14: "OFFBOARD", 17: "TAKEOFF", 18: "LAND"
            }
            mode = mode_map.get(nav_state, f"NAV_{nav_state}")

            # --- MODIFIED: Using "soc_estimate" ---
            remaining_float = battery_status.get("soc_estimate", -1.0)
            battery_percent = -1
            if 0.0 <= remaining_float <= 1.0:
                battery_percent = int(remaining_float * 100)

            gui_data[drone] = {
                "name": drone, "agent_id": agent_id,
                "armed": vehicle_status.get("armed", False),
                "connected": vehicle_status.get("connected", False),
                "mode": mode, "state": drone_state.get("state", "UNKNOWN"),
                "last_update": drone_state.get("timestamp", 0),
                "battery_percent": battery_percent,
                "voltage_v": battery_status.get("voltage_v", 0.0), # Added voltage
                "battery_warning": battery_status.get("warning", 0),
                "latency_ms": latency_ms
            }
        self.socketio.emit("drone_update", gui_data)

    def publish_global_command(self, command: str, enable: bool = True):
        msg = Trigger()
        msg.stamp = self.get_clock().now().to_msg()
        msg.enable = enable

        publisher_dict = None
        if command == "takeoff":
            publisher_dict = self.takeoff_pubs
        elif command == "land":
            publisher_dict = self.land_pubs
            self.stop_roaming_loop() # Stop roaming on global land
        elif command == "arm":
            publisher_dict = self.arm_pubs
        elif command == "disarm":
            msg.enable = False
            publisher_dict = self.arm_pubs
        elif command == "kill":
            publisher_dict = self.kill_pubs
            self.stop_roaming_loop() # Stop roaming on global kill

        if publisher_dict:
            for drone in publisher_dict.keys():
                self.call_individual_service(drone, command)
            self.get_logger().info(f"Published global {command} command to {len(publisher_dict)} drones.")
            return

        elif command == "planning_start":
            self.start_planning_all()
        elif command == "planning_stop":
            self.stop_planning_all()
        elif command == "swap_positions":
            self.swap_positions()

        self.get_logger().info(f"Published global {command} command")

    def start_planning_all(self):
        for drone in self.drone_list:
            self.call_planning_service(drone, "start")
        self.get_logger().info("Started planning for all drones")

    def stop_planning_all(self):
        for drone in self.drone_list:
            self.call_planning_service(drone, "stop")
        self.get_logger().info("Stopped planning for all drones")

    def call_planning_service(self, drone: str, command: str):
        try:
            if command == "start":
                publisher = self.planning_start_publishers.get(drone)
                if publisher:
                    msg = StartPlanning()
                    pos = self.drone_positions.get(drone, Point(x=0.0, y=0.0, z=0.0))
                    msg.initial_state = [
                        float(pos.x), float(pos.y), float(pos.z),
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0
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
        """
        Calculates the 2D centroid of all drones and sends each drone
        to its symmetrical position relative to the centroid,
        while maintaining its original Z altitude.
        """
        valid_drones = []
        for drone_name in self.drone_list:
            pos = self.drone_positions.get(drone_name)
            if pos is None:
                self.get_logger().warn(f"Could not get position for {drone_name}, skipping.")
                continue
            valid_drones.append((drone_name, pos))

        if not valid_drones:
            self.get_logger().error("Swap positions failed: No valid drone positions available.")
            return

        n_drones = len(valid_drones)
        sum_x = sum(p.x for _, p in valid_drones)
        sum_y = sum(p.y for _, p in valid_drones)

        centroid_x = sum_x / n_drones
        centroid_y = sum_y / n_drones

        self.get_logger().info(f"Calculated 2D centroid for {n_drones} drones: ({centroid_x:.2f}, {centroid_y:.2f})")

        goals_to_publish = []
        log_goals = {}
        for drone_name, pos in valid_drones:
            pub = self.goal_publishers.get(drone_name)
            if pub is None:
                self.get_logger().warn(f"No goal publisher for {drone_name}, cannot send swap goal.")
                continue

            # Calculate symmetrical position (2D)
            goal_x = 2.0 * centroid_x - pos.x
            goal_y = 2.0 * centroid_y - pos.y
            goal_z = pos.z  # Maintain original altitude

            goal_msg = PointStamped()
            goal_msg.header.frame_id = "world"
            goal_msg.point.x = goal_x
            goal_msg.point.y = goal_y
            goal_msg.point.z = goal_z

            goals_to_publish.append((pub, goal_msg))
            log_goals[drone_name] = (goal_x, goal_y, goal_z)

        # Log all goals
        for drone, goal_pos in log_goals.items():
             self.get_logger().info(f"  {drone} -> ({goal_pos[0]:.2f}, {goal_pos[1]:.2f}, {goal_pos[2]:.2f})")

        # Publish goals multiple times
        for i in range(5):
            current_time = self.get_clock().now().to_msg()
            for pub, msg in goals_to_publish:
                msg.header.stamp = current_time
                pub.publish(msg)

            if i < 4:
                time.sleep(0.1) # Use time.sleep

        self.get_logger().info(f"Published symmetrical swap goals for {len(goals_to_publish)} drones.")

    def call_individual_service(self, drone: str, command: str):
        try:
            msg = Trigger()
            msg.stamp = self.get_clock().now().to_msg()
            msg.enable = True

            enable_msg = Bool() # For controller
            pub = None

            if command == "arm":
                pub = self.arm_pubs.get(drone)
                if pub and drone in self.controller_enable_pubs:
                    enable_msg.data = True
                    self.controller_enable_pubs[drone].publish(enable_msg)
                    self.get_logger().info(f"Enabled controller for {drone} (on arm)")
            elif command == "disarm":
                msg.enable = False
                pub = self.arm_pubs.get(drone)
                if pub and drone in self.controller_enable_pubs:
                    enable_msg.data = False
                    self.controller_enable_pubs[drone].publish(enable_msg)
                    self.get_logger().info(f"Enabled controller for {drone} (on arm)")
            elif command == "takeoff":
                pub = self.takeoff_pubs.get(drone)
                if pub and drone in self.controller_enable_pubs:
                    enable_msg.data = True
                    self.controller_enable_pubs[drone].publish(enable_msg)
                    self.get_logger().info(f"Enabled controller for {drone}")
            elif command == "land":
                pub = self.land_pubs.get(drone)
                if pub:
                    if drone in self.controller_enable_pubs:
                        enable_msg.data = False
                        self.controller_enable_pubs[drone].publish(enable_msg)
                        self.get_logger().info(f"Disabled controller for {drone}")
                    self.call_planning_service(drone, "stop")
                    self.stop_roaming_loop() # Stop roaming on individual land
            elif command == "kill":
                pub = self.kill_pubs.get(drone)
                if pub:
                    self.call_planning_service(drone, "stop")
                    if drone in self.controller_enable_pubs:
                        enable_msg.data = False
                        self.controller_enable_pubs[drone].publish(enable_msg)
                        self.get_logger().info(f"Disabled controller for {drone} (KILL)")
                    self.stop_roaming_loop() # Stop roaming on individual kill
            elif command == "planning_start":
                self.call_planning_service(drone, "start")
                return
            elif command == "planning_stop":
                self.call_planning_service(drone, "stop")
                return
            else:
                self.get_logger().warn(f"Unknown individual command: {command}")
                return

            if pub:
                pub.publish(msg)
                self.get_logger().info(f"Sent {command} command for {drone}")
            else:
                self.get_logger().error(f"No publisher for {command} on {drone}")

        except Exception as e:
            self.get_logger().error(f"Error sending {command} command for {drone}: {e}")

    def publish_individual_goal(self, drone: str, x: float, y: float, z: float):
        try:
            publisher = self.goal_publishers.get(drone)
            if not publisher:
                self.get_logger().error(f"No goal publisher found for {drone}")
                return

            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"
            msg.point.x, msg.point.y, msg.point.z = float(x), float(y), float(z)

            publisher.publish(msg)
            self.get_logger().info(f"Published goal for {drone}: [x: {x}, y: {y}, z: {z}]")

        except Exception as e:
            self.get_logger().error(f"Error publishing goal for {drone}: {e}")

    # --- ROAMING METHODS ---
    def start_roaming_loop(self, data):
        """Start the roaming behavior loop"""
        if self.roaming_active:
            self.get_logger().warn("Roaming is already active. Stopping first.")
            self.stop_roaming_loop()

        self.roaming_params['radius'] = data.get('radius', 2.0)
        self.roaming_params['center_height'] = data.get('center_height', -2.5)
        self.roaming_params['cylinder_height'] = data.get('cylinder_height', 1.0)
        self.roaming_active = True
        self.roaming_goals = {} # Clear previous goals

        self.get_logger().info(f"Starting roaming with params: {self.roaming_params}")
        self.socketio.emit('roaming_status', {'active': True})

        # Start the loop
        self.roaming_update()

    def stop_roaming_loop(self):
        """Stop the roaming behavior loop"""
        if not self.roaming_active:
            return

        self.roaming_active = False
        if self.roaming_loop_timer:
            self.roaming_loop_timer.cancel()
            self.roaming_loop_timer = None

        self.stop_planning_all()

        self.get_logger().info("Roaming stopped.")
        self.socketio.emit('roaming_status', {'active': False})

    def sample_point_on_cylinder(self) -> Point:
        """Sample a random 3D point on the surface of a cylinder"""
        radius = self.roaming_params['radius']
        center_h = self.roaming_params['center_height']
        cyl_h = self.roaming_params['cylinder_height']

        angle = random.uniform(0, 2 * math.pi)
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        z = center_h + random.uniform(-cyl_h / 2.0, cyl_h / 2.0)

        return Point(x=x, y=y, z=z)

    def roaming_update(self):
        """The core logic loop for roaming"""
        if not self.roaming_active:
            return

        try:
            for drone_name in self.drone_list:
                current_pos = self.drone_positions.get(drone_name)
                current_goal = self.roaming_goals.get(drone_name)
                goal_reached = False

                if current_pos and current_goal:
                    dist = math.sqrt(
                        (current_pos.x - current_goal.x)**2 +
                        (current_pos.y - current_goal.y)**2 +
                        (current_pos.z - current_goal.z)**2
                    )
                    if dist < self.roaming_threshold:
                        goal_reached = True

                if current_goal is None or goal_reached:
                    new_goal = self.sample_point_on_cylinder()
                    self.roaming_goals[drone_name] = new_goal

                    self.publish_individual_goal(
                        drone_name, new_goal.x, new_goal.y, new_goal.z
                    )
                    if goal_reached:
                        self.get_logger().info(f"Drone {drone_name} reached goal. Sending new goal.")
                    else:
                        self.get_logger().info(f"Sending initial goal for {drone_name}.")

        except Exception as e:
            self.get_logger().error(f"Error in roaming_update: {e}")

        if self.roaming_active:
            self.roaming_loop_timer = threading.Timer(0.2, self.roaming_update) # Loop every 200ms
            self.roaming_loop_timer.start()


# Flask web server setup
app = Flask(__name__)
app.config["SECRET_KEY"] = "drone_gui_secret"
socketio = SocketIO(app, cors_allowed_origins="*")
ros_node = None

@app.route("/")
def index():
    try:
        web_dir = get_package_share_directory("drone_gui_ros2") + "/web"
        return send_from_directory(web_dir, "index.html")
    except:
        return send_from_directory(
            "/home/niel/Documents/repos/lis/omni-nxt/ros_packages/drone_gui_ros2/web", "index.html",
        )

@app.route("/<path:filename>")
def serve_static(filename):
    try:
        web_dir = get_package_share_directory("drone_gui_ros2") + "/web"
        return send_from_directory(web_dir, filename)
    except:
        return send_from_directory(
            "/home/niel/Documents/repos/lis/omni-nxt/ros_packages/drone_gui_ros2/web", filename,
        )

@socketio.on("global_command")
def handle_global_command(data):
    if ros_node:
        ros_node.publish_global_command(data.get("command"))

@socketio.on("individual_command")
def handle_individual_command(data):
    if ros_node:
        ros_node.call_individual_service(data.get("drone"), data.get("command"))

@socketio.on("individual_goal")
def handle_individual_goal(data):
    if ros_node:
        drone, x, y, z = data.get("drone"), data.get("x"), data.get("y"), data.get("z")
        if all([drone is not None, x is not None, y is not None, z is not None]):
            ros_node.publish_individual_goal(drone, x, y, z)
        else:
            ros_node.get_logger().warn(f"Received incomplete goal data: {data}")

@socketio.on("start_roaming")
def handle_start_roaming(data):
    if ros_node:
        ros_node.start_roaming_loop(data)

@socketio.on("stop_roaming")
def handle_stop_roaming(data):
    if ros_node:
        ros_node.stop_roaming_loop()

def run_flask_app(port=8080):
    socketio.run(
        app, host="0.0.0.0", port=port, debug=False, allow_unsafe_werkzeug=True
    )

def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = DroneGUINode(socketio)

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
        if ros_node.roaming_active:
            ros_node.stop_roaming_loop() # Ensure loop stops on shutdown
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
