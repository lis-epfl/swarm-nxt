#!/usr/bin/env python3

from functools import partial
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from swarmnxt_msgs.msg import Trigger, ControllerCommand, DroneState
from px4_msgs.msg import (
    VehicleCommand,
    VehicleStatus,
    OffboardControlMode,
    VehicleOdometry,  
)
from geometry_msgs.msg import (
    PoseStamped,
    Point,
    Quaternion,
    Vector3,  
)
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from mpc_controller_ros2_msgs.msg import Trajectory, TrajectoryState
from std_msgs.msg import Header  # Added for trajectory header
import numpy as np


def make_drone_state(state_const):
    msg = DroneState()
    msg.state = state_const
    return msg


class DroneStateManager(Node):
    def __init__(self):
        super().__init__("drone_state_manager")
        self.get_logger().info("DroneStateManager node has been started.")

        self.declare_parameter('takeoff_altitude', 0.5)
        self.declare_parameter('takeoff_duration', 3.0) 
        self.declare_parameter('takeoff_dt', 0.1)

        # Initialize drone state and vehicle status
        self.mode = make_drone_state(DroneState.IDLE)
        self.vehicle_status = None
        self.current_pose = None
        self.takeoff_altitude = None

        self.loop_timer = self.create_timer(0.1, self.loop_cb)

        self.create_control_cmd_sub()
        namespace = self.get_namespace()
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, depth=10)

        self.global_takeoff_sub_ = self.create_subscription(
            Trigger, "/global/takeoff", self.takeoff_cb, reliable_qos
        )
        self.global_land_sub_ = self.create_subscription(
            Trigger, "/global/land", self.land_cb, reliable_qos
        )

        self.global_arm_sub_ = self.create_subscription(
            Trigger, "/global/arm", self.arm_cb, reliable_qos
        )

        # Subscribe to PX4 topics
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)

        self.vehicle_status_sub_ = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v1", self.vehicle_status_cb, best_effort_qos
        )

        self.vehicle_odometry_sub_ = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry", self.vehicle_odometry_cb, best_effort_qos
        )

        # Publishers for PX4 commands
        self.vehicle_command_pub_ = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10
        )

        self.offboard_control_mode_pub_ = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )

        self.drone_state_pub_ = self.create_publisher(
            DroneState, namespace + "/manager/state", 10
        )

        # Publisher for the MPC Trajectory
        self.trajectory_pub_ = self.create_publisher(
            Trajectory, "/planner/trajectory", reliable_qos
        )

    def create_control_cmd_sub(self):
        self.get_logger().info("Resetting control messages")
        self.control_msgs = 0
        self.control_cmd_sub_ = self.create_subscription(
            ControllerCommand,
            "/controller/cmd",
            self.control_cmd_cb,
            10,
        )

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Helper to publish VehicleCommand messages"""
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
        self.vehicle_command_pub_.publish(cmd)

    def set_mode(self, mode: DroneState):
        if type(mode) != DroneState:
            self.get_logger().error(
                f"Did not get a valid mode to set. Got {mode}")
            return
        self.get_logger().info(f"Attempting to set the mode to {mode}")

        # Map DroneState to PX4 nav_state
        mode_map = {
            DroneState.IDLE: 2,  # NAVIGATION_STATE_POSCTL
            DroneState.OFFBOARD: 14,  # NAVIGATION_STATE_OFFBOARD
            DroneState.LANDING: 18,  # NAVIGATION_STATE_AUTO_LAND
        }

        nav_state = mode_map[mode.state]

        if nav_state == 14:  # OFFBOARD
            if self.control_msgs < 10:
                self.get_logger().warning(
                    f"Did not switch to offboard mode since there were too few control msgs"
                )
                return
            else:
                if self.control_cmd_sub_:
                    self.destroy_subscription(self.control_cmd_sub_)
                    self.control_cmd_sub_ = None
                else:
                    self.get_logger().warning("No control cmd subscription to destroy")

        if nav_state != 14:  # Not OFFBOARD
            self.create_control_cmd_sub()

        # Send SET_NAV_STATE command
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_SET_NAV_STATE,
            param1=float(nav_state)
        )
        self.mode = mode

    def takeoff_cb(self, msg: Trigger):
        if not msg.enable:
            return

        if self.vehicle_status is None or self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().warning("Takeoff command rejected: Drone is not armed.")
            return

        # Check if we have a valid pose (both position and orientation)
        if self.current_pose is None or self.current_pose.pose.orientation.w == 0.0:
             self.get_logger().warning("Takeoff command rejected: Current pose is not available.")
             return

        self.get_logger().info("Received takeoff command. Switching to OFFBOARD and publishing trajectory.")

        # 1. Switch to Offboard mode
        self.set_mode(make_drone_state(DroneState.OFFBOARD))

        # 2. Generate and publish the takeoff trajectory
        self.generate_and_publish_takeoff_trajectory()
    
    def generate_and_publish_takeoff_trajectory(self):
        # Get parameters
        takeoff_alt = self.get_parameter('takeoff_altitude').get_parameter_value().double_value
        takeoff_time = self.get_parameter('takeoff_duration').get_parameter_value().double_value
        dt = self.get_parameter('takeoff_dt').get_parameter_value().double_value

        if takeoff_time <= 0.0:
            self.get_logger().error(f"Takeoff duration must be positive. Is {takeoff_time}")
            return

        if dt <= 0.0:
            self.get_logger().error(f"Takeoff dt must be positive. Is {dt}")
            return

        num_steps = int(takeoff_time / dt)
        if num_steps < 1:
             self.get_logger().error(f"Not enough steps for trajectory. Steps: {num_steps}")
             return

        # Current state
        start_pos = self.current_pose.pose.position
        start_orientation = self.current_pose.pose.orientation

        # Target state
        target_pos_z = start_pos.z + takeoff_alt
        target_vel_z = (target_pos_z - start_pos.z) / takeoff_time

        # Create trajectory message
        traj_msg = Trajectory()
        now = self.get_clock().now()
        traj_msg.header.stamp = now.to_msg()
        traj_msg.header.frame_id = self.current_pose.header.frame_id
        traj_msg.t_0 = float(now.nanoseconds / 1e9)
        traj_msg.dt = dt

        self.get_logger().info(f"Generating trajectory: {num_steps} points, alt: {takeoff_alt}, time: {takeoff_time}s, dt: {dt}s")

        # Generate trajectory states
        for i in range(num_steps):
            progress = (i + 1) / num_steps # 1-based progress

            state = TrajectoryState()

            # Position: Interpolate Z, hold X, Y
            state.position.x = start_pos.x
            state.position.y = start_pos.y
            state.position.z = start_pos.z + (target_pos_z - start_pos.z) * progress

            # Orientation: Hold starting orientation
            state.orientation = start_orientation

            # Velocity: Constant Z velocity, zero X, Y
            state.velocity.x = 0.0
            state.velocity.y = 0.0
            state.velocity.z = target_vel_z

            # Angular Velocity: Zero
            state.angular_velocity.x = 0.0
            state.angular_velocity.y = 0.0
            state.angular_velocity.z = 0.0

            traj_msg.states.append(state)

        # Add a final state at the target with zero velocity
        final_state = TrajectoryState()
        final_state.position.x = start_pos.x
        final_state.position.y = start_pos.y
        final_state.position.z = target_pos_z
        final_state.orientation = start_orientation
        final_state.velocity = Vector3()  # Zero
        final_state.angular_velocity = Vector3() # Zero
        traj_msg.states.append(final_state)

        # Publish the trajectory
        self.trajectory_pub_.publish(traj_msg)
        self.get_logger().info(f"Published takeoff trajectory with {len(traj_msg.states)} states.")

    def land_cb(self, msg):
        # call the local land service
        if msg.enable:
            self.set_mode(make_drone_state(DroneState.LANDING))

    def arm_cb(self, msg):
        # Publish arm/disarm command
        arm_value = 1.0 if msg.enable else 0.0
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=arm_value
        )
        self.get_logger().info(f"Sent arm command: {msg.enable}")

    def vehicle_status_cb(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def vehicle_odometry_cb(self, msg: VehicleOdometry):
        # Populate position, converting FRD to FLU
        self.current_pose.pose.position.x = msg.position[0]
        self.current_pose.pose.position.y = -msg.position[1]
        self.current_pose.pose.position.z = -msg.position[2]

        # Populate orientation
        self.current_pose.pose.orientation.w = float(msg.q[0])
        self.current_pose.pose.orientation.x = float(msg.q[1])
        self.current_pose.pose.orientation.y = -float(msg.q[2])
        self.current_pose.pose.orientation.z = -float(msg.q[3])

        # Update the stamp
        self.current_pose.header.stamp = self.get_clock().now().to_msg()

    def control_cmd_cb(self, msg):
        self.control_msgs += 1

    def loop_cb(self):
        # Skip state logic if vehicle status not available yet
        if self.vehicle_status is None:
            return

        elif self.mode.state == DroneState.OFFBOARD:
            # nothing to automatically do
            pass
        elif self.mode.state == DroneState.LANDING:
            # nothing to automatically do
            pass

        # if at any point that we think we're flying but we get disarmed, move back to idle.
        if (
            self.mode.state in [DroneState.OFFBOARD, DroneState.LANDING]
            and self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED
        ):
            self.get_logger().warning(
                "Drone got disarmed, switching drone state to idle"
            )
            self.set_mode(make_drone_state(DroneState.IDLE))

        self.drone_state_pub_.publish(self.mode)


def main(args=None):
    rclpy.init(args=args)
    node = DroneStateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
