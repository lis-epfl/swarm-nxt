#!/usr/bin/env python3

from functools import partial
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from swarmnxt_msgs.msg import Trigger, ControllerCommand, DroneState
from px4_msgs.msg import VehicleCommand, VehicleStatus, VehicleLocalPosition, OffboardControlMode
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


def make_drone_state(state_const):
    msg = DroneState()
    msg.state = state_const
    return msg


class DroneStateManager(Node):
    def __init__(self):
        super().__init__("drone_state_manager")
        self.get_logger().info("DroneStateManager node has been started.")

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
            VehicleStatus, namespace +
            "/fmu/out/vehicle_status", self.vehicle_status_cb, best_effort_qos
        )

        self.vehicle_local_position_sub_ = self.create_subscription(
            VehicleLocalPosition, namespace +
            "/fmu/out/vehicle_local_position", self.vehicle_local_position_cb, best_effort_qos
        )

        # Publishers for PX4 commands
        self.vehicle_command_pub_ = self.create_publisher(
            VehicleCommand, namespace + "/fmu/in/vehicle_command", 10
        )

        self.offboard_control_mode_pub_ = self.create_publisher(
            OffboardControlMode, namespace + "/fmu/in/offboard_control_mode", 10
        )

        self.drone_state_pub_ = self.create_publisher(
            DroneState, namespace + "/manager/state", 10
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
            DroneState.TAKING_OFF: 17,  # NAVIGATION_STATE_AUTO_TAKEOFF
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

            # Publish offboard control mode before switching
            # offboard_msg = OffboardControlMode()
            # offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            # offboard_msg.position = True
            # offboard_msg.velocity = False
            # offboard_msg.acceleration = False
            # offboard_msg.attitude = False
            # offboard_msg.body_rate = False
            # self.offboard_control_mode_pub_.publish(offboard_msg)

        if nav_state != 14:  # Not OFFBOARD
            self.create_control_cmd_sub()

        # Send SET_NAV_STATE command
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_SET_NAV_STATE,
            param1=float(nav_state)
        )
        self.mode = mode

    def takeoff_cb(self, msg: Trigger):
        if msg.enable:
            self.set_mode(make_drone_state(DroneState.TAKING_OFF))
            self.control_msgs = 0
            # start the controller

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

    def vehicle_local_position_cb(self, msg: VehicleLocalPosition):
        # Convert NED to ENU and store as PoseStamped
        if self.current_pose is None:
            self.current_pose = PoseStamped()
        self.current_pose.pose.position.x = msg.x
        self.current_pose.pose.position.y = msg.y
        self.current_pose.pose.position.z = -msg.z  # NED to ENU
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.current_pose.header.frame_id = "world"

    def control_cmd_cb(self, msg):
        self.control_msgs += 1

    def loop_cb(self):
        # Skip state logic if vehicle status not available yet
        if self.vehicle_status is None:
            return

        if self.mode.state == DroneState.TAKING_OFF:
            # check if the takeoff is finished.
            # nav_state changes or altitude reached?
            if (self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER or
                    (self.current_pose is not None and self.current_pose.pose.position.z >= 0.8)):
                # we've finished takeoff...
                if self.control_msgs > 50:
                    self.set_mode(make_drone_state(DroneState.OFFBOARD))

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
