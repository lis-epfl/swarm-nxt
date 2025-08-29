#!/usr/bin/env python3

from functools import partial
import rclpy
from rclpy.node import Node
from swarmnxt_msgs.msg import Trigger, ControllerCommand, DroneState
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode
from mavros_msgs.msg import State
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


def make_drone_state(state_const):
    msg = DroneState()
    msg.state = state_const
    return msg


class DroneStateManager(Node):
    def __init__(self):
        super().__init__("drone_state_manager")
        self.get_logger().info("DroneStateManager node has been started.")
        
        # Initialize drone state and MAVROS state
        self.mode = make_drone_state(DroneState.IDLE)
        self.mavros_state = None

        self.loop_timer = self.create_timer(0.1, self.loop_cb)
            
        self.create_control_cmd_sub()
        namespace = self.get_namespace()
        reliable_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=10)


        self.global_takeoff_sub_ = self.create_subscription(
            Trigger, "/global/takeoff", self.takeoff_cb, reliable_qos
        )
        self.global_land_sub_ = self.create_subscription(
            Trigger, "/global/land", self.land_cb, reliable_qos
        )

        self.global_arm_sub_ = self.create_subscription(
            Trigger, "/global/arm", self.arm_cb, reliable_qos
        )

        self.mavros_state_client_ = self.create_subscription(
            State, namespace + "/mavros/state", self.mavros_state_cb, 10
        )

        self.set_mode_client_ = self.create_client(
            SetMode, namespace + "/mavros/set_mode"
        )

        self.local_takeoff_client_ = self.create_client(
            CommandTOL, namespace + "/mavros/cmd/takeoff"
        )
        self.local_land_client_ = self.create_client(
            CommandTOL, namespace + "/mavros/cmd/land"
        )
        self.local_arm_client_ = self.create_client(
            CommandBool, namespace + "/mavros/cmd/arming"
        )
        
        self.drone_state_pub_ = self.create_publisher(
            DroneState, namespace + "/manager/state", 10
        )


    def create_control_cmd_sub(self):
        self.get_logger().info("Resetting control messages")
        self.control_msgs = 0
        self.control_cmd_sub_ = self.create_subscription(
            ControllerCommand, self.get_namespace() + "/controller/cmd", self.control_cmd_cb, 10
        )

    def set_mode_future_cb(self, result: SetMode.Response, mode: DroneState):
        if result.mode_sent:
            self.mode = mode.state
            self.get_logger().info(f"Mode {mode} sent successfully")
        else:
            self.get_logger().warning(f"Mode {mode} not sent!")


    def set_mode(self, mode: DroneState):
        if type(mode) != DroneState:
            self.get_logger().error(f"Did not get a valid mode to set. Got {mode}")
            return
        self.get_logger().info(f"Attempting to set the mode to {mode.state}")
        
        mode_map = {
            DroneState.IDLE: "AUTO.LAND",
            DroneState.TAKING_OFF: "AUTO.TAKEOFF",
            DroneState.HOVERING: "AUTO.LOITER",
            DroneState.OFFBOARD: "OFFBOARD",
            DroneState.LANDING: "AUTO.LAND"
        }

        px4_mode = mode_map[mode.state]


        if px4_mode == "OFFBOARD": 
            if self.control_msgs < 10: 
                self.get_logger().warning(f"Did not switch to offboard mode since there were too few control msgs")
                return
            else: 
                if self.control_cmd_sub_:
                    self.destroy_subscription(self.control_cmd_sub_)
                    self.control_cmd_sub_ = None
                else: 
                    self.get_logger().warning("No control cmd subscription to destroy")
        if px4_mode != "OFFBOARD":
            self.create_control_cmd_sub()
        req = SetMode.Request()
        req.custom_mode = px4_mode

        self.set_mode_client_.call_async(req).add_done_callback(partial(self.set_mode_future_cb, mode=mode))

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
        # call the local arming service
        req = CommandBool.Request()
        req.value = msg.enable
        self.local_arm_client_.call_async(req).add_done_callback(
            lambda res: self.get_logger().info(res)
        )

    def mavros_state_cb(self, msg: State): 
        self.mavros_state = msg

    def control_cmd_cb(self, msg):
        self.control_msgs += 1

    def loop_cb(self):
        # Skip state logic if MAVROS state not available yet
        if self.mavros_state is None:
            return
            
        if self.mode == DroneState.TAKING_OFF:
            # check if the takeoff is finished.
            # px4 mode changes? 
            if self.mavros_state.mode == "AUTO.LOITER": 
                # we've finished takeoff... 
                self.set_mode(make_drone_state(DroneState.HOVERING))
        elif self.mode == DroneState.HOVERING:
            # wait n seconds and then initiate the switch to offboard mode
            if self.control_msgs > 50:
                self.set_mode(make_drone_state(DroneState.OFFBOARD))
        elif self.mode == DroneState.OFFBOARD:
            # nothing to automatically do 
            pass
        elif self.mode == DroneState.LANDING: 
            # nothign to automatically do 
            pass
        

        # if at any point that we think we're flying but we get disarmed, move back to idle. 
        if self.mode != DroneState.IDLE and not self.mavros_state.armed:
            self.get_logger().warning("Drone got disarmed, switching drone state to idle")
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
