import rclpy
from rclpy.node import Node
from swarmnxt_msgs.msg import Trigger, ControllerCommand
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class DroneStateManager(Node):
    def __init__(self):
        super().__init__("drone_state_manager")
        self.get_logger().info("DroneStateManager node has been started.")

        self.loop_timer = self.create_timer(0.1, self.loop_cb)
            
        self.create_control_cmd_sub()
        namespace = self.get_namespace()
        reliable_qos = QoSProfile(reliablity=QoSReliabilityPolicy.RELIABLE, depth=10)
        self.global_takeoff_sub_ = self.create_subscription(
            Trigger, "/global/takeoff", self.takeoff_cb, reliable_qos
        )
        self.global_land_sub_ = self.create_subscription(
            Trigger, "/global/takeoff", self.land_cb, reliable_qos
        )

        self.global_start_flying_sub_ = self.create_subscription(
            Trigger,  "/global/fly", self.start_flying_cb, reliable_qos
        )       

        self.global_arm_sub_ = self.create_subscription(
            Trigger, "/global/arm", self.arm_cb, reliable_qos
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

    def create_control_cmd_sub(self):
        self.get_logger().info("Resetting control messages")
        self.control_msgs = 0
        self.control_cmd_sub_ = self.create_subscription(
            ControllerCommand, self.get_namespace() + "/controller/cmd", self.control_cmd_cb, 10
        )


    def set_mode(self, mode: str):
        def set_mode_future_cb(result: SetMode.Response):
            if result.mode_sent:
                self.get_logger().info(f"Mode {mode} sent successfully")
            else:
                self.get_logger().warning(f"Mode {mode} not sent!")

        if mode not in ["AUTO.TAKEOFF", "AUTO.LAND", "OFFBOARD"]:
            self.get_logger().warning(f"Mode {mode} not recognized. Not changing mode")
            return
        
        if mode == "OFFBOARD": 
            if self.control_msgs < 10: 
                self.get_logger().warning(f"Did not switch to offboard mode since there were too few control msgs")
                return
            else: 
                if self.control_cmd_sub_:
                    self.destroy_subscription(self.control_cmd_sub_)
                    self.control_cmd_sub_ = None
                else: 
                    self.get_logger().warning("No control cmd subscription to destroy")
        if mode != "OFFBOARD":
            self.create_control_cmd_sub()
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client_.call_async(req).add_done_callback(set_mode_future_cb)

    def takeoff_cb(self, msg: Trigger):
        if msg.enable:
            self.set_mode("AUTO.TAKEOFF")
            self.control_msgs = 0
            # start the controller

    def land_cb(self, msg):
        # call the local land service
        if msg.enable:
            self.set_mode("AUTO.LAND")

    def start_flying_cb(self, msg):
        if msg.enable: 
            self.set_mode("OFFBOARD")
    
    def arm_cb(self, msg):
        # call the local arming service
        req = CommandBool.Request()
        req.value = msg.enable
        self.local_arm_client_.call_async(req).add_done_callback(
            lambda res: self.get_logger().info(res)
        )
    
    def control_cmd_cb(self, msg):
        self.control_msgs += 1

    def loop_cb(self):
        # check the state?
        # if at goal: 
            # land()
        pass


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
