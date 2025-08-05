import rclpy
from rclpy.node import Node
from swarmnxt_msgs.msg import Trigger
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class DroneStateManager(Node):
    def __init__(self):
        super().__init__("drone_state_manager")
        self.get_logger().info("DroneStateManager node has been started.")

        self.loop_timer = self.create_timer(0.1, self.loop_cb)


        namespace = self.get_namespace()
        reliable_qos = QoSProfile(reliablity=QoSReliabilityPolicy.RELIABLE, depth=10)
        self.global_takeoff_sub_ = self.create_subscription(
            Trigger, "/global/takeoff", self.takeoff_cb, reliable_qos
        )
        self.global_land_sub_ = self.create_subscription(
            Trigger, "/global/takeoff", self.land_cb, reliable_qos
        )
        self.global_arm_sub_ = self.create_subscription(
            Trigger, "global/arm", self.arm_cb, reliable_qos
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

    def set_mode(self, mode: str):
        def set_mode_future_cb(result: SetMode.Response):
            if result.mode_sent:
                self.get_logger().info(f"Mode {mode} sent successfully")
            else:
                self.get_logger().warning(f"Mode {mode} not sent!")

        if mode not in ["AUTO.TAKEOFF", "AUTO.LAND", "OFFBOARD"]:
            self.get_logger().warning(f"Mode {mode} not recognized. Not changing mode")
            return
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client_.call_async(req).add_done_callback(set_mode_future_cb)

    def takeoff_cb(self, msg: Trigger):
        if msg.enable:
            self.set_mode("AUTO.TAKEOFF")

    def land_cb(self, msg):
        # call the local land service
        if msg.enable:
            self.set_mode("AUTO.LAND")

    def arm_cb(self, msg):
        # call the local arming service
        req = CommandBool.Request()
        req.value = msg.enable
        self.local_arm_client_.call_async(req).add_done_callback(
            lambda res: self.get_logger().info(res)
        )
    
    def loop_cb(self):
        # check the state?

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
