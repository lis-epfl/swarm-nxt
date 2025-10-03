#!/usr/bin/env python3

from rclpy.node import Node
import rclpy
from geometry_msgs.msg import PointStamped
import time


class GoalGiver(Node):
    def __init__(self):
        super().__init__("goal_giver")
        self.get_logger().info("GoalGiver node has been started.")
        self.declare_parameter("agent_name", "agent_0")

        agent_name = self.get_parameter("agent_name").get_parameter_value().string_value

        self.clicked_point_sub_ = self.create_subscription(
            PointStamped, "/clicked_point", self.point_cb, 10
        )

        self.agent_goal_pub_ = self.create_publisher(
            PointStamped, agent_name + "/goal", 10
        )

    def point_cb(self, msg: PointStamped):
        point = msg
        point.point.z = float(1)

        for i in range(5): 

            self.agent_goal_pub_.publish(
                point
            )
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = GoalGiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
