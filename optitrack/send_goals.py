#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Goals

class MultiRobotGoalManager(Node):
    def __init__(self):
        super().__init__('multi_robot_goal_manager')
        
        # Declare parameters
        self.declare_parameter('robot_names', ['nxt1', 'nxt2', 'nxt3'])
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('publish_rate', 10.0)
        
        # Get parameters
        self.robot_names = self.get_parameter('robot_names').get_parameter_value().string_array_value
        if not self.robot_names:  # Fallback if parameter is empty
            self.robot_names = ['nxt1', 'nxt2', 'nxt3']
        
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Define waypoints for each robot (x, y, z, orientation_w)
        # You can customize these waypoints for each robot
        self.waypoints = {
            'nxt1': [
                (0.0, 0.0, 0.0, 1.0),
                (0.2, 0.0, 0.0, 1.0),
                (0.2, 0.2, 0.0, 1.0),
                (0.0, 0.2, 0.0, 1.0)
            ],
            'nxt2': [
                (1.0, 1.0, 0.0, 1.0),
                (3.0, 1.0, 0.0, 1.0),
                (3.0, 3.0, 0.0, 1.0),
                (1.0, 3.0, 0.0, 1.0)
            ],
            'nxt3': [
                (-1.0, -1.0, 0.0, 1.0),
                (1.0, -1.0, 0.0, 1.0),
                (1.0, 1.0, 0.0, 1.0),
                (-1.0, 1.0, 0.0, 1.0)
            ]
        }
        
        # Initialize robot states
        self.robot_states = {}
        self.goal_publishers = {}
        self.pose_subscribers = {}
        
        for robot_name in self.robot_names:
            self.robot_states[robot_name] = {
                'current_goal_index': 0,
                'current_pose': None,
                'goal_reached': False
            }
            
            # Create goal publisher for each robot
            goal_topic = f'/{robot_name}/goals'
            self.goal_publishers[robot_name] = self.create_publisher(
                Goals, goal_topic, qos_profile
            )
            
            # Create pose subscriber for each robot
            pose_topic = f'/{robot_name}/mavros/local_position/pose'
            self.pose_subscribers[robot_name] = self.create_subscription(
                PoseStamped, pose_topic,
                lambda msg, name=robot_name: self.pose_callback(msg, name),
                qos_profile
            )
        
        # Create timer for periodic goal publishing and status updates
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.status_timer = self.create_timer(5.0, self.print_status)
        
        # Publish initial goals after a short delay
        self.create_timer(1.0, self.publish_initial_goals)
        
        self.get_logger().info("Multi-Robot Goal Manager initialized")
        self.get_logger().info(f"Managing {len(self.robot_names)} robots: {self.robot_names}")
        self.get_logger().info(f"Goal tolerance: {self.goal_tolerance} meters")
    
    def publish_initial_goals(self):
        """Publish initial goals once after startup"""
        self.publish_current_goals()
        self.destroy_timer(self.timer)  # Remove the one-time timer
    
    def pose_callback(self, msg, robot_name):
        """Callback function for pose updates from each robot"""
        self.robot_states[robot_name]['current_pose'] = msg
        
        # Check if current goal is reached
        if self.is_goal_reached(robot_name):
            self.advance_to_next_goal(robot_name)
    
    def is_goal_reached(self, robot_name):
        """Check if the robot has reached its current goal"""
        state = self.robot_states[robot_name]
        
        if state['current_pose'] is None:
            return False
        
        current_goal_index = state['current_goal_index']
        waypoints = self.waypoints.get(robot_name, [])
        
        if current_goal_index >= len(waypoints):
            return True  # All goals completed
        
        # Get current goal position
        goal_x, goal_y, goal_z, _ = waypoints[current_goal_index]
        
        # Get current robot position
        current_pos = state['current_pose'].pose.position
        
        # Calculate distance to goal
        distance = math.sqrt(
            (current_pos.x - goal_x) ** 2 +
            (current_pos.y - goal_y) ** 2 +
            (current_pos.z - goal_z) ** 2
        )
        
        return distance <= self.goal_tolerance
    
    def advance_to_next_goal(self, robot_name):
        """Advance robot to the next goal in sequence"""
        state = self.robot_states[robot_name]
        waypoints = self.waypoints.get(robot_name, [])
        
        current_index = state['current_goal_index']
        
        self.get_logger().info(f"{robot_name}: Goal {current_index + 1} reached!")
        
        # Advance to next goal
        state['current_goal_index'] += 1
        
        if state['current_goal_index'] >= len(waypoints):
            self.get_logger().info(f"{robot_name}: All goals completed! Cycling back to start.")
            state['current_goal_index'] = 0  # Cycle back to first goal
        
        # Publish the new goal
        self.publish_goal(robot_name)
    
    def publish_goal(self, robot_name):
        """Publish current goal for a specific robot"""
        state = self.robot_states[robot_name]
        waypoints = self.waypoints.get(robot_name, [])
        
        if not waypoints:
            self.get_logger().warn(f"No waypoints defined for {robot_name}")
            return
        
        current_index = state['current_goal_index']
        if current_index >= len(waypoints):
            return
        
        # Create goal message
        pose_stamped = PoseStamped()
        pose_stamped.header = Header()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "map"  # Adjust frame_id as needed
        
        # Set goal position and orientation
        x, y, z, w = waypoints[current_index]
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = w

        goal_msg = Goals()

        goal_msg.header = Header()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"

        goal_msg.goals.append(pose_stamped)
        
        # Publish goal
        self.goal_publishers[robot_name].publish(goal_msg)
        
        self.get_logger().info(f"{robot_name}: Published goal {current_index + 1}/{len(waypoints)} "
                              f"at ({x:.2f}, {y:.2f}, {z:.2f})")
    
    def publish_current_goals(self):
        """Publish current goals for all robots"""
        for robot_name in self.robot_names:
            self.publish_goal(robot_name)
    
    def timer_callback(self):
        """Timer callback for periodic goal publishing"""
        self.publish_current_goals()
    
    def print_status(self):
        """Print current status of all robots"""
        self.get_logger().info("=== Robot Status ===")
        for robot_name in self.robot_names:
            state = self.robot_states[robot_name]
            waypoints = self.waypoints.get(robot_name, [])
            current_index = state['current_goal_index']
            
            if state['current_pose']:
                pos = state['current_pose'].pose.position
                self.get_logger().info(f"{robot_name}: Goal {current_index + 1}/{len(waypoints)}, "
                                     f"Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
            else:
                self.get_logger().info(f"{robot_name}: Goal {current_index + 1}/{len(waypoints)}, "
                                     f"Position: No pose data")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        manager = MultiRobotGoalManager()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in Multi-Robot Goal Manager: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()