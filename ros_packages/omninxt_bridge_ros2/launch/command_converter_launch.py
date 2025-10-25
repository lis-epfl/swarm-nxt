#!/usr/bin/env python3
"""
Launch file for command converter, trajectory converter, and MPC controller
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Configuration files
    bridge_config_file = PathJoinSubstitution([
        FindPackageShare('omninxt_bridge_ros2'),
        'config',
        'command_converter_params.yaml'
    ])
    
    # Launch arguments
    cmd_output_topic_arg = DeclareLaunchArgument(
        'cmd_output_topic',
        default_value='/controller/cmd',
        description='Output topic for controller commands (swarmnxt_msgs)'
    )
    
    # Command converter node
    command_converter_node = Node(
        package='omninxt_bridge_ros2',
        executable='command_converter_node',
        name='command_converter',
        output='screen',
        parameters=[
            bridge_config_file,
            {
                'output_topic': LaunchConfiguration('cmd_output_topic'),
            }
        ]
    )
    
    return LaunchDescription([
        cmd_output_topic_arg,
        command_converter_node,
    ])
