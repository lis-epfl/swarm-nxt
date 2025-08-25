from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='logger_ros2',
            executable='logger_node',
            name='logger_node',
            output='screen',
            parameters=[{'topics_to_log': []}],  # You can specify default topics here
        ),
    ])