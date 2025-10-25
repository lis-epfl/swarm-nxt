from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    safety_checker_node = Node(
        package='safety_checker_ros2',
        executable='safety_checker_node',
        name='safety_checker_node',
        parameters=[],
        # prefix=['xterm -e gdb -ex run --args'],
        output='screen',
        emulate_tty=True
    )

    ld.add_action(safety_checker_node)
    return ld
