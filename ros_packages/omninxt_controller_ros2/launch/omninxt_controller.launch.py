from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    omninxt_controller = Node(
        package='omninxt_controller_ros2',
        executable='omninxt_controller_node',
        name='omninxt_controller_node',
        parameters=[],
        # prefix=['xterm -e gdb -ex run --args'],
        output='screen',
        emulate_tty=True
    )

    ld.add_action(omninxt_controller)
    return ld