from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    drone_planner = Node(
        package='drone_planner_ros2',
        executable='drone_planner_node',
        name='drone_planner_node',
        parameters=[],
        # prefix=['xterm -e gdb -ex run --args'],
        output='screen',
        emulate_tty=True
    )

    ld.add_action(drone_planner)
    return ld