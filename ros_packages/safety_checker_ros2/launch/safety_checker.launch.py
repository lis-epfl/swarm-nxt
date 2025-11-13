from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    safety_params_file = PathJoinSubstitution([
        FindPackageShare('safety_checker_ros2'),
        'config',
        'safety_params.yaml'
    ])

    safety_checker_node = Node(
        package='safety_checker_ros2',
        executable='safety_checker_node',
        name='safety_checker',
        parameters=[safety_params_file],
        # prefix=['xterm -e gdb -ex run --args'],
        output='screen',
        emulate_tty=True
    )

    ld.add_action(safety_checker_node)
    return ld
