from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8080',
        description='Port for the web server'
    )
    
    peers_file_arg = DeclareLaunchArgument(
        'peers_file',
        default_value='~/ros/config/peers.yaml',
        description='Path to the peers.yaml file containing drone list'
    )
    
    # Create the drone GUI node
    drone_gui_node = Node(
        package='drone_gui_ros2',
        executable='drone_gui_node.py',
        name='drone_gui_node',
        output='screen',
        parameters=[
            {'port': LaunchConfiguration('port')},
            {'peers_file': LaunchConfiguration('peers_file')}
        ]
    )
    
    return LaunchDescription([
        port_arg,
        peers_file_arg,
        drone_gui_node
    ])