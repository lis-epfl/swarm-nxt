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
        default_value='~/data/ros2_swarmnxt_ws/config/peer_list.yaml',
        description='Path to the peers.yaml file containing drone list'
    )
    
    hdsm_mapping_file_arg = DeclareLaunchArgument(
        'hdsm_mapping_file',
        default_value='~/data/ros2_swarmnxt_ws/config/hdsm_planner_map.yaml',
        description='Path to the HDSM planner mapping JSON file'
    )
    
    # Create the drone GUI node
    drone_gui_node = Node(
        package='drone_gui_ros2',
        executable='drone_gui_node.py',
        name='drone_gui_node',
        output='screen',
        parameters=[
            {'port': LaunchConfiguration('port')},
            {'peers_file': LaunchConfiguration('peers_file')},
            {'hdsm_mapping_file': LaunchConfiguration('hdsm_mapping_file')}
        ]
    )
    
    return LaunchDescription([
        port_arg,
        peers_file_arg,
        hdsm_mapping_file_arg,
        drone_gui_node
    ])
