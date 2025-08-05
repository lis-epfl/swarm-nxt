from setuptools import setup

package_name = 'drone_state_manager_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=[
        'rclpy',
        'std_msgs',
        'geometry_msgs',
        'swarmnxt_msgs'
    ],
    entry_points={
        'console_scripts': [
            'drone_state_manager = drone_state_manager_ros2.main:main',
        ],
    },
    data_files=[    
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/drone_state_manager_ros2', ['package.xml'])
    ]
)