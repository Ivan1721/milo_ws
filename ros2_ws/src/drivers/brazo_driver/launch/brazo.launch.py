import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('brazo_driver'), 'config',
                          'brazo_driver.yaml')
    return LaunchDescription([
        Node(package='brazo_driver', executable='driver', name='arm_driver_node',
             parameters=[config], output='screen', emulate_tty=True),
    ])
