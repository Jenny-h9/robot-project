from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('supermarket_capture')) / 'config' / 'capture.yaml'
    return LaunchDescription([
        Node(
            package='supermarket_capture',
            executable='scheduler_node',
            name='supermarket_capture',
            output='screen',
            parameters=[str(config)],
        )
    ])
