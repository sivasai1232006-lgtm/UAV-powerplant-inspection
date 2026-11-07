import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('uav_allocator'),
        'config',
        'allocator.yaml',
    )
    return LaunchDescription([
        Node(
            package='uav_allocator',
            executable='auction_allocator',
            name='auction_allocator',
            parameters=[config_path],
            output='screen',
        ),
    ])
