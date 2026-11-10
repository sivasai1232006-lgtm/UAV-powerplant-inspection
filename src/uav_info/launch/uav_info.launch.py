import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('uav_info'),
        'config',
        'uav.yaml',
    )
    frames_path = os.path.join(
        get_package_share_directory('uav_info'),
        'config',
        'coordinate_frames.yaml',
    )

    nodes = [
        Node(
            package='uav_info',
            executable='uav_info',
            namespace=uav_id,
            name='uav_info',
            parameters=[
                config_path,
                frames_path,
                {
                    'uav.id': uav_id,
                    'frames.local': f'{uav_id}/odom',
                    'frames.body': f'{uav_id}/base_link',
                    'use_sim_time': True,
                },
            ],
            output='screen',
        )
        for uav_id in ('uav_1', 'uav_2')
    ]
    nodes.extend(
        Node(
            package='uav_info',
            executable='scan_layer_relay',
            namespace=uav_id,
            parameters=[{'uav_id': uav_id}],
            output='screen',
        )
        for uav_id in ('uav_1', 'uav_2')
    )

    return LaunchDescription(nodes)
