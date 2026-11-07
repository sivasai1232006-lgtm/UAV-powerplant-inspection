import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('uav_control'),
        'config',
        'uav_param.yaml',
    )

    controllers = [
        Node(
            package='uav_control',
            executable='controller',      # name from setup.py console_scripts
            namespace=ns,
            name='offboard_controller',   # same name used in super().__init__()
            parameters=[params_file, {'use_sim_time': True}],
            output='screen',
        )
        for ns in ('uav_1', 'uav_2')
    ]

    return LaunchDescription(controllers)
