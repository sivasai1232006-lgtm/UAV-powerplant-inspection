from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_mission',
            executable='mission_manager',
            name='mission_manager',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
