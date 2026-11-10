"""Launch isolated benchmark process; this launch file has no UAV clients."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    mission = Path(get_package_share_directory('uav_mission')) / 'config/mission.yaml'
    allocator = Path(get_package_share_directory('uav_allocator')) / 'config/allocator.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('mission_config', default_value=str(mission)),
        DeclareLaunchArgument('allocator_config', default_value=str(allocator)),
        DeclareLaunchArgument('results_dir', default_value=''),
        DeclareLaunchArgument('milp_time_limit_sec', default_value='10.0'),
        DeclareLaunchArgument('max_tasks_per_uav', default_value='0'),
        Node(
            package='benchmarks', executable='run_benchmark',
            output='screen',
            arguments=[
                '--mission-config', LaunchConfiguration('mission_config'),
                '--allocator-config', LaunchConfiguration('allocator_config'),
                '--results-dir', LaunchConfiguration('results_dir'),
                '--milp-time-limit-sec', LaunchConfiguration('milp_time_limit_sec'),
                '--max-tasks-per-uav', LaunchConfiguration('max_tasks_per_uav'),
            ],
        ),
    ])
