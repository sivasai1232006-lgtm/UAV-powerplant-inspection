from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('uav_planner'))
    nodes = []
    for altitude, map_name, planner_name in (
        (8, 'static_map_8m', 'planner_8m'),
        (10, 'static_map', 'planner_10m'),
    ):
        params_path = share / f'config/nav2_planner_{altitude}m.yaml'
        if altitude == 8:
            params_path = share / 'config/nav2_planner.yaml'
        map_path = share / f'maps/{map_name}.yaml'
        nodes.extend([
            Node(
                package='nav2_map_server',
                executable='map_server',
                namespace=planner_name,
                name='map_server',
                parameters=[{
                    'use_sim_time': True,
                    'yaml_filename': str(map_path),
                    'frame_id': 'map',
                }],
                output='screen',
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                namespace=planner_name,
                name='planner_server',
                parameters=[str(params_path)],
                output='screen',
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                namespace=planner_name,
                name='lifecycle_manager_navigation',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': ['map_server', 'planner_server'],
                }],
                output='screen',
            ),
        ])
    return LaunchDescription(nodes)
