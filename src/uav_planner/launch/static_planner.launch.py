from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

from uav_planner.altitude_layers import altitude_label, altitude_layers


def generate_launch_description():
    share = Path(get_package_share_directory('uav_planner'))
    with (share / 'config/static_map.yaml').open(
        encoding='utf-8'
    ) as config_file:
        map_config = yaml.safe_load(config_file) or {}
    layers = altitude_layers(map_config.get('flight_altitudes', ()))
    nodes = []
    for altitude, map_name, planner_name in layers:
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
                remappings=[('map', 'structural_map')],
                output='screen',
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                namespace=planner_name,
                name='lifecycle_manager_map',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': ['map_server'],
                }],
                output='screen',
            ),
        ])
        for uav_id in ('uav_1', 'uav_2'):
            namespace = f'{uav_id}/{planner_name}'
            params_path = share / (
                f'config/nav2_planner_{uav_id}_'
                f'{altitude_label(altitude)}m.yaml'
            )
            nodes.extend([
                Node(
                    package='uav_planner',
                    executable='altitude_map_fuser',
                    namespace=uav_id,
                    name=f'altitude_map_fuser_{altitude_label(altitude)}m',
                    parameters=[{
                        'structural_map_topic': (
                            f'/{planner_name}/structural_map'
                        ),
                        'base_map_topic': '/map',
                        'output_map_topic': (
                            f'/{uav_id}/{planner_name}/map'
                        ),
                    }],
                    output='screen',
                ),
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    namespace=namespace,
                    name='planner_server',
                    parameters=[str(params_path)],
                    output='screen',
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    namespace=namespace,
                    name='lifecycle_manager_navigation',
                    parameters=[{
                        'use_sim_time': True,
                        'autostart': True,
                        'node_names': ['planner_server'],
                    }],
                    output='screen',
                ),
            ])
    return LaunchDescription(nodes)
