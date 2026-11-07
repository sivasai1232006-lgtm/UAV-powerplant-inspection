import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start_simulation(context):
    px4_dir_value = LaunchConfiguration('px4_dir').perform(context).strip()
    if not px4_dir_value:
        px4_dir_value = os.environ.get('PX4_AUTOPILOT_DIR', '').strip()
    if not px4_dir_value:
        raise RuntimeError(
            'Set PX4_AUTOPILOT_DIR or pass px4_dir:=/path/to/PX4-Autopilot.'
        )
    px4_dir = Path(px4_dir_value).expanduser()
    build_dir = px4_dir / 'build/px4_sitl_default'
    px4_binary = build_dir / 'bin/px4'
    if not px4_binary.is_file():
        raise RuntimeError(f'PX4 SITL binary not found: {px4_binary}')

    sim_share = Path(get_package_share_directory('uav_sim'))
    world_value = LaunchConfiguration('world').perform(context).strip()
    world = (
        Path(world_value).expanduser()
        if world_value
        else sim_share / 'worlds/electrical_substation.world'
    )
    gazebo_world_name = LaunchConfiguration(
        'gazebo_world_name'
    ).perform(context)
    models = px4_dir / 'Tools/simulation/gz/models'
    plugin_dir = build_dir / 'src/modules/simulation/gz_plugins'
    server_config = px4_dir / 'src/modules/simulation/gz_bridge/server.config'
    resource_root = sim_share.parent
    resource_paths = [str(resource_root), str(models)]
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH')
    if existing:
        resource_paths.append(existing)
    plugin_paths = [str(plugin_dir)]
    existing_plugins = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH')
    if existing_plugins:
        plugin_paths.append(existing_plugins)

    actions = [
        SetEnvironmentVariable('GZ_VERSION', 'harmonic'),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', ':'.join(resource_paths)
        ),
        SetEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH', ':'.join(plugin_paths)
        ),
        SetEnvironmentVariable(
            'GZ_SIM_SERVER_CONFIG_PATH', str(server_config)
        ),
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', str(world)],
            output='screen',
        ),
        TimerAction(
            period=3.0,
            actions=[Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='gazebo_clock_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                output='screen',
            )],
        ),
    ]

    vehicles = (
        (
            LaunchConfiguration('uav_1_instance').perform(context),
            'uav_1',
            LaunchConfiguration('uav_1_spawn_pose').perform(context),
        ),
        (
            LaunchConfiguration('uav_2_instance').perform(context),
            'uav_2',
            LaunchConfiguration('uav_2_spawn_pose').perform(context),
        ),
    )
    for index, (instance, uav_id, spawn_pose) in enumerate(vehicles):
        env = {
            'PX4_SYS_AUTOSTART': '4013',
            'PX4_SIM_MODEL': 'gz_x500_lidar_2d',
            'PX4_GZ_WORLD': gazebo_world_name,
            'PX4_GZ_MODEL_POSE': spawn_pose,
            'PX4_UXRCE_DDS_NS': uav_id,
            'PX4_UXRCE_DDS_PORT': '8888',
        }
        actions.append(TimerAction(
            period=6.0 + index * 4.0,
            actions=[ExecuteProcess(
                cmd=[str(px4_binary), '-i', instance],
                cwd=str(build_dir),
                additional_env=env,
                output='screen',
            )],
        ))

        topic = (
            f'/world/{gazebo_world_name}/model/x500_lidar_2d_{instance}/link/link/'
            'sensor/lidar_2d_v2/scan'
        )
        actions.append(TimerAction(
            period=14.0 + index * 4.0,
            actions=[Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'{uav_id}_scan_bridge',
                arguments=[
                    f'{topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
                ],
                remappings=[(topic, f'/{uav_id}/scan/raw')],
                output='screen',
            )],
        ))

    enable_obstacle = LaunchConfiguration(
        'enable_dynamic_obstacle'
    ).perform(context).lower() == 'true'
    if enable_obstacle:
        obstacle_sdf = sim_share / 'models/demo_dynamic_obstacle/model.sdf'
        request = (
            f'sdf_filename: "{obstacle_sdf}", name: "demo_dynamic_obstacle", '
            'allow_renaming: false, pose: {position: {x: 0, y: 2, z: 8}, '
            'orientation: {w: 1}}'
        )
        actions.append(TimerAction(
            period=12.0,
            actions=[ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s',
                    f'/world/{gazebo_world_name}/create',
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean', '--timeout', '5000',
                    '--req', request,
                ],
                output='screen',
            )],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'px4_dir',
            default_value=os.environ.get('PX4_AUTOPILOT_DIR', ''),
        ),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument(
            'gazebo_world_name', default_value='powerplant'
        ),
        DeclareLaunchArgument('uav_1_instance', default_value='1'),
        DeclareLaunchArgument('uav_2_instance', default_value='2'),
        DeclareLaunchArgument(
            'uav_1_spawn_pose', default_value='-5,0,0.2,0,0,0'
        ),
        DeclareLaunchArgument(
            'uav_2_spawn_pose', default_value='5,0,0.2,0,0,0'
        ),
        DeclareLaunchArgument(
            'enable_dynamic_obstacle', default_value='false'
        ),
        OpaqueFunction(function=_start_simulation),
    ])
