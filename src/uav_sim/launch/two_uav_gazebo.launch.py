import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def _start_simulation(context):
    #retrieves PX4 launch argument
    px4_dir = LaunchConfiguration('px4_dir').perform(context).strip()
    # If not set, take from env variables
    if not px4_dir:
        px4_dir = os.environ.get('PX4_AUTOPILOT_DIR', '').strip()
    if not px4_dir:
        raise RuntimeError('Set PX4_AUTOPILOT environment')
    # path for px4 executable and error handling
    px4_dir = Path(px4_dir).expanduser()
    build_dir = px4_dir / 'build/px4_sitl_default'
    px4_binary = build_dir / 'bin/px4'
    if not px4_binary.is_file():
        raise RuntimeError(f'PX4 SITL binary not found: {px4_binary}')
    # path for world and error handling
    sim_share = Path(get_package_share_directory('uav_sim'))
    world_value = LaunchConfiguration('world').perform(context).strip()
    world = (Path(world_value).expanduser() if world_value else sim_share / 'worlds/electrical_substation.world')
    gazebo_world_name = LaunchConfiguration('gazebo_world_name').perform(context)
    # path for models and plugins
    models = px4_dir / 'Tools/simulation/gz/models'
    plugin_dir = build_dir / 'src/modules/simulation/gz_plugins'
    server_config = px4_dir / 'src/modules/simulation/gz_bridge/server.config'
    # env variables for gz sim
    resource_root = sim_share.parent
    resource_paths = [str(resource_root), str(models)]
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH')
    if existing:
        resource_paths.append(existing)

    plugin_paths = [str(plugin_dir)]
    existing_plugins = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH')
    if existing_plugins:
        plugin_paths.append(existing_plugins)

    # list of actions to be done.
    actions = [
        SetEnvironmentVariable('GZ_VERSION', 'harmonic'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join(resource_paths)),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', ':'.join(plugin_paths)),
        SetEnvironmentVariable('GZ_SIM_SERVER_CONFIG_PATH', str(server_config)),
        ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],output='screen',),
        ExecuteProcess(cmd=['gz', 'sim', '-r', str(world)],output='screen',),
        TimerAction(period=3.0,
            actions=[Node(package='ros_gz_bridge',
                executable='parameter_bridge',
                name='gazebo_clock_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                output='screen',
            )]
        )
    ]

    vehicles = (
        (LaunchConfiguration('uav_1_instance').perform(context), 'uav_1',LaunchConfiguration('uav_1_spawn_pose').perform(context)),
        (LaunchConfiguration('uav_2_instance').perform(context),'uav_2',LaunchConfiguration('uav_2_spawn_pose').perform(context)),
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
                additional_env= {key: TextSubstitution(text=value) for key, value in env.items()},
                output='screen',
            )],
        ))

        topic = (f'/world/{gazebo_world_name}/model/x500_lidar_2d_{instance}/link/link/' 'sensor/lidar_2d_v2/scan')
        actions.append(TimerAction(
            period=14.0 + index * 4.0,
            actions=[Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'{uav_id}_scan_bridge',
                arguments=[f'{topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
                remappings=[(topic, f'/{uav_id}/scan/raw')],
                output='screen',
            )],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('px4_dir',default_value=os.environ.get('PX4_AUTOPILOT_DIR', '')),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('gazebo_world_name', default_value='powerplant'),
        DeclareLaunchArgument('uav_1_instance', default_value='1'),
        DeclareLaunchArgument('uav_2_instance', default_value='2'),
        DeclareLaunchArgument('uav_1_spawn_pose', default_value='-5,0,0.2,0,0,0'),
        DeclareLaunchArgument('uav_2_spawn_pose', default_value='5,0,0.2,0,0,0'),
        OpaqueFunction(function=_start_simulation),
    ])
