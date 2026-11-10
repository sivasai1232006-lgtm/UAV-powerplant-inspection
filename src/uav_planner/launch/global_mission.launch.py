import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package, launch_file, arguments=None):
    launch_path = (
        Path(get_package_share_directory(package)) / 'launch' / launch_file
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_path)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    px4_dir = LaunchConfiguration('px4_dir')
    world = LaunchConfiguration('world')
    gazebo_world_name = LaunchConfiguration('gazebo_world_name')
    uav_1_instance = LaunchConfiguration('uav_1_instance')
    uav_2_instance = LaunchConfiguration('uav_2_instance')
    uav_1_spawn_pose = LaunchConfiguration('uav_1_spawn_pose')
    uav_2_spawn_pose = LaunchConfiguration('uav_2_spawn_pose')
    dynamic_obstacle = LaunchConfiguration('enable_dynamic_obstacle')
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
        DeclareLaunchArgument('results_dir', default_value=''),
        DeclareLaunchArgument(
            'save_task_level_results', default_value='true'
        ),
        DeclareLaunchArgument(
            'simulation_shutdown_delay_sec', default_value='2.0'
        ),
        DeclareLaunchArgument(
            'require_algorithm_comparison_results',
            default_value='true',
        ),
        # The comparison runner is independent of the live UAV clients and
        # finishes its complete three-algorithm output before normal shutdown.
        _include(
            'benchmarks', 'benchmark.launch.py',
            {'results_dir': LaunchConfiguration('results_dir')},
        ),
        _include(
            'uav_sim',
            'two_uav_gazebo.launch.py',
            {
                'px4_dir': px4_dir,
                'world': world,
                'gazebo_world_name': gazebo_world_name,
                'uav_1_instance': uav_1_instance,
                'uav_2_instance': uav_2_instance,
                'uav_1_spawn_pose': uav_1_spawn_pose,
                'uav_2_spawn_pose': uav_2_spawn_pose,
                'enable_dynamic_obstacle': dynamic_obstacle,
            },
        ),
        TimerAction(
            period=12.0,
            actions=[
                _include('uav_info', 'uav_info.launch.py'),
                _include('uav_control', 'two_uav.launch.py'),
            ],
        ),
        TimerAction(
            period=16.0,
            actions=[_include('uav_planner', 'static_planner.launch.py')],
        ),
        TimerAction(
            period=21.0,
            actions=[_include(
                'uav_mission', 'mission.launch.py',
                {
                    'results_dir': LaunchConfiguration('results_dir'),
                    'save_task_level_results': LaunchConfiguration(
                        'save_task_level_results'
                    ),
                    'simulation_shutdown_delay_sec': LaunchConfiguration(
                        'simulation_shutdown_delay_sec'
                    ),
                    'require_algorithm_comparison_results':
                        LaunchConfiguration(
                        'require_algorithm_comparison_results'
                    ),
                },
            )],
        ),
        TimerAction(
            period=24.0,
            actions=[_include('uav_allocator', 'allocator.launch.py')],
        ),
    ])
