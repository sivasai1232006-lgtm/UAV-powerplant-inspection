from functools import partial

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _start_orderly_nav2_shutdown(event, context, coordinator):
    if event.returncode == 0:
        return [coordinator]
    return []


def _shutdown_after_nav2_exit(event, context):
    return [EmitEvent(event=Shutdown(
        reason=(
            'Both UAVs landed, Nav2 lifecycle nodes were shut down, '
            'and final mission results were saved.'
        )
    ))]


def generate_launch_description():
    planner_altitudes = (3.0, 5.0, 7.0, 10.0)
    manager_services = []
    for altitude in planner_altitudes:
        planner_name = f'planner_{altitude:g}m'
        manager_services.extend(
            f'/{uav_id}/{planner_name}/lifecycle_manager_navigation/'
            'manage_nodes'
            for uav_id in ('uav_1', 'uav_2')
        )
        manager_services.append(
            f'/{planner_name}/lifecycle_manager_map/manage_nodes'
        )

    mission_manager = Node(
        package='uav_mission',
        executable='mission_manager',
        name='mission_manager',
        parameters=[{
            'use_sim_time': True,
            'results_dir': LaunchConfiguration('results_dir'),
            'save_task_level_results': ParameterValue(
                LaunchConfiguration('save_task_level_results'),
                value_type=bool,
            ),
            'simulation_shutdown_delay_sec': ParameterValue(
                LaunchConfiguration('simulation_shutdown_delay_sec'),
                value_type=float,
            ),
            'require_algorithm_comparison_results': ParameterValue(
                LaunchConfiguration('require_algorithm_comparison_results'),
                value_type=bool,
            ),
        }],
        # A successful mission exits once, then the parent launch shuts down
        # all child ROS, PX4 SITL, Gazebo, and XRCE processes cleanly.
        respawn=False,
        output='screen',
    )
    nav2_shutdown_coordinator = Node(
        package='uav_mission',
        executable='nav2_shutdown_coordinator',
        name='nav2_shutdown_coordinator',
        parameters=[{'manager_services': manager_services}],
        respawn=False,
        output='screen',
    )
    start_shutdown_handler = RegisterEventHandler(OnProcessExit(
        target_action=mission_manager,
        on_exit=partial(
            _start_orderly_nav2_shutdown,
            coordinator=nav2_shutdown_coordinator,
        ),
    ))
    finish_shutdown_handler = RegisterEventHandler(OnProcessExit(
        target_action=nav2_shutdown_coordinator,
        on_exit=_shutdown_after_nav2_exit,
    ))
    return LaunchDescription([
        DeclareLaunchArgument('results_dir', default_value=''),
        DeclareLaunchArgument(
            'save_task_level_results', default_value='true'
        ),
        DeclareLaunchArgument(
            'simulation_shutdown_delay_sec', default_value='2.0'
        ),
        DeclareLaunchArgument(
            'require_algorithm_comparison_results', default_value='false'
        ),
        start_shutdown_handler,
        finish_shutdown_handler,
        mission_manager,
    ])
