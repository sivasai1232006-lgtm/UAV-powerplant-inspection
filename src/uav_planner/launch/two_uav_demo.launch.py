import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package, launch_file, arguments=None):
    launch_path = os.path.join(
        get_package_share_directory(package), 'launch', launch_file
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_path)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'px4_dir',
            default_value=os.environ.get('PX4_AUTOPILOT_DIR', ''),
        ),
        DeclareLaunchArgument(
            'enable_dynamic_obstacle', default_value='false'
        ),
        _include(
            'uav_planner',
            'global_mission.launch.py',
            {
                'px4_dir': LaunchConfiguration('px4_dir'),
                'enable_dynamic_obstacle': LaunchConfiguration(
                    'enable_dynamic_obstacle'
                ),
            },
        ),
    ])
