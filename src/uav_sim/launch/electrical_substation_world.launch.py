
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():
    sim_share = Path(get_package_share_directory('uav_sim'))
    world = sim_share / 'worlds/electrical_substation.world'
    resource_paths = [str(sim_share.parent)]
    px4_dir = os.environ.get('PX4_AUTOPILOT_DIR')
    if px4_dir:
        px4_models = Path(px4_dir) / 'Tools/simulation/gz/models'
        if px4_models.is_dir():
            resource_paths.append(str(px4_models))

    existing_paths = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if existing_paths:
        resource_paths.append(existing_paths)

    return LaunchDescription([
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=":".join(resource_paths),
        ),

        SetEnvironmentVariable(
            name="GZ_VERSION",
            value="harmonic",
        ),

        ExecuteProcess(
            cmd=["gz", "sim", "-r", str(world)],
            output="screen",
        ),
    ])
