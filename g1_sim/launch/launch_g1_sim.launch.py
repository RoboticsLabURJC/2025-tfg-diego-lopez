from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('g1_sim')

    # Ruta al launch "antiguo" (ahora fuera de launch/)
    core_launch = os.path.join(pkg_share, 'g1_sim', 'sim_core.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core_launch),
            launch_arguments={
                'world': 'empty',
                'robot_model': 'g1_free_roam',
                'bridge_config': 'auto',
                'spawn_x': '0.0',
                'spawn_y': '0.0',
                'spawn_z': '0.80',
                'gui': 'true',
                'run': 'true',
                'use_software_rendering': 'false',
                'verbose': '1',
            }.items()
        )
    ])