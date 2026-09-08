from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('g1_sim')

    core_launch = os.path.join(
        pkg_share,
        'g1_sim',
        'sim_core.py'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'world',
            default_value='empty'
        ),

        DeclareLaunchArgument(
            'robot_model',
            default_value='g1_free_roam'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core_launch),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'robot_model': LaunchConfiguration('robot_model'),

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