from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo, EmitEvent
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

import os


def generate_launch_description():

    try:
        sim_path = get_package_share_directory('g1_sim')
    except PackageNotFoundError:
        return LaunchDescription([
            LogInfo(msg="[ERROR] g1_sim package not found.\n\nThis package is required for running the simulation.\n\nYou can install it from:\nhttps://github.com/RoboticsLabURJC/2025-tfg-diego-lopez\n\nOr run the controller without simulation:\nros2 run g1_locomotion g1_locomotion_main"),
            EmitEvent(event=Shutdown())
        ])

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_path, 'launch', 'launch_g1_sim.launch.py')
        )
    )

    locomotion_node = Node(
        package='g1_locomotion',
        executable='g1_locomotion_main',
        output='screen'
    )

    return LaunchDescription([
        locomotion_node,
        TimerAction(period=2.0, actions=[sim_launch])
    ])