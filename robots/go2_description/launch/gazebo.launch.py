from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('go2_description')

    os.environ["GZ_SIM_RESOURCE_PATH"] = (
        os.environ.get("GZ_SIM_RESOURCE_PATH", "") + ":" + pkg_share
    )

    urdf_file = os.path.join(pkg_share, 'urdf', 'go2_description.urdf')

    with open(urdf_file, 'r') as f:
        urdf_content = f.read()

    urdf_fixed = urdf_content.replace(
        'package://go2_description/dae',
        os.path.join(pkg_share, 'meshes')
    )

    tmp_urdf = '/tmp/go2_fixed.urdf'
    with open(tmp_urdf, 'w') as f:
        f.write(urdf_fixed)

    gui = os.environ.get("DISPLAY") is not None
    gazebo_cmd = ['gz', 'sim', '-r', 'empty.sdf']
    if not gui:
        gazebo_cmd.append('-s')

    return LaunchDescription([
        ExecuteProcess(
            cmd=gazebo_cmd,
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_go2',
            arguments=['-file', tmp_urdf, '-name', 'go2_description', '-world', 'empty', '-z', '1'],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='joint_state_bridge',
            arguments=[
                '/model/go2_description/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model'
            ],
            output='screen'
        )
    ])
