from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share = get_package_share_directory('oculus_hmd_sim')
    world = os.path.join(share, 'worlds', 'hmd_world.sdf')
    urdf  = os.path.join(share, 'urdf',  'hmd_head.urdf')
    yaml  = os.path.join(share, 'config','bridge.yaml')

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world}'}.items()
    )

    # Spawn directo del URDF en el mundo (sin robot_description)
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', 'hmd_world',
                   '-file', urdf,
                   '-name', 'hmd_head']
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={yaml}'],
        output='screen'
    )


    return LaunchDescription([gz, spawn, bridge])
