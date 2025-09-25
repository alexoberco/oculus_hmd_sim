from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share = get_package_share_directory('oculus_hmd_sim')
    world = os.path.join(share, 'worlds', 'hmd_world.sdf')
    urdf  = os.path.join(share, 'urdf',  'hmd_head.urdf')
    yaml  = os.path.join(share, 'config','bridge.yaml')
    rviz_cfg = os.path.join(share, 'config', 'rviz_config.rviz')
    img_yaml = os.path.join(share, 'config', 'bridge_image.yaml')



    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world}'}.items()
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(urdf).read()},
                    {'use_sim_time': True}],
        output='screen'
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'hmd_world',
            '-file', urdf,
            '-name', 'hmd_head',
            '-x', '0', '-y', '0', '-z', '0.12'   # << aquí subes 0.5 en Z
        ]
    )


    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        parameters=[{'config_file': yaml}],
        output='screen'
    )
    
    gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='ros_gz_image_bridge',
        arguments=['/hmd_cam/image'],   # topic de la cámara en GZ
        parameters=[img_yaml],
        output='screen'
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([gz, spawn, bridge, robot_state_pub, rviz, gz_image_bridge])
