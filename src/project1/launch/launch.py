import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_file = os.path.join(
        get_package_share_directory('project1'),
        'worlds',
        'world.sdf'
    )

    tb4_launch = os.path.join(
        get_package_share_directory('turtlebot4_gz_bringup'),
        'launch',
        'turtlebot4_gz.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb4_launch),
            launch_arguments={
                'gazebo': 'ignition',
                'gz_args': world_file,
                'x': '1.0',
                'y': '2.0',
                'z': '0.0',
                'yaw': '0.0',
                'model': 'standard',
            }.items()
        ),
    ])
