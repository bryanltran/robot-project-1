from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('project1')
    default_world = os.path.join(pkg_share, 'worlds', 'world.sdf')

    world = LaunchConfiguration('world')
    slam = LaunchConfiguration('slam')
    headless = LaunchConfiguration('headless')

    nav2_share = get_package_share_directory('nav2_bringup')
    tb4_sim_launch = os.path.join(nav2_share, 'launch', 'tb4_simulation_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('slam', default_value='True'),
        DeclareLaunchArgument('headless', default_value='False'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb4_sim_launch),
            launch_arguments={
                'world': world,
                'slam': slam,
                'headless': headless,
                'x_pose': '2.0',
                'y_pose': '4.0',
                'z_pose': '0.0',
                'yaw': '1.6',
            }.items()
        ),
    ])
