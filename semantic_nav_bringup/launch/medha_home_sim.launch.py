from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('semantic_nav_bringup')
    nav2_dir = get_package_share_directory('nav2_bringup')

    world = os.path.join(bringup_dir, 'worlds', 'medha_home.world')
    map_file = os.path.join(bringup_dir, 'maps', 'medha_home_map.yaml')
    tb3_launch = os.path.join(nav2_dir, 'launch', 'tb3_simulation_launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_launch),
            launch_arguments={
                'slam': 'False',
                'headless': 'False',
                'world': world,
                'map': map_file,
                'robot_name': 'turtlebot3_waffle',
                'robot_sdf': '/opt/ros/humble/share/nav2_bringup/worlds/waffle.model',
                'use_composition': 'False',
            }.items()
        )
    ])
