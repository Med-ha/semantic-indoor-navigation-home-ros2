from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    planner_params = os.path.join(
        get_package_share_directory('semantic_nav_bringup'),
        'config',
        'planner_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='semantic_nav_perception',
            executable='go_to_named_place_node',
            name='go_to_named_place_node',
            output='screen'
        ),
        Node(
            package='semantic_nav_planning',
            executable='approach_pose_generator',
            name='approach_pose_generator',
            output='screen',
            parameters=[planner_params]
        ),
        Node(
            package='semantic_nav_execution',
            executable='nav_executor',
            name='nav_executor',
            output='screen'
        ),
        Node(
            package='semantic_nav_eval',
            executable='evaluation_logger',
            name='evaluation_logger',
            output='screen'
        ),
    ])
