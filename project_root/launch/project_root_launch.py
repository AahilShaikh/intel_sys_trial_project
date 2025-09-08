import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='project_root',
            executable='map_publisher',
            name='map_publisher'
        ),
        Node(
            package='project_root',
            executable='goal_publisher',
            name='goal_publisher',
            remappings=[
                ('goal', 'generated_goal')
            ]
        ),
        Node(
            package='project_root',
            executable='start_publisher',
            name='start_publisher',
            remappings=[
                ('start', 'generated_start')
            ]
        ),
        Node(
            package='project_root',
            executable='path_publisher',
            name='path_publisher',
        )
    ])
