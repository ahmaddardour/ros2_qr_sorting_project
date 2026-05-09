from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='qr_sorting_project', executable='scene_publisher',
             name='scene_publisher', output='screen'),
        Node(package='qr_sorting_project', executable='main_node',
             name='main_node', output='screen'),
    ])
