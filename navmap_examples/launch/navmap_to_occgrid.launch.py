from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='navmap_examples', executable='02_to_occgrid', output='screen')
    ])
