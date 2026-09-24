from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='urc_can', executable='feedback', name='urc_can',
             parameters=[{'mode': 'simulated'}], output='screen'),
    ])
