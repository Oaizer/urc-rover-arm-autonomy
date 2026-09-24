"""No hardware is opened by default; camera is a separate explicit opt-in."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_model = str(Path(get_package_share_directory('urc_kinematics')) / 'config' / 'demo.json')
    return LaunchDescription([
        DeclareLaunchArgument('ik_config', default_value=default_model),
        Node(package='urc_kinematics', executable='kinematics_node',
             parameters=[{'config_path': LaunchConfiguration('ik_config')}], output='screen'),
        Node(package='urc_can', executable='feedback',
             parameters=[{'mode': 'simulated'}], output='screen'),
    ])
