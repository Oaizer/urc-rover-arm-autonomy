"""Simulation CAN + IK; opt-in USB camera and detector in a camera namespace."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    core = str(Path(get_package_share_directory('urc_bringup')) / 'launch' / 'core.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('camera_enabled', default_value='false'),
        DeclareLaunchArgument('camera_namespace', default_value='wrist_camera'),
        DeclareLaunchArgument('device', default_value='/dev/video0'),
        DeclareLaunchArgument('dictionary', default_value='',
                              description='Required when camera enabled; select actual tag dictionary'),
        DeclareLaunchArgument('calibration_file', default_value=''),
        DeclareLaunchArgument('detector_config', default_value=str(
            Path(get_package_share_directory('urc_perception')) / 'config' / 'detection_only.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(core)),
        Node(package='urc_perception', executable='usb_camera',
             namespace=LaunchConfiguration('camera_namespace'),
             condition=IfCondition(LaunchConfiguration('camera_enabled')),
             parameters=[{'device': ParameterValue(LaunchConfiguration('device'), value_type=str),
                          'calibration_file': ParameterValue(LaunchConfiguration('calibration_file'), value_type=str)}], output='screen'),
        Node(package='urc_perception', executable='aruco_detector',
             namespace=LaunchConfiguration('camera_namespace'),
             condition=IfCondition(LaunchConfiguration('camera_enabled')),
             parameters=[LaunchConfiguration('detector_config'),
                         {'dictionary': ParameterValue(LaunchConfiguration('dictionary'), value_type=str),
                          'calibration_file': ParameterValue(LaunchConfiguration('calibration_file'), value_type=str)}], output='screen'),
    ])
