"""Read-only viewer for an existing joint-state source. No simulated feedback."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from urc_description.model import make_urdf


def generate_launch_description():
    default = str(Path(get_package_share_directory('urc_kinematics')) / 'config' / 'demo.json')
    share = Path(get_package_share_directory('urc_simulation'))

    def nodes(context):
        path = LaunchConfiguration('ik_config').perform(context)
        return [
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'robot_description': make_urdf(path), 'publish_frequency': 50.0}],
                 remappings=[('joint_states', LaunchConfiguration('joint_states_topic'))]),
            Node(package='rviz2', executable='rviz2', arguments=['-d', str(share / 'config' / 'live_arm.rviz')]),
        ]
    return LaunchDescription([
        DeclareLaunchArgument('ik_config', default_value=default),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        OpaqueFunction(function=nodes),
    ])
