"""Isolated kinematic testbed. Intentionally launches NO physical CAN/camera."""
from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from urc_description.model import make_urdf


def generate_launch_description():
    share = Path(get_package_share_directory('urc_simulation'))
    default_model = str(Path(get_package_share_directory('urc_kinematics')) / 'config' / 'demo.json')

    def nodes(context):
        config = LaunchConfiguration('ik_config').perform(context)
        description = make_urdf(config, include_sim_camera=True)
        fixture = LaunchConfiguration('fixture_path').perform(context)
        processes = [
            Node(package='urc_simulation', executable='simulation_clock', output='screen'),
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'robot_description': description, 'publish_frequency': 50.0}], output='screen'),
            Node(package='urc_kinematics', executable='kinematics_node',
                 parameters=[{'config_path': config, 'publish_tcp_tf': False}], output='screen'),
            Node(package='urc_autonomy', executable='arm_controller',
                 parameters=[{'config_path': config}], output='screen'),
            Node(package='urc_simulation', executable='joint_executor',
                 parameters=[{'config_path': config}], output='screen'),
            Node(package='urc_autonomy', executable='tag_mapper',
                 parameters=[{'config_path': config, 'fixture_path': fixture}],
                 remappings=[('tag_detections', '/sim_camera/tag_detections'),
                             ('camera_info', '/sim_camera/camera_info')], output='screen'),
            Node(package='urc_autonomy', executable='hover_mission',
                 parameters=[{'config_path': config, 'fixture_path': fixture,
                              'sequence': LaunchConfiguration('sequence')}], output='screen'),
            Node(package='urc_simulation', executable='simulator', parameters=[{
                'config_path': config, 'auto_demo': ParameterValue(LaunchConfiguration('auto_demo'), value_type=bool)}], output='screen'),
            Node(package='urc_perception', executable='aruco_detector', namespace='sim_camera',
                 parameters=[{'dictionary': 'DICT_4X4_50', 'tag_sizes': ['0:0.02', '1:0.02', '2:0.02', '3:0.02'],
                              'calibration_file': str(share / 'config' / 'camera.yaml'),
                              'publish_annotated': True, 'annotated_fps': 5.0, 'annotated_scale': .5,
                              'max_image_age_s': .5, 'ambiguity_policy': 'reject'}], output='screen'),
            Node(package='rviz2', executable='rviz2', arguments=['-d', str(share / 'config' / 'arm.rviz')],
                 condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
        ]
        # Closing RViz stops this local testbed; a crashed critical node cannot
        # leave a frozen arm looking like a live simulation.
        handlers = [RegisterEventHandler(OnProcessExit(target_action=process,
                    on_exit=[EmitEvent(event=Shutdown(reason='Simulation component exited'))]))
                    for process in processes]
        return processes + handlers
    return LaunchDescription([
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',
            os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', str(share / 'config' / 'fastdds.xml'))),
        SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument('ik_config', default_value=default_model),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('auto_demo', default_value='false'),
        DeclareLaunchArgument('sequence', default_value='ABC'),
        DeclareLaunchArgument('fixture_path', default_value=str(
            Path(get_package_share_directory('urc_autonomy')) / 'config' / 'demo_panel.json')),
        OpaqueFunction(function=nodes),
    ])
