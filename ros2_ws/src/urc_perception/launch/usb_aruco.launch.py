"""Namespaced camera and detector with one shared calibration file."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument('dictionary', description='Required explicit OpenCV DICT_* name'),
        DeclareLaunchArgument('namespace', default_value='camera'),
        DeclareLaunchArgument('device', default_value='/dev/video0'),
        DeclareLaunchArgument('calibration_file', default_value=''),
        DeclareLaunchArgument('params_file', default_value='',
                              description='Optional ROS YAML; use tag_sizes here'),
    ]
    # Empty params_file is intentionally omitted, avoiding invalid empty file paths.
    from launch.actions import OpaqueFunction

    def nodes(context):
        params_file = LaunchConfiguration('params_file').perform(context)
        common = [params_file] if params_file else []
        calibration = {'calibration_file': ParameterValue(
            LaunchConfiguration('calibration_file'), value_type=str)}
        namespace = LaunchConfiguration('namespace')
        return [
            Node(package='urc_perception', executable='usb_camera', namespace=namespace,
                 parameters=common + [calibration, {'device': ParameterValue(
                     LaunchConfiguration('device'), value_type=str)}], output='screen'),
            Node(package='urc_perception', executable='aruco_detector', namespace=namespace,
                 parameters=common + [calibration, {'dictionary': ParameterValue(
                     LaunchConfiguration('dictionary'), value_type=str)}], output='screen'),
        ]
    return LaunchDescription(declarations + [OpaqueFunction(function=nodes)])
