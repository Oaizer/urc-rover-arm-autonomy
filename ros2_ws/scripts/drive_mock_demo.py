#!/usr/bin/env python3
"""Send one visible pose goal only to the isolated ros2_control mock demo."""

import argparse
import os
from pathlib import Path
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListHardwareComponents
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

from urc_autonomy.common import pose_message
from urc_interfaces.action import MoveArm
from urc_kinematics.core import load_config


TARGETS = {
    'home': [0., .5, -1., 0., .5, 0.],
    'far': [.55, .25, -.65, .35, .25, -.1],
}


def wait(node, future, seconds=15.):
    rclpy.spin_until_future_complete(node, future, timeout_sec=seconds)
    if not future.done() or future.exception():
        raise RuntimeError('ROS request timed out or failed')
    return future.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('target', choices=TARGETS)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '71':
        raise SystemExit('Refusing motion outside the isolated demo domain 71')

    rclpy.init()
    node = rclpy.create_node('mock_pose_demo',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
    try:
        hardware = node.create_client(ListHardwareComponents,
                                      '/controller_manager/list_hardware_components')
        if not hardware.wait_for_service(timeout_sec=15.):
            raise RuntimeError('No controller manager')
        components = wait(node, hardware.call_async(ListHardwareComponents.Request())).component
        if not (len(components) == 1 and components[0].name == 'URCMockArm'
                and components[0].plugin_name == 'mock_components/GenericSystem'
                and components[0].state.id == 3):
            raise RuntimeError('Refusing motion: active mock hardware was not verified')

        action = ActionClient(node, MoveArm, '/arm/move_to_pose')
        if not action.wait_for_server(timeout_sec=15.):
            raise RuntimeError('Pose controller unavailable')
        path = Path(get_package_share_directory('urc_kinematics')) / 'config' / 'demo.json'
        model, config = load_config(path)
        clock_deadline = time.monotonic() + 10.
        while node.get_clock().now().nanoseconds == 0 and time.monotonic() < clock_deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if node.get_clock().now().nanoseconds == 0:
            raise RuntimeError('Simulation clock unavailable')
        target = pose_message(model.forward(TARGETS[args.target]), config['base_frame'],
                              node.get_clock().now().to_msg())
        goal = MoveArm.Goal(target=target, source='manual', timeout_s=30.,
                            max_joint_delta_rad=1.5)
        handle = wait(node, action.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('Pose goal rejected')
        result = wait(node, handle.get_result_async(), seconds=35.)
        print(result.result.message, flush=True)
        if not result.result.success:
            raise RuntimeError('Pose motion failed')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
