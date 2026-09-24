"""Exercise the installed simulation graph without opening RViz/hardware."""
import os
from pathlib import Path
import signal
import subprocess
import time
os.environ['ROS_DOMAIN_ID'] = '179'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(Path(__file__).resolve().parent / 'src/urc_simulation/config/fastdds.xml')

import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from visualization_msgs.msg import InteractiveMarkerFeedback
from tf2_ros import Buffer, TransformListener, TransformException
from urc_interfaces.msg import TagDetections


def spin(node, condition, seconds=20):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)
        if condition():
            return
    raise AssertionError('ROS simulation test timed out')


def main():
    rclpy.init()
    node = rclpy.create_node('simulation_test', parameter_overrides=[rclpy.Parameter('use_sim_time', value=True)])
    process = subprocess.Popen(['ros2', 'launch', 'urc_simulation', 'demo.launch.py',
                                'rviz:=false', 'auto_demo:=false'], start_new_session=True)
    try:
        states, tags, images, status = [], [], [], []
        node.create_subscription(JointState, '/joint_states', states.append, 10)
        node.create_subscription(TagDetections, '/sim_camera/tag_detections', tags.append, qos_profile_sensor_data)
        node.create_subscription(Image, '/sim_camera/image_raw', images.append, qos_profile_sensor_data)
        node.create_subscription(String, '/sim/status', status.append, 10)
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        # Cold WSL startup from the Windows-mounted workspace can take longer
        # than the steady-state timing checks below.
        spin(node, lambda: states and images and any(len(t.detections) == 4 for t in tags), seconds=60)
        assert node.count_publishers('/joint_states') == 1
        assert 'urc_can' not in node.get_node_names()
        assert {d.id for d in tags[-1].detections} == {0, 1, 2, 3}
        print('PASS one simulated joint source, no CAN; rendered camera -> real ArUco IDs', flush=True)
        available = lambda: buffer.can_transform('world', 'arm_tcp', Time())
        spin(node, available)
        assert buffer.can_transform('world', 'sim_camera_optical_frame', Time())
        print('PASS robot_state_publisher supplies full arm and camera TF', flush=True)

        initial = np.array(states[-1].position)
        demo = node.create_client(Trigger, '/sim/demo')
        assert demo.wait_for_service(timeout_sec=5)
        future = demo.call_async(Trigger.Request())
        spin(node, future.done)
        assert future.result().success
        spin(node, lambda: np.linalg.norm(np.array(states[-1].position) - initial) > .05)
        spin(node, lambda: status and 'target reached' in status[-1].data)
        np.testing.assert_allclose(states[-1].position, [.12, .42, -.88, .12, .46, .08], atol=.01)
        assert max(abs(v) for m in states for v in m.velocity) <= .351
        print('PASS IK service -> bounded smooth joint movement -> live TF', flush=True)

        pause = node.create_client(SetBool, '/sim/pause')
        assert pause.wait_for_service(timeout_sec=5)
        future = pause.call_async(SetBool.Request(data=True))
        spin(node, future.done)
        frozen = np.array(states[-1].position)
        n = len(states)
        spin(node, lambda: len(states) >= n + 10)
        np.testing.assert_allclose(states[-1].position, frozen)
        print('PASS simulation pause holds state', flush=True)

        feedback = node.create_publisher(InteractiveMarkerFeedback, '/sim/controls/feedback', 10)
        spin(node, lambda: feedback.get_subscription_count() > 0)
        message = InteractiveMarkerFeedback()
        message.header.frame_id = 'arm_base_legacy'
        message.client_id = 'smoke_test'
        message.marker_name = 'tag_panel'
        message.event_type = InteractiveMarkerFeedback.POSE_UPDATE
        message.pose.position.x, message.pose.position.y, message.pose.position.z = 1., .22, 2.
        message.pose.orientation.y, message.pose.orientation.w = -2**-.5, 2**-.5
        old = len(tags)
        feedback.publish(message)
        spin(node, lambda: len(tags) > old + 3 and not tags[-1].detections)
        print('PASS moving RViz panel changes image and removes out-of-view detections', flush=True)
        assert process.poll() is None
        print('Simulation graph smoke test passed', flush=True)
    finally:
        if process.poll() is None:
            # Signal the launch parent only; it owns and shuts down its children.
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
