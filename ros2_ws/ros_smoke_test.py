"""Hardware-free ROS graph smoke test. Run after sourcing install/setup.bash.

Starts only simulation/service/perception nodes, never a physical camera or CAN.
Uses its own ROS domain and cleans up every subprocess it starts.
"""
import os
import signal
import subprocess
import sys
import time

# Isolate the test from a running robot. Override only deliberately for diagnostics.
os.environ['ROS_DOMAIN_ID'] = os.environ.get('URC_TEST_DOMAIN_ID', '177')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Image
from urc_interfaces.msg import TagDetections
from urc_interfaces.srv import ComputeFK, SolveIK


def spin_until(node, predicate, timeout=15):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not predicate():
        raise AssertionError('Timed out waiting for ROS data/service')


def call(node, client, request):
    assert client.wait_for_service(timeout_sec=15), client.srv_name
    future = client.call_async(request)
    spin_until(node, future.done)
    return future.result()


def main():
    children = []
    rclpy.init()
    node = rclpy.create_node('urc_core_smoke_test')
    try:
        for module in ('urc_kinematics.node', 'urc_can.node'):
            children.append(subprocess.Popen(
                [sys.executable, '-c', f'from {module} import main; main()'],
                start_new_session=True))
        samples = []
        sub = node.create_subscription(JointState, '/joint_states', samples.append, 10)
        spin_until(node, lambda: len(samples) >= 2)
        assert samples[-1].name == [f'joint_{i}' for i in range(1, 7)]
        assert list(samples[-1].position) == [0.0] * 6
        assert len(samples[-1].velocity) == 6
        print('PASS simulated CAN publishes six-joint feedback', flush=True)

        fk = node.create_client(ComputeFK, '/compute_fk')
        ik = node.create_client(SolveIK, '/solve_ik')
        req = ComputeFK.Request()
        req.positions = [0.15, -0.35, 0.6, 0.2, -0.4, 0.1]
        pose = call(node, fk, req).pose
        request = SolveIK.Request()
        request.target = pose
        request.seed = [0.1, -0.3, 0.55, 0.15, -0.35, 0.05]
        result = call(node, ik, request)
        assert result.success, result.message
        assert result.position_error_m <= 1e-4
        assert result.orientation_error_rad <= 1e-3
        print('PASS full-pose IK/FK through generated ROS services', flush=True)
        request.target.orientation.w = 0.0
        request.target.orientation.x = 0.0
        request.target.orientation.y = 0.0
        request.target.orientation.z = 0.0
        assert not call(node, ik, request).success
        print('PASS invalid quaternion rejected by ROS service', flush=True)

        # Near full extension: TCP position is reachable, but not with flat wrist.
        bend = float(np.arctan2(.1, .35))
        req.positions = [0., .4, -bend, 0., bend, 0.]
        actual = call(node, fk, req).pose
        request.target = actual
        request.target.orientation.x = request.target.orientation.y = request.target.orientation.z = 0.
        request.target.orientation.w = 1.
        request.seed = req.positions
        request.require_orientation = False
        relaxed = call(node, ik, request)
        assert relaxed.success and relaxed.orientation_relaxed
        assert relaxed.position_error_m <= 1e-4
        assert relaxed.orientation_error_rad > .1
        assert abs(relaxed.achieved_pose.position.x - actual.position.x) <= 1e-4
        request.require_orientation = True
        assert not call(node, ik, request).success
        print('PASS flat-wrist fallback, achieved pose and strict-orientation opt-out', flush=True)

        # A reachable requested pose can still require a distant wrist branch.
        # The bounded solver must choose a nearby position-only solution instead.
        req.positions = [0., .5, -1., 0., .5, np.pi]
        request.target = call(node, fk, req).pose
        request.seed = [0., .5, -1., 0., .5, 0.]
        request.max_joint_delta_rad = .8
        request.require_orientation = False
        nearby = call(node, ik, request)
        assert nearby.success and nearby.orientation_relaxed, nearby.message
        assert nearby.position_error_m <= 1e-4
        assert np.max(np.abs(np.asarray(nearby.positions) - request.seed)) <= .8 + 1e-9
        request.require_orientation = True
        assert not call(node, ik, request).success
        print('PASS reachable distant-wrist pose uses nearby free-wrist branch', flush=True)

        children.append(subprocess.Popen([
            sys.executable, '-c', 'from urc_perception.detector_node import main; main()',
            '--ros-args', '-p', 'dictionary:=DICT_4X4_50'], start_new_session=True))
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        detections = []
        tag_sub = node.create_subscription(TagDetections, '/tag_detections', detections.append, qos)
        publisher = node.create_publisher(Image, '/image_raw', qos)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco, 'generateImageMarker'):
            marker = cv2.aruco.generateImageMarker(dictionary, 7, 160)
        else:
            marker = cv2.aruco.drawMarker(dictionary, 7, 160)
        frame = np.full((480, 640, 3), 255, dtype=np.uint8)
        frame[160:320, 240:400] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        bridge = CvBridge()
        sent_stamps = set()

        def publish_image():
            msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'test_camera_optical_frame'
            sent_stamps.add((msg.header.stamp.sec, msg.header.stamp.nanosec))
            publisher.publish(msg)

        timer = node.create_timer(0.1, publish_image)
        spin_until(node, lambda: any(m.detections for m in detections))
        detected = next(m for m in detections if m.detections)
        assert detected.dictionary == 'DICT_4X4_50'
        assert detected.header.frame_id == 'test_camera_optical_frame'
        assert (detected.header.stamp.sec, detected.header.stamp.nanosec) in sent_stamps
        assert detected.detections[0].id == 7
        assert len(detected.detections[0].corners) == 4
        assert not detected.detections[0].pose_valid
        print('PASS synthetic image -> ROS ArUco ID/corners, no fabricated metric pose', flush=True)
        node.destroy_timer(timer)
        node.destroy_subscription(tag_sub)
        node.destroy_subscription(sub)
        print('ROS core smoke test passed', flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        for process in children:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in children:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)


if __name__ == '__main__':
    main()
