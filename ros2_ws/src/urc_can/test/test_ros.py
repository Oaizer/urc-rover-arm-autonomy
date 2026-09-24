"""Optional ROS integration checks; pure suite runs without ROS installed."""
import time
from unittest.mock import patch

import pytest

rclpy = pytest.importorskip('rclpy')
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from sensor_msgs.msg import JointState
from rclpy.executors import SingleThreadedExecutor

from urc_can.core import Status
from urc_can.node import FeedbackNode


def test_simulated_ros_feedback_and_stale_suppression():
    rclpy.init()
    node = FeedbackNode()
    observer = rclpy.create_node('urc_can_test_observer')
    samples, diagnostics = [], []
    observer.create_subscription(JointState, '/joint_states', samples.append, 10)
    observer.create_subscription(DiagnosticArray, '/diagnostics', diagnostics.append, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(observer)
    try:
        deadline = time.monotonic() + 5
        while (not samples or not diagnostics or
               diagnostics[-1].status[0].level != DiagnosticStatus.OK) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert samples and diagnostics
        assert samples[-1].name == [f'joint_{i}' for i in range(1, 7)]
        assert list(samples[-1].position) == [0.0] * 6
        assert list(samples[-1].velocity) == [0.0] * 6
        assert not samples[-1].effort
        assert diagnostics[-1].status[0].hardware_id == 'simulated/stationary'
        assert diagnostics[-1].status[0].level == DiagnosticStatus.OK
        assert node.count_subscribers('/joint_targets') == 0
        node.worker.close()
        stale = Status(sequence=100, last_good_at=time.monotonic() - 10,
                       error='missing replies', failures=1)
        # Direct publisher spies verify no stale publication, without DDS queue ambiguity.
        with patch.object(node.worker, 'snapshot', return_value=stale), \
                patch.object(node.publisher, 'publish') as publish, \
                patch.object(node.diagnostics, 'publish') as diagnostic:
            node.publish_feedback()
            publish.assert_not_called()
            assert diagnostic.call_args.args[0].status[0].level == DiagnosticStatus.STALE
    finally:
        executor.shutdown()
        node.destroy_node()
        observer.destroy_node()
        rclpy.try_shutdown()
