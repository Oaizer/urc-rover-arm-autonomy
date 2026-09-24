"""ROS adapter. All configured parameters are immutable after startup."""

import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState

from .core import Joint, finite, validate_joints, validate_mode
from .transport import MoteusQueryTransport, StationaryTransport
from .worker import FeedbackWorker


class FeedbackNode(Node):
    def __init__(self):
        super().__init__('urc_can')

        def param(name, default):
            return self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True)).value

        self.mode = param('mode', 'simulated')
        backend = param('backend', '')
        channel = param('channel', '')
        validate_mode(self.mode, backend, channel)
        names = param('joint_names', [f'joint_{i}' for i in range(1, 7)])
        ids = param('can_ids', list(range(1, 7)))
        ratios = param('ratios', [1.0] * 6)
        signs = param('signs', [1] * 6)
        offsets = param('offsets_rad', [0.0] * 6)
        if any(len(values) != 6 for values in (names, ids, ratios, signs, offsets)):
            raise ValueError('all joint parameter arrays must have exactly six entries')
        joints = validate_joints(Joint(*values) for values in zip(names, ids, ratios, signs, offsets))
        rate = finite(param('rate_hz', 20.0))
        timeout = finite(param('cycle_timeout_s', 0.1))
        self.stale = finite(param('stale_after_s', 0.5))
        if not 0 < rate <= 200 or not 0 < timeout <= 5 or self.stale <= max(timeout, 1 / rate):
            raise ValueError('rate must be (0,200], timeout (0,5], stale_after > timeout and period')
        self.identity = 'simulated/stationary' if self.mode == 'simulated' else f'{backend}:{channel}'
        factory = (lambda: StationaryTransport(joints)) if self.mode == 'simulated' else (
            lambda: MoteusQueryTransport(joints, backend, channel))
        self.worker = FeedbackWorker(joints, factory, period=1 / rate, timeout=timeout,
                                     latch_failures=self.mode == 'read_only')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.last_sequence = -1
        self.timer = self.create_timer(min(1 / rate, 0.1), self.publish_feedback)
        self.get_logger().info(f'mode={self.mode}, {self.identity}; query/feedback only, no targets')
        self.worker.start()

    def publish_feedback(self):
        state = self.worker.snapshot()
        now = time.monotonic()
        stamp = self.get_clock().now()
        if state.publishable(now, self.stale, self.last_sequence):
            sample = state.sample
            msg = JointState()
            # Approximate receipt time in ROS clock domain, not timer publication time.
            from rclpy.duration import Duration
            msg.header.stamp = (stamp - Duration(seconds=now - sample.observed_at)).to_msg()
            msg.name = list(sample.names)
            msg.position = list(sample.positions)
            msg.velocity = list(sample.velocities)
            # Effort intentionally absent: no commissioned torque conversion.
            self.publisher.publish(msg)
            self.last_sequence = state.sequence
        level, message = state.diagnostic(now, self.stale)
        status = DiagnosticStatus()
        status.name = 'urc_can/feedback'
        status.hardware_id = self.identity
        status.level = (DiagnosticStatus.OK, DiagnosticStatus.WARN,
                        DiagnosticStatus.ERROR, DiagnosticStatus.STALE)[level]
        status.message = f'{self.mode}: {message}'
        age = 'never' if state.last_good_at is None else str(now - state.last_good_at)
        status.values = [KeyValue(key=k, value=v) for k, v in (
            ('mode', self.mode), ('sample_age_s', age), ('failures', str(state.failures)),
            ('last_error', state.error), ('recovery', 'restart required after any real cycle failure'),
            ('actuation', 'disabled; no target interface'))]
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = stamp.to_msg()
        diagnostic.status = [status]
        self.diagnostics.publish(diagnostic)

    def destroy_node(self):
        try:
            self.worker.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FeedbackNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            rclpy.try_shutdown()
