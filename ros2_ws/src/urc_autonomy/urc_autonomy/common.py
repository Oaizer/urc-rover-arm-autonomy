from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.task import Future
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from urc_kinematics.core import load_config
from urc_kinematics.service_adapter import ordered_joint_positions

NAMES = [f'joint_{i}' for i in range(1, 7)]


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def get_model(node):
    path = str(Path(get_package_share_directory('urc_kinematics')) / 'config' / 'demo.json')
    return load_config(node.declare_parameter('config_path', path).value)


def pose_message(core_pose, frame, stamp):
    msg = PoseStamped()
    msg.header.frame_id = frame
    if stamp is not None:
        msg.header.stamp = stamp
    p, q = msg.pose.position, msg.pose.orientation
    p.x, p.y, p.z = map(float, core_pose.position)
    q.x, q.y, q.z, q.w = map(float, core_pose.quaternion_xyzw)
    return msg


class Feedback:
    def __init__(self, model):
        self.model, self.q, self.v, self.stamp, self.received = model, None, None, 0., 0.

    def update(self, msg, now_ros):
        stamp = stamp_seconds(msg.header.stamp)
        if stamp <= self.stamp or not 0 <= now_ros - stamp < .5:
            return
        try:
            q = self.model.validate_positions(ordered_joint_positions(msg.name, msg.position, NAMES))
            v = np.asarray(ordered_joint_positions(msg.name, msg.velocity, NAMES))
            if not np.isfinite(v).all():
                return
        except (ValueError, TypeError):
            return
        self.q, self.v, self.stamp, self.received = q, v, stamp, time.monotonic()

    def fresh(self, now_ros, limit=.5):
        return self.q is not None and 0 <= now_ros - self.stamp < limit and time.monotonic() - self.received < limit


async def tick(node, seconds=.02):
    future = Future()
    timer = node.create_timer(seconds, lambda: future.set_result(True) if not future.done() else None)
    try:
        await future
    finally:
        node.destroy_timer(timer)


def run(factory, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
