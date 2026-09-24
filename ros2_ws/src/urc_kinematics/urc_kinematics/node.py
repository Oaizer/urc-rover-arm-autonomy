"""IK/FK services and opt-in measured-state TF; never commands motion."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from urc_interfaces.srv import ComputeFK, SolveIK

from .core import load_config
from .service_adapter import ServiceAdapter, ordered_joint_positions


class KinematicsNode(Node):
    def __init__(self):
        super().__init__('urc_kinematics')
        default = str(Path(get_package_share_directory('urc_kinematics')) / 'config' / 'demo.json')
        self.declare_parameter('config_path', default, ParameterDescriptor(read_only=True))
        model, config = load_config(self.get_parameter('config_path').value)
        descriptor = ParameterDescriptor(read_only=True)
        self.declare_parameter('publish_tcp_tf', False, descriptor)
        self.declare_parameter('joint_states_topic', '/joint_states', descriptor)
        self.declare_parameter('joint_names', [f'joint_{i}' for i in range(1, 7)], descriptor)
        self.declare_parameter('joint5_frame', 'arm_joint5_output', descriptor)
        self.joint_names = list(self.get_parameter('joint_names').value)
        if (len(self.joint_names) != 6 or len(set(self.joint_names)) != 6
                or any(not name.strip() for name in self.joint_names)):
            raise ValueError('joint_names must be six distinct nonempty names in J1..J6 order')
        self.joint5_frame = self.get_parameter('joint5_frame').value
        self.base_frame, self.tcp_frame = config['base_frame'], config['tcp_frame']
        if (not self.joint5_frame.strip()
                or self.joint5_frame in [self.base_frame, self.tcp_frame]):
            raise ValueError('joint5_frame must be nonempty and distinct from base/TCP')
        self.model = model
        if not config['commissioned']:
            self.get_logger().warning('UNCOMMISSIONED demo geometry/limits/tool; kinematics use only.')
        self.get_logger().info(
            f"Pose convention: {config['tcp_frame']} in {config['base_frame']}; meters/radians. "
            'Collision checking is not provided.')
        self.adapter = ServiceAdapter(model, self.get_logger().error)
        self.ik_service = self.create_service(SolveIK, 'solve_ik', self.adapter.solve_ik)
        self.fk_service = self.create_service(ComputeFK, 'compute_fk', self.adapter.compute_fk)
        if self.get_parameter('publish_tcp_tf').value:
            self.broadcaster = TransformBroadcaster(self)
            self.subscription = self.create_subscription(
                JointState, self.get_parameter('joint_states_topic').value,
                self.on_joint_states, qos_profile_sensor_data)

    def on_joint_states(self, message):
        try:
            q = ordered_joint_positions(message.name, message.position, self.joint_names)
            poses = [(self.tcp_frame, self.model.forward(q)),
                     (self.joint5_frame, self.model.joint5_output(q))]
        except (ValueError, TypeError, OverflowError) as exc:
            self.get_logger().warning(f'JointState rejected; no TF published: {exc}', throttle_duration_sec=5.0)
            return
        transforms = []
        for child, pose in poses:
            transform = TransformStamped()
            transform.header.stamp = message.header.stamp
            transform.header.frame_id = self.base_frame
            transform.child_frame_id = child
            t, r = transform.transform.translation, transform.transform.rotation
            t.x, t.y, t.z = map(float, pose.position)
            r.x, r.y, r.z, r.w = map(float, pose.quaternion_xyzw)
            transforms.append(transform)
        self.broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KinematicsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
