"""Synthetic environment and RViz controls; joint motion lives in another node."""
import time
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState, Image, CameraInfo
from std_msgs.msg import String, Int32MultiArray
from std_srvs.srv import Trigger, SetBool
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from urc_kinematics.core import Pose as CorePose
from urc_perception.ros_common import sensor_qos
from urc_autonomy.common import Feedback, get_model, pose_message as stamped_pose, stamp_seconds, run
from .scene import Scene, camera_pose, START, TAG_CENTERS, SIZE, K


def pose_message(position, rotation):
    return stamped_pose(CorePose(position, Rotation.from_matrix(rotation).as_quat()), '', None).pose


class Simulator(Node):
    def __init__(self):
        super().__init__('urc_simulator')
        cv2.setNumThreads(1)
        self.model, config = get_model(self)
        self.base = config['base_frame']
        self.feedback_state = Feedback(self.model)
        self.auto_demo = self.declare_parameter('auto_demo', False).value
        self.scene, self.bridge = Scene(), CvBridge()
        self.state, self.mission_state = 'WAITING for controller', 'IDLE'
        self.camera_enabled = True
        self.images = self.create_publisher(Image, '/sim_camera/image_raw', sensor_qos())
        self.camera_info = self.create_publisher(CameraInfo, '/sim_camera/camera_info', sensor_qos())
        self.visuals = self.create_publisher(MarkerArray, '/sim/scene', 10)
        self.status = self.create_publisher(String, '/sim/status', 1)
        self.target = self.create_publisher(PoseStamped, '/arm/target_pose', 1)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(String, '/arm/status', lambda m: setattr(self, 'state', m.data), 1)
        self.create_subscription(String, '/mission/status', lambda m: setattr(self, 'mission_state', m.data), 1)
        self.create_subscription(PoseStamped, '/arm/achieved_pose', self.show_target, 1)
        self.create_subscription(PoseStamped, '/mission/target_pose', self.show_target, 1)
        self.create_subscription(PoseStamped, '/sim/target_pose', self.target.publish, 1)
        self.create_service(SetBool, '/sim/camera_enabled', self.set_camera)
        self.create_subscription(Int32MultiArray, '/sim/visible_tag_ids',
                                 lambda m: setattr(self.scene, 'visible_ids', set(m.data) & set(TAG_CENTERS)), 1)
        self.service_clients = {name: self.create_client(Trigger, name) for name in
                        ['/arm/demo', '/arm/reset', '/mission/start', '/mission/abort']}
        self.pause_client = self.create_client(SetBool, '/arm/pause')
        self.server = InteractiveMarkerServer(self, '/sim/controls')
        self.menu = MenuHandler()
        self.menu.insert('Start hover mission (A B C)', callback=lambda _: self.call('/mission/start'))
        self.menu.insert('Abort mission', callback=lambda _: self.call('/mission/abort'))
        self.menu.insert('Run manual IK demo', callback=lambda _: self.call('/arm/demo'))
        self.menu.insert('Pause / cancel arm', callback=lambda _: self.pause(True))
        self.menu.insert('Resume arm', callback=lambda _: self.pause(False))
        self.menu.insert('Reset arm to observation pose', callback=lambda _: self.call('/arm/reset'))
        tcp = self.model.forward(START)
        self.add_control('ik_target', 'IK target / mission menu',
                         pose_message(tcp.position, Rotation.from_quat(tcp.quaternion_xyzw).as_matrix()), .16, [.95, .40, .12])
        self.add_control('tag_panel', 'Tag panel',
                         pose_message(self.scene.position, self.scene.rotation), .22, [.4, .65, .9])
        self.server.applyChanges()
        self.create_timer(.1, self.tick)
        self.get_logger().info('Synthetic environment only. Arm commands go through ROS motion actions.')

    def on_joints(self, msg):
        self.feedback_state.update(msg, self.get_clock().now().nanoseconds * 1e-9)
        if self.feedback_state.stamp == stamp_seconds(msg.header.stamp):
            self.joint_stamp = msg.header.stamp

    def call(self, name):
        client = self.service_clients[name]
        if client.service_is_ready():
            future = client.call_async(Trigger.Request())
            future.add_done_callback(lambda f: self.get_logger().info(f.result().message))

    def pause(self, value):
        if self.pause_client.service_is_ready():
            self.pause_client.call_async(SetBool.Request(data=value))

    def set_camera(self, request, response):
        self.camera_enabled = request.data
        response.success, response.message = True, 'Synthetic camera enabled' if request.data else 'Synthetic camera disabled'
        return response

    def add_control(self, name, description, pose, scale, color):
        marker = InteractiveMarker()
        marker.header.frame_id = self.base
        marker.name, marker.description, marker.pose, marker.scale = name, description, pose, scale
        visible = InteractiveMarkerControl()
        visible.always_visible = True
        visible.interaction_mode = InteractiveMarkerControl.MENU
        cube = Marker(type=Marker.CUBE)
        cube.pose.orientation.w = 1.
        cube.scale.x = cube.scale.y = cube.scale.z = .025
        cube.color.r, cube.color.g, cube.color.b = color
        cube.color.a = .9
        visible.markers.append(cube)
        marker.controls.append(visible)
        for axis, xyz in [('x', [1, 0, 0]), ('y', [0, 1, 0]), ('z', [0, 0, 1])]:
            for mode, prefix in [(InteractiveMarkerControl.MOVE_AXIS, 'move'),
                                 (InteractiveMarkerControl.ROTATE_AXIS, 'rotate')]:
                control = InteractiveMarkerControl()
                control.name = f'{prefix}_{axis}'
                control.orientation.w = 2**-.5
                control.orientation.x, control.orientation.y, control.orientation.z = [v * 2**-.5 for v in xyz]
                control.interaction_mode = mode
                marker.controls.append(control)
        self.server.insert(marker, feedback_callback=self.feedback)
        self.menu.apply(self.server, name)

    def feedback(self, feedback):
        if feedback.header.frame_id not in ('', self.base):
            return
        if feedback.marker_name == 'tag_panel' and feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            p, q = feedback.pose.position, feedback.pose.orientation
            try:
                pose = CorePose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
                self.scene.position = pose.position
                self.scene.rotation = Rotation.from_quat(pose.quaternion_xyzw).as_matrix()
            except ValueError:
                pass
        if feedback.marker_name == 'ik_target' and feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            message = PoseStamped()
            message.header.frame_id, message.header.stamp = self.base, self.get_clock().now().to_msg()
            message.pose = feedback.pose
            self.target.publish(message)
        self.server.applyChanges()

    def show_target(self, message):
        if message.header.frame_id == self.base:
            self.server.setPose('ik_target', message.pose)
            self.server.applyChanges()

    def tick(self):
        now = self.get_clock().now()
        if self.auto_demo and self.feedback_state.fresh(now.nanoseconds * 1e-9) and self.service_clients['/arm/demo'].service_is_ready():
            self.auto_demo = False
            self.call('/arm/demo')
        if self.camera_enabled and self.feedback_state.fresh(now.nanoseconds * 1e-9):
            p, r = camera_pose(self.model, self.feedback_state.q)
            image = self.bridge.cv2_to_imgmsg(self.scene.render(p, r, grayscale=True), encoding='mono8')
            image.header.stamp, image.header.frame_id = self.joint_stamp, 'sim_camera_optical_frame'
            self.images.publish(image)
            info = CameraInfo()
            info.header = image.header
            info.width, info.height, info.distortion_model = 640, 480, 'plumb_bob'
            info.d, info.k = [0.] * 5, K.flatten().tolist()
            info.r = np.eye(3).flatten().tolist()
            info.p = np.column_stack([K, np.zeros(3)]).flatten().tolist()
            self.camera_info.publish(info)
        self.publish_scene(now.to_msg())
        self.status.publish(String(data=self.state))

    def marker(self, name, marker_id, kind, stamp, color):
        marker = Marker()
        marker.header.frame_id, marker.header.stamp = self.base, stamp
        marker.ns, marker.id, marker.type = name, marker_id, kind
        marker.pose.orientation.w = 1.
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def publish_scene(self, stamp):
        markers = []
        board = self.marker('ground_truth_panel', 0, Marker.CUBE, stamp, [.2, .35, .5, .65])
        board.pose = pose_message(self.scene.position, self.scene.rotation)
        board.scale.x, board.scale.y, board.scale.z = .15, .10, .003
        markers.append(board)
        for i, center in TAG_CENTERS.items():
            tag = self.marker('ground_truth_tags', i, Marker.CUBE, stamp, [.95, .6, .15, 1.])
            tag.pose = pose_message(self.scene.position + self.scene.rotation @ center, self.scene.rotation)
            tag.scale.x = tag.scale.y = SIZE
            tag.scale.z = .004
            markers.append(tag)
        for i, (label, center) in enumerate([('A', [-.02, 0., 0.]), ('B', [0., 0., 0.]), ('C', [.02, 0., 0.])]):
            target = self.marker('fixture_targets', i, Marker.SPHERE, stamp, [.8, .3, 1., .9])
            target.pose = pose_message(self.scene.position + self.scene.rotation @ center, self.scene.rotation)
            target.scale.x = target.scale.y = target.scale.z = .009
            markers.append(target)
            label_marker = self.marker('fixture_labels', i, Marker.TEXT_VIEW_FACING, stamp, [1., 1., 1., 1.])
            label_marker.pose = pose_message(self.scene.position + self.scene.rotation @ (np.array(center) + [0, .01, .002]), np.eye(3))
            label_marker.scale.z, label_marker.text = .012, label
            markers.append(label_marker)
        text = self.marker('status', 0, Marker.TEXT_VIEW_FACING, stamp, [.9, .95, 1., 1.])
        text.pose.position.x, text.pose.position.y, text.pose.position.z = .35, .65, 0.
        text.scale.z = .018
        text.text = 'SIM ONLY | ' + self.state
        markers.append(text)
        self.visuals.publish(MarkerArray(markers=markers))

    def destroy_node(self):
        self.server.shutdown()
        return super().destroy_node()


def main(args=None):
    run(Simulator, args)
