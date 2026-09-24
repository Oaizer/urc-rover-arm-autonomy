"""Timestamped camera observations -> base-frame tag map and registered panel."""
from collections import deque
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import JointState, CameraInfo
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
from urc_interfaces.msg import TagDetections, TagMap as MapMessage, MappedTag, PanelObservation
from urc_perception.core import Calibration
from urc_perception.ros_common import sensor_qos
from urc_kinematics.core import Pose
from urc_kinematics.service_adapter import ordered_joint_positions
from .common import NAMES, get_model, pose_message, stamp_seconds, run
from .mapping import load_fixture, board_pose, TagMap


class Mapper(Node):
    def __init__(self):
        super().__init__('tag_mapper')
        self.model, config = get_model(self)
        self.base = config['base_frame']
        path = str(Path(get_package_share_directory('urc_autonomy')) / 'config' / 'demo_panel.json')
        self.fixture = load_fixture(self.declare_parameter('fixture_path', path).value)
        self.map = TagMap(self.fixture)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pending, self.calibration, self.calibration_frame = None, None, ''
        self.calibration_stamp, self.last_camera_stamp, self.last_accepted_stamp = 0., 0., 0.
        self.last_callback, self.last_input_stamp, self.input_count = 0., 0., 0
        self.history = deque(maxlen=150)
        self.reason = 'Waiting for camera and joint feedback'
        self.create_subscription(TagDetections, 'tag_detections', self.detected, sensor_qos())
        self.create_subscription(CameraInfo, 'camera_info', self.camera_info, sensor_qos())
        self.create_subscription(JointState, '/joint_states', self.joints, sensor_qos())
        self.tags = self.create_publisher(MapMessage, '/perception/tag_map', 1)
        self.panel = self.create_publisher(PanelObservation, '/perception/panel', 1)
        self.markers = self.create_publisher(MarkerArray, '/perception/markers', 1)
        self.create_service(Trigger, '/perception/reset_map', self.reset)
        self.create_timer(.02, self.process)
        self.create_timer(.1, self.publish)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def reset(self, request, response):
        self.map.reset()
        self.pending = None
        self.last_accepted_stamp = self.now()
        response.success, response.message = True, 'Tag history cleared'
        return response

    def joints(self, msg):
        try:
            v = np.asarray(ordered_joint_positions(msg.name, msg.velocity, NAMES))
            q = self.model.validate_positions(ordered_joint_positions(msg.name, msg.position, NAMES))
            stamp = stamp_seconds(msg.header.stamp)
            if np.isfinite(v).all() and 0 <= self.now() - stamp < .5:
                self.history.append((stamp, v, q))
        except ValueError:
            pass

    def camera_info(self, msg):
        if msg.distortion_model not in ('plumb_bob', 'rational_polynomial'):
            return
        try:
            self.calibration = Calibration(msg.width, msg.height, msg.k, msg.d)
            self.calibration_frame, self.calibration_stamp = msg.header.frame_id, stamp_seconds(msg.header.stamp)
        except ValueError:
            self.calibration = None

    def detected(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        self.last_callback, self.last_input_stamp = time.monotonic(), stamp
        self.input_count += 1
        if (msg.dictionary == self.fixture['dictionary'] and msg.header.frame_id
                and 0 <= self.now() - stamp <= .5 and stamp > self.last_camera_stamp):
            self.last_camera_stamp = stamp
            self.pending = msg

    def process(self):
        msg = self.pending
        if msg is None:
            return
        stamp = stamp_seconds(msg.header.stamp)
        if stamp <= self.last_accepted_stamp or self.now() - stamp > .5:
            self.pending = None
            return
        nearby = [entry for entry in self.history if abs(entry[0] - stamp) <= .06]
        if not nearby:
            self.reason = 'Waiting for time-matched joint feedback'
            return
        # Only accumulate settled observations, as required by the eye-in-hand model.
        if any(np.max(np.abs(entry[1])) > .02 for entry in nearby):
            self.pending = None
            self.reason = 'Arm moving; retaining settled observations'
            return
        try:
            tf = self.buffer.lookup_transform(self.base, msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except TransformException:
            self.reason = 'Waiting for transform at image timestamp'
            return
        t, q = tf.transform.translation, tf.transform.rotation
        camera_p = np.array([t.x, t.y, t.z])
        camera_r = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        observations = {}
        corners = {d.id: [[p.x, p.y] for p in d.corners] for d in msg.detections if len(d.corners) == 4}
        board = None
        if (self.calibration is not None and self.calibration_frame == msg.header.frame_id
                and abs(self.calibration_stamp - stamp) < .5):
            board = board_pose(corners, self.fixture, self.calibration)
        if board is not None:
            p, r, ids, _ = board
            for i in ids:
                observations[i] = (camera_p + camera_r @ (p + r @ self.fixture['tags'][i]), camera_r @ r[:, 2])
        else:
            for d in msg.detections:
                if not d.pose_valid or d.ambiguous or not np.isfinite(d.reprojection_error_px) or d.reprojection_error_px > 1.5:
                    continue
                p, q = d.pose.position, d.pose.orientation
                try:
                    pose = Pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
                    r = Rotation.from_quat(pose.quaternion_xyzw).as_matrix()
                    observations[d.id] = (camera_p + camera_r @ pose.position, camera_r @ r[:, 2])
                except ValueError:
                    continue
        self.map.add(observations, stamp)
        self.last_accepted_stamp = stamp
        self.pending = None
        self.reason = 'Accepted settled observations' if observations else 'No unambiguous metric observations'

    def publish(self):
        now = self.now()
        registration = self.map.registration(now)
        if not 0 <= now - self.last_camera_stamp < 1.:
            registration.valid, registration.reason = False, 'Camera stream stale or absent'
            if self.input_count:
                self.get_logger().warning(
                    f'Detection stream stale: accepted_age={now-self.last_camera_stamp:.3f}s '
                    f'input_age={now-self.last_input_stamp:.3f}s callback_age={time.monotonic()-self.last_callback:.3f}s '
                    f'count={self.input_count}', throttle_duration_sec=3.)
        if not self.history or not 0 <= now - self.history[-1][0] < .5:
            registration.valid, registration.reason = False, 'Joint feedback stale or absent'
        msg = PanelObservation()
        msg.header.frame_id = self.base
        msg.header.stamp = Time(seconds=registration.stamp).to_msg()
        msg.valid, msg.reason, msg.generation = registration.valid, registration.reason, self.map.generation
        msg.tag_ids, msg.registration_error_m = list(registration.ids), registration.error
        if registration.position is not None:
            msg.pose = pose_message(Pose(registration.position, Rotation.from_matrix(registration.rotation).as_quat()),
                                    self.base, msg.header.stamp).pose
        self.panel.publish(msg)
        tagmap = MapMessage(header=msg.header, generation=msg.generation)
        visuals = []
        for i, (p, _, count, spread, stamp) in self.map.landmarks(now).items():
            tagmap.tags.append(MappedTag(id=i, position=Point(x=float(p[0]), y=float(p[1]), z=float(p[2])),
                                        sample_count=count, spread_m=spread, last_seen=Time(seconds=stamp).to_msg()))
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = self.base, self.get_clock().now().to_msg()
            marker.ns, marker.id, marker.type = 'mapped_tags', i, Marker.SPHERE
            marker.pose.position = tagmap.tags[-1].position
            marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = marker.scale.z = .008
            marker.color.g, marker.color.a = 1., .95
            marker.lifetime = Duration(seconds=.3).to_msg()
            visuals.append(marker)
        if msg.valid:
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = self.base, self.get_clock().now().to_msg()
            marker.ns, marker.id, marker.type, marker.pose = 'registered_panel', 0, Marker.CUBE, msg.pose
            marker.scale.x, marker.scale.y, marker.scale.z = .13, .08, .001
            marker.color.r, marker.color.g, marker.color.a = 1., 1., .3
            marker.lifetime = Duration(seconds=.3).to_msg()
            visuals.append(marker)
        self.tags.publish(tagmap)
        self.markers.publish(MarkerArray(markers=visuals))


def main(args=None):
    run(Mapper, args)
