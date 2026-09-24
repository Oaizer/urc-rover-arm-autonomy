"""ROS adapter: latest image mailbox and optional throttled annotation."""
import math
import time

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point32
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from urc_interfaces.msg import TagDetection, TagDetections

from .capture import LatestSlot
from .core import Detector, image_stamp_is_fresh, load_calibration, quaternion_xyzw
from .ros_common import parameter, positive, run, sensor_qos


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        threads = parameter(self, 'opencv_threads', 1)
        if isinstance(threads, bool) or not isinstance(threads, int) or not 1 <= threads <= 8:
            raise ValueError('opencv_threads must be an integer from 1 to 8')
        cv2.setNumThreads(threads)
        self.dictionary_name = parameter(self, 'dictionary', '')
        sizes = parameter(self, 'tag_sizes', Parameter.Type.STRING_ARRAY) or []
        calibration = load_calibration(parameter(self, 'calibration_file', ''))
        policy = parameter(self, 'ambiguity_policy', 'reject')
        gap = parameter(self, 'ambiguity_gap_px', 0.2)
        ratio = parameter(self, 'ambiguity_ratio', 1.5)
        max_error = positive(parameter(self, 'max_reprojection_error_px', 3.0),
                             'max_reprojection_error_px')
        if policy not in ('reject', 'flag') or not math.isfinite(gap) or gap < 0:
            raise ValueError('Invalid ambiguity_policy or ambiguity_gap_px')
        if not math.isfinite(ratio) or ratio < 1:
            raise ValueError('ambiguity_ratio must be finite and >= 1')
        self.detector = Detector(self.dictionary_name, sizes, calibration,
                                 ambiguity_policy=policy, ambiguity_gap_px=gap,
                                 ambiguity_ratio=ratio, max_error_px=max_error)
        detection_fps = positive(parameter(self, 'detection_fps', 30.0), 'detection_fps')
        self.max_image_age_s = positive(parameter(self, 'max_image_age_s', 0.25),
                                        'max_image_age_s')
        publish_annotated = parameter(self, 'publish_annotated', False)
        self.annotated_fps = positive(parameter(self, 'annotated_fps', 5.0), 'annotated_fps')
        self.annotated_scale = positive(parameter(self, 'annotated_scale', 1.0), 'annotated_scale')
        if self.annotated_scale > 1.:
            raise ValueError('annotated_scale must not exceed 1')
        self.annotations = (self.create_publisher(Image, 'image_annotated', sensor_qos())
                            if publish_annotated else None)
        self.publisher = self.create_publisher(TagDetections, 'tag_detections', sensor_qos())
        self.bridge = CvBridge()
        self.slot = LatestSlot()
        self.subscription = self.create_subscription(Image, 'image_raw', self.slot.put, sensor_qos())
        self.timer = self.create_timer(1.0 / detection_fps, self.process_latest)
        self.last_annotation = -math.inf
        self.last_dimensions = None
        if calibration is None:
            self.get_logger().warning('No calibration: pixel detections only, no metric poses')
        if not self.detector.sizes:
            self.get_logger().warning('No tag_sizes: pixel detections only, no metric poses')

    def process_latest(self):
        message = self.slot.take()
        if message is None:
            return
        if not message.header.frame_id:
            return
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if not image_stamp_is_fresh(stamp_ns, self.get_clock().now().nanoseconds,
                                    self.max_image_age_s):
            return
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='mono8' if message.encoding == 'mono8' else 'bgr8')
        except (CvBridgeError, ValueError) as exc:
            self.get_logger().error(f'Image conversion failed: {exc}')
            return
        height, width = image.shape[:2]
        calibration = self.detector.calibration
        if self.last_dimensions != (width, height):
            self.last_dimensions = (width, height)
            if calibration is not None and not calibration.matches(width, height):
                self.get_logger().error('Calibration resolution mismatch: pixel detections only')
        detections = self.detector.detect(image)
        result = TagDetections()
        result.header = message.header
        result.dictionary = self.dictionary_name
        for detection in detections:
            tag = TagDetection()
            tag.id = detection.tag_id
            tag.corners = [Point32(x=float(x), y=float(y), z=0.0)
                           for x, y in detection.corners]
            tag.pose_valid = detection.pose.valid
            tag.reprojection_error_px = detection.pose.error
            tag.ambiguous = detection.pose.ambiguous
            if detection.pose.valid:
                tag.pose.position.x, tag.pose.position.y, tag.pose.position.z = map(
                    float, detection.pose.tvec)
                q = quaternion_xyzw(detection.pose.rvec)
                (tag.pose.orientation.x, tag.pose.orientation.y,
                 tag.pose.orientation.z, tag.pose.orientation.w) = q
            result.detections.append(tag)
        # Processing time counts toward freshness too. Never restamp stale observations.
        if not image_stamp_is_fresh(stamp_ns, self.get_clock().now().nanoseconds,
                                    self.max_image_age_s):
            return
        self.publisher.publish(result)
        now = time.monotonic()
        if self.annotations is not None and now - self.last_annotation >= 1 / self.annotated_fps:
            self.last_annotation = now
            annotated = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if image.ndim == 2 else image.copy()
            for detection in detections:
                points = np.rint(detection.corners).astype(np.int32)
                color = (0, 200, 0) if detection.pose.valid else (0, 180, 255)
                cv2.polylines(annotated, [points], True, color, 2)
                label = f'{detection.tag_id}: ' + (
                    'ambiguous' if detection.pose.ambiguous else
                    'pose' if detection.pose.valid else 'pixels only')
                cv2.putText(annotated, label, tuple(points[0]), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, color, 1, cv2.LINE_AA)
            if self.annotated_scale < 1.:
                annotated = cv2.resize(annotated, None, fx=self.annotated_scale, fy=self.annotated_scale,
                                       interpolation=cv2.INTER_AREA)
            output = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            output.header = message.header
            if image_stamp_is_fresh(stamp_ns, self.get_clock().now().nanoseconds,
                                     self.max_image_age_s):
                self.annotations.publish(output)


def main(args=None):
    run(ArucoDetectorNode, args)
