"""USB V4L2 camera: uninterrupted acquisition, bounded latest-frame publication."""
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from .capture import CaptureWorker
from .core import load_calibration
from .ros_common import parameter, positive, run, sensor_qos


def camera_info(calibration, width, height, header):
    message = CameraInfo()
    message.header = header
    message.width, message.height = width, height
    if calibration is not None and calibration.matches(width, height):
        message.distortion_model = ('plumb_bob' if len(calibration.distortion) <= 5
                                    else 'rational_polynomial')
        message.d = calibration.distortion.tolist()
        message.k = calibration.matrix.flatten().tolist()
        message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        k = calibration.matrix
        message.p = [float(k[0, 0]), 0.0, float(k[0, 2]), 0.0,
                     0.0, float(k[1, 1]), float(k[1, 2]), 0.0,
                     0.0, 0.0, 1.0, 0.0]
    # Otherwise K[0] == 0 is the ROS CameraInfo uncalibrated convention.
    return message


class UsbCamera(Node):
    def __init__(self):
        super().__init__('usb_camera')
        self.worker = None
        device = parameter(self, 'device', '/dev/video0')
        width = positive(parameter(self, 'width', 640), 'width')
        height = positive(parameter(self, 'height', 480), 'height')
        fps = positive(parameter(self, 'capture_fps', 30.0), 'capture_fps')
        publish_fps = positive(parameter(self, 'publish_fps', 30.0), 'publish_fps')
        fourcc = parameter(self, 'fourcc', 'MJPG')
        self.frame_id = parameter(self, 'frame_id', 'camera_optical_frame')
        if not self.frame_id or len(fourcc) != 4:
            raise ValueError('frame_id must be nonempty and fourcc must have four characters')
        self.calibration = load_calibration(parameter(self, 'calibration_file', ''))
        self.bridge = CvBridge()
        self.images = self.create_publisher(Image, 'image_raw', sensor_qos())
        self.infos = self.create_publisher(CameraInfo, 'camera_info', sensor_qos())
        self._last_dimensions = None
        self.timer = self.create_timer(1.0 / publish_fps, self.publish_latest)
        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        try:
            if not capture.isOpened():
                raise RuntimeError(f'Cannot open V4L2 device {device}')
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            capture.set(cv2.CAP_PROP_FPS, fps)
            if not capture.set(cv2.CAP_PROP_BUFFERSIZE, 1):
                self.get_logger().warning('Driver declined buffer size 1; continuous drain remains enabled')
            self.worker = CaptureWorker(capture, lambda: self.get_clock().now().to_msg())
            self.worker.start()
        except Exception:
            capture.release()
            raise
        self.get_logger().info('Timestamps are ROS-clock receipt after read/decode, not exposure time')

    def publish_latest(self):
        if self.worker.error:
            self.timer.cancel()
            raise RuntimeError(self.worker.error)
        frame = self.worker.slot.take()
        if frame is None:
            return
        height, width = frame.image.shape[:2]
        if self._last_dimensions != (width, height):
            self._last_dimensions = (width, height)
            self.get_logger().info(f'Actual camera resolution: {width}x{height}')
            if self.calibration is not None and not self.calibration.matches(width, height):
                self.get_logger().error('Calibration resolution mismatch: publishing uncalibrated CameraInfo')
        header = Header(stamp=frame.stamp, frame_id=self.frame_id)
        message = self.bridge.cv2_to_imgmsg(frame.image, encoding='bgr8')
        message.header = header
        self.infos.publish(camera_info(self.calibration, width, height, header))
        self.images.publish(message)

    def destroy_node(self):
        if self.worker is not None and not self.worker.close():
            self.get_logger().error('V4L2 read did not return during shutdown; daemon reader will exit with process')
        return super().destroy_node()


def main(args=None):
    run(UsbCamera, args)
