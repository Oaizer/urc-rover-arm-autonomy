#!/usr/bin/env python3
"""Capture multiple cameras, detect/annotate ArUco tags, and send UDP JPEGs."""

import argparse
import socket
import struct
import time
from typing import Dict, Tuple

import cv2
import numpy as np

try:
    from picamera2 import Picamera2
except ImportError:  # USB mode does not need Picamera2 installed.
    Picamera2 = None


MAGIC = b"ARU2"
# magic, stream/camera id, reserved, frame id, packet index, packet count, timestamp
HEADER = struct.Struct("!4sBBIHHQ")
MAX_DATAGRAM = 1200
PAYLOAD_SIZE = MAX_DATAGRAM - HEADER.size
DICTIONARIES: Dict[str, int] = {
    name: getattr(cv2.aruco, name)
    for name in dir(cv2.aruco)
    if name.startswith("DICT_") and isinstance(getattr(cv2.aruco, name), int)
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True, help="Laptop Ethernet IPv4 address")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--backend", choices=("usb", "picamera2", "auto"), default="usb",
                        help="Camera backend; USB/V4L2 is the default")
    parser.add_argument("--camera-devices", default="/dev/video0",
                        help="Comma-separated USB V4L2 devices, e.g. /dev/video0,/dev/video2")
    parser.add_argument("--camera-nums", default="0",
                        help="Comma-separated Picamera2 numbers, used with --backend picamera2")
    parser.add_argument("--camera-num", type=int, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--fps", type=int, default=30, help="Target FPS per camera")
    parser.add_argument("--detect-width", type=int, default=640)
    parser.add_argument("--detect-every", type=int, default=2)
    parser.add_argument("--dictionary", default="DICT_4X4_50", choices=sorted(DICTIONARIES))
    parser.add_argument("--jpeg-quality", type=int, default=65)
    return parser.parse_args()


def camera_numbers(args: argparse.Namespace) -> list[int]:
    raw = str(args.camera_num) if args.camera_num is not None else args.camera_nums
    try:
        numbers = [int(item.strip()) for item in raw.split(",") if item.strip()]
    except ValueError as exc:
        raise SystemExit("camera numbers must be comma-separated integers") from exc
    if not numbers or len(set(numbers)) != len(numbers) or any(n < 0 or n > 255 for n in numbers):
        raise SystemExit("camera numbers must be unique integers from 0 to 255")
    return numbers


def camera_devices(args: argparse.Namespace) -> list[str]:
    devices = [item.strip() for item in args.camera_devices.split(",") if item.strip()]
    if not devices:
        raise SystemExit("camera-devices must contain at least one /dev/video device")
    if len(devices) > 256:
        raise SystemExit("at most 256 camera devices are supported")
    return devices


def make_detector(dictionary_name: str):
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARIES[dictionary_name])
    if hasattr(cv2.aruco, "DetectorParameters"):
        params = cv2.aruco.DetectorParameters()
    else:
        params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    params.adaptiveThreshWinSizeStep = 10
    params.minMarkerPerimeterRate = 0.02
    params.maxMarkerPerimeterRate = 4.0
    if hasattr(cv2.aruco, "ArucoDetector"):
        return cv2.aruco.ArucoDetector(dictionary, params)
    return dictionary, params


def detect_markers(detector, gray: np.ndarray):
    if isinstance(detector, tuple):
        dictionary, params = detector
        return cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    return detector.detectMarkers(gray)


def annotate(frame_bgr: np.ndarray, detector, detect_width: int,
             previous: Tuple[np.ndarray, np.ndarray], should_detect: bool,
             camera_num: int) -> Tuple[np.ndarray, Tuple[np.ndarray, np.ndarray]]:
    height, width = frame_bgr.shape[:2]
    corners, ids = previous
    if should_detect:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        scale = min(1.0, detect_width / float(width))
        if scale != 1.0:
            small = cv2.resize(gray, (int(width * scale), int(height * scale)), interpolation=cv2.INTER_AREA)
        else:
            small = gray
        found_corners, found_ids, _ = detect_markers(detector, small)
        if found_ids is not None:
            corners = [c.astype(np.float32) / scale for c in found_corners]
            ids = found_ids.reshape(-1).astype(int)
        else:
            corners, ids = [], np.empty((0,), dtype=int)

    for marker_corners, marker_id in zip(corners, ids):
        pts = np.round(marker_corners.reshape(4, 2)).astype(np.int32)
        cv2.polylines(frame_bgr, [pts], True, (0, 255, 0), 2, cv2.LINE_AA)
        x, y = int(pts[:, 0].min()), int(pts[:, 1].min())
        cv2.putText(frame_bgr, f"id {int(marker_id)}", (x, max(20, y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(frame_bgr, f"cam {camera_num} | ArUco n={len(ids)}",
                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2, cv2.LINE_AA)
    return frame_bgr, (corners, ids)


def send_frame(sock: socket.socket, destination: Tuple[str, int], camera_id: int,
               frame_id: int, timestamp_ns: int, jpeg: bytes) -> int:
    packet_count = (len(jpeg) + PAYLOAD_SIZE - 1) // PAYLOAD_SIZE
    if packet_count > 65535:
        raise ValueError("JPEG frame is too large for the UDP packet format")
    for packet_index in range(packet_count):
        start = packet_index * PAYLOAD_SIZE
        header = HEADER.pack(MAGIC, camera_id, 0, frame_id & 0xFFFFFFFF,
                             packet_index, packet_count, timestamp_ns)
        sock.sendto(header + jpeg[start:start + PAYLOAD_SIZE], destination)
    return packet_count


def main() -> None:
    args = parse_args()
    if args.detect_every < 1 or args.fps < 1 or args.jpeg_quality < 1 or args.jpeg_quality > 100:
        raise SystemExit("detect-every/fps must be positive and jpeg-quality must be 1..100")

    backend = args.backend
    if backend == "auto":
        backend = "usb" if args.camera_devices else "picamera2"

    detector = make_detector(args.dictionary)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
    destination = (args.host, args.port)
    cameras = []
    camera_labels = []
    try:
        if backend == "usb":
            camera_labels = camera_devices(args)
            for device in camera_labels:
                cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
                if not cap.isOpened():
                    raise RuntimeError(f"cannot open {device}; check it is a capture node")
                # MJPG keeps USB bandwidth low. The camera performs the JPEG
                # compression; OpenCV decodes the frame locally on the Pi.
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
                cap.set(cv2.CAP_PROP_FPS, args.fps)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                ok, frame = cap.read()
                if not ok or frame is None:
                    cap.release()
                    raise RuntimeError(f"{device} opened but returned no frames")
                cameras.append(cap)
        else:
            if Picamera2 is None:
                raise RuntimeError("Picamera2 is not installed; use --backend usb for USB cameras")
            picamera_numbers = camera_numbers(args)
            camera_labels = [f"picamera2:{number}" for number in picamera_numbers]
            for camera_num in picamera_numbers:
                picam = Picamera2(camera_num)
                config = picam.create_video_configuration(
                    main={"size": (args.width, args.height), "format": "RGB888"},
                    buffer_count=3,
                )
                picam.configure(config)
                cameras.append(picam)
            for picam in cameras:
                picam.start()
            time.sleep(0.5)

        previous = [([], np.empty((0,), dtype=int)) for _ in cameras]
        frame_ids = [0 for _ in cameras]
        next_frame_times = [time.monotonic() for _ in cameras]
        last_report = time.monotonic()
        captured = [0 for _ in cameras]
        packets = [0 for _ in cameras]
        print(f"Sending {backend} cameras {camera_labels} to {args.host}:{args.port}; "
              f"{args.width}x{args.height}@{args.fps} each; detect {args.detect_width}px "
              f"every {args.detect_every} frame(s); {args.dictionary}", flush=True)

        while True:
            camera_index = min(range(len(cameras)), key=lambda i: next_frame_times[i])
            now = time.monotonic()
            if now < next_frame_times[camera_index]:
                time.sleep(next_frame_times[camera_index] - now)
            next_frame_times[camera_index] = max(
                next_frame_times[camera_index] + (1.0 / args.fps), time.monotonic()
            )

            frame_id = frame_ids[camera_index]
            if backend == "usb":
                ok, frame = cameras[camera_index].read()
                if not ok or frame is None:
                    continue
            else:
                # Picamera2 RGB888 is converted once so the rest of the path
                # uses the same BGR representation as V4L2/OpenCV.
                frame_rgb = cameras[camera_index].capture_array("main")
                frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            should_detect = frame_id % args.detect_every == 0
            frame, previous[camera_index] = annotate(
                frame, detector, args.detect_width, previous[camera_index],
                should_detect, camera_index
            )
            ok, encoded_frame = cv2.imencode(
                ".jpg", frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality],
            )
            if not ok:
                continue
            packets[camera_index] += send_frame(
                sock, destination, camera_index, frame_id,
                time.monotonic_ns(), encoded_frame.tobytes()
            )
            captured[camera_index] += 1
            frame_ids[camera_index] += 1

            now = time.monotonic()
            if now - last_report >= 5.0:
                details = " ".join(
                    f"cam{i}:frames={captured[i]} packets={packets[i]} tags={len(previous[i][1])}"
                    for i in range(len(cameras))
                )
                print(details, flush=True)
                captured = [0 for _ in cameras]
                packets = [0 for _ in cameras]
                last_report = now
    except KeyboardInterrupt:
        pass
    finally:
        for camera in cameras:
            if backend == "usb":
                camera.release()
            else:
                camera.stop()
        sock.close()


if __name__ == "__main__":
    main()
