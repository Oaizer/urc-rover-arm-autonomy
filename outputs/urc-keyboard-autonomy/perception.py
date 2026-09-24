"""USB camera capture, ArUco keyboard pose, and display verification."""

from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np

from interfaces import KeyboardObservation, TagDetection


class LatestCamera:
    def __init__(self, device: str, width: int, height: int, fps: int) -> None:
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open USB camera {device}")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def read(self) -> tuple[np.ndarray, int]:
        ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError("USB camera returned no frame")
        return frame, time.monotonic_ns()

    def close(self) -> None:
        self.cap.release()


class ArucoTagDetector:
    """Estimate each visible tag independently for multi-view 3D fusion."""

    def __init__(self, camera_matrix: np.ndarray, distortion: np.ndarray,
                 dictionary_name: str, marker_size_mm: float,
                 allowed_ids: tuple[int, ...] = (0, 1, 2, 3, 4)) -> None:
        self.K = np.asarray(camera_matrix, dtype=float)
        self.D = np.asarray(distortion, dtype=float)
        if self.K.shape != (3, 3):
            raise ValueError("camera_matrix must be 3x3")
        if marker_size_mm <= 0:
            raise ValueError("marker_size_mm must be positive")
        dictionary_id = getattr(cv2.aruco, dictionary_name)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.marker_size = float(marker_size_mm)
        self.allowed_ids = frozenset(int(x) for x in allowed_ids)
        self.params = cv2.aruco.DetectorParameters()
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)

    def detect(self, frame: np.ndarray, timestamp_ns: int) -> list[TagDetection]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return []
        half = self.marker_size / 2.0
        # Corner order matches ArUco: top-left, top-right, bottom-right, bottom-left.
        object_points = np.asarray([
            [-half, half, 0.0], [half, half, 0.0],
            [half, -half, 0.0], [-half, -half, 0.0],
        ], dtype=np.float32)
        detections: list[TagDetection] = []
        for corner, raw_id in zip(corners, ids.reshape(-1)):
            marker_id = int(raw_id)
            if marker_id not in self.allowed_ids:
                continue
            image_points = np.asarray(corner, dtype=np.float32).reshape(4, 2)
            ok, rvec, tvec = cv2.solvePnP(
                object_points, image_points, self.K, self.D,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok or float(tvec.reshape(3)[2]) <= 0.0:
                continue
            projected, _ = cv2.projectPoints(object_points, rvec, tvec, self.K, self.D)
            error = float(np.sqrt(np.mean(np.sum(
                (projected.reshape(4, 2) - image_points) ** 2, axis=1))))
            rotation, _ = cv2.Rodrigues(rvec)
            transform = np.eye(4)
            transform[:3, :3] = rotation
            transform[:3, 3] = tvec.reshape(3)
            pixel_area = abs(float(cv2.contourArea(image_points)))
            detections.append(TagDetection(
                timestamp_ns, marker_id, transform, error, pixel_area))
        return detections


class KeyboardLocalizer:
    def __init__(self, camera_matrix: np.ndarray, distortion: np.ndarray,
                 dictionary_name: str, marker_size_mm: float,
                 tag_centers_mm: dict[int, tuple[float, float]]) -> None:
        if not tag_centers_mm:
            raise ValueError("measured tag_centers_mm is required")
        self.K = np.asarray(camera_matrix, dtype=float)
        self.D = np.asarray(distortion, dtype=float)
        dictionary_id = getattr(cv2.aruco, dictionary_name)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.marker_size = float(marker_size_mm)
        self.tag_centers = {int(k): np.asarray(v, dtype=float) for k, v in tag_centers_mm.items()}
        self.params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)

    def observe(self, frame: np.ndarray, timestamp_ns: int) -> KeyboardObservation | None:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return None
        object_points, image_points, visible = [], [], []
        half = self.marker_size / 2.0
        for corner, raw_id in zip(corners, ids.reshape(-1)):
            marker_id = int(raw_id)
            if marker_id not in self.tag_centers:
                continue
            cx, cy = self.tag_centers[marker_id]
            object_points.extend([[cx-half, cy-half, 0], [cx+half, cy-half, 0],
                                  [cx+half, cy+half, 0], [cx-half, cy+half, 0]])
            image_points.extend(corner.reshape(4, 2).tolist())
            visible.append(marker_id)
        if len(visible) < 4:
            return None
        ok, rvec, tvec = cv2.solvePnP(np.asarray(object_points, dtype=np.float32),
                                      np.asarray(image_points, dtype=np.float32),
                                      self.K, self.D, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            return None
        projected, _ = cv2.projectPoints(np.asarray(object_points, dtype=np.float32), rvec, tvec, self.K, self.D)
        error = float(np.mean(np.linalg.norm(projected.reshape(-1, 2) - np.asarray(image_points), axis=1)))
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, tvec.reshape(3)
        return KeyboardObservation(timestamp_ns, T, tuple(visible), error, (frame.shape[1], frame.shape[0]))


class DisplayVerifier:
    def __init__(self, roi: tuple[int, int, int, int] | None) -> None:
        self.roi = roi

    def read(self, frame: np.ndarray) -> str | None:
        if self.roi is None:
            return None
        try:
            import pytesseract
        except ImportError:
            return None
        x, y, w, h = self.roi
        crop = frame[y:y+h, x:x+w]
        if crop.size == 0:
            return None
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        text = pytesseract.image_to_string(binary, config="--psm 7")
        cleaned = "".join(ch for ch in text.upper() if ch.isalnum())
        return cleaned or None
