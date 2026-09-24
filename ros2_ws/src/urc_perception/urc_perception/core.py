"""ROS-independent ArUco detection, calibration and square IPPE estimation."""
from dataclasses import dataclass
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


def image_stamp_is_fresh(stamp_ns, now_ns, max_age_s):
    """Strict receipt-age boundary in ONE ROS clock domain; zero is unavailable."""
    if not math.isfinite(max_age_s) or max_age_s <= 0:
        raise ValueError('max_image_age_s must be finite and positive')
    return stamp_ns > 0 and 0 <= now_ns - stamp_ns <= int(max_age_s * 1e9)


@dataclass(frozen=True)
class Calibration:
    width: int
    height: int
    matrix: np.ndarray
    distortion: np.ndarray

    def __post_init__(self):
        k = np.asarray(self.matrix, dtype=np.float64).reshape(3, 3).copy()
        d = np.asarray(self.distortion, dtype=np.float64).reshape(-1).copy()
        if self.width <= 0 or self.height <= 0:
            raise ValueError('Calibration dimensions must be positive')
        if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
            raise ValueError('Calibration must have finite, positive focal lengths')
        if not np.allclose(k[2], [0, 0, 1]) or k[0, 1] != 0 or k[1, 0] != 0:
            raise ValueError('Expected an OpenCV pinhole camera matrix without skew')
        if len(d) not in (4, 5, 8, 12, 14) or not np.isfinite(d).all():
            raise ValueError('Invalid OpenCV distortion coefficients')
        k.setflags(write=False)
        d.setflags(write=False)
        object.__setattr__(self, 'matrix', k)
        object.__setattr__(self, 'distortion', d)

    def matches(self, width, height):
        return (self.width, self.height) == (width, height)


def load_calibration(filename):
    """Empty path means detection-only; an explicitly bad file fails startup."""
    if not filename:
        return None
    with Path(filename).expanduser().open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if data['distortion_model'] not in ('plumb_bob', 'rational_polynomial'):
        raise ValueError('Only raw pinhole plumb_bob/rational_polynomial is supported')
    distortion = data['distortion_coefficients']['data']
    if data['distortion_model'] == 'plumb_bob' and len(distortion) not in (4, 5):
        raise ValueError('plumb_bob requires 4 or 5 coefficients')
    if data['distortion_model'] == 'rational_polynomial' and len(distortion) != 8:
        raise ValueError('rational_polynomial requires 8 coefficients')
    return Calibration(int(data['image_width']), int(data['image_height']),
                       data['camera_matrix']['data'], distortion)


def get_dictionary(name):
    if not hasattr(cv2, 'aruco'):
        raise RuntimeError('OpenCV with the aruco module is required')
    if not name.startswith('DICT_') or not hasattr(cv2.aruco, name):
        raise ValueError('dictionary must name an explicit OpenCV DICT_* dictionary')
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def parse_sizes(entries, dictionary):
    """String list 'ID:metres'; no default sizes or competition ID assumptions."""
    sizes = {}
    for entry in entries:
        try:
            tag_id_text, size_text = entry.split(':')
            tag_id, size = int(tag_id_text), float(size_text)
        except (ValueError, AttributeError) as exc:
            raise ValueError('tag_sizes entries must be ID:metres') from exc
        if tag_id < 0 or tag_id >= len(dictionary.bytesList):
            raise ValueError('Tag ID is outside the selected dictionary')
        if tag_id in sizes or not math.isfinite(size) or size <= 0:
            raise ValueError('Tag sizes must be unique, finite and positive')
        sizes[tag_id] = size
    return sizes


def square_points(size):
    half = size / 2.0
    return np.array([[-half, half, 0], [half, half, 0],
                     [half, -half, 0], [-half, -half, 0]], dtype=np.float64)


@dataclass
class PoseEstimate:
    valid: bool = False
    rvec: object = None
    tvec: object = None
    error: float = float('nan')
    ambiguous: bool = False


def estimate_pose(corners, size, calibration, *, ambiguity_gap_px=0.2,
                  ambiguity_ratio=1.5, ambiguity_policy='reject', max_error_px=3.0):
    """Evaluate BOTH IPPE solutions, including depth of every transformed corner.

    Error is sqrt(mean(||projected - observed||**2)), not OpenCV's per-axis RMS.
    Near ties are ambiguous by absolute gap OR ratio, including exact zero ties.
    """
    if ambiguity_policy not in ('reject', 'flag'):
        raise ValueError('ambiguity_policy must be reject or flag')
    if (not all(math.isfinite(v) for v in
                (size, ambiguity_gap_px, ambiguity_ratio, max_error_px))
            or size <= 0 or ambiguity_gap_px < 0 or ambiguity_ratio < 1
            or max_error_px <= 0):
        raise ValueError('Invalid pose thresholds or marker size')
    points = square_points(size)
    pixels = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    if not np.isfinite(pixels).all():
        return PoseEstimate()
    try:
        solved = cv2.solvePnPGeneric(points, pixels, calibration.matrix,
                                    calibration.distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    except cv2.error:
        return PoseEstimate()
    candidates = []
    if not solved[0]:
        return PoseEstimate()
    for rotation, translation in zip(solved[1], solved[2]):
        rotation = np.asarray(rotation).reshape(3)
        translation = np.asarray(translation).reshape(3)
        if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
            continue
        matrix = cv2.Rodrigues(rotation)[0]
        depths = (matrix @ points.T).T[:, 2] + translation[2]
        if np.any(depths <= 1e-9):
            continue
        projected = cv2.projectPoints(points, rotation, translation,
                                      calibration.matrix, calibration.distortion)[0].reshape(4, 2)
        error = float(np.sqrt(np.mean(np.sum((projected - pixels) ** 2, axis=1))))
        if math.isfinite(error):
            candidates.append((error, rotation, translation))
    if not candidates:
        return PoseEstimate()
    candidates.sort(key=lambda item: item[0])
    error, rotation, translation = candidates[0]
    ambiguous = False
    if len(candidates) > 1:
        second = candidates[1][0]
        ambiguous = (second - error <= ambiguity_gap_px
                     or second <= ambiguity_ratio * max(error, 1e-12))
    valid = error <= max_error_px and not (ambiguous and ambiguity_policy == 'reject')
    return PoseEstimate(valid, rotation if valid else None,
                        translation if valid else None, error, ambiguous)


def quaternion_xyzw(rvec):
    vector = np.asarray(rvec, dtype=np.float64).reshape(3)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    xyz = vector * (math.sin(angle / 2) / angle)
    return (*map(float, xyz), math.cos(angle / 2))


@dataclass
class Detection:
    tag_id: int
    corners: np.ndarray
    pose: PoseEstimate


class Detector:
    def __init__(self, dictionary, sizes=(), calibration=None, **pose_options):
        self.dictionary = get_dictionary(dictionary)
        self.sizes = parse_sizes(sizes, self.dictionary)
        self.calibration = calibration
        self.pose_options = pose_options
        # OpenCV 4.6 exposes DetectorParameters as a type but its bare constructor
        # can leave the C++ pointer uninitialized. Prefer its factory when present.
        parameters = (cv2.aruco.DetectorParameters_create()
                      if hasattr(cv2.aruco, 'DetectorParameters_create')
                      else cv2.aruco.DetectorParameters())
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters = parameters
        self.backend = (cv2.aruco.ArucoDetector(self.dictionary, parameters)
                        if hasattr(cv2.aruco, 'ArucoDetector') else None)

    def detect(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        if gray.dtype != np.uint8 or gray.ndim != 2:
            raise ValueError('Expected uint8 grayscale or BGR image')
        if self.backend is not None:
            corners, ids, _ = self.backend.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.dictionary, parameters=self.parameters)
        if ids is None:
            return []
        calibrated = self.calibration is not None and self.calibration.matches(
            gray.shape[1], gray.shape[0])
        detections = []
        for tag_id, corner in zip(ids.flatten(), corners):
            tag_id = int(tag_id)
            pixels = np.asarray(corner, dtype=np.float64).reshape(4, 2)
            pose = (estimate_pose(pixels, self.sizes[tag_id], self.calibration,
                                  **self.pose_options)
                    if calibrated and tag_id in self.sizes else PoseEstimate())
            detections.append(Detection(tag_id, pixels, pose))
        return detections
