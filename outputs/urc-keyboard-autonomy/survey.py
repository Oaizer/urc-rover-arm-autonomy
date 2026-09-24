"""Multi-view ArUco fusion and keyboard-plane registration.

All distances are millimetres and all poses are 4x4 homogeneous transforms.
This module contains no camera or motor I/O, so it can be tested before the
hardware is available.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
import time
from typing import Callable, Sequence

import numpy as np

from interfaces import (JointFeedback, JointInterface, KeyboardSurveyResult,
                        MotionCommand, TagDetection, TagDetectorInterface,
                        TagObservation)


@dataclass(frozen=True)
class SurveyConfig:
    allowed_marker_ids: tuple[int, ...] = (0, 1, 2, 3, 4)
    min_unique_tags: int = 3
    min_total_observations: int = 5
    min_viewpoints: int = 2
    min_viewpoint_separation_deg: float = 8.0
    max_reprojection_error_px: float = 1.5
    min_marker_pixel_area: float = 100.0
    max_timestamp_delta_ms: float = 10.0
    min_triangle_area_mm2: float = 500.0
    max_plane_rms_mm: float = 3.0
    max_registration_rms_mm: float = 3.0
    max_position_uncertainty_mm: float = 2.0
    require_normal_faces_base: bool = True


@dataclass(frozen=True)
class SurveyViewpoint:
    name: str
    yaw_deg: float
    pitch_deg: float


def default_front_viewpoints() -> tuple[SurveyViewpoint, ...]:
    """Front-only search order, with early exit as soon as confidence passes."""
    return (
        SurveyViewpoint("center", 0.0, 0.0),
        SurveyViewpoint("left", -15.0, 0.0),
        SurveyViewpoint("right", 15.0, 0.0),
        SurveyViewpoint("up", 0.0, 12.0),
        SurveyViewpoint("down", 0.0, -12.0),
        SurveyViewpoint("left_up", -15.0, 12.0),
        SurveyViewpoint("right_up", 15.0, 12.0),
        SurveyViewpoint("left_down", -15.0, -12.0),
        SurveyViewpoint("right_down", 15.0, -12.0),
    )


class ActiveSurveyExecutor:
    """Safely move through commissioned poses and stop as soon as survey passes.

    The viewpoint joint positions must be collision-checked and commissioned
    before live use. Camera timestamps are paired with interpolated encoder
    feedback from samples immediately before and after each capture.
    """

    def __init__(self, joints: JointInterface, camera, detector: TagDetectorInterface,
                 estimator: "ActiveKeyboardSurvey",
                 base_from_camera: Callable[[JointFeedback], np.ndarray],
                 velocity_limit_rad_s: float = 0.2,
                 acceleration_limit_rad_s2: float = 0.4,
                 torque_limit: float = 0.0,
                 tracking_tolerance_rad: float = 0.03,
                 feedback_timeout_ms: float = 100.0,
                 move_timeout_s: float = 8.0,
                 settle_s: float = 0.3,
                 frames_per_viewpoint: int = 3,
                 telemetry=None) -> None:
        self.joints, self.camera, self.detector = joints, camera, detector
        self.estimator, self.base_from_camera = estimator, base_from_camera
        self.velocity_limit = float(velocity_limit_rad_s)
        self.acceleration_limit = float(acceleration_limit_rad_s2)
        self.torque_limit = float(torque_limit)
        self.tracking_tolerance = float(tracking_tolerance_rad)
        self.feedback_timeout_ms = float(feedback_timeout_ms)
        self.move_timeout_s = float(move_timeout_s)
        self.settle_s = float(settle_s)
        self.frames_per_viewpoint = int(frames_per_viewpoint)
        self.telemetry = telemetry
        self.sequence = 0
        if self.frames_per_viewpoint < 1:
            raise ValueError("frames_per_viewpoint must be positive")

    def run(self, viewpoints: Sequence[tuple[str, np.ndarray]]) -> tuple[KeyboardSurveyResult, list[np.ndarray]]:
        if not viewpoints:
            raise ValueError("at least one commissioned survey viewpoint is required")
        retained_frames: list[np.ndarray] = []
        for name, target in viewpoints:
            target = np.asarray(target, dtype=float)
            if target.shape != (6,) or not np.all(np.isfinite(target)):
                raise ValueError(f"viewpoint {name} must contain six finite joint angles")
            self._move(target)
            time.sleep(self.settle_s)
            accepted = 0
            for _ in range(self.frames_per_viewpoint):
                before = self._safe_feedback()
                frame, image_timestamp_ns = self.camera.read()
                after = self._safe_feedback()
                feedback = _feedback_at(image_timestamp_ns, before, after)
                detections = self.detector.detect(frame, image_timestamp_ns)
                T_base_camera = self.base_from_camera(feedback)
                accepted += self.estimator.add_frame(
                    detections, image_timestamp_ns, feedback,
                    T_base_camera, str(name))
                retained_frames.append(frame)
                retained_frames = retained_frames[-8:]
            ready, reason = self.estimator.status()
            if self.telemetry:
                self.telemetry.write("survey_viewpoint", viewpoint=name,
                                     accepted=accepted, ready=ready, reason=reason)
            if ready:
                return self.estimator.build_result(), retained_frames
        self.joints.stop()
        _, reason = self.estimator.status()
        raise RuntimeError(f"survey exhausted commissioned viewpoints: {reason}")

    def _safe_feedback(self) -> JointFeedback:
        feedback = self.joints.feedback()
        if not feedback.fresh(self.feedback_timeout_ms):
            self.joints.stop()
            raise RuntimeError("stale joint feedback during survey")
        if feedback.faults:
            self.joints.stop()
            raise RuntimeError("; ".join(feedback.faults))
        if feedback.positions_rad.shape != (6,) or not np.all(np.isfinite(feedback.positions_rad)):
            self.joints.stop()
            raise RuntimeError("invalid joint feedback during survey")
        return feedback

    def _move(self, target: np.ndarray) -> None:
        self.sequence += 1
        command = MotionCommand(
            self.sequence, target, self.velocity_limit, self.acceleration_limit,
            self.torque_limit, time.monotonic_ns() + int(self.move_timeout_s * 1e9))
        self.joints.send(command)
        deadline = time.monotonic() + self.move_timeout_s
        while True:
            feedback = self._safe_feedback()
            error = float(np.max(np.abs(feedback.positions_rad - target)))
            if error <= self.tracking_tolerance:
                return
            if time.monotonic() >= deadline:
                self.joints.stop()
                raise RuntimeError(f"survey viewpoint tracking error {error:.4f} rad")
            time.sleep(0.01)


class ActiveKeyboardSurvey:
    """Accumulate timestamped tag poses and register the keyboard in base."""

    def __init__(self, tag_centers_keyboard_mm: dict[int, tuple[float, float]],
                 config: SurveyConfig | None = None) -> None:
        if len(tag_centers_keyboard_mm) < 3:
            raise ValueError("at least three measured keyboard tag centers are required")
        self.tag_centers = {
            int(marker_id): np.asarray((xy[0], xy[1], 0.0), dtype=float)
            for marker_id, xy in tag_centers_keyboard_mm.items()
        }
        self.cfg = config or SurveyConfig()
        self.observations: list[TagObservation] = []
        self.rejections: list[str] = []

    def clear(self) -> None:
        self.observations.clear()
        self.rejections.clear()

    def add_frame(self, detections: list[TagDetection], image_timestamp_ns: int,
                  feedback: JointFeedback, T_base_camera: np.ndarray,
                  viewpoint_id: str) -> int:
        """Transform accepted detections into base coordinates.

        The supplied joint feedback must correspond to the image timestamp.
        A production capture loop should retrieve it from a timestamped joint
        history rather than using whichever feedback happens to arrive later.
        """
        T_base_camera = np.asarray(T_base_camera, dtype=float)
        if T_base_camera.shape != (4, 4) or not np.all(np.isfinite(T_base_camera)):
            raise ValueError("T_base_camera must be a finite 4x4 transform")
        delta_ms = abs(feedback.timestamp_ns - image_timestamp_ns) / 1e6
        if delta_ms > self.cfg.max_timestamp_delta_ms:
            self.rejections.append(f"stale synchronization: {delta_ms:.3f} ms")
            return 0
        accepted = 0
        camera_position = T_base_camera[:3, 3].copy()
        camera_forward = _unit(T_base_camera[:3, 2])
        for detection in detections:
            if detection.marker_id not in self.cfg.allowed_marker_ids:
                self.rejections.append(f"unexpected marker id {detection.marker_id}")
                continue
            if detection.marker_id not in self.tag_centers:
                self.rejections.append(f"marker {detection.marker_id} has no keyboard coordinate")
                continue
            if not detection.valid:
                self.rejections.append(f"invalid marker {detection.marker_id}")
                continue
            if detection.reprojection_error_px > self.cfg.max_reprojection_error_px:
                self.rejections.append(f"marker {detection.marker_id} reprojection error")
                continue
            if detection.pixel_area < self.cfg.min_marker_pixel_area:
                self.rejections.append(f"marker {detection.marker_id} too small")
                continue
            T_base_tag = T_base_camera @ detection.T_camera_tag
            normal = _unit(T_base_tag[:3, 2])
            self.observations.append(TagObservation(
                detection.marker_id, image_timestamp_ns, feedback.timestamp_ns,
                str(viewpoint_id), T_base_tag[:3, 3].copy(), normal,
                camera_position, camera_forward,
                float(detection.reprojection_error_px), float(detection.pixel_area)))
            accepted += 1
        return accepted

    def status(self) -> tuple[bool, str]:
        try:
            self.build_result()
        except ValueError as exc:
            return False, str(exc)
        return True, "keyboard survey complete"

    def build_result(self) -> KeyboardSurveyResult:
        if len(self.observations) < self.cfg.min_total_observations:
            raise ValueError("insufficient accepted tag observations")
        fused = self._fused_tags()
        if len(fused) < self.cfg.min_unique_tags:
            raise ValueError("fewer than three reliable tag IDs")
        marker_ids = tuple(sorted(fused))
        measured = np.array([fused[i][0] for i in marker_ids])
        known = np.array([self.tag_centers[i] for i in marker_ids])
        if _max_triangle_area(measured) < self.cfg.min_triangle_area_mm2:
            raise ValueError("tag centers are collinear or too closely spaced")
        viewpoint_count = len({o.viewpoint_id for o in self.observations})
        if viewpoint_count < self.cfg.min_viewpoints:
            raise ValueError("observations do not include enough viewpoints")
        separation = self._viewpoint_separation_deg()
        if separation < self.cfg.min_viewpoint_separation_deg:
            raise ValueError("camera viewpoints are not sufficiently separated")

        plane_origin, plane_normal, plane_rms = _fit_plane(measured)
        if plane_rms > self.cfg.max_plane_rms_mm:
            raise ValueError(f"plane residual {plane_rms:.3f} mm is too high")
        T_base_keyboard, registration_rms = _rigid_registration(known, measured)
        if registration_rms > self.cfg.max_registration_rms_mm:
            raise ValueError(f"keyboard registration residual {registration_rms:.3f} mm is too high")

        # Keep the registered normal consistent with the measured plane normal.
        if np.dot(T_base_keyboard[:3, 2], plane_normal) < 0.0:
            plane_normal = -plane_normal
        normal_faces_base = bool(np.dot(T_base_keyboard[:3, 2], -plane_origin) > 0.0)
        if self.cfg.require_normal_faces_base and not normal_faces_base:
            raise ValueError("keyboard normal does not face the arm base")
        fusion_spread = float(np.sqrt(np.mean([fused[i][1] ** 2 for i in marker_ids])))
        uncertainty = float(np.hypot(registration_rms, fusion_spread))
        if uncertainty > self.cfg.max_position_uncertainty_mm:
            raise ValueError(f"keyboard position uncertainty {uncertainty:.3f} mm is too high")
        return KeyboardSurveyResult(
            T_base_keyboard, marker_ids, len(self.observations), viewpoint_count,
            separation, plane_rms, registration_rms, uncertainty, normal_faces_base)

    def _fused_tags(self) -> dict[int, tuple[np.ndarray, float]]:
        result: dict[int, tuple[np.ndarray, float]] = {}
        for marker_id in sorted({o.marker_id for o in self.observations}):
            rows = [o for o in self.observations if o.marker_id == marker_id]
            points = np.array([o.position_base_mm for o in rows])
            median = np.median(points, axis=0)
            distance = np.linalg.norm(points - median, axis=1)
            mad = float(np.median(np.abs(distance - np.median(distance))))
            threshold = max(2.0, 3.0 * 1.4826 * mad)
            keep = distance <= threshold
            if not np.any(keep):
                continue
            kept_rows = [row for row, use in zip(rows, keep) if use]
            kept_points = points[keep]
            weights = np.array([
                min(row.pixel_area, 10000.0) / max(row.reprojection_error_px, 0.1) ** 2
                for row in kept_rows
            ])
            center = np.average(kept_points, axis=0, weights=weights)
            spread = float(np.sqrt(np.average(
                np.sum((kept_points - center) ** 2, axis=1), weights=weights)))
            result[marker_id] = (center, spread)
        return result

    def _viewpoint_separation_deg(self) -> float:
        directions: dict[str, np.ndarray] = {}
        for observation in self.observations:
            directions.setdefault(observation.viewpoint_id, observation.camera_forward_base)
        maximum = 0.0
        for first, second in combinations(directions.values(), 2):
            cosine = float(np.clip(np.dot(_unit(first), _unit(second)), -1.0, 1.0))
            maximum = max(maximum, float(np.degrees(np.arccos(cosine))))
        return maximum


def _unit(vector: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    length = float(np.linalg.norm(vector))
    if length <= 1e-12:
        raise ValueError("zero-length direction vector")
    return vector / length


def _feedback_at(timestamp_ns: int, before: JointFeedback,
                 after: JointFeedback) -> JointFeedback:
    """Interpolate a stationary/moving joint sample to the image timestamp."""
    if before.timestamp_ns <= timestamp_ns <= after.timestamp_ns and after.timestamp_ns > before.timestamp_ns:
        alpha = (timestamp_ns - before.timestamp_ns) / (after.timestamp_ns - before.timestamp_ns)
        positions = before.positions_rad + alpha * (after.positions_rad - before.positions_rad)
        velocities = before.velocities_rad_s + alpha * (after.velocities_rad_s - before.velocities_rad_s)
        torque = None
        if before.torque_or_current is not None and after.torque_or_current is not None:
            torque = before.torque_or_current + alpha * (after.torque_or_current - before.torque_or_current)
        return JointFeedback(timestamp_ns, positions, velocities,
                             tuple(dict.fromkeys(before.faults + after.faults)), torque)
    return min((before, after), key=lambda item: abs(item.timestamp_ns - timestamp_ns))


def _max_triangle_area(points: np.ndarray) -> float:
    maximum = 0.0
    for a, b, c in combinations(np.asarray(points, dtype=float), 3):
        maximum = max(maximum, float(np.linalg.norm(np.cross(b - a, c - a)) / 2.0))
    return maximum


def _fit_plane(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    points = np.asarray(points, dtype=float)
    origin = np.mean(points, axis=0)
    _, _, vh = np.linalg.svd(points - origin)
    normal = _unit(vh[-1])
    distances = (points - origin) @ normal
    return origin, normal, float(np.sqrt(np.mean(distances ** 2)))


def _rigid_registration(source: np.ndarray, destination: np.ndarray) -> tuple[np.ndarray, float]:
    source, destination = np.asarray(source, dtype=float), np.asarray(destination, dtype=float)
    source_center, destination_center = np.mean(source, axis=0), np.mean(destination, axis=0)
    covariance = (source - source_center).T @ (destination - destination_center)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1] *= -1.0
        rotation = vt.T @ u.T
    translation = destination_center - rotation @ source_center
    predicted = (rotation @ source.T).T + translation
    rms = float(np.sqrt(np.mean(np.sum((predicted - destination) ** 2, axis=1))))
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = rotation, translation
    return transform, rms
