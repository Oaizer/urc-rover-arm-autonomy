"""Typed boundaries between perception, IK, hardware, and mission logic."""

from dataclasses import dataclass, field
from enum import Enum
from typing import Mapping, Protocol, Sequence
import time

import numpy as np


class MissionState(str, Enum):
    IDLE = "IDLE"
    ACQUIRE = "ACQUIRE"
    SURVEY_PLAN = "SURVEY_PLAN"
    MOVE_TO_VIEWPOINT = "MOVE_TO_VIEWPOINT"
    SETTLE = "SETTLE"
    DETECT_TAGS = "DETECT_TAGS"
    FIT_PLANE = "FIT_PLANE"
    REGISTER_KEYBOARD = "REGISTER_KEYBOARD"
    PLAN = "PLAN"
    APPROACH = "APPROACH"
    PRESS = "PRESS"
    RETRACT = "RETRACT"
    VERIFY = "VERIFY"
    COMPLETE = "COMPLETE"
    ABORT = "ABORT"
    FAULT = "FAULT"


@dataclass(frozen=True)
class JointFeedback:
    timestamp_ns: int
    positions_rad: np.ndarray
    velocities_rad_s: np.ndarray
    faults: tuple[str, ...] = ()
    torque_or_current: np.ndarray | None = None

    def fresh(self, timeout_ms: float) -> bool:
        return time.monotonic_ns() - self.timestamp_ns <= int(timeout_ms * 1e6)


@dataclass(frozen=True)
class MotionCommand:
    sequence: int
    positions_rad: np.ndarray
    velocity_limit_rad_s: float
    acceleration_limit_rad_s2: float
    torque_limit: float
    expires_ns: int


@dataclass(frozen=True)
class KeyboardObservation:
    timestamp_ns: int
    T_camera_keyboard: np.ndarray
    marker_ids: tuple[int, ...]
    reprojection_error_px: float
    image_size: tuple[int, int]

    @property
    def valid(self) -> bool:
        return bool(self.marker_ids) and np.isfinite(self.reprojection_error_px)


@dataclass(frozen=True)
class TagDetection:
    """One marker pose measured in the camera optical frame."""

    timestamp_ns: int
    marker_id: int
    T_camera_tag: np.ndarray
    reprojection_error_px: float
    pixel_area: float

    @property
    def valid(self) -> bool:
        return (self.T_camera_tag.shape == (4, 4)
                and np.all(np.isfinite(self.T_camera_tag))
                and np.isfinite(self.reprojection_error_px)
                and self.pixel_area > 0.0
                and self.T_camera_tag[2, 3] > 0.0)


@dataclass(frozen=True)
class TagObservation:
    """A marker observation transformed into the arm-base frame."""

    marker_id: int
    image_timestamp_ns: int
    joint_timestamp_ns: int
    viewpoint_id: str
    position_base_mm: np.ndarray
    normal_base: np.ndarray
    camera_position_base_mm: np.ndarray
    camera_forward_base: np.ndarray
    reprojection_error_px: float
    pixel_area: float


@dataclass(frozen=True)
class KeyboardSurveyResult:
    """Confidence-gated keyboard registration produced by active surveying."""

    T_base_keyboard: np.ndarray
    marker_ids: tuple[int, ...]
    observation_count: int
    viewpoint_count: int
    viewpoint_separation_deg: float
    plane_rms_mm: float
    registration_rms_mm: float
    position_uncertainty_mm: float
    normal_faces_base: bool


@dataclass(frozen=True)
class KeyResult:
    expected: str
    observed: str | None
    success: bool
    reason: str = ""


class JointInterface(Protocol):
    def feedback(self) -> JointFeedback: ...
    def send(self, command: MotionCommand) -> None: ...
    def stop(self) -> None: ...


class KeyboardLocalizerInterface(Protocol):
    def observe(self, frame: np.ndarray, timestamp_ns: int) -> KeyboardObservation | None: ...


class TagDetectorInterface(Protocol):
    def detect(self, frame: np.ndarray, timestamp_ns: int) -> list[TagDetection]: ...


class DisplayVerifierInterface(Protocol):
    def read(self, frame: np.ndarray) -> str | None: ...


class IKSolverInterface(Protocol):
    def solve(self, position_mm: Sequence[float], orientation: np.ndarray | None,
              seed: np.ndarray) -> tuple[np.ndarray, float]: ...
