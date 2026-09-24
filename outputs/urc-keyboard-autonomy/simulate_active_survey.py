#!/usr/bin/env python3
"""Run the poor-visibility multi-view survey without physical hardware."""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from interfaces import JointFeedback, TagDetection
from survey import ActiveKeyboardSurvey, SurveyConfig, default_front_viewpoints


TAG_LAYOUT_MM = {
    0: (-155.0, -50.0), 1: (155.0, -50.0),
    2: (155.0, 50.0), 3: (-155.0, 50.0), 4: (0.0, 0.0),
}

# Poor visibility: no single view contains enough corner tags by itself.
VISIBLE = {
    "center": (4,), "left": (0, 3), "right": (1, 2),
    "up": (0, 1, 4), "down": (2, 3, 4),
    "left_up": (0, 4), "right_up": (1, 4),
    "left_down": (3, 4), "right_down": (2, 4),
}


def T(rotation: np.ndarray | None = None, translation=(0.0, 0.0, 0.0)) -> np.ndarray:
    result = np.eye(4)
    if rotation is not None:
        result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def camera_pose(yaw_deg: float, pitch_deg: float) -> np.ndarray:
    # Optical +Z points approximately from the arm base toward the keyboard.
    base = np.array([[0.0, 0.0, 1.0],
                     [1.0, 0.0, 0.0],
                     [0.0, 1.0, 0.0]])
    rotation = Rotation.from_euler("ZY", [yaw_deg, -pitch_deg], degrees=True).as_matrix() @ base
    return T(rotation, (40.0, -20.0, 80.0))


def detections(T_base_camera: np.ndarray, T_base_keyboard: np.ndarray,
               marker_ids: tuple[int, ...], timestamp_ns: int,
               rng: np.random.Generator) -> list[TagDetection]:
    output = []
    for marker_id in marker_ids:
        x, y = TAG_LAYOUT_MM[marker_id]
        T_base_tag = T_base_keyboard @ T(translation=(x, y, 0.0))
        T_camera_tag = np.linalg.inv(T_base_camera) @ T_base_tag
        # Simulate sub-millimetre pose noise after calibrated marker detection.
        T_camera_tag[:3, 3] += rng.normal(0.0, 0.35, 3)
        error = float(rng.uniform(0.25, 0.75))
        output.append(TagDetection(timestamp_ns, marker_id, T_camera_tag,
                                   error, float(rng.uniform(650.0, 1100.0))))
    return output


def run_simulated_survey(verbose: bool = True):
    rng = np.random.default_rng(2026)
    R_base_keyboard = np.array([[0.0, 0.0, -1.0],
                                [1.0, 0.0, 0.0],
                                [0.0, -1.0, 0.0]])
    truth = T(R_base_keyboard, (610.0, 15.0, 110.0))
    config = SurveyConfig(require_normal_faces_base=True)
    survey = ActiveKeyboardSurvey(TAG_LAYOUT_MM, config)

    if verbose:
        print("phase 1: active keyboard survey")
    result = None
    for viewpoint in default_front_viewpoints():
        camera = camera_pose(viewpoint.yaw_deg, viewpoint.pitch_deg)
        timestamp = time.monotonic_ns()
        seen = VISIBLE[viewpoint.name]
        frame_detections = detections(camera, truth, seen, timestamp, rng)
        # The simulated encoders and image are synchronized exactly.
        feedback = JointFeedback(timestamp, np.zeros(6), np.zeros(6))
        accepted = survey.add_frame(frame_detections, timestamp, feedback,
                                    camera, viewpoint.name)
        ready, reason = survey.status()
        if verbose:
            print(f"  {viewpoint.name:12s} saw {seen!s:14s} accepted={accepted} | {reason}")
        if ready:
            result = survey.build_result()
            break

    if result is None:
        raise RuntimeError("survey exhausted all viewpoints")
    return result, truth


def main() -> int:
    result, truth = run_simulated_survey(verbose=True)
    translation_error = float(np.linalg.norm(
        result.T_base_keyboard[:3, 3] - truth[:3, 3]))
    payload = {
        "marker_ids": result.marker_ids,
        "observation_count": result.observation_count,
        "viewpoint_count": result.viewpoint_count,
        "viewpoint_separation_deg": result.viewpoint_separation_deg,
        "plane_rms_mm": result.plane_rms_mm,
        "registration_rms_mm": result.registration_rms_mm,
        "position_uncertainty_mm": result.position_uncertainty_mm,
        "translation_error_mm": translation_error,
        "T_base_keyboard": result.T_base_keyboard.tolist(),
    }
    Path("active_survey_result.json").write_text(
        json.dumps(payload, indent=2), encoding="utf-8")
    print("phase 1 complete: keyboard registered")
    print(json.dumps(payload, indent=2))
    print("phase 2 handoff: T_base_keyboard is ready for IK typing")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
