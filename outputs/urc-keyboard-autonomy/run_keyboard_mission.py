#!/usr/bin/env python3
"""Run the safe simulation or the guarded live mission."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from hardware import MoteusCanBridge, SimulatedJoints
from ik_adapter import ExistingIK
from interfaces import MissionState
from mission import KeyboardMission, MissionConfig
from perception import ArucoTagDetector, DisplayVerifier, KeyboardLocalizer, LatestCamera
from survey import ActiveKeyboardSurvey, ActiveSurveyExecutor, SurveyConfig
from telemetry import JsonlTelemetry


def load(path: str) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


class SimulatedLocalizer:
    def observe(self, frame, timestamp_ns):
        from interfaces import KeyboardObservation
        return KeyboardObservation(timestamp_ns, np.eye(4), (1, 2, 3, 4), 0.5, (640, 480))


class SimulatedVerifier:
    def __init__(self, key: str) -> None:
        self.text = ""
        self.key = key.upper()

    def read(self, frame):
        index = min(len(self.text), len(self.key) - 1)
        self.text = self.key[:index + 1]
        return self.text


class CameraDisplayVerifier:
    """Read a fresh camera frame for every post-press verification."""

    def __init__(self, camera: LatestCamera, verifier: DisplayVerifier) -> None:
        self.camera, self.verifier = camera, verifier

    def read(self, _frame):
        frame, _ = self.camera.read()
        return self.verifier.read(frame)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config.json")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--simulate", action="store_true")
    mode.add_argument("--live", action="store_true")
    parser.add_argument("--key", default="ABC")
    args = parser.parse_args()
    cfg = load(args.config)
    telemetry = JsonlTelemetry(cfg["motion"].get("telemetry_path", "telemetry.jsonl"))
    camera = None
    if args.simulate:
        joints, localizer, verifier = SimulatedJoints(), SimulatedLocalizer(), SimulatedVerifier(args.key)
        class SimIK:
            def solve(self, position, orientation, seed): return np.zeros(6), 0.0
        ik = SimIK()
        key_centers = {character: (index * 19.05, 0.0)
                       for index, character in enumerate(args.key.upper())}
        frames = [np.zeros((480, 640, 3), dtype=np.uint8)]
        from simulate_active_survey import run_simulated_survey
        survey_result, _ = run_simulated_survey(verbose=True)
    else:
        if cfg.get("calibration_status") != "MEASURED_AND_VERIFIED":
            raise SystemExit(
                "live mode refused: calibration_status must be MEASURED_AND_VERIFIED; "
                "starter values are simulation-only")
        arm = cfg["arm"]
        viewpoint_rows = cfg.get("survey", {}).get("viewpoints", [])
        commissioned_viewpoints = [row.get("joint_positions_deg") for row in viewpoint_rows]
        required = (arm["moteus_ids"], arm["output_ratios"], arm["zero_offsets_rad"], arm["direction_signs"], arm["limits_rad"],
                    arm["T_j5_camera"], arm["T_wrist_tip"], cfg["keyboard"]["tag_centers_mm"],
                    cfg["keyboard"]["key_centers_mm"], commissioned_viewpoints,
                    cfg["camera"].get("camera_matrix"), cfg["camera"].get("distortion"))
        if (any(not value for value in required)
                or any(row is None for row in commissioned_viewpoints)
                or cfg["motion"].get("torque_limit", 0.0) <= 0):
            raise SystemExit("live mode refused: complete calibration, motors, and all survey joint poses in config.json")
        if any(len(row) != 6 for row in commissioned_viewpoints):
            raise SystemExit("live mode refused: every survey joint_positions_deg must contain six values")
        camera_cfg = cfg["camera"]
        joints = MoteusCanBridge(
            arm["moteus_ids"], cfg["can"]["serial_port"],
            arm["output_ratios"], arm["zero_offsets_rad"], arm["direction_signs"],
            arm["limits_rad"])
        camera = LatestCamera(camera_cfg["device"], camera_cfg["width"], camera_cfg["height"], camera_cfg["fps"])
        localizer = KeyboardLocalizer(np.asarray(camera_cfg["camera_matrix"]), np.asarray(camera_cfg["distortion"]),
                                      camera_cfg["dictionary"], camera_cfg["marker_size_mm"],
                                      {int(k): tuple(v) for k, v in cfg["keyboard"]["tag_centers_mm"].items()})
        verifier = CameraDisplayVerifier(camera, DisplayVerifier(cfg["keyboard"]["display_roi_px"]))
        ik = ExistingIK(cfg["ik_project"])
        T_j5_camera = np.asarray(arm["T_j5_camera"], dtype=float)
        base_from_camera = lambda feedback: ik.base_from_j5_camera(
            feedback.positions_rad, T_j5_camera,
            bool(arm.get("camera_rotates_with_j5", True)))
        detector = ArucoTagDetector(
            np.asarray(camera_cfg["camera_matrix"]), np.asarray(camera_cfg["distortion"]),
            camera_cfg["dictionary"], camera_cfg["marker_size_mm"],
            tuple(cfg["survey"]["allowed_marker_ids"]))
        survey_fields = {k: v for k, v in cfg["survey"].items()
                         if k in SurveyConfig.__dataclass_fields__}
        survey_fields["allowed_marker_ids"] = tuple(survey_fields["allowed_marker_ids"])
        estimator = ActiveKeyboardSurvey(
            {int(k): tuple(v) for k, v in cfg["keyboard"]["tag_centers_mm"].items()},
            SurveyConfig(**survey_fields))
        executor = ActiveSurveyExecutor(
            joints, camera, detector, estimator, base_from_camera,
            velocity_limit_rad_s=cfg["motion"]["approach_speed_rad_s"],
            acceleration_limit_rad_s2=cfg["motion"]["approach_speed_rad_s"] * 2.0,
            torque_limit=cfg["motion"]["torque_limit"],
            tracking_tolerance_rad=cfg["motion"]["max_tracking_error_rad"],
            feedback_timeout_ms=cfg["motion"]["feedback_timeout_ms"],
            settle_s=0.3,
            frames_per_viewpoint=cfg["motion"]["observation_stability_frames"],
            telemetry=telemetry)
        survey_poses = [(row["name"], np.radians(row["joint_positions_deg"]))
                        for row in viewpoint_rows]
        try:
            survey_result, frames = executor.run(survey_poses)
        except Exception:
            camera.close()
            raise
    if args.simulate:
        base_from_camera = None
    else:
        survey_result = None
    mission = KeyboardMission(joints, localizer, verifier, ik,
                              key_centers if args.simulate else cfg["keyboard"]["key_centers_mm"],
                              MissionConfig(**{k: v for k, v in cfg["motion"].items()
                                               if k in MissionConfig.__dataclass_fields__}),
                              base_from_camera=base_from_camera, telemetry=telemetry)
    try:
        results = mission.run(args.key, frames, survey_result=survey_result)
    finally:
        if camera is not None:
            camera.close()
    print({"state": mission.state.value, "results": [r.__dict__ for r in results]})


if __name__ == "__main__":
    main()
