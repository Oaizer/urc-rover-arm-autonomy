"""Autonomous typing state machine with stale-data and fault stops."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable

import numpy as np

from interfaces import (DisplayVerifierInterface, IKSolverInterface, JointFeedback,
                        JointInterface, KeyboardLocalizerInterface, KeyResult,
                        KeyboardSurveyResult, MissionState, MotionCommand)
from telemetry import JsonlTelemetry


@dataclass
class MissionConfig:
    clearance_mm: float = 20.0
    press_speed_mm_s: float = 2.0
    torque_limit: float = 0.0
    max_tracking_error_rad: float = 0.08
    feedback_timeout_ms: float = 100.0
    max_corrections: int = 2
    timeout_s: float = 120.0
    settle_timeout_s: float = 5.0


class KeyboardMission:
    def __init__(self, joints: JointInterface, localizer: KeyboardLocalizerInterface,
                 verifier: DisplayVerifierInterface, ik: IKSolverInterface,
                 key_centers_mm: dict[str, tuple[float, float]],
                 config: MissionConfig | None = None,
                 base_from_camera: Callable[[JointFeedback], np.ndarray] | None = None,
                 telemetry: JsonlTelemetry | None = None) -> None:
        self.joints, self.localizer, self.verifier, self.ik = joints, localizer, verifier, ik
        self.key_centers = {k.upper(): np.asarray(v, dtype=float) for k, v in key_centers_mm.items()}
        self.cfg = config or MissionConfig()
        self.base_from_camera = base_from_camera or (lambda _fb: np.eye(4))
        self.telemetry = telemetry
        self.state = MissionState.IDLE
        self.sequence = 0
        self.corrections = 0
        self.started_ns = 0

    def abort(self, reason: str) -> None:
        self.state = MissionState.ABORT
        self.joints.stop()
        if self.telemetry:
            self.telemetry.write("abort", reason=reason)
        raise RuntimeError(f"mission aborted: {reason}")

    def _safe_feedback(self) -> JointFeedback:
        fb = self.joints.feedback()
        if not fb.fresh(self.cfg.feedback_timeout_ms):
            self.abort("stale joint feedback")
        if fb.faults:
            self.abort("; ".join(fb.faults))
        if not np.all(np.isfinite(fb.positions_rad)):
            self.abort("invalid joint position")
        return fb

    def _send(self, q: np.ndarray, velocity: float = 0.25) -> None:
        self.sequence += 1
        command = MotionCommand(self.sequence, q, velocity, velocity * 2.0, self.cfg.torque_limit,
                                time.monotonic_ns() + int(self.cfg.settle_timeout_s * 1e9))
        self.joints.send(command)
        deadline = time.monotonic() + self.cfg.settle_timeout_s
        while True:
            fb = self._safe_feedback()
            error = float(np.max(np.abs(fb.positions_rad - q)))
            if self.telemetry:
                self.telemetry.write("joint_tracking", sequence=self.sequence,
                                     error_rad=error, state=self.state.value)
            if error <= self.cfg.max_tracking_error_rad:
                return
            if time.monotonic() >= deadline:
                self.abort(f"joint tracking error {error:.4f} rad")
            time.sleep(0.01)

    def run(self, launch_key: str, frames: list[np.ndarray],
            survey_result: KeyboardSurveyResult | None = None) -> list[KeyResult]:
        launch_key = launch_key.upper()
        if not 3 <= len(launch_key) <= 6 or any(ch not in self.key_centers for ch in launch_key):
            raise ValueError("launch key must contain 3-6 configured characters")
        self.state, self.started_ns = MissionState.ACQUIRE, time.monotonic_ns()
        if not frames:
            self.abort("no camera frames")
        observation = None
        if survey_result is None:
            observations = []
            for frame in frames:
                obs = self.localizer.observe(frame, time.monotonic_ns())
                if obs is not None and obs.valid:
                    observations.append(obs)
            if len(observations) < 1:
                self.abort("keyboard pose not acquired")
            observation = min(observations, key=lambda o: o.reprojection_error_px)
            if self.telemetry:
                self.telemetry.write("keyboard_observation", marker_ids=observation.marker_ids,
                                     reprojection_error_px=observation.reprojection_error_px,
                                     image_age_ms=(time.monotonic_ns() - observation.timestamp_ns) / 1e6)
            if observation.reprojection_error_px > 2.0:
                self.abort("keyboard pose reprojection error too high")
            registered_pose = None
        else:
            registered_pose = np.asarray(survey_result.T_base_keyboard, dtype=float)
            if registered_pose.shape != (4, 4) or not np.all(np.isfinite(registered_pose)):
                self.abort("invalid survey keyboard transform")
            if not survey_result.normal_faces_base:
                self.abort("survey keyboard normal does not face base")
            if self.telemetry:
                self.telemetry.write(
                    "keyboard_survey", marker_ids=survey_result.marker_ids,
                    observation_count=survey_result.observation_count,
                    viewpoint_count=survey_result.viewpoint_count,
                    viewpoint_separation_deg=survey_result.viewpoint_separation_deg,
                    plane_rms_mm=survey_result.plane_rms_mm,
                    registration_rms_mm=survey_result.registration_rms_mm,
                    position_uncertainty_mm=survey_result.position_uncertainty_mm)

        results = []
        verified_text = ""

        def press_key(character: str, seed: np.ndarray) -> np.ndarray:
            fb = self._safe_feedback()
            if registered_pose is None:
                T_base_camera = self.base_from_camera(fb)
                T_base_keyboard = T_base_camera @ observation.T_camera_keyboard
            else:
                T_base_keyboard = registered_pose
            keyboard_point = np.r_[self.key_centers[character], 0.0, 1.0]
            contact = (T_base_keyboard @ keyboard_point)[:3]
            normal = T_base_keyboard[:3, 2]
            normal /= max(np.linalg.norm(normal), 1e-9)
            approach = contact + normal * self.cfg.clearance_mm
            orientation = T_base_keyboard[:3, :3]
            q_approach, approach_error = self.ik.solve(approach, orientation, seed)
            q_contact, contact_error = self.ik.solve(contact, orientation, q_approach)
            if max(approach_error, contact_error) > 2.0:
                self.abort(f"IK error {max(approach_error, contact_error):.2f} mm")
            self.state = MissionState.APPROACH
            self._send(q_approach)
            self.state = MissionState.PRESS
            self._send(q_contact, self.cfg.press_speed_mm_s)
            self.state = MissionState.RETRACT
            self._send(q_approach)
            return q_approach

        for character in launch_key:
            if (time.monotonic_ns() - self.started_ns) / 1e9 > self.cfg.timeout_s:
                self.abort("mission timeout")
            self.state = MissionState.PLAN
            fb = self._safe_feedback()
            q_seed = press_key(character, fb.positions_rad)
            self.state = MissionState.VERIFY
            observed = self.verifier.read(frames[-1])
            expected_text = verified_text + character
            success = observed == expected_text
            if not success and self.corrections < self.cfg.max_corrections:
                self.corrections += 1
                backspace = "BACKSPACE" if "BACKSPACE" in self.key_centers else "BS"
                if backspace in self.key_centers:
                    press_key(backspace, q_seed)
                    observed = self.verifier.read(frames[-1])
                    if observed == verified_text:
                        q_seed = press_key(character, q_seed)
                        observed = self.verifier.read(frames[-1])
                        success = observed == expected_text
            if success:
                verified_text = expected_text
            results.append(KeyResult(character, observed, success, "display readback"))
            if self.telemetry:
                self.telemetry.write("key_result", expected=character, observed=observed,
                                     success=success, corrections=self.corrections)
            if not success and self.corrections >= self.cfg.max_corrections:
                self.abort("verification failed")
        self.state = MissionState.COMPLETE
        if self.telemetry:
            self.telemetry.write("complete", text=verified_text)
        return results
