#!/usr/bin/env python3
"""Report simulation readiness and the remaining live-mode blockers."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config.starter.json")
    args = parser.parse_args()
    path = Path(args.config)
    config = json.loads(path.read_text(encoding="utf-8"))
    blockers: list[str] = []

    camera = config.get("camera", {})
    keyboard = config.get("keyboard", {})
    arm = config.get("arm", {})
    survey = config.get("survey", {})
    motion = config.get("motion", {})

    if config.get("calibration_status") != "MEASURED_AND_VERIFIED":
        blockers.append("calibration_status is not MEASURED_AND_VERIFIED")
    if np.asarray(camera.get("camera_matrix", [])).shape != (3, 3):
        blockers.append("camera.camera_matrix is missing or malformed")
    if len(camera.get("distortion", [])) < 4:
        blockers.append("camera.distortion is missing")
    if set(map(int, keyboard.get("tag_centers_mm", {}).keys())) != set(range(5)):
        blockers.append("keyboard.tag_centers_mm must contain IDs 0-4")
    if len(keyboard.get("key_centers_mm", {})) < 28:
        blockers.append("keyboard.key_centers_mm is incomplete")
    if np.asarray(arm.get("T_j5_camera", [])).shape != (4, 4):
        blockers.append("arm.T_j5_camera is missing or malformed")
    if np.asarray(arm.get("T_wrist_tip", [])).shape != (4, 4):
        blockers.append("arm.T_wrist_tip is missing or malformed")
    for field in ("moteus_ids", "output_ratios", "zero_offsets_rad", "direction_signs"):
        if len(arm.get(field, [])) != 6:
            blockers.append(f"arm.{field} must contain six values")
    if np.asarray(arm.get("limits_rad", [])).shape != (6, 2):
        blockers.append("arm.limits_rad must be 6x2")
    viewpoints = survey.get("viewpoints", [])
    if not viewpoints or any(len(row.get("joint_positions_deg") or []) != 6 for row in viewpoints):
        blockers.append("every survey viewpoint needs six commissioned joint angles")
    if float(motion.get("torque_limit", 0.0)) <= 0.0:
        blockers.append("motion.torque_limit is zero")

    print(f"config: {path.resolve()}")
    print("simulation: READY")
    if blockers:
        print("live motion: LOCKED")
        for blocker in blockers:
            print(f"  - {blocker}")
    else:
        print("live configuration fields: COMPLETE")
        print("Perform the physical preflight before applying motor power.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
