#!/usr/bin/env python3
"""Exercise ArUco pose estimation with a generated camera image.

This uses no camera, tags, motors, CAN adapter, or network.
"""

from __future__ import annotations

import sys
import time

import cv2
import numpy as np

from perception import KeyboardLocalizer


def make_frame() -> np.ndarray:
    frame = np.full((480, 640, 3), 255, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for marker_id, (x, y) in zip((0, 1, 2, 3), ((80, 120), (480, 120), (480, 360), (80, 360))):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 80)
        frame[y-40:y+40, x-40:x+40] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    return frame


def main() -> int:
    camera_matrix = np.array([[800.0, 0, 320.0], [0, 800.0, 240.0], [0, 0, 1.0]])
    localizer = KeyboardLocalizer(
        camera_matrix, np.zeros(5), "DICT_4X4_50", 20.0,
        {0: (0, 0), 1: (100, 0), 2: (100, 60), 3: (0, 60)},
    )
    frame = make_frame()
    observation = localizer.observe(frame, time.monotonic_ns())
    if observation is None or len(observation.marker_ids) != 4:
        print("FAIL: fewer than four expected markers detected")
        return 1
    print({
        "marker_ids": observation.marker_ids,
        "reprojection_error_px": round(observation.reprojection_error_px, 3),
        "translation_mm": np.round(observation.T_camera_keyboard[:3, 3], 2).tolist(),
    })
    if observation.reprojection_error_px >= 2.0:
        print("FAIL: synthetic pose error is above the 2 px acceptance threshold")
        return 1
    for marker_id, center in zip(observation.marker_ids, ((480, 360), (80, 360), (480, 120), (80, 120))):
        x, y = center
        cv2.rectangle(frame, (x - 40, y - 40), (x + 40, y + 40), (0, 180, 0), 3)
        cv2.putText(frame, f"id {marker_id}", (x - 35, y - 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 120, 0), 2)
    cv2.rectangle(frame, (80, 120), (480, 360), (255, 0, 0), 2)
    cv2.putText(frame, f"keyboard pose | error {observation.reprojection_error_px:.2f}px",
                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 0, 0), 2)
    output = "offline_test_output.png"
    cv2.imwrite(output, frame)
    print(f"visual output: {output}")
    print("offline ArUco perception test: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
