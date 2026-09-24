#!/usr/bin/env python3
"""Offline perspective stress test for the keyboard ArUco localizer."""

from __future__ import annotations

import math
import json
import time
from pathlib import Path

import cv2
import numpy as np

from perception import KeyboardLocalizer


IMAGE_SIZE = (640, 480)
K = np.array([[800.0, 0.0, 320.0], [0.0, 800.0, 240.0], [0.0, 0.0, 1.0]])
TAG_SIZE_MM = 20.0
TAG_CENTERS = {0: (0.0, 0.0), 1: (100.0, 0.0), 2: (100.0, 60.0), 3: (0.0, 60.0)}


def project(points: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    camera_points = (R @ points.T + t.reshape(3, 1)).T
    image_points, _ = cv2.projectPoints(camera_points, np.zeros(3), np.zeros(3), K, None)
    return image_points.reshape(-1, 2)


def render_scene(rng: np.random.Generator, angles: tuple[float, float, float],
                 distance: float, blur: int, noise: float, dark: float,
                 occlude: bool) -> tuple[np.ndarray, np.ndarray, bool]:
    rx, ry, rz = np.deg2rad(angles)
    R, _ = cv2.Rodrigues(np.array([rx, ry, rz], dtype=float))
    t = np.array([rng.uniform(-25, 25), rng.uniform(-18, 18), distance], dtype=float)
    frame = np.full((IMAGE_SIZE[1], IMAGE_SIZE[0], 3), 225, np.uint8)

    board = np.array([[0, 0, 0], [100, 0, 0], [100, 60, 0], [0, 60, 0]], dtype=float)
    board_px = np.round(project(board, R, t)).astype(np.int32)
    visible = bool(np.all(board_px[:, 0] >= 12) and np.all(board_px[:, 0] < IMAGE_SIZE[0] - 12)
                  and np.all(board_px[:, 1] >= 12) and np.all(board_px[:, 1] < IMAGE_SIZE[1] - 12))
    cv2.fillConvexPoly(frame, board_px, (190, 190, 190))
    cv2.polylines(frame, [board_px], True, (80, 80, 80), 2)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    half = TAG_SIZE_MM / 2.0
    outer_half = half * (140.0 / 120.0)
    for marker_id, (cx, cy) in TAG_CENTERS.items():
        marker = np.full((140, 140), 255, np.uint8)
        marker[10:130, 10:130] = cv2.aruco.generateImageMarker(dictionary, marker_id, 120)
        marker = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        object_corners = np.array([[cx-outer_half, cy-outer_half, 0], [cx+outer_half, cy-outer_half, 0],
                                   [cx+outer_half, cy+outer_half, 0], [cx-outer_half, cy+outer_half, 0]], dtype=float)
        image_corners = project(object_corners, R, t).astype(np.float32)
        source = np.array([[0, 0], [139, 0], [139, 139], [0, 139]], dtype=np.float32)
        H = cv2.getPerspectiveTransform(source, image_corners)
        cv2.warpPerspective(marker, H, IMAGE_SIZE, frame, borderMode=cv2.BORDER_TRANSPARENT)

    # Simulate exposure variation, sensor noise, and modest motion blur.
    frame = np.clip(frame.astype(np.float32) * dark, 0, 255).astype(np.uint8)
    if noise:
        frame = np.clip(frame.astype(np.float32) + rng.normal(0, noise, frame.shape), 0, 255).astype(np.uint8)
    if blur > 1:
        frame = cv2.GaussianBlur(frame, (blur, blur), 0)
    if occlude:
        # Occlude board interior while leaving the four corner tags visible.
        cv2.rectangle(frame, (250, 205), (390, 275), (110, 110, 110), -1)
    return frame, np.vstack((R, t.reshape(1, 3))), visible


def annotate(frame: np.ndarray, observation) -> np.ndarray:
    out = frame.copy()
    for marker_id in observation.marker_ids:
        # Marker location is visually represented by the detected tag polygon.
        pass
    cv2.putText(out, f"tags={len(observation.marker_ids)} error={observation.reprojection_error_px:.2f}px",
                (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    return out


def main() -> int:
    rng = np.random.default_rng(21)
    localizer = KeyboardLocalizer(K, np.zeros(5), "DICT_4X4_50", TAG_SIZE_MM, TAG_CENTERS)
    cases = []
    attempts = 0
    while len(cases) < 60 and attempts < 1000:
        attempts += 1
        index = len(cases)
        angles = (rng.uniform(-40, 40), rng.uniform(-40, 40), rng.uniform(-25, 25))
        distance = rng.uniform(220, 420)
        blur = int(rng.choice([1, 1, 1, 3, 5]))
        noise = float(rng.choice([0, 0, 2, 5, 8]))
        dark = float(rng.uniform(0.55, 1.15))
        occlude = bool(index % 7 == 0)
        frame, pose, visible = render_scene(rng, angles, distance, blur, noise, dark, occlude)
        if not visible:
            continue
        observation = localizer.observe(frame, time.monotonic_ns())
        passed = observation is not None and len(observation.marker_ids) == 4
        error_px = None if observation is None else observation.reprojection_error_px
        if passed and error_px > 2.0:
            passed = False
        cases.append((passed, error_px, angles, frame, observation))

    passed = sum(item[0] for item in cases)
    errors = [item[1] for item in cases if item[1] is not None]
    output = Path("stress_test_montage.png")
    selected = sorted(cases, key=lambda item: (item[0], -(item[1] or 999)))[:12]
    tiles = []
    for ok, error, angles, frame, observation in selected:
        tile = cv2.resize(frame, (320, 240))
        status = "PASS" if ok else "FAIL"
        cv2.putText(tile, f"{status} angles={tuple(round(x) for x in angles)}",
                    (8, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                    (0, 130, 0) if ok else (0, 0, 220), 1)
        tiles.append(tile)
    rows = [np.hstack(tiles[i:i+4]) for i in range(0, len(tiles), 4)]
    cv2.imwrite(str(output), np.vstack(rows))
    detected = sum(item[1] is not None for item in cases)
    report = {"passed": passed, "detected": detected, "total": len(cases),
           "pass_rate": round(passed / len(cases), 3),
           "detected_rate": round(detected / len(cases), 3),
           "max_detected_reprojection_error_px": None if not errors else round(max(errors), 3),
           "mean_detected_reprojection_error_px": None if not errors else round(float(np.mean(errors)), 3),
           "montage": str(output)}
    print(report)
    Path("stress_test_results.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
