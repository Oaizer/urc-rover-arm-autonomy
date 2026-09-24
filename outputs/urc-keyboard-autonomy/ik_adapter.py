"""Adapter for the existing Y-Z-Z-X-Z-X IK project."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np


class ExistingIK:
    def __init__(self, project: str | Path) -> None:
        root = Path(project).resolve()
        if not (root / "arm_ik.py").exists():
            raise FileNotFoundError(f"arm_ik.py not found in {root}")
        sys.path.insert(0, str(root))
        spec = importlib.util.spec_from_file_location("urc_existing_arm_ik", root / "arm_ik.py")
        if spec is None or spec.loader is None:
            raise ImportError("cannot load existing IK")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.module = module
        self.geometry, self.limits = module.load_config(root / "arm_config.json")

    def solve(self, position_mm, orientation, seed):
        result = self.module.solve_ik(position_mm, orientation=orientation,
                                      geometry=self.geometry, seed=np.asarray(seed),
                                      limits_deg=self.limits, starts=8)
        if not result.success:
            raise RuntimeError(result.message)
        return result.q, float(result.position_error_mm)

    def base_from_joint_frame(self, q, T_wrist_camera):
        """Return base-to-camera using the calibrated wrist-camera transform.

        The existing model's TCP frame is the calibrated wrist frame for this
        first integration slice. Once the physical wrist datum is measured,
        insert that fixed transform here without changing mission logic.
        """
        position, rotation, *_ = self.module.forward_kinematics(q, self.geometry)
        T_base_wrist = np.eye(4)
        T_base_wrist[:3, :3] = rotation
        T_base_wrist[:3, 3] = position
        return T_base_wrist @ np.asarray(T_wrist_camera, dtype=float)

    def base_from_j5_camera(self, q, T_j5_camera, rotates_with_j5: bool = True):
        """Return base-to-camera for a camera physically mounted at J5.

        ``rotates_with_j5`` is true when the mount is attached to the J5 output.
        Set it false only when the camera is fixed to the upstream J5 housing.
        The translation origin is the J5 joint center in both cases.
        """
        q = np.asarray(q, dtype=float)
        if q.shape != (6,):
            raise ValueError("q must contain six joint angles")
        T_j5_camera = np.asarray(T_j5_camera, dtype=float)
        if T_j5_camera.shape != (4, 4) or not np.all(np.isfinite(T_j5_camera)):
            raise ValueError("T_j5_camera must be a finite 4x4 transform")
        _, _, centers, _, _ = self.module.forward_kinematics(q, self.geometry)
        frames = self.module.link_frames(q)
        T_base_j5 = np.eye(4)
        T_base_j5[:3, :3] = frames[4] if rotates_with_j5 else frames[3]
        T_base_j5[:3, 3] = centers[4]
        return T_base_j5 @ T_j5_camera
