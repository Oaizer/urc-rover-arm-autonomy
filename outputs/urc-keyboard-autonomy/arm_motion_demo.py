#!/usr/bin/env python3
"""Render a slow simulated six-joint arm move using the existing IK model."""

from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np


def load_arm(project: Path):
    sys.path.insert(0, str(project))
    spec = importlib.util.spec_from_file_location("existing_arm_ik", project / "arm_ik.py")
    if spec is None or spec.loader is None:
        raise ImportError("could not load arm_ik.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    geometry, limits = module.load_config(project / "arm_config.json")
    return module, geometry, limits


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project", default=None,
                        help="path to the existing i-need-to-design-an-inverse project")
    parser.add_argument("--target", nargs=3, type=float, default=[150, 70, 300])
    parser.add_argument("--frames", type=int, default=90)
    parser.add_argument("--fps", type=int, default=15)
    args = parser.parse_args()

    if args.project:
        project = Path(args.project).resolve()
    else:
        project = Path(__file__).resolve().parents[4] / "2026-09-19" / "i-need-to-design-an-inverse"
    module, geometry, limits = load_arm(project)
    q_start = np.zeros(6)
    result = module.solve_ik(args.target, geometry=geometry, seed=q_start,
                             limits_deg=limits, starts=16, check_collisions=True)
    if not result.success:
        raise SystemExit(f"target did not pass IK/collision checks: {result.message}")
    q_goal = result.q

    figure = plt.figure(figsize=(9, 7), dpi=110)
    axis = figure.add_subplot(111, projection="3d")
    axis.set_title("Simulated arm motion: zero pose -> target")
    axis.set_xlabel("X (mm)"); axis.set_ylabel("Y (mm)"); axis.set_zlabel("Z (mm)")
    axis.set_xlim(-350, 500); axis.set_ylim(-250, 450); axis.set_zlim(-350, 350)
    axis.set_box_aspect((850, 700, 700))
    target = np.asarray(args.target, dtype=float)
    axis.scatter(*target, color="red", marker="x", s=70, label="target")
    axis.legend(loc="upper left")
    line, = axis.plot([], [], [], color="royalblue", linewidth=4)
    joints, = axis.plot([], [], [], "o", color="orange", markersize=6)
    trace, = axis.plot([], [], [], "--", color="gray", linewidth=1)
    status = axis.text2D(0.02, 0.94, "", transform=axis.transAxes)
    trace_points = []

    def update(frame_index: int):
        progress = frame_index / max(args.frames - 1, 1)
        smooth = progress * progress * (3.0 - 2.0 * progress)
        q = (1.0 - smooth) * q_start + smooth * q_goal
        position, _rotation, centers, _axes, polyline = module.forward_kinematics(q, geometry)
        trace_points.append(position.copy())
        polyline = np.asarray(polyline)
        centers = np.asarray(centers)
        line.set_data(polyline[:, 0], polyline[:, 1])
        line.set_3d_properties(polyline[:, 2])
        joints.set_data(centers[:, 0], centers[:, 1])
        joints.set_3d_properties(centers[:, 2])
        trail = np.asarray(trace_points)
        trace.set_data(trail[:, 0], trail[:, 1])
        trace.set_3d_properties(trail[:, 2])
        status.set_text(f"progress {progress:5.1%}   TCP error {np.linalg.norm(position-target):.2f} mm")
        return line, joints, trace, status

    animation = FuncAnimation(figure, update, frames=args.frames, interval=1000 / args.fps,
                              blit=False, repeat=False)
    output_dir = Path(__file__).resolve().parent
    gif_path = output_dir / "arm_motion_demo.gif"
    png_path = output_dir / "arm_motion_demo_final.png"
    animation.save(gif_path, writer=PillowWriter(fps=args.fps))
    update(args.frames - 1)
    figure.savefig(png_path, bbox_inches="tight")
    plt.close(figure)
    print(f"start_q_deg={np.round(np.rad2deg(q_start), 2).tolist()}")
    print(f"goal_q_deg={np.round(np.rad2deg(q_goal), 2).tolist()}")
    print(f"target_mm={target.tolist()}")
    print(f"gif={gif_path}")
    print(f"final_frame={png_path}")


if __name__ == "__main__":
    main()
