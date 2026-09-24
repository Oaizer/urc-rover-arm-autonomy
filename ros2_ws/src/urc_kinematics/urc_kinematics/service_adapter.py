"""ROS-message duck-typed callbacks, testable without a ROS installation."""

import numpy as np

from .core import Pose


def write_pose(message, position, quaternion):
    message.position.x, message.position.y, message.position.z = map(float, position)
    (message.orientation.x, message.orientation.y,
     message.orientation.z, message.orientation.w) = map(float, quaternion)


def ordered_joint_positions(names, positions, expected_names):
    """Require one complete sample; ignore unrelated joints, never stale-fill."""
    if len(names) != len(positions) or len(set(names)) != len(names):
        raise ValueError('JointState names/positions mismatch or duplicate names')
    lookup = dict(zip(names, positions))
    if any(name not in lookup for name in expected_names):
        raise ValueError('JointState is missing required arm joints')
    return [lookup[name] for name in expected_names]


class ServiceAdapter:
    def __init__(self, model, report_error):
        self.model = model
        self.report_error = report_error

    def solve_ik(self, request, response):
        # Invalid requests have no meaningful candidate or residual.
        response.success = False
        response.positions = [float('nan')] * 6
        response.position_error_m = float('inf')
        response.orientation_error_rad = float('inf')
        response.orientation_relaxed = False
        try:
            p, q = request.target.position, request.target.orientation
            result = self.model.solve(Pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w]), request.seed,
                                      allow_orientation_fallback=not getattr(request, 'require_orientation', False),
                                      max_joint_delta_rad=(getattr(request, 'max_joint_delta_rad', 0.0) or None))
            response.success = result.success
            response.message = result.message
            response.positions = result.positions.tolist()
            response.position_error_m = result.position_error_m
            response.orientation_error_rad = result.orientation_error_rad
            response.orientation_relaxed = result.orientation_relaxed
            if hasattr(response, 'achieved_pose'):
                actual = self.model.forward(result.positions)
                write_pose(response.achieved_pose, actual.position, actual.quaternion_xyzw)
        except (ValueError, TypeError, OverflowError, FloatingPointError, np.linalg.LinAlgError) as exc:
            response.message = f'IK rejected: {exc}'
            self.report_error(response.message)
        return response

    def compute_fk(self, request, response):
        try:
            pose = self.model.forward(request.positions)
            write_pose(response.pose, pose.position, pose.quaternion_xyzw)
        except (ValueError, TypeError, OverflowError, FloatingPointError, np.linalg.LinAlgError) as exc:
            # ComputeFK's fixed contract has no success/message fields. Explicit
            # nonfinite sentinel avoids returning a plausible default zero pose.
            write_pose(response.pose, [float('nan')]*3, [float('nan')]*4)
            self.report_error(f'FK rejected; returning NaN pose: {exc}')
        return response
