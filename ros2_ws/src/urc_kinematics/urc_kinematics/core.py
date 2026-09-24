"""YZZXZX geometry adapted from the original arm_ik.py; no ROS imports.

At each joint: rotate about its local axis, then translate by its local offset.
The explicit fixed tool transform is applied AFTER all six link offsets.
Base is legacy Y-up: for coincident ROS X-forward/Y-left/Z-up axes,
p_ros = [p_legacy.x, -p_legacy.z, p_legacy.y], R_ros = Rx(pi/2) @ R_legacy.
This module intentionally retains the legacy frame, never silently relabels it.
"""

from dataclasses import dataclass
import json
from pathlib import Path
import warnings

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


AXES = np.array([[0, 1, 0], [0, 0, 1], [0, 0, 1],
                 [1, 0, 0], [0, 0, 1], [1, 0, 0]], dtype=float)


def vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must contain {size} finite numbers')
    return result.copy()


def quaternion(value):
    q = vector(value, 4, 'quaternion_xyzw')
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or abs(norm - 1.0) > 1e-6:
        raise ValueError('quaternion_xyzw must be unit length (within 1e-6)')
    return q / norm


@dataclass(frozen=True)
class Geometry:
    d1: float
    L2: float
    L3x: float
    L3y: float
    L4: float
    L5: float
    L6: float

    def __post_init__(self):
        lengths = vector(list(vars(self).values()), 7, 'geometry_m')
        if np.any(lengths < 0) or lengths.sum() <= 0:
            raise ValueError('geometry_m must be nonnegative and not all zero')

    @property
    def offsets(self):
        return np.array([[0, self.d1, 0], [self.L2, 0, 0],
                         [self.L3x, self.L3y, 0], [self.L4, 0, 0],
                         [self.L5, 0, 0], [self.L6, 0, 0]])


@dataclass(frozen=True)
class Pose:
    position: np.ndarray
    quaternion_xyzw: np.ndarray

    def __post_init__(self):
        object.__setattr__(self, 'position', vector(self.position, 3, 'position_m'))
        object.__setattr__(self, 'quaternion_xyzw', quaternion(self.quaternion_xyzw))


@dataclass(frozen=True)
class IKResult:
    success: bool
    message: str
    positions: np.ndarray
    position_error_m: float
    orientation_error_rad: float
    orientation_relaxed: bool = False


class Kinematics:
    def __init__(self, geometry, limits_rad, tool, *, position_tolerance_m=1e-4,
                 orientation_tolerance_rad=1e-3, starts=16, max_nfev=300):
        if not isinstance(geometry, Geometry) or not isinstance(tool, Pose):
            raise ValueError('geometry and tool must be Geometry and Pose')
        limits = np.asarray(limits_rad, dtype=float)
        if (limits.shape != (6, 2) or not np.all(np.isfinite(limits))
                or np.any(limits[:, 0] >= limits[:, 1])):
            raise ValueError('limits_rad must be six finite [lower, upper] pairs, lower < upper')
        for name, value in [('position_tolerance_m', position_tolerance_m),
                            ('orientation_tolerance_rad', orientation_tolerance_rad)]:
            if not np.isscalar(value) or not np.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
        for name, value in [('starts', starts), ('max_nfev', max_nfev)]:
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f'{name} must be a positive integer')
        self.geometry = geometry
        self.lower, self.upper = limits.T.copy()
        self.tool_p = tool.position.copy()
        self.tool_r = Rotation.from_quat(tool.quaternion_xyzw).as_matrix()
        self.position_tolerance_m = position_tolerance_m
        self.orientation_tolerance_rad = orientation_tolerance_rad
        self.starts, self.max_nfev = starts, max_nfev

    def validate_positions(self, positions):
        q = vector(positions, 6, 'positions/seed_rad')
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise ValueError('positions/seed_rad violate configured joint limits')
        return q

    def _chain(self, q):
        p, r = np.zeros(3), np.eye(3)
        for angle, axis, offset in zip(q, AXES, self.geometry.offsets):
            r = r @ Rotation.from_rotvec(axis * angle).as_matrix()
            p = p + r @ offset
        return p, r

    def _tcp(self, q):
        p, r = self._chain(q)
        return p + r @ self.tool_p, r @ self.tool_r

    def forward(self, positions):
        p, r = self._tcp(self.validate_positions(positions))
        return Pose(p, Rotation.from_matrix(r).as_quat())

    def joint5_output(self, positions):
        """J5 center with orientation AFTER J5 rotation, BEFORE L5 offset.

        J6 and the tool transform do not affect this frame. A future camera
        needs a separately measured fixed transform from this frame.
        """
        q = self.validate_positions(positions)
        p, r = np.zeros(3), np.eye(3)
        for index, (angle, axis, offset) in enumerate(zip(q, AXES, self.geometry.offsets)):
            r = r @ Rotation.from_rotvec(axis * angle).as_matrix()
            if index == 4:
                return Pose(p, Rotation.from_matrix(r).as_quat())
            p = p + r @ offset

    def _geometric_seeds(self, target_p, target_r, seed):
        # Remove the arbitrary fixed tool transform before using original branches.
        chain_r = target_r @ self.tool_r.T
        chain_p = target_p - chain_r @ self.tool_p
        g = self.geometry
        point = chain_p - (g.L5 + g.L6) * chain_r[:, 0]
        radial = float(np.hypot(point[0], point[2]))
        yaw = -np.arctan2(point[2], point[0]) if radial > 1e-12 else seed[0]
        length = float(np.hypot(g.L3x + g.L4, g.L3y))
        if g.L2 * length < 1e-15:
            return []
        beta = np.arctan2(g.L3y, g.L3x + g.L4)
        height = point[1] - g.d1
        cosine = (radial**2 + height**2 - g.L2**2 - length**2) / (2*g.L2*length)
        if abs(cosine) > 1 + 1e-10:
            return []
        elbow_angle = np.arccos(np.clip(cosine, -1, 1))
        candidates = []
        for base, radius in [(yaw, radial), (yaw + np.pi, -radial)]:
            for elbow in [elbow_angle, -elbow_angle]:
                shoulder = np.arctan2(height, radius) - np.arctan2(
                    length*np.sin(elbow), g.L2 + length*np.cos(elbow))
                q3 = elbow - beta
                pre = (Rotation.from_rotvec([0, base, 0]).as_matrix()
                       @ Rotation.from_rotvec([0, 0, shoulder + q3]).as_matrix())
                with warnings.catch_warnings():
                    warnings.filterwarnings('ignore', message='Gimbal lock detected.*')
                    a, b, c = Rotation.from_matrix(pre.T @ chain_r).as_euler('XZX')
                for wrist in [(a, b, c), (a + np.pi, -b, c + np.pi)]:
                    q = np.array([base, shoulder, q3, *wrist])
                    k_min = np.ceil((self.lower - q - 1e-10)/(2*np.pi))
                    k_max = np.floor((self.upper - q + 1e-10)/(2*np.pi))
                    if np.any(k_min > k_max):
                        continue
                    turns = np.clip(np.round((seed - q)/(2*np.pi)), k_min, k_max)
                    q = np.clip(q + turns*2*np.pi, self.lower, self.upper)
                    if not any(np.linalg.norm(q - other) < 1e-8 for other in candidates):
                        candidates.append(q)
        return sorted(candidates, key=lambda q: float(np.linalg.norm(q - seed)))

    def solve(self, target, seed, *, allow_orientation_fallback=True, max_joint_delta_rad=None):
        """Prefer the requested full pose; optionally relax ONLY orientation.

        Position tolerance, joint limits and fixed tool geometry never change.
        Failure of a finite full-pose search is not proof of unreachability.
        """
        seed = self.validate_positions(seed)
        target = Pose(target.position, target.quaternion_xyzw)
        if max_joint_delta_rad is not None:
            if (not np.isscalar(max_joint_delta_rad) or not np.isfinite(max_joint_delta_rad)
                    or max_joint_delta_rad <= 0):
                raise ValueError('max_joint_delta_rad must be positive and finite when set')
            lower = np.maximum(self.lower, seed - max_joint_delta_rad)
            upper = np.minimum(self.upper, seed + max_joint_delta_rad)
        else:
            lower, upper = self.lower, self.upper

        strict = self._solve_full_pose(target, seed)
        strict_near = strict.success and np.all(strict.positions >= lower) and np.all(strict.positions <= upper)
        if strict_near:
            return strict
        if not allow_orientation_fallback:
            if strict.success:
                return IKResult(False, 'Requested pose needs a joint jump beyond the configured limit.',
                                strict.positions, strict.position_error_m, strict.orientation_error_rad)
            return strict
        bound = np.linalg.norm(self.geometry.offsets, axis=1).sum() + np.linalg.norm(self.tool_p)
        if np.linalg.norm(target.position) > bound + self.position_tolerance_m:
            return strict
        target_r = Rotation.from_quat(target.quaternion_xyzw).as_matrix()

        def evaluate(q):
            q = self.validate_positions(q)
            p, r = self._tcp(q)
            pe = float(np.linalg.norm(p - target.position))
            ae = float(Rotation.from_matrix(target_r @ r.T).magnitude())
            inside = np.all(q >= lower - 1e-10) and np.all(q <= upper + 1e-10)
            return IKResult(inside and pe <= self.position_tolerance_m,
                            'Position reached; wrist orientation relaxed after full-pose search failed; collision unchecked.',
                            q, pe, ae, True)

        def residual(q):
            return (self._tcp(q)[0] - target.position) / self.position_tolerance_m

        # Start near measured joints, then reuse the rejected full-pose candidate.
        # Orientation is completely unconstrained, not weakly penalized.
        rng = np.random.default_rng(17)
        for index in range(self.starts):
            guess = seed if index == 0 else np.clip(strict.positions, lower, upper) if index == 1 else rng.uniform(lower, upper)
            candidate = evaluate(guess)
            if candidate.success:
                return candidate
            fit = least_squares(residual, guess, bounds=(lower, upper),
                                max_nfev=self.max_nfev, ftol=1e-10, xtol=1e-10, gtol=1e-10)
            candidate = evaluate(fit.x)
            if candidate.success:
                return candidate
        return IKResult(False, 'Neither requested pose nor position-only fallback found a verified solution.',
                        strict.positions, strict.position_error_m, strict.orientation_error_rad)

    def _solve_full_pose(self, target, seed):
        """Return a verified full-pose result. Invalid inputs raise ValueError.

        The seed must be within limits; it is tried first. Other branches are
        ordered near it, without a global nearest-solution guarantee. Failed
        results contain a rejected candidate, never a command to move.
        """
        seed = self.validate_positions(seed)
        target = Pose(target.position, target.quaternion_xyzw)
        target_r = Rotation.from_quat(target.quaternion_xyzw).as_matrix()

        def errors(q):
            p, r = self._tcp(q)
            return p - target.position, Rotation.from_matrix(target_r @ r.T).as_rotvec()

        def residual(q):
            dp, dr = errors(q)
            return np.r_[dp/self.position_tolerance_m, dr/self.orientation_tolerance_rad]

        def result(q):
            q = self.validate_positions(q)
            dp, dr = errors(q)
            pe, ae = float(np.linalg.norm(dp)), float(np.linalg.norm(dr))
            success = pe <= self.position_tolerance_m and ae <= self.orientation_tolerance_rad
            return IKResult(success, 'Pose reached within tolerances; collision unchecked.' if success
                            else 'No verified pose found; returned candidate is rejected.', q, pe, ae)

        best = result(seed)
        if best.success:
            return best
        bound = np.linalg.norm(self.geometry.offsets, axis=1).sum() + np.linalg.norm(self.tool_p)
        if np.linalg.norm(target.position) > bound + self.position_tolerance_m:
            return IKResult(False, 'Target exceeds maximum reach bound; seed returned.',
                            seed, best.position_error_m, best.orientation_error_rad)
        guesses = [seed] + self._geometric_seeds(target.position, target_r, seed)
        rng = np.random.default_rng(7)
        best_score = np.linalg.norm(residual(seed))
        for index in range(self.starts):
            guess = guesses[index] if index < len(guesses) else rng.uniform(self.lower, self.upper)
            exact = result(guess)
            if exact.success:
                return exact
            fit = least_squares(residual, guess, bounds=(self.lower, self.upper),
                                max_nfev=self.max_nfev, ftol=1e-10, xtol=1e-10, gtol=1e-10)
            candidate = result(fit.x)
            if candidate.success:
                return candidate
            score = np.linalg.norm(residual(candidate.positions))
            if score < best_score:
                best, best_score = candidate, score
        return best


def load_config(path):
    """Load an explicit SI JSON model; no implicit geometry or tool defaults."""
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    required = {'commissioned', 'base_frame', 'tcp_frame', 'geometry_m', 'limits_rad',
                'tool_transform', 'solver'}
    if not isinstance(data, dict) or set(data) != required:
        raise ValueError(f'Config must contain exactly {sorted(required)}')
    if not isinstance(data['commissioned'], bool):
        raise ValueError('commissioned must be boolean')
    for name in ['base_frame', 'tcp_frame']:
        if not isinstance(data[name], str) or not data[name].strip():
            raise ValueError(f'{name} must be a nonempty frame name')
    if not data['base_frame'].endswith('_legacy'):
        raise ValueError('base_frame must end in _legacy: model uses legacy Y-up axes')
    if data['base_frame'] == data['tcp_frame']:
        raise ValueError('base_frame and tcp_frame must differ')
    tool = data['tool_transform']
    if not isinstance(tool, dict) or set(tool) != {'translation_m', 'quaternion_xyzw'}:
        raise ValueError('tool_transform requires translation_m and quaternion_xyzw')
    solver = data['solver']
    if not isinstance(solver, dict) or set(solver) != {
            'position_tolerance_m', 'orientation_tolerance_rad', 'starts', 'max_nfev'}:
        raise ValueError('solver requires explicit tolerances, starts and max_nfev')
    try:
        model = Kinematics(Geometry(**data['geometry_m']), data['limits_rad'],
                           Pose(tool['translation_m'], tool['quaternion_xyzw']), **solver)
    except (TypeError, OverflowError) as exc:
        raise ValueError(f'Invalid model configuration: {exc}') from exc
    return model, data
