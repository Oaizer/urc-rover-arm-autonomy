"""Metric registration from observed tag centres; no scene pose is an input."""
from collections import deque
from dataclasses import dataclass
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from urc_perception.core import square_points


def load_fixture(path):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    data['tags'] = {int(i): np.asarray(p, float) for i, p in data['tags'].items()}
    data['targets'] = {str(i): np.asarray(p, float) for i, p in data['targets'].items()}
    for p in [*data['tags'].values(), *data['targets'].values()]:
        if p.shape != (3,) or not np.isfinite(p).all() or abs(p[2]) > 1e-8:
            raise ValueError('fixture points must be finite, planar xyz positions in metres')
    if len(data['tags']) < 3 or not data['targets'] or any(i < 0 for i in data['tags']):
        raise ValueError('at least three distinct IDs and one target required')
    points = np.array(list(data['tags'].values()))
    if np.linalg.svd(points - points.mean(axis=0), compute_uv=False)[1] < .005:
        raise ValueError('fixture tags must be noncollinear and sufficiently separated')
    for name in ('tag_size_m', 'clearance_m', 'max_registration_error_m', 'tag_ttl_s',
                 'movement_threshold_m', 'attempt_timeout_s'):
        if not np.isfinite(data[name]) or data[name] <= 0:
            raise ValueError(f'{name} must be positive and finite')
    if not isinstance(data['min_samples'], int) or data['min_samples'] < 2:
        raise ValueError('min_samples must be at least two')
    return data


def rigid_fit(reference, observed):
    source, destination = np.asarray(reference, float), np.asarray(observed, float)
    if source.shape != destination.shape or source.ndim != 2 or source.shape[1] != 3 or len(source) < 3:
        raise ValueError('three corresponding 3D points required')
    if not np.isfinite([source, destination]).all():
        raise ValueError('nonfinite correspondences')
    a, b = source - source.mean(0), destination - destination.mean(0)
    if min(np.linalg.svd(a, compute_uv=False)[1], np.linalg.svd(b, compute_uv=False)[1]) < .005:
        raise ValueError('degenerate or poorly separated tag centres')
    u, _, vt = np.linalg.svd(a.T @ b)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ correction @ u.T
    position = destination.mean(0) - rotation @ source.mean(0)
    residuals = np.linalg.norm(source @ rotation.T + position - destination, axis=1)
    return position, rotation, float(np.sqrt(np.mean(residuals**2))), float(max(residuals))


def board_pose(corners_by_id, fixture, calibration):
    """Resolve a known planar layout using all corners, including ambiguous single tags.

    Both planar PnP solutions are checked. Near-equivalent poses are acceptable;
    different poses with indistinguishable image evidence are withheld.
    """
    ids = sorted(set(corners_by_id) & fixture['tags'].keys())
    if len(ids) < 3:
        return None
    objects = np.vstack([square_points(fixture['tag_size_m']) + fixture['tags'][i] for i in ids])
    pixels = np.vstack([corners_by_id[i] for i in ids]).astype(float)
    if pixels.shape != (len(objects), 2) or not np.isfinite(pixels).all():
        return None
    try:
        solved = cv2.solvePnPGeneric(objects, pixels, calibration.matrix,
                                    calibration.distortion, flags=cv2.SOLVEPNP_IPPE)
    except cv2.error:
        return None
    candidates = []
    if not solved[0]:
        return None
    for rv, tv in zip(solved[1], solved[2]):
        r, t = cv2.Rodrigues(rv)[0], np.asarray(tv).reshape(3)
        projected3d = objects @ r.T + t
        if not np.isfinite(projected3d).all() or np.any(projected3d[:, 2] <= .01) or r[:, 2] @ t >= 0:
            continue
        projected = cv2.projectPoints(objects, rv, tv, calibration.matrix, calibration.distortion)[0].reshape(-1, 2)
        error = float(np.sqrt(np.mean(np.sum((projected - pixels)**2, axis=1))))
        candidates.append((error, t, r))
    candidates.sort(key=lambda x: x[0])
    if not candidates or candidates[0][0] > 1.5:
        return None
    if len(candidates) > 1:
        e0, p0, r0 = candidates[0]
        e1, p1, r1 = candidates[1]
        similar_evidence = e1 - e0 < .05 or e1 < 1.15 * max(e0, 1e-9)
        different_pose = (np.max(np.linalg.norm(objects @ r0.T + p0 - objects @ r1.T - p1, axis=1)) > .002
                          or Rotation.from_matrix(r0.T @ r1).magnitude() > .04)
        if similar_evidence and different_pose:
            return None
    error, position, rotation = candidates[0]
    return position, rotation, ids, error


@dataclass
class Registration:
    valid: bool
    reason: str
    stamp: float = 0.
    position: object = None
    rotation: object = None
    error: float = float('inf')
    ids: tuple = ()


class TagMap:
    def __init__(self, fixture):
        self.fixture, self.samples, self.generation = fixture, {}, 0
        self.last_stamp = 0.

    def reset(self):
        self.samples.clear()
        self.generation += 1

    def add(self, observations, stamp):
        """Observations map ID -> (base position, base outward normal)."""
        if not np.isfinite(stamp) or stamp <= self.last_stamp:
            return False
        self.last_stamp = stamp
        accepted = {}
        for i, (p, n) in observations.items():
            p, n = np.asarray(p, float), np.asarray(n, float)
            if i in self.fixture['tags'] and p.shape == (3,) and n.shape == (3,) and np.isfinite([p, n]).all() and np.linalg.norm(n) > .9:
                accepted[i] = (p.copy(), n / np.linalg.norm(n))
        # One displaced previously seen tag invalidates the entire old layout.
        moved = any(i in self.samples and np.linalg.norm(p - np.mean([s[1] for s in self.samples[i]], axis=0)) > self.fixture['movement_threshold_m']
                    for i, (p, _) in accepted.items())
        if moved:
            self.reset()
        for i, (p, n) in accepted.items():
            self.samples.setdefault(i, deque(maxlen=8)).append((stamp, p, n))
        return moved

    def landmarks(self, now):
        result = {}
        for i in list(self.samples):
            samples = self.samples[i]
            while samples and now - samples[0][0] > self.fixture['tag_ttl_s']:
                samples.popleft()
            if not samples:
                del self.samples[i]
                continue
            xyz = np.array([s[1] for s in samples])
            center = xyz.mean(0)
            result[i] = (center, np.mean([s[2] for s in samples], axis=0), len(samples),
                         float(np.max(np.linalg.norm(xyz - center, axis=1))), samples[-1][0])
        return result

    def registration(self, now):
        landmarks = self.landmarks(now)
        ids = sorted(i for i, (_, _, count, spread, _) in landmarks.items()
                     if count >= self.fixture['min_samples'] and spread <= self.fixture['max_registration_error_m'])
        if len(ids) < 3:
            return Registration(False, 'Need three stable, noncollinear tag IDs', ids=tuple(ids))
        try:
            p, r, rms, worst = rigid_fit([self.fixture['tags'][i] for i in ids], [landmarks[i][0] for i in ids])
        except ValueError as exc:
            return Registration(False, str(exc), ids=tuple(ids))
        if worst > self.fixture['max_registration_error_m']:
            return Registration(False, 'Tag layout differs from calibrated fixture', error=rms, ids=tuple(ids))
        if any(r[:, 2] @ landmarks[i][1] < .8 for i in ids):
            return Registration(False, 'Inconsistent tag face normals', error=rms, ids=tuple(ids))
        return Registration(True, 'Registered calibrated fixture', max(landmarks[i][4] for i in ids), p, r, rms, tuple(ids))


def target_pose(registration, fixture, label):
    if not registration.valid:
        raise ValueError('valid panel registration required')
    offset = fixture['targets'][label].copy()
    offset[2] += fixture['clearance_m']
    position = registration.position + registration.rotation @ offset
    # Tool +X points into the panel; tool +Y follows the panel's up direction.
    x, y = -registration.rotation[:, 2], registration.rotation[:, 1]
    rotation = np.column_stack([x, y, np.cross(x, y)])
    return position, rotation
