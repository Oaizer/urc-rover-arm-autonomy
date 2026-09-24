"""Hardware-free regression tests, compatible with unittest and pytest."""

import json
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from urc_kinematics import Geometry, Kinematics, Pose, load_config
from urc_kinematics.service_adapter import ServiceAdapter, ordered_joint_positions


ROOT = Path(__file__).resolve().parents[1]


def pose_message():
    return NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))


def independent_fk(q):
    """Independent explicit homogeneous matrices; original mm config converted once."""
    offsets_mm = [[0, 50, 0], [350, 0, 0], [50, 100, 0],
                  [300, 0, 0], [30, 0, 0], [100, 0, 0]]
    transform = np.eye(4)
    for angle, axis, offset in zip(q, 'YZZXZX', offsets_mm):
        c, s = np.cos(angle), np.sin(angle)
        rotation = {'X': [[1, 0, 0], [0, c, -s], [0, s, c]],
                    'Y': [[c, 0, s], [0, 1, 0], [-s, 0, c]],
                    'Z': [[c, -s, 0], [s, c, 0], [0, 0, 1]]}[axis]
        joint, link = np.eye(4), np.eye(4)
        joint[:3, :3] = rotation
        link[:3, 3] = np.array(offset) / 1000
        transform = transform @ joint @ link
    return transform


class KinematicsTests(unittest.TestCase):
    def setUp(self):
        self.model, self.config = load_config(ROOT / 'config/demo.json')

    def assert_pose(self, actual, expected, pos=1e-4, angle=1e-3):
        self.assertLessEqual(np.linalg.norm(actual.position - expected.position), pos)
        relative = Rotation.from_quat(actual.quaternion_xyzw).inv() * Rotation.from_quat(expected.quaternion_xyzw)
        self.assertLessEqual(relative.magnitude(), angle)

    def test_zero_pose_and_si(self):
        pose = self.model.forward(np.zeros(6))
        np.testing.assert_allclose(pose.position, [.83, .15, 0], atol=1e-14)
        np.testing.assert_allclose(pose.quaternion_xyzw, [0, 0, 0, 1], atol=1e-14)
        self.assertFalse(self.config['commissioned'])
        self.assertEqual(self.config['base_frame'], 'arm_base_legacy')

    def test_geometry_against_independent_homogeneous_fk(self):
        rng = np.random.default_rng(41)
        for q in rng.uniform(-np.pi, np.pi, (30, 6)):
            reference = independent_fk(q)
            pose = self.model.forward(q)
            np.testing.assert_allclose(pose.position, reference[:3, 3], atol=1e-14)
            np.testing.assert_allclose(Rotation.from_quat(pose.quaternion_xyzw).as_matrix(),
                                       reference[:3, :3], atol=1e-14)

    def test_y_up_mapping(self):
        mapping = Rotation.from_rotvec([np.pi/2, 0, 0]).as_matrix()
        np.testing.assert_allclose(mapping @ self.model.forward(np.zeros(6)).position,
                                   [.83, 0, .15], atol=1e-14)

    def test_seed_preserved_and_sign_equivalent_quaternion(self):
        q = np.array([.4, -.6, 1.1, .5, -.4, .7])
        pose = self.model.forward(q)
        answer = self.model.solve(Pose(pose.position, -pose.quaternion_xyzw), q)
        self.assertTrue(answer.success)
        np.testing.assert_array_equal(answer.positions, q)

    def test_random_full_pose_roundtrips_from_different_seeds(self):
        rng = np.random.default_rng(11)
        for q in rng.uniform(-2.7, 2.7, (16, 6)):
            target = self.model.forward(q)
            answer = self.model.solve(target, np.zeros(6))
            self.assertTrue(answer.success, answer.message)
            self.assert_pose(self.model.forward(answer.positions), target)
            self.assertLessEqual(answer.position_error_m, self.model.position_tolerance_m)
            self.assertLessEqual(answer.orientation_error_rad, self.model.orientation_tolerance_rad)

    def test_arbitrary_tool_translation_and_rotation(self):
        tool = Pose([.08, -.03, .04], Rotation.from_euler('xyz', [.2, -.5, .9]).as_quat())
        model = Kinematics(self.model.geometry, self.config['limits_rad'], tool)
        q = np.array([.4, -.6, 1.1, .5, -.4, .7])
        reference = independent_fk(q)
        target = model.forward(q)
        np.testing.assert_allclose(target.position,
                                   reference[:3, 3] + reference[:3, :3] @ tool.position, atol=1e-14)
        expected_r = reference[:3, :3] @ Rotation.from_quat(tool.quaternion_xyzw).as_matrix()
        np.testing.assert_allclose(Rotation.from_quat(target.quaternion_xyzw).as_matrix(), expected_r, atol=1e-14)
        answer = model.solve(target, np.zeros(6))
        self.assertTrue(answer.success)
        self.assert_pose(model.forward(answer.positions), target)
        seeds = model._geometric_seeds(target.position, expected_r, np.zeros(6))
        self.assertGreater(len(seeds), 0)
        for candidate in seeds:
            self.assert_pose(model.forward(candidate), target, pos=1e-12, angle=1e-12)

    def test_singular_wrist_and_limit_boundary(self):
        for q in [np.zeros(6), np.array([np.pi, .2, -.4, .8, 0., -.6]),
                  np.array([.3, -.1, .5, .4, np.pi, -.7])]:
            target = self.model.forward(q)
            answer = self.model.solve(target, q)
            self.assertTrue(answer.success)
            self.assert_pose(self.model.forward(answer.positions), target)

    def test_unreachable_position(self):
        result = self.model.solve(Pose([10, 0, 0], [0, 0, 0, 1]), np.zeros(6))
        self.assertFalse(result.success)
        self.assertIn('reach bound', result.message)
        self.assertTrue(np.isfinite(result.position_error_m))

    def test_orientation_failure_within_reach_and_tight_limits(self):
        model = Kinematics(self.model.geometry, [[-.001, .001]]*6,
                           Pose([0, 0, 0], [0, 0, 0, 1]), starts=2, max_nfev=30)
        result = model.solve(Pose([.83, .15, 0], [1, 0, 0, 0]), np.zeros(6), allow_orientation_fallback=False)
        self.assertFalse(result.success)
        self.assertGreater(result.orientation_error_rad, 3.)
        self.assertTrue(np.all(np.abs(result.positions) <= .001))

    def test_invalid_inputs(self):
        for q in [[0]*5, [0]*7, [np.nan]*6, [np.inf]*6, [4]*6]:
            with self.assertRaises(ValueError):
                self.model.forward(q)
            with self.assertRaises(ValueError):
                self.model.solve(Pose([0, 0, 0], [0, 0, 0, 1]), q)
        for q in [[0]*4, [0, 0, 0, 2], [np.nan, 0, 0, 1]]:
            with self.assertRaises(ValueError):
                Pose([0, 0, 0], q)
        with self.assertRaises(ValueError):
            Pose([np.inf, 0, 0], [0, 0, 0, 1])

    def test_orientation_fallback_preserves_position_and_limits(self):
        model = Kinematics(self.model.geometry, [[-.001, .001]]*6,
                           Pose([0, 0, 0], [0, 0, 0, 1]), starts=2, max_nfev=30)
        target = Pose([.83, .15, 0], [1, 0, 0, 0])
        result = model.solve(target, np.zeros(6))
        self.assertTrue(result.success)
        self.assertTrue(result.orientation_relaxed)
        self.assertLessEqual(result.position_error_m, model.position_tolerance_m)
        self.assertGreater(result.orientation_error_rad, 3.)
        np.testing.assert_allclose(result.positions, np.zeros(6))

    def test_fallback_moves_to_position_with_nonzero_tool(self):
        model = Kinematics(self.model.geometry, [[-.35, .35]]*6,
                           Pose([.03, .01, .02], [0, 0, 0, 1]), starts=3, max_nfev=100)
        wanted = model.forward([.12, .15, -.2, .1, .08, -.1])
        target = Pose(wanted.position, [1, 0, 0, 0])
        result = model.solve(target, np.zeros(6))
        self.assertTrue(result.success, result.message)
        self.assertTrue(result.orientation_relaxed)
        self.assertLessEqual(np.linalg.norm(model.forward(result.positions).position - wanted.position), 1e-4)
        self.assertTrue(np.all(np.abs(result.positions) <= .35))

    def test_full_pose_remains_first_choice(self):
        target = self.model.forward([.1, .2, -.3, .1, .2, -.1])
        result = self.model.solve(target, np.zeros(6))
        self.assertTrue(result.success)
        self.assertFalse(result.orientation_relaxed)
        self.assert_pose(self.model.forward(result.positions), target)

    def test_fallback_cannot_bypass_joint_limits(self):
        model = Kinematics(self.model.geometry, [[-.001, .001]]*6,
                           Pose([0, 0, 0], [0, 0, 0, 1]), starts=2, max_nfev=30)
        result = model.solve(Pose([.75, .2, 0], [0, 0, 0, 1]), np.zeros(6))
        self.assertFalse(result.success)
        self.assertFalse(result.orientation_relaxed)
        self.assertTrue(np.all(np.abs(result.positions) <= .001))

    def test_service_reports_relaxation_and_strict_opt_out(self):
        model = Kinematics(self.model.geometry, [[-.001, .001]]*6,
                           Pose([0, 0, 0], [0, 0, 0, 1]), starts=2, max_nfev=30)
        adapter = ServiceAdapter(model, lambda _: None)
        target = pose_message()
        target.position.x, target.position.y = .83, .15
        target.orientation.x, target.orientation.w = 1., 0.
        for strict in (False, True):
            reply = adapter.solve_ik(NS(target=target, seed=[0]*6, require_orientation=strict),
                                     NS(achieved_pose=pose_message()))
            self.assertEqual(reply.success, not strict)
            self.assertEqual(reply.orientation_relaxed, not strict)
            self.assertAlmostEqual(reply.achieved_pose.position.x, .83, places=3)

    def test_large_strict_branch_uses_nearby_free_wrist(self):
        model = Kinematics(self.model.geometry, self.config['limits_rad'],
                           Pose([0, 0, 0], [0, 0, 0, 1]), starts=4, max_nfev=80)
        seed = np.array([0, .5, -1, 0, .5, 0])
        far_wrist = seed.copy()
        far_wrist[5] = np.pi
        target = model.forward(far_wrist)
        relaxed = model.solve(target, seed, max_joint_delta_rad=.8)
        self.assertTrue(relaxed.success, relaxed.message)
        self.assertTrue(relaxed.orientation_relaxed)
        self.assertLessEqual(np.max(np.abs(relaxed.positions - seed)), .8 + 1e-9)
        np.testing.assert_allclose(model.forward(relaxed.positions).position, target.position, atol=1e-4)
        strict = model.solve(target, seed, max_joint_delta_rad=.8, allow_orientation_fallback=False)
        self.assertFalse(strict.success)

    def test_invalid_model(self):
        for limits in [[[0, 0]]*6, [[1, -1]]*6, [[0, np.inf]]*6, [[-1, 1]]*5]:
            with self.assertRaises(ValueError):
                Kinematics(self.model.geometry, limits, Pose([0, 0, 0], [0, 0, 0, 1]))
        for kwargs in [{'starts': 0}, {'max_nfev': True}, {'position_tolerance_m': np.nan},
                       {'orientation_tolerance_rad': -1}]:
            with self.assertRaises(ValueError):
                Kinematics(self.model.geometry, self.config['limits_rad'],
                           Pose([0, 0, 0], [0, 0, 0, 1]), **kwargs)
        with self.assertRaises(ValueError):
            Geometry(-1, 0, 0, 0, 0, 0, 0)

    def test_j5_output_includes_j5_but_not_j6_or_tool(self):
        q = np.zeros(6)
        before = self.model.joint5_output(q)
        np.testing.assert_allclose(before.position, [.7, .15, 0], atol=1e-14)
        q[4] = .7
        after = self.model.joint5_output(q)
        np.testing.assert_allclose(after.position, before.position)
        self.assertGreater(np.linalg.norm(after.quaternion_xyzw - before.quaternion_xyzw), .1)
        q[5] = 1.2
        self.assert_pose(self.model.joint5_output(q), after, pos=1e-14, angle=1e-14)

    def test_joint_state_name_mapping(self):
        names = [f'joint_{i}' for i in range(1, 7)]
        self.assertEqual(ordered_joint_positions(names[::-1] + ['gripper'], list(range(6)) + [9], names),
                         [5, 4, 3, 2, 1, 0])
        for actual, positions in [(names[:5], [0]*5), (names, [0]*5), (names + names, [0]*12)]:
            with self.assertRaises(ValueError):
                ordered_joint_positions(actual, positions, names)

    def test_service_callbacks_and_invalid_sentinels(self):
        errors = []
        adapter = ServiceAdapter(self.model, errors.append)
        fk = adapter.compute_fk(NS(positions=[0]*6), NS(pose=pose_message()))
        self.assertAlmostEqual(fk.pose.position.x, .83)
        ik = adapter.solve_ik(NS(target=fk.pose, seed=[0]*6), NS())
        self.assertTrue(ik.success)
        fk = adapter.compute_fk(NS(positions=[4]*6), NS(pose=pose_message()))
        self.assertTrue(np.isnan(fk.pose.position.x))
        self.assertTrue(np.isnan(fk.pose.orientation.w))
        ik = adapter.solve_ik(NS(target=fk.pose, seed=[0]*6), NS())
        self.assertFalse(ik.success)
        self.assertTrue(np.all(np.isnan(ik.positions)))
        self.assertTrue(np.isinf(ik.position_error_m))
        self.assertEqual(len(errors), 2)

    def test_config_schema_rejects_bad_frames_and_missing_tool(self):
        # Mock file reads to keep all tests free of filesystem writes.
        from unittest.mock import patch
        for modification in [{'base_frame': 'base_link'}, {'tool_transform': {}},
                             {'commissioned': 'false'}, {'extra': 1}]:
            data = dict(self.config, **modification)
            with patch.object(Path, 'read_text', return_value=json.dumps(data)):
                with self.assertRaises(ValueError):
                    load_config('unused.json')


if __name__ == '__main__':
    unittest.main()
