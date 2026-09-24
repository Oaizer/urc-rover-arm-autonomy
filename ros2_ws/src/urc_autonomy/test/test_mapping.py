from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from urc_autonomy.mapping import load_fixture, rigid_fit, board_pose, TagMap, target_pose
from urc_autonomy.motion import Motion
from urc_kinematics.core import load_config
from urc_perception.core import Calibration, Detector
from urc_simulation.scene import Scene, camera_pose, START, K

ROOT = Path(__file__).resolve().parents[2]
FIXTURE = ROOT / 'urc_autonomy' / 'config' / 'demo_panel.json'
CONFIG = ROOT / 'urc_kinematics' / 'config' / 'demo.json'


def observations(fixture, position, rotation, ids=None):
    return {i: (position + rotation @ p, rotation[:, 2]) for i, p in fixture['tags'].items() if ids is None or i in ids}


def test_sequential_views_register_same_base_frame():
    fixture = load_fixture(FIXTURE)
    mapping = TagMap(fixture)
    r = Rotation.from_euler('xyz', [.1, -.5, .2]).as_matrix()
    p = np.array([.7, .2, .02])
    for index, tag_id in enumerate([0, 1, 2]):
        for sample in range(3):
            mapping.add(observations(fixture, p, r, [tag_id]), 10. + index + sample * .1)
    result = mapping.registration(13.)
    assert result.valid and result.ids == (0, 1, 2)
    np.testing.assert_allclose(result.position, p, atol=1e-10)
    np.testing.assert_allclose(result.rotation, r, atol=1e-10)
    hover, orientation = target_pose(result, fixture, 'A')
    assert np.dot(hover - (p + r @ fixture['targets']['A']), r[:, 2]) == pytest.approx(.02)
    np.testing.assert_allclose(orientation[:, 0], -r[:, 2])


def test_motion_invalidates_whole_previous_map():
    fixture = load_fixture(FIXTURE)
    mapping = TagMap(fixture)
    p, r = np.array([.7, .2, 0]), np.eye(3)
    for t in [1., 1.1, 1.2]:
        mapping.add(observations(fixture, p, r), t)
    assert mapping.registration(1.3).valid
    assert mapping.add(observations(fixture, p + [.03, 0, 0], r, [0]), 1.4)
    assert mapping.generation == 1
    assert not mapping.registration(1.5).valid
    assert set(mapping.samples) == {0}


def test_duplicate_frames_do_not_manufacture_stability_and_old_tags_expire():
    fixture = load_fixture(FIXTURE)
    mapping = TagMap(fixture)
    batch = observations(fixture, np.zeros(3), np.eye(3))
    for _ in range(10):
        mapping.add(batch, 1.)
    assert not mapping.registration(1.1).valid
    mapping.add(batch, 1.2)
    mapping.add(batch, 1.3)
    assert mapping.registration(1.4).valid
    assert not mapping.registration(30.).valid


def test_inconsistent_layout_and_normals_rejected():
    fixture = load_fixture(FIXTURE)
    mapping = TagMap(fixture)
    batch = observations(fixture, np.zeros(3), np.eye(3))
    batch[0] = (batch[0][0] + [.03, 0, 0], batch[0][1])
    for t in [1., 1.1, 1.2]:
        mapping.add(batch, t)
    assert not mapping.registration(1.3).valid
    mapping = TagMap(fixture)
    batch = observations(fixture, np.zeros(3), np.eye(3))
    batch[0] = (batch[0][0], -batch[0][1])
    for t in [1., 1.1, 1.2]:
        mapping.add(batch, t)
    assert not mapping.registration(1.3).valid


def test_collinear_registration_is_not_a_panel_frame():
    with pytest.raises(ValueError):
        rigid_fit([[0, 0, 0], [.1, 0, 0], [.2, 0, 0]], [[0, 1, 0], [.1, 1, 0], [.2, 1, 0]])


@pytest.mark.parametrize('angle', [0., .12, -.12])
def test_rendered_pixels_register_without_scene_pose_input(angle):
    fixture = load_fixture(FIXTURE)
    model, _ = load_config(CONFIG)
    scene = Scene()
    scene.rotation = scene.rotation @ Rotation.from_euler('y', angle).as_matrix()
    camera_p, camera_r = camera_pose(model, START)
    image = scene.render(camera_p, camera_r)
    detected = Detector('DICT_4X4_50').detect(image)
    result = board_pose({d.tag_id: d.corners for d in detected}, fixture, Calibration(640, 480, K, np.zeros(5)))
    assert result is not None
    p, r, ids, _ = result
    assert len(ids) >= 3
    assert np.linalg.norm(camera_p + camera_r @ p - scene.position) < .002
    assert Rotation.from_matrix((camera_r @ r).T @ scene.rotation).magnitude() < .04


def test_backend_cannot_shorten_bounded_trajectory():
    with pytest.raises(ValueError):
        Motion(np.zeros(6), np.ones(6), 0., duration=.1)


def test_actual_partial_images_fuse_across_joint_positions():
    fixture = load_fixture(FIXTURE)
    model, _ = load_config(CONFIG)
    scene, mapping = Scene(), TagMap(fixture)
    detector = Detector('DICT_4X4_50', [f'{i}:0.02' for i in range(4)], Calibration(640, 480, K, np.zeros(5)))
    for view, yaw in enumerate([.08, -.08]):
        q = START.copy()
        q[0] = yaw
        camera_p, camera_r = camera_pose(model, q)
        detected = detector.detect(scene.render(camera_p, camera_r))
        assert len(detected) == 2  # Neither view sees enough IDs on its own.
        batch = {}
        for d in detected:
            assert d.pose.valid and not d.pose.ambiguous
            batch[d.tag_id] = (camera_p + camera_r @ d.pose.tvec,
                               camera_r @ Rotation.from_rotvec(d.pose.rvec).as_matrix()[:, 2])
        for sample in range(3):
            mapping.add(batch, 1. + view + sample * .1)
        if view == 0:
            assert not mapping.registration(1.3).valid
    result = mapping.registration(2.3)
    assert result.valid
    np.testing.assert_allclose(result.position, scene.position, atol=.002)
    for label in fixture['targets']:
        point, _ = target_pose(result, fixture, label)
        truth = scene.position + scene.rotation @ (fixture['targets'][label] + [0, 0, .02])
        assert np.linalg.norm(point - truth) < .002
