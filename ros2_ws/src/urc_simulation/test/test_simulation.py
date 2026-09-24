from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation
from urc_description.model import make_urdf, CAMERA_TRANSLATION, CAMERA_ROTATION
from urc_kinematics.core import load_config
from urc_perception.core import Detector
from urc_simulation.scene import Scene, camera_pose, START
from urc_simulation.trajectory import Motion

CONFIG = Path(__file__).resolve().parents[2] / 'urc_kinematics' / 'config' / 'demo.json'


def walk_urdf(xml, q):
    joints = list(ET.fromstring(xml).findall('joint'))
    transforms = {'world': np.eye(4)}
    for joint in joints:
        parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
        origin = joint.find('origin')
        local = np.eye(4)
        local[:3, 3] = np.fromstring(origin.get('xyz'), sep=' ')
        local[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy'), sep=' ')).as_matrix()
        if joint.get('type') == 'revolute':
            axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
            rot = np.eye(4)
            rot[:3, :3] = Rotation.from_rotvec(axis * q[int(joint.get('name').split('_')[1]) - 1]).as_matrix()
            local = local @ rot
        transforms[child] = transforms[parent] @ local
    return transforms


def test_urdf_matches_ik_and_j5_in_world():
    model, _ = load_config(CONFIG)
    world = Rotation.from_euler('x', np.pi / 2).as_matrix()
    xml = make_urdf(CONFIG, include_sim_camera=True)
    for q in [START, np.zeros(6), *np.random.default_rng(3).uniform(-1, 1, (10, 6))]:
        frames = walk_urdf(xml, q)
        tcp = model.forward(q)
        np.testing.assert_allclose(frames['arm_tcp'][:3, 3], world @ tcp.position, atol=1e-10)
        np.testing.assert_allclose(frames['arm_tcp'][:3, :3], world @ Rotation.from_quat(tcp.quaternion_xyzw).as_matrix(), atol=1e-10)
        j5 = model.joint5_output(q)
        np.testing.assert_allclose(frames['arm_joint5_output'][:3, 3], world @ j5.position, atol=1e-10)
        p, r = camera_pose(model, q)
        np.testing.assert_allclose(frames['sim_camera_optical_frame'][:3, 3], world @ p, atol=1e-10)
        np.testing.assert_allclose(frames['sim_camera_optical_frame'][:3, :3], world @ r, atol=1e-10)


def test_physical_description_does_not_invent_camera():
    assert 'sim_camera' not in make_urdf(CONFIG)


def test_rviz_jazzy_interactive_namespace():
    path = Path(__file__).resolve().parents[1] / 'config' / 'arm.rviz'
    config = yaml.safe_load(path.read_text())
    display = next(d for d in config['Visualization Manager']['Displays']
                   if d['Class'] == 'rviz_default_plugins/InteractiveMarkers')
    assert display['Interactive Markers Namespace'] == '/sim/controls'
    assert 'Update Topic' not in display


def test_quintic_motion_limits_and_endpoints():
    motion = Motion(np.zeros(6), np.array([1, -.4, .5, 0, -.1, .3]), 10)
    samples = [motion.sample(t) for t in np.linspace(10, 10 + motion.duration, 1001)]
    velocities = np.array([s[1] for s in samples])
    assert np.max(np.abs(velocities)) <= .35 + 1e-8
    accelerations = np.diff(velocities, axis=0) / (motion.duration / 1000)
    assert np.max(np.abs(accelerations)) <= .5 + 1e-5
    np.testing.assert_allclose(samples[0][0], motion.start)
    np.testing.assert_allclose(samples[-1][0], motion.end)
    assert samples[-1][2]


@pytest.mark.parametrize('value', [float('nan'), float('inf')])
def test_nonfinite_trajectory_rejected(value):
    with pytest.raises(ValueError):
        Motion(np.zeros(6), [value] * 6, 0)


def test_synthetic_scene_passes_through_real_detector():
    model, _ = load_config(CONFIG)
    scene = Scene()
    p, r = camera_pose(model, START)
    image = scene.render(p, r)
    detections = Detector('DICT_4X4_50').detect(image)
    assert {d.tag_id for d in detections} == {0, 1, 2, 3}
    # Moving the camera changes actual rendered pixels, not just a 3D overlay.
    moved = scene.render(p, r @ Rotation.from_euler('y', .2).as_matrix())
    assert not np.array_equal(image, moved)
    assert not Detector('DICT_4X4_50').detect(scene.render(p, r @ Rotation.from_euler('y', np.pi).as_matrix()))
