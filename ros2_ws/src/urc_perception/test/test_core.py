import math

import cv2
import numpy as np
import pytest

from urc_perception.core import (Calibration, Detector, estimate_pose, get_dictionary,
                                 image_stamp_is_fresh, load_calibration, parse_sizes,
                                 quaternion_xyzw, square_points)


@pytest.fixture
def calibration():
    return Calibration(640, 480, [[800, 0, 320], [0, 805, 240], [0, 0, 1]], np.zeros(5))


def marker_image():
    dictionary = get_dictionary('DICT_4X4_50')
    if hasattr(cv2.aruco, 'generateImageMarker'):
        marker = cv2.aruco.generateImageMarker(dictionary, 7, 160)
    else:
        marker = cv2.aruco.drawMarker(dictionary, 7, 160)
    image = np.full((480, 640), 255, np.uint8)
    image[160:320, 240:400] = marker
    return image


def project(calibration, rvec=(2.8, 0.3, -0.1), tvec=(0.03, -0.02, 0.8)):
    return cv2.projectPoints(square_points(0.15), np.array(rvec, dtype=float),
                             np.array(tvec, dtype=float), calibration.matrix,
                             calibration.distortion)[0].reshape(4, 2)


def test_synthetic_tag_detection_without_calibration():
    detector = Detector('DICT_4X4_50', ['7:0.15'])
    assert detector.parameters.cornerRefinementMethod == cv2.aruco.CORNER_REFINE_SUBPIX
    found = detector.detect(marker_image())
    assert len(found) == 1 and found[0].tag_id == 7
    assert found[0].corners.shape == (4, 2)
    assert not found[0].pose.valid and found[0].pose.tvec is None
    assert math.isnan(found[0].pose.error)
    np.testing.assert_allclose(found[0].corners, [[240, 160], [399, 160],
                                               [399, 319], [240, 319]], atol=1)


def test_bgr_and_blank_detection():
    detector = Detector('DICT_4X4_50')
    assert detector.detect(cv2.cvtColor(marker_image(), cv2.COLOR_GRAY2BGR))[0].tag_id == 7
    assert detector.detect(np.full((480, 640), 255, np.uint8)) == []


def test_unknown_size_has_no_pose(calibration):
    result = Detector('DICT_4X4_50', ['8:0.15'], calibration).detect(marker_image())[0]
    assert not result.pose.valid and math.isnan(result.pose.error)


def test_resolution_mismatch_has_no_pose(calibration):
    image = cv2.resize(marker_image(), (320, 240), interpolation=cv2.INTER_NEAREST)
    result = Detector('DICT_4X4_50', ['7:0.15'], calibration).detect(image)[0]
    assert not result.pose.valid and math.isnan(result.pose.error)


def test_projected_corners_recover_metric_pose(calibration):
    result = estimate_pose(project(calibration), 0.15, calibration)
    assert result.valid and not result.ambiguous
    assert result.error < 1e-5
    np.testing.assert_allclose(result.tvec, [0.03, -0.02, 0.8], atol=1e-6)
    expected = cv2.Rodrigues(np.array([2.8, 0.3, -0.1]))[0]
    np.testing.assert_allclose(cv2.Rodrigues(result.rvec)[0], expected, atol=1e-6)


def test_distorted_projected_corners(calibration):
    calibration = Calibration(640, 480, calibration.matrix, [-0.15, 0.03, 0.001, 0, 0])
    result = estimate_pose(project(calibration), 0.15, calibration)
    assert result.valid and result.error < 1e-4
    np.testing.assert_allclose(result.tvec, [0.03, -0.02, 0.8], atol=1e-5)


def test_front_parallel_ambiguity_reject_and_flag(calibration):
    # Tiny tilt avoids a singular exact-frontoparallel IPPE implementation case.
    corners = project(calibration, (math.pi - 0.005, 0, 0), (0, 0, 1.5))
    rejected = estimate_pose(corners, 0.15, calibration)
    assert rejected.ambiguous and not rejected.valid and rejected.tvec is None
    flagged = estimate_pose(corners, 0.15, calibration, ambiguity_policy='flag')
    assert flagged.ambiguous and flagged.valid and flagged.tvec[2] > 0


def test_reprojection_threshold_rejects_inconsistent_square(calibration):
    corners = project(calibration)
    corners[0] += [10, -9]
    result = estimate_pose(corners, 0.15, calibration, max_error_px=0.01)
    assert not result.valid and result.error > 0.01


def test_rejects_negative_depth_hypotheses(monkeypatch, calibration):
    rotations = [np.zeros((3, 1)), np.zeros((3, 1))]
    translations = [np.array([[0.0], [0.0], [-1.0]])] * 2
    monkeypatch.setattr(cv2, 'solvePnPGeneric', lambda *a, **k: (2, rotations, translations))
    assert not estimate_pose(project(calibration), 0.15, calibration).valid


def test_rejects_corner_behind_camera_even_when_center_positive(monkeypatch, calibration):
    rotation = np.array([[math.pi / 2], [0.0], [0.0]])
    translation = np.array([[0.0], [0.0], [0.01]])
    monkeypatch.setattr(cv2, 'solvePnPGeneric',
                        lambda *a, **k: (2, [rotation, rotation], [translation, translation]))
    assert not estimate_pose(project(calibration), 0.15, calibration).valid


def test_legacy_detector_api(monkeypatch):
    if not hasattr(cv2.aruco, 'ArucoDetector'):
        assert Detector('DICT_4X4_50').detect(marker_image())[0].tag_id == 7
        return
    # Current OpenCV no longer exports the free function: emulate the 4.6 surface.
    backend = cv2.aruco.ArucoDetector
    params = cv2.aruco.DetectorParameters
    monkeypatch.setattr(cv2.aruco, 'DetectorParameters_create', params, raising=False)
    monkeypatch.setattr(cv2.aruco, 'detectMarkers',
                        lambda gray, dictionary, parameters:
                        backend(dictionary, parameters).detectMarkers(gray), raising=False)
    monkeypatch.delattr(cv2.aruco, 'ArucoDetector')
    monkeypatch.delattr(cv2.aruco, 'DetectorParameters')
    assert Detector('DICT_4X4_50').detect(marker_image())[0].tag_id == 7


@pytest.mark.parametrize('entries', [['7:0'], ['7:-1'], ['7:nan'], ['7:inf'],
                                   ['7:0.1', '7:0.2'], ['50:0.1'], ['-1:0.1'], ['bad']])
def test_bad_sizes(entries):
    with pytest.raises(ValueError):
        parse_sizes(entries, get_dictionary('DICT_4X4_50'))


def test_dictionary_required():
    with pytest.raises(ValueError):
        Detector('')


def test_bad_calibration(calibration):
    with pytest.raises(ValueError):
        Calibration(640, 480, np.zeros((3, 3)), np.zeros(5))
    with pytest.raises(ValueError):
        Calibration(640, 480, calibration.matrix, [float('nan')] * 5)
    assert load_calibration('') is None


def test_nonfinite_corners(calibration):
    assert not estimate_pose(np.full((4, 2), np.nan), 0.15, calibration).valid


def test_quaternion():
    assert quaternion_xyzw([0, 0, 0]) == (0, 0, 0, 1)
    np.testing.assert_allclose(quaternion_xyzw([math.pi, 0, 0]), [1, 0, 0, 0], atol=1e-12)


@pytest.mark.parametrize('stamp,now,expected', [
    (1_000_000_000, 1_000_000_000, True),
    (1_000_000_000, 1_250_000_000, True),
    (1_000_000_000, 1_250_000_001, False),
    (1_000_000_001, 1_000_000_000, False),
    (0, 0, False),
    (0, 1_000_000_000, False),
    (-1, 1_000_000_000, False),
])
def test_timestamp_freshness(stamp, now, expected):
    assert image_stamp_is_fresh(stamp, now, 0.25) is expected


@pytest.mark.parametrize('limit', [0, -1, float('nan'), float('inf')])
def test_timestamp_invalid_limit(limit):
    with pytest.raises(ValueError):
        image_stamp_is_fresh(1, 1, limit)
