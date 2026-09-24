"""Perspective image renderer for a synthetic four-marker panel.

Ground truth is used ONLY here, never passed to the detector. No arm occlusion,
rolling shutter, lens blur, physics or contact simulation is claimed.
"""
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from urc_description.model import CAMERA_TRANSLATION, CAMERA_ROTATION
from urc_perception.core import square_points

K = np.array([[600., 0., 320.], [0., 600., 240.], [0., 0., 1.]])
SIZE = .02
TAG_CENTERS = {0: [-.05, .03, 0], 1: [.05, .03, 0],
               2: [.05, -.03, 0], 3: [-.05, -.03, 0]}
BOARD_POSITION = np.array([.82, .19, 0.0])
BOARD_ROTATION = np.array([[0., 0., -1.], [0., 1., 0.], [1., 0., 0.]])
START = np.array([0., .5, -1., 0., .5, 0.])


def camera_pose(model, q):
    j5 = model.joint5_output(q)
    r = Rotation.from_quat(j5.quaternion_xyzw).as_matrix()
    return j5.position + r @ CAMERA_TRANSLATION, r @ CAMERA_ROTATION


class Scene:
    def __init__(self):
        self.position = BOARD_POSITION.copy()
        self.rotation = BOARD_ROTATION.copy()
        self.visible_ids = set(TAG_CENTERS)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        generate = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
        self.markers = {i: generate(dictionary, i, 128) for i in TAG_CENTERS}

    def render(self, camera_p, camera_r, grayscale=False):
        frame = np.full((480, 640), 185, np.uint8)
        rc = camera_r.T @ self.rotation
        tc = camera_r.T @ (self.position - camera_p)

        def project(points):
            camera = np.asarray(points) @ rc.T + tc
            if np.any(camera[:, 2] <= .02):
                return None
            pixels = camera @ K.T
            pixels = pixels[:, :2] / pixels[:, 2:]
            # Prevent enormous off-screen integer coordinates/warp transforms.
            if not np.isfinite(pixels).all() or np.max(np.abs(pixels)) > 1e5:
                return None
            return pixels.astype(np.float32)

        if np.dot(rc[:, 2], tc) >= 0:
            return frame if grayscale else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        outline = project([[-.075, .05, 0], [.075, .05, 0], [.075, -.05, 0], [-.075, -.05, 0]])
        if outline is not None:
            cv2.fillConvexPoly(frame, np.rint(outline).astype(np.int32), 245)
        for marker_id, center in TAG_CENTERS.items():
            if marker_id not in self.visible_ids:
                continue
            points = project(square_points(SIZE) + center)
            if points is None:
                continue
            src = np.array([[0, 0], [127, 0], [127, 127], [0, 127]], np.float32)
            h = cv2.getPerspectiveTransform(src, points)
            warped = cv2.warpPerspective(self.markers[marker_id], h, (640, 480), flags=cv2.INTER_NEAREST, borderValue=255)
            mask = np.zeros_like(frame)
            cv2.fillConvexPoly(mask, np.rint(points).astype(np.int32), 255)
            frame[mask > 0] = warped[mask > 0]
        return frame if grayscale else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
