import unittest

import cv2
import numpy as np

from perception import ArucoTagDetector


class TestArucoTagDetector(unittest.TestCase):
    def test_individual_marker_pose(self):
        frame = np.full((480, 640, 3), 255, dtype=np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        marker = cv2.aruco.generateImageMarker(dictionary, 2, 100)
        frame[190:290, 270:370] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        K = np.array([[800.0, 0.0, 320.0],
                      [0.0, 800.0, 240.0],
                      [0.0, 0.0, 1.0]])
        detector = ArucoTagDetector(K, np.zeros(5), "DICT_4X4_50", 20.0)
        detections = detector.detect(frame, 123)
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0].marker_id, 2)
        self.assertTrue(detections[0].valid)
        self.assertGreater(detections[0].T_camera_tag[2, 3], 0.0)
        self.assertLess(detections[0].reprojection_error_px, 1.0)


if __name__ == "__main__":
    unittest.main()
