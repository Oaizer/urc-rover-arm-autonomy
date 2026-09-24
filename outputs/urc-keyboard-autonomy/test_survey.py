import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from interfaces import JointFeedback, TagDetection
from survey import ActiveKeyboardSurvey, SurveyConfig, _feedback_at


def transform(rotation=None, translation=(0.0, 0.0, 0.0)):
    result = np.eye(4)
    if rotation is not None:
        result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


class SurveyFixture:
    layout = {0: (-120.0, -50.0), 1: (120.0, -50.0),
              2: (120.0, 50.0), 3: (-120.0, 50.0), 4: (0.0, 0.0)}
    R_base_keyboard = np.array([[0.0, 0.0, -1.0],
                                [1.0, 0.0, 0.0],
                                [0.0, -1.0, 0.0]])
    T_base_keyboard = transform(R_base_keyboard, (600.0, 20.0, 80.0))

    @staticmethod
    def camera(yaw_deg):
        base_rotation = np.array([[0.0, 0.0, 1.0],
                                  [1.0, 0.0, 0.0],
                                  [0.0, 1.0, 0.0]])
        rotation = Rotation.from_euler("Z", yaw_deg, degrees=True).as_matrix() @ base_rotation
        return transform(rotation, (30.0, -20.0, 40.0))

    @classmethod
    def detections(cls, T_base_camera, ids=(0, 1, 2, 3, 4), offset=None):
        output = []
        for marker_id in ids:
            x, y = cls.layout[marker_id]
            T_keyboard_tag = transform(translation=(x, y, 0.0))
            T_camera_tag = np.linalg.inv(T_base_camera) @ cls.T_base_keyboard @ T_keyboard_tag
            if offset is not None and marker_id == offset[0]:
                T_camera_tag = T_camera_tag.copy()
                T_camera_tag[:3, 3] += np.asarray(offset[1], dtype=float)
            output.append(TagDetection(1_000_000_000, marker_id, T_camera_tag, 0.35, 900.0))
        return output


class TestActiveKeyboardSurvey(unittest.TestCase):
    def feedback(self, timestamp=1_000_000_000):
        return JointFeedback(timestamp, np.zeros(6), np.zeros(6))

    def test_multiview_registration(self):
        survey = ActiveKeyboardSurvey(SurveyFixture.layout)
        for name, yaw in (("left", -6.0), ("right", 6.0)):
            camera = SurveyFixture.camera(yaw)
            accepted = survey.add_frame(SurveyFixture.detections(camera), 1_000_000_000,
                                        self.feedback(), camera, name)
            self.assertEqual(accepted, 5)
        result = survey.build_result()
        np.testing.assert_allclose(result.T_base_keyboard, SurveyFixture.T_base_keyboard, atol=1e-8)
        self.assertEqual(result.marker_ids, (0, 1, 2, 3, 4))
        self.assertGreaterEqual(result.viewpoint_separation_deg, 11.9)
        self.assertLess(result.registration_rms_mm, 1e-8)
        self.assertTrue(result.normal_faces_base)

    def test_outlier_is_rejected_during_fusion(self):
        survey = ActiveKeyboardSurvey(SurveyFixture.layout)
        for index, yaw in enumerate((-8.0, 0.0, 8.0)):
            camera = SurveyFixture.camera(yaw)
            offset = (0, (0.0, 70.0, 0.0)) if index == 1 else None
            survey.add_frame(SurveyFixture.detections(camera, offset=offset),
                             1_000_000_000, self.feedback(), camera, f"v{index}")
        result = survey.build_result()
        self.assertLess(result.registration_rms_mm, 1e-6)

    def test_stale_joint_sample_is_rejected(self):
        survey = ActiveKeyboardSurvey(SurveyFixture.layout)
        camera = SurveyFixture.camera(0.0)
        accepted = survey.add_frame(SurveyFixture.detections(camera), 1_000_000_000,
                                    self.feedback(1_020_000_000), camera, "center")
        self.assertEqual(accepted, 0)
        self.assertIn("stale synchronization", survey.rejections[-1])

    def test_collinear_tags_do_not_define_keyboard(self):
        layout = {0: (-100.0, 0.0), 1: (0.0, 0.0), 2: (100.0, 0.0)}
        config = SurveyConfig(min_total_observations=6, require_normal_faces_base=False)
        survey = ActiveKeyboardSurvey(layout, config)
        T_base_keyboard = SurveyFixture.T_base_keyboard
        for name, yaw in (("left", -6.0), ("right", 6.0)):
            camera = SurveyFixture.camera(yaw)
            detections = []
            for marker_id, (x, y) in layout.items():
                T_camera_tag = np.linalg.inv(camera) @ T_base_keyboard @ transform(translation=(x, y, 0.0))
                detections.append(TagDetection(1_000_000_000, marker_id, T_camera_tag, 0.3, 900.0))
            survey.add_frame(detections, 1_000_000_000, self.feedback(), camera, name)
        with self.assertRaisesRegex(ValueError, "collinear"):
            survey.build_result()

    def test_joint_feedback_is_interpolated_to_image_time(self):
        before = JointFeedback(100, np.zeros(6), np.zeros(6))
        after = JointFeedback(200, np.ones(6), np.ones(6) * 2.0)
        sample = _feedback_at(150, before, after)
        self.assertEqual(sample.timestamp_ns, 150)
        np.testing.assert_allclose(sample.positions_rad, 0.5)
        np.testing.assert_allclose(sample.velocities_rad_s, 1.0)


if __name__ == "__main__":
    unittest.main()
