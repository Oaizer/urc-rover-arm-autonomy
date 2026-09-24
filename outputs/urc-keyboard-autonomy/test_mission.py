import unittest

import numpy as np

from interfaces import JointFeedback, KeyboardObservation, KeyboardSurveyResult
from mission import KeyboardMission
from hardware import (SimulatedJoints, controller_to_joint,
                      joint_to_controller)


class Localizer:
    def observe(self, frame, timestamp_ns):
        return KeyboardObservation(timestamp_ns, np.eye(4), (1, 2, 3, 4), 0.5, (10, 10))


class Verifier:
    def __init__(self): self.text = ""
    def read(self, frame):
        self.text += "ABC"[len(self.text)]
        return self.text


class IK:
    def solve(self, position, orientation, seed): return np.zeros(6), 0.0


class TestMission(unittest.TestCase):
    def test_simulated_key_sequence(self):
        mission = KeyboardMission(SimulatedJoints(), Localizer(), Verifier(), IK(),
                                  {"A": (0, 0), "B": (10, 0), "C": (20, 0)})
        result = mission.run("ABC", [np.zeros((10, 10, 3), dtype=np.uint8)])
        self.assertEqual(mission.state.value, "COMPLETE")
        self.assertEqual(len(result), 3)

    def test_feedback_shape_is_preserved(self):
        fb = SimulatedJoints().feedback()
        self.assertEqual(fb.positions_rad.shape, (6,))

    def test_typing_accepts_completed_survey(self):
        result = KeyboardSurveyResult(np.eye(4), (0, 1, 2), 6, 2, 12.0,
                                      0.4, 0.5, 0.7, True)
        mission = KeyboardMission(SimulatedJoints(), Localizer(), Verifier(), IK(),
                                  {"A": (0, 0), "B": (10, 0), "C": (20, 0)})
        keys = mission.run("ABC", [np.zeros((10, 10, 3), dtype=np.uint8)],
                           survey_result=result)
        self.assertEqual(mission.state.value, "COMPLETE")
        self.assertEqual(len(keys), 3)

    def test_controller_joint_calibration_round_trip(self):
        joint = np.array([0.1, -0.2, 0.3, -0.4, 0.5, -0.6])
        ratios = np.array([1.0, 6.0, 9.0, 1.0, 3.0, 1.0])
        offsets = np.array([0.2, 0.1, -0.1, 0.0, 0.3, -0.2])
        signs = np.array([1.0, -1.0, 1.0, -1.0, 1.0, -1.0])
        raw = joint_to_controller(joint, ratios, offsets, signs)
        recovered = controller_to_joint(raw, ratios, offsets, signs)
        np.testing.assert_allclose(recovered, joint)


if __name__ == "__main__":
    unittest.main()
