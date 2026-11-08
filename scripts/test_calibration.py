"""Hardware-free regression checks for shifted STS3215 calibration ranges."""

import unittest

from lerobot.motors import MotorCalibration
from lerobot_robot_xlerobot.calibration import center_sts3215_calibration


class CalibrationTests(unittest.TestCase):
    def test_coordinate_translation_preserves_span_and_normalized_position(self):
        for low, high, offset in [
            (-168, 2057, 0), (2027, 4347, 200), (3, 4095, -100),
            (-166, 2050, -1284), (2035, 4345, 1284),
        ]:
            with self.subTest(low=low, high=high):
                original = MotorCalibration(1, 0, offset, low, high)
                actual = center_sts3215_calibration({"joint": original})["joint"]
                self.assertGreaterEqual(actual.range_min, 0)
                self.assertLessEqual(actual.range_max, 4095)
                self.assertGreaterEqual(actual.homing_offset, -2047)
                self.assertLessEqual(actual.homing_offset, 2047)
                self.assertEqual(actual.range_max - actual.range_min, high - low)
                for position in (low, (low + high) / 2, high):
                    encoder = position + offset
                    new_position = encoder - actual.homing_offset
                    self.assertAlmostEqual(
                        position - (low + high) / 2,
                        new_position - (actual.range_min + actual.range_max) / 2,
                    )
                    self.assertAlmostEqual(
                        (position - low) / (high - low),
                        (new_position - actual.range_min) / (actual.range_max - actual.range_min),
                    )
                self.assertEqual((original.range_min, original.range_max), (low, high))

    def test_reported_elbow_failure_has_a_valid_translation(self):
        original = MotorCalibration(3, 0, -1284, -166, 2050)
        actual = center_sts3215_calibration({"elbow": original})["elbow"]
        self.assertEqual((actual.homing_offset, actual.range_min, actual.range_max), (-2047, 597, 2813))

    def test_representable_calibration_is_unchanged(self):
        original = MotorCalibration(1, 0, -500, 100, 3000)
        self.assertEqual(center_sts3215_calibration({"joint": original})["joint"], original)

    def test_translation_existence_matches_register_constraints(self):
        for low in (-1000, -166, 0, 2000, 4095):
            for span in (1, 2000, 4095, 4096):
                for offset in (-2047, -1284, 0, 1284, 2047):
                    high = low + span
                    # Independent check: all legal offsets, rather than delta bounds.
                    feasible = any(
                        0 <= low + offset - new_offset < high + offset - new_offset <= 4095
                        for new_offset in range(-2047, 2048)
                    )
                    with self.subTest(low=low, span=span, offset=offset):
                        original = MotorCalibration(1, 0, offset, low, high)
                        if feasible:
                            actual = center_sts3215_calibration({"joint": original})["joint"]
                            self.assertTrue(0 <= actual.range_min < actual.range_max <= 4095)
                            self.assertTrue(-2047 <= actual.homing_offset <= 2047)
                        else:
                            with self.assertRaises(ValueError):
                                center_sts3215_calibration({"joint": original})

    def test_invalid_spans_and_unrepresentable_offsets_are_rejected(self):
        for low, high, offset in [(1, 1, 0), (10, 5, 0), (-168, 4347, 0), (-168, 2057, -2000)]:
            with self.subTest(low=low, high=high, offset=offset):
                with self.assertRaises(ValueError):
                    center_sts3215_calibration({"joint": MotorCalibration(1, 0, offset, low, high)})


if __name__ == "__main__":
    unittest.main()
