"""Hardware-free checks for leader routing, alignment and motion limits."""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

from dual_leader import DualLeader, JOINTS, validate_leader_ports


class DualLeaderTests(unittest.TestCase):
    def setUp(self):
        self.leader = DualLeader.__new__(DualLeader)
        self.leader.previous = None
        self.values = {side: {f"{joint}.pos": 0.0 for joint in JOINTS}
                       for side in ("left", "right")}
        self.leader.arms = {side: Mock(get_action=Mock(return_value=values))
                            for side, values in self.values.items()}
        self.obs = self.leader.read()

    def test_independent_sides_and_gripper_limits(self):
        self.values["left"]["shoulder_pan.pos"] = 8
        self.values["right"]["shoulder_pan.pos"] = -6
        self.values["left"]["gripper.pos"] = 120
        result = self.leader.read()
        self.assertEqual(result["left_arm_shoulder_pan.pos"], 8)
        self.assertEqual(result["right_arm_shoulder_pan.pos"], -6)
        self.assertEqual(result["left_arm_gripper.pos"], 100)

    def test_startup_mismatch_refuses_to_follow(self):
        self.values["right"]["elbow_flex.pos"] = 30
        with self.assertRaisesRegex(ValueError, "未对齐"):
            self.leader.align(self.obs)
        with self.assertRaises(RuntimeError):
            self.leader.action(self.obs, 30)

    def test_each_episode_alignment_resets_previous(self):
        self.leader.align(self.obs)
        self.leader.previous = dict.fromkeys(self.obs, 8)
        self.leader.align(self.obs)
        self.assertEqual(self.leader.previous, self.obs)

    def test_rate_and_tracking_error_are_limited(self):
        self.leader.align(self.obs)
        self.values["left"]["shoulder_pan.pos"] = 90
        self.values["left"]["gripper.pos"] = 100
        for _ in range(100):  # Stalled follower must not accumulate a large target error.
            result = self.leader.action(self.obs, 30)
            self.assertEqual(result["left_arm_shoulder_pan.pos"], 1)
            self.assertEqual(result["left_arm_gripper.pos"], 2)
            self.assertEqual(result["right_arm_shoulder_pan.pos"], 0)
        updated = dict(result)
        result = self.leader.action(updated, 30)
        self.assertEqual(result["left_arm_shoulder_pan.pos"], 2)

    def test_bad_read_aborts_entire_action(self):
        self.leader.align(self.obs)
        self.values["right"]["wrist_roll.pos"] = float("nan")
        with self.assertRaises(ValueError):
            self.leader.action(self.obs, 30)
        self.assertEqual(self.leader.previous, self.obs)

    def test_disconnect_attempts_both_arms(self):
        self.leader.arms["left"].disconnect.side_effect = OSError("lost USB")
        with self.assertRaises(OSError):
            self.leader.disconnect()
        self.leader.arms["right"].disconnect.assert_called_once()

    def test_aliases_cannot_reuse_follower_serial(self):
        with tempfile.TemporaryDirectory() as tmp:
            ports = [Path(tmp) / str(i) for i in range(5)]
            for port in ports:
                port.touch()
            validate_leader_ports(*ports[:2], ports[2:])
            alias = Path(tmp) / "alias"
            alias.symlink_to(ports[2])
            with self.assertRaises(ValueError):
                validate_leader_ports(alias, ports[1], ports[2:])


if __name__ == "__main__":
    unittest.main()
