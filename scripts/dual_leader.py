"""Two SO-100/101 leaders mapped to XLeRobot arms (degrees, gripper 0–100)."""

import math
from pathlib import Path

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


class DualLeader:
    def __init__(self, left_port, right_port, calibration_dir):
        from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig

        self.arms = {}
        for side, port in (("left", left_port), ("right", right_port)):
            self.arms[side] = SOLeader(SOLeaderTeleopConfig(
                id=f"leader_{side}", port=port, use_degrees=True,
                calibration_dir=calibration_dir,
            ))
        self.previous = None

    def connect(self):
        for arm in self.arms.values():
            arm.connect()

    def disconnect(self):
        # Attempt both cleanups even if one serial connection has failed.
        try:
            if self.arms["left"].is_connected:
                self.arms["left"].disconnect()
        finally:
            if self.arms["right"].is_connected:
                self.arms["right"].disconnect()

    def read(self):
        result = {}
        for side, arm in self.arms.items():
            values = arm.get_action()
            for joint in JOINTS:
                value = float(values[f"{joint}.pos"])
                if not math.isfinite(value):
                    raise ValueError(f"Invalid leader position: {side}/{joint}")
                if joint == "gripper":
                    value = max(0.0, min(100.0, value))
                result[f"{side}_arm_{joint}.pos"] = value
        return result

    def align(self, observation):
        targets = self.read()
        errors = [key for key, value in targets.items()
                  if abs(value - observation[key]) > (15 if "gripper" in key else 10)]
        if errors:
            raise ValueError(
                "主从臂姿态未对齐（关节需在 10 度内，夹爪在 15% 内）。"
                "请退出后调整姿态再启动；检查校准和左右接线：" + ", ".join(errors)
            )
        self.previous = {key: observation[key] for key in targets}

    def action(self, observation, fps):
        if self.previous is None:
            raise RuntimeError("Align leaders before enabling follow mode")
        targets = self.read()  # Read both successfully before sending any commands.
        result = {}
        for key, target in targets.items():
            step = (60.0 if "gripper" in key else 30.0) / fps
            # Bound changes against both the last command and current measured position.
            previous = self.previous[key]
            command = max(previous - step, min(previous + step, target))
            measured = observation[key]
            result[key] = max(measured - step, min(measured + step, command))
        self.previous = result.copy()
        return result


def validate_leader_ports(left, right, follower_ports):
    ports = [Path(port).resolve(strict=True) for port in (left, right, *follower_ports)]
    if len(set(ports)) != len(ports):
        raise ValueError("两个主臂和三块从端控制板必须使用五个不同的串口")
