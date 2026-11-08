#!/usr/bin/env python3
"""Hardware-free checks of plugins, keyboard controls, CUDA and dataset round trips."""

import importlib
import tempfile
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np
import torch

from keyboard_collect import Keyboard, control_action, load_controls, make_controls
from lerobot.configs.video import RGBEncoderConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.feature_utils import build_dataset_frame, hw_to_dataset_features


def main():
    for name in [
        "lerobot_robot_xlerobot", "lerobot_robot_xlerobot_2wheels",
        "lerobot_robot_xlerobot_mecanum", "lerobot_teleoperator_xlerobot_vr",
        "lerobot.scripts.lerobot_record", "lerobot.scripts.lerobot_teleoperate",
    ]:
        importlib.import_module(name)
        print(f"IMPORT OK: {name}")
    from lerobot_robot_xlerobot import XLerobot, XLerobotConfig

    robot = XLerobot(XLerobotConfig(
        id="install_check", use_degrees=True,
        calibration_dir=Path(__file__).resolve().parents[1] / "calibration",
    ))
    obs = {name: 0.0 for name in robot.action_features}
    obs["left_arm_shoulder_lift.pos"] = 10.0
    obs["right_arm_wrist_flex.pos"] = 15.0
    module = load_controls()
    controls = make_controls(module, obs, 30)
    action = control_action(module, controls, robot, obs, set())
    assert set(action) == set(robot.action_features)
    for key in action:
        assert np.isclose(action[key], obs[key]), (key, action[key], obs[key])
    moving = control_action(module, controls, robot, obs, {"i", "q"})
    assert moving["x.vel"] > 0
    assert moving["left_arm_shoulder_pan.pos"] > obs["left_arm_shoulder_pan.pos"]
    print("CONTROLS OK: initial pose preserved; base and arm keys produce actions")

    import pygame

    keyboard = Keyboard()
    keyboard.connect()
    try:
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_i, unicode="i"))
        assert "i" in keyboard.get_action()
        pygame.event.post(pygame.event.Event(pygame.WINDOWFOCUSLOST))
        assert not keyboard.get_action()
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_ESCAPE, unicode=""))
        assert keyboard.get_action() == {"b"}
    finally:
        keyboard.disconnect()
    print("KEYBOARD OK: press, focus loss, exit")

    print(f"TORCH: {torch.__version__}; CUDA available: {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        assert (torch.ones(4, device="cuda") @ torch.ones(4, device="cuda")).item() == 4
        print(f"CUDA OK: {torch.cuda.get_device_name(0)}")

    with tempfile.TemporaryDirectory(prefix="xlerobot_check_", dir=Path(__file__).resolve().parents[1] / "logs") as tmp:
        root = Path(tmp) / "dataset"
        features = {
            **hw_to_dataset_features({**robot.observation_features, "head": (64, 64, 3)}, "observation"),
            **hw_to_dataset_features(robot.action_features, "action"),
        }
        ds = LeRobotDataset.create(
            "local/install_check", root=root, fps=30, features=features,
            robot_type="xlerobot", video_backend="pyav", image_writer_threads=1,
            rgb_encoder=RGBEncoderConfig(vcodec="h264", preset="fast"),
        )
        for index in range(12):
            camera_obs = {**obs, "head": np.full((64, 64, 3), index * 10, dtype=np.uint8)}
            ds.add_frame({
                **build_dataset_frame(features, camera_obs, "observation"),
                **build_dataset_frame(features, action, "action"),
                "task": "Synthetic installation check",
            })
        assert ds.has_pending_frames()
        ds.save_episode()
        assert not ds.has_pending_frames()
        ds.finalize()
        for backend in ("pyav", "torchcodec"):
            loaded = LeRobotDataset("local/install_check", root=root, video_backend=backend)
            assert loaded.num_frames == 12 and loaded.num_episodes == 1
            assert loaded[0]["observation.images.head"].shape == (3, 64, 64)
            assert loaded[0]["action"].shape == (17,)
            print(f"DATASET OK: parquet + H.264 video + {backend} readback")

        # Run the actual CLI loop against simulated hardware; exit during an episode.
        import keyboard_collect

        class FakeRobot:
            observation_features = robot.observation_features
            action_features = robot.action_features
            name = "xlerobot"
            is_connected = False
            stopped = False

            def __init__(self, config):
                pass

            def connect(self):
                self.is_connected = True

            def disconnect(self):
                self.stopped = True
                self.is_connected = False

            def stop_base(self):
                pass

            def get_observation(self):
                return obs.copy()

            def send_action(self, action):
                return action

            def _from_keyboard_to_base_action(self, keys):
                return robot._from_keyboard_to_base_action(keys)

        class FakeKeyboard:
            is_connected = False
            ticks = 0

            def connect(self):
                self.is_connected = True

            def disconnect(self):
                self.is_connected = False

            def get_action(self):
                self.ticks += 1
                return set() if self.ticks <= 3 else {"b"}

        argv = ["keyboard_collect.py", "--port1=SIMULATED1", "--port2=SIMULATED2","--port3=SIMULATED3", 
                "--record", "--task=Synthetic CLI check", "--dataset-name=cli_check"]
        with patch("lerobot_robot_xlerobot.XLerobot", FakeRobot), \
            patch.object(keyboard_collect, "Keyboard", FakeKeyboard), \
            patch.object(keyboard_collect, "ROOT", Path(tmp)), \
            patch.object(keyboard_collect, "load_controls", lambda: module), \
            patch("builtins.input", return_value=""), patch("sys.argv", argv):
            keyboard_collect.main()
        cli_ds = LeRobotDataset("local/cli_check", root=Path(tmp) / "data/cli_check", video_backend="pyav")
        assert cli_ds.num_frames == 3 and cli_ds.num_episodes == 1
        print("CLI OK: interrupted episode saved with 3 frames")

        # Exercise the dual-leader CLI with real routing and simulated serial arms.
        serial_paths = [Path(tmp) / f"serial_{i}" for i in range(5)]
        for path in serial_paths:
            path.touch()
        leaders = []

        def simulated_leader(config):
            side = "left" if config.id == "leader_left" else "right"
            from dual_leader import JOINTS
            arm = Mock(is_connected=False)
            arm.connect.side_effect = lambda: setattr(arm, "is_connected", True)
            arm.disconnect.side_effect = lambda: setattr(arm, "is_connected", False)
            arm.get_action.return_value = {
                f"{joint}.pos": obs[f"{side}_arm_{joint}.pos"] for joint in JOINTS
            }
            leaders.append(arm)
            return arm

        argv = ["keyboard_collect.py", "--control=dual-leader",
                f"--port1={serial_paths[2]}", f"--port2={serial_paths[3]}", f"--port3={serial_paths[4]}",
                f"--leader-left-port={serial_paths[0]}", f"--leader-right-port={serial_paths[1]}",
                "--record", "--task=Synthetic dual leader check", "--dataset-name=dual_cli_check"]
        with patch("lerobot_robot_xlerobot.XLerobot", FakeRobot), \
            patch("lerobot.teleoperators.so_leader.SOLeader", simulated_leader), \
            patch.object(keyboard_collect, "Keyboard", FakeKeyboard), \
            patch.object(keyboard_collect, "ROOT", Path(tmp)), \
            patch.object(keyboard_collect, "load_controls", lambda: module), \
            patch("builtins.input", return_value=""), patch("sys.argv", argv):
            keyboard_collect.main()
        dual_ds = LeRobotDataset("local/dual_cli_check", root=Path(tmp) / "data/dual_cli_check", video_backend="pyav")
        assert dual_ds.num_frames == 3 and dual_ds.num_episodes == 1
        assert len(leaders) == 2 and all(not arm.is_connected for arm in leaders)
        print("DUAL LEADER CLI OK: both arms routed, recorded and disconnected")

    print("ALL CHECKS PASSED; physical robot/cameras not tested.")


if __name__ == "__main__":
    main()
