#!/usr/bin/env python3
"""USB XLeRobot keyboard or dual-leader teleoperation and local recording."""

import argparse
import importlib.util
import json
import time
from pathlib import Path
from types import SimpleNamespace

from serial_config import DEFAULT_CONFIG, resolve_ports
from terminal_keyboard import TerminalKeyboard
from dual_leader import DualLeader, validate_leader_ports

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


class Keyboard:
    """Capture press/release events in a focused window, including on Wayland."""

    def __init__(self):
        self.pressed = {}
        self.is_connected = False
        self.dual_leader = False

    def connect(self):
        import pygame

        self.pg = pygame
        pygame.display.init()
        pygame.font.init()
        screen = pygame.display.set_mode((820, 250))
        pygame.display.set_caption("XLeRobot keyboard control - keep this window focused")
        screen.fill((25, 30, 40))
        font = pygame.font.Font(None, 26)
        for index, line in enumerate([
            "Keep this window focused. ESC / B: stop and save.",
            ("Arms: left leader -> left arm; right leader -> right arm" if self.dual_leader
             else "Left arm: W/S A/D Q/E R/F T/G Z/X"),
            ("Leaders stay active when this window loses focus. B / ESC: exit." if self.dual_leader
             else "Right arm (NumLock on): 8/2 4/6 7/9 / * + - 1/3"),
            "Base: I/K J/L U/O. Speed: N/M. Head: < > , .",
            "Return to terminal for ENTER between episodes.",
        ]):
            screen.blit(font.render(line, True, (230, 235, 245)), (20, 22 + 40 * index))
        pygame.display.flip()
        self.is_connected = True

    def get_action(self):
        for event in self.pg.event.get():
            if event.type == self.pg.QUIT:
                self.is_connected = False
                self.pressed.clear()
            elif event.type == self.pg.WINDOWFOCUSLOST:
                self.pressed.clear()
            elif event.type == self.pg.KEYDOWN:
                if event.key == self.pg.K_ESCAPE:
                    self.is_connected = False
                if event.unicode:
                    self.pressed[event.key] = event.unicode
            elif event.type == self.pg.KEYUP:
                self.pressed.pop(event.key, None)
        return set(self.pressed.values()) if self.is_connected else {"b"}

    def disconnect(self):
        self.pg.quit()
        self.is_connected = False


def load_controls():
    path = ROOT / "XLeRobot/software/examples/4_xlerobot_teleop_keyboard.py"
    spec = importlib.util.spec_from_file_location("upstream_keyboard", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_controls(module, obs, fps):
    arms = []
    for prefix, mapping in [("left", module.LEFT_JOINT_MAP), ("right", module.RIGHT_JOINT_MAP)]:
        arm = module.SimpleTeleopArm(module.SO101Kinematics(), mapping, obs, prefix=prefix)
        # Start from measured positions; all kinematics helpers use degrees.
        arm.target_positions = arm.joint_positions.copy()
        arm.current_x, arm.current_y = arm.kinematics.forward_kinematics(
            arm.target_positions["shoulder_lift"], arm.target_positions["elbow_flex"]
        )
        arm.pitch = sum(arm.target_positions[j] for j in ("shoulder_lift", "elbow_flex", "wrist_flex"))
        arm.degree_step = 30.0 / fps
        arm.xy_step = 0.03 / fps
        arms.append(arm)
    head = module.SimpleHeadControl(obs)
    head.degree_step = 15.0 / fps
    return *arms, head


def control_action(module, controls, robot, obs, keys, leader=None, fps=30):
    left, right, head = controls
    left_keys = {action: key in keys for action, key in module.LEFT_KEYMAP.items()}
    right_keys = {action: key in keys for action, key in module.RIGHT_KEYMAP.items()}
    if leader is None:
        left.handle_keys(left_keys)
        right.handle_keys(right_keys)
    head.handle_keys(left_keys)
    # Reuse one observation for each control tick and its recorded state.
    view = SimpleNamespace(get_observation=lambda: obs)
    arm_action = (leader.action(obs, fps) if leader is not None else {
        **left.p_control_action(view), **right.p_control_action(view),
    })
    return {
        **arm_action,
        **head.p_control_action(view),
        **robot._from_keyboard_to_base_action(np.array(list(keys))),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port1", help="Left arm + head: 8 motors; overrides saved mapping")
    parser.add_argument("--port2", help="Right arm: 6 motors; overrides saved mapping")
    parser.add_argument("--port3", help="Base wheels: 3 motors; overrides saved mapping")
    parser.add_argument("--ports-config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--keyboard", choices=("window", "terminal"), default="window",
                        help="window: graphical control; terminal: key pulses over SSH")
    parser.add_argument("--control", choices=("keyboard", "dual-leader"), default="keyboard")
    parser.add_argument("--leader-left-port", help="Left SO-100/101 leader serial device")
    parser.add_argument("--leader-right-port", help="Right SO-100/101 leader serial device")
    parser.add_argument("--id", default="xlerobot_usb")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--cameras", type=Path, help="JSON mapping of camera names to OpenCVCameraConfig fields")
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--task", default="")
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--episode-seconds", type=float, default=30)
    parser.add_argument("--dataset-name", default=time.strftime("xlerobot_%Y%m%d_%H%M%S"))
    args = parser.parse_args()
    if args.fps <= 0 or args.episodes <= 0 or args.episode_seconds <= 0:
        parser.error("fps, episodes and episode-seconds must be positive")
    if args.record and not args.task.strip():
        parser.error("--record requires a meaningful --task description")
    if Path(args.dataset_name).name != args.dataset_name or args.dataset_name in (".", ".."):
        parser.error("dataset-name must be a single directory name")
    try:
        ports = resolve_ports(args.ports_config, {key: getattr(args, key) for key in ("port1", "port2", "port3")})
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    if args.control == "dual-leader":
        if not args.leader_left_port or not args.leader_right_port:
            parser.error("dual-leader requires --leader-left-port and --leader-right-port")
        try:
            validate_leader_ports(args.leader_left_port, args.leader_right_port, ports.values())
        except (OSError, ValueError) as exc:
            parser.error(str(exc))
    elif args.leader_left_port or args.leader_right_port:
        parser.error("Leader ports require --control dual-leader")
    for role, port in ports.items():
        print(f"{role}: {port}")

    from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
    from lerobot.configs.video import RGBEncoderConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.utils.feature_utils import build_dataset_frame, hw_to_dataset_features
    from lerobot_robot_xlerobot import XLerobot, XLerobotConfig

    dataset_root = ROOT / "data" / args.dataset_name
    if args.record and dataset_root.exists():
        parser.error(f"Dataset already exists: {dataset_root}; choose another --dataset-name")
    cameras = {}
    if args.cameras:
        cameras = {name: OpenCVCameraConfig(**cfg) for name, cfg in json.loads(args.cameras.read_text()).items()}
    module = load_controls()
    keyboard = TerminalKeyboard() if args.keyboard == "terminal" else Keyboard()
    keyboard.dual_leader = args.control == "dual-leader"
    robot = XLerobot(XLerobotConfig(
        id=args.id, **ports, cameras=cameras,
        use_degrees=True, calibration_dir=ROOT / "calibration",
    ))
    leader = (DualLeader(args.leader_left_port, args.leader_right_port, ROOT / "calibration")
              if args.control == "dual-leader" else None)
    dataset = None
    try:
        if leader is not None:
            print("Dual leaders: left → left arm, right → right arm. Arm keyboard keys are disabled. "
                  "Space/focus loss does not pause leaders; B/ESC/Ctrl+C exits.")
            leader.connect()
        robot.connect()
        keyboard.connect()
        controls = make_controls(module, robot.get_observation(), args.fps)
        if args.record:
            features = {
                **hw_to_dataset_features(robot.observation_features, "observation", use_video=True),
                **hw_to_dataset_features(robot.action_features, "action", use_video=True),
            }
            dataset = LeRobotDataset.create(
                repo_id=f"local/{args.dataset_name}", root=dataset_root, fps=args.fps,
                features=features, robot_type=robot.name, video_backend="pyav",
                rgb_encoder=RGBEncoderConfig(vcodec="h264", preset="fast"),
                image_writer_threads=max(1, len(cameras)),
            )
        episode = 0
        while not args.record or episode < args.episodes:
            if args.record:
                robot.stop_base()
                input(f"Episode {episode + 1}/{args.episodes}: position robot, then press ENTER to record. ")
            if leader is not None:
                leader.align(robot.get_observation())
            start = time.perf_counter()
            while keyboard.is_connected and (not args.record or time.perf_counter() - start < args.episode_seconds):
                tick = time.perf_counter()
                keys = set(keyboard.get_action())
                if "b" in keys:
                    return
                obs = robot.get_observation()
                action = control_action(module, controls, robot, obs, keys, leader, args.fps)
                sent = robot.send_action(action)
                if dataset is not None:
                    dataset.add_frame({
                        **build_dataset_frame(dataset.features, obs, prefix="observation"),
                        **build_dataset_frame(dataset.features, sent, prefix="action"),
                        "task": args.task,
                    })
                time.sleep(max(0, 1 / args.fps - (time.perf_counter() - tick)))
            robot.stop_base()
            if dataset is not None and dataset.has_pending_frames():
                dataset.save_episode()
            episode += 1
            if not keyboard.is_connected:
                break
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        # Stop hardware before video encoding/finalization, even on errors.
        try:
            if robot.is_connected:
                robot.disconnect()
        finally:
            try:
                if leader is not None:
                    leader.disconnect()
            finally:
                try:
                    if keyboard.is_connected:
                        keyboard.disconnect()
                finally:
                    if dataset is not None:
                        try:
                            if dataset.has_pending_frames():
                                dataset.save_episode()
                        finally:
                            dataset.finalize()
                        print(f"Saved locally: {dataset_root}")


if __name__ == "__main__":
    main()
