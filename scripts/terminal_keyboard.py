"""Timed key pulses for SSH terminals, which do not report key releases."""

import os
import select
import sys
import termios
import time
import tty


class TerminalKeyboard:
    """Use cbreak input, restoring terminal settings on exit."""

    pulse_seconds = 0.12

    def __init__(self):
        self.is_connected = False
        self.expires = {}
        self.fd = None
        self.saved_settings = None

    def connect(self):
        if not sys.stdin.isatty():
            raise RuntimeError("终端控制需要交互式终端；SSH 请使用 ssh -t。")
        self.fd = sys.stdin.fileno()
        self.saved_settings = termios.tcgetattr(self.fd)
        try:
            # Keep ISIG so Ctrl+C reaches the application's cleanup handler.
            tty.setcbreak(self.fd)
            self.is_connected = True
            print(
                "\nSSH 终端控制已启动（英文输入法）。\n"
                "左臂: W/S A/D Q/E R/F T/G Z/X；右臂: 数字 8/2 4/6 7/9 / * + - 1/3\n"
                "底盘: I/K J/L U/O；速度: N/M；云台: < > , .\n"
                "每次输入产生短脉冲，停止输入后自动停止底盘；空格立即停止，Esc/B/Ctrl+C 退出。\n",
                flush=True,
            )
        except BaseException:
            self.disconnect()
            raise

    def get_action(self):
        now = time.monotonic()
        self.expires = {key: end for key, end in self.expires.items() if end > now}
        if select.select([self.fd], [], [], 0)[0]:
            data = os.read(self.fd, 4096)
            if not data:
                self.is_connected = False
            for key in data.decode("utf-8", errors="ignore"):
                if key in ("b", "B", "\x1b"):
                    self.is_connected = False
                    break
                if key in (" ", "\r", "\n"):
                    # Discard the rest of this batch after an explicit stop.
                    self.expires.clear()
                    break
                if key in "wasdqerftgzx824679/*+-13ijkluonm<>,.":
                    self.expires[key] = now + self.pulse_seconds
        if not self.is_connected:
            self.expires.clear()
            return {"b"}
        return set(self.expires)

    def disconnect(self):
        try:
            if self.saved_settings is not None:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved_settings)
        finally:
            self.saved_settings = None
            self.expires.clear()
            self.is_connected = False
