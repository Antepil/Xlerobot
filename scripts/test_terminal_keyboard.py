"""Exercise SSH input and terminal restoration using a real pseudo terminal."""

import os
import pty
import termios
import unittest
from unittest.mock import patch

from terminal_keyboard import TerminalKeyboard


class TerminalKeyboardTests(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = pty.openpty()
        self.stream = os.fdopen(os.dup(self.slave), "r")
        self.original = termios.tcgetattr(self.slave)
        self.keyboard = TerminalKeyboard()
        with patch("terminal_keyboard.sys.stdin", self.stream), patch("builtins.print"):
            self.keyboard.connect()

    def tearDown(self):
        self.keyboard.disconnect()
        self.assertEqual(termios.tcgetattr(self.slave), self.original)
        self.stream.close()
        os.close(self.master)
        os.close(self.slave)

    def send(self, data):
        os.write(self.master, data)
        import select
        self.assertTrue(select.select([self.slave], [], [], 1)[0])

    def test_pulse_expires_without_release_event(self):
        with patch("terminal_keyboard.time.monotonic", return_value=10):
            self.send(b"i")
            self.assertEqual(self.keyboard.get_action(), {"i"})
        with patch("terminal_keyboard.time.monotonic", return_value=10.2):
            self.assertEqual(self.keyboard.get_action(), set())

    def test_stop_discards_remaining_batch(self):
        self.send(b"i")
        self.assertEqual(self.keyboard.get_action(), {"i"})
        self.send(b" j")
        self.assertEqual(self.keyboard.get_action(), set())

    def test_escape_exits_and_clears_keys(self):
        self.send(b"i\x1b")
        self.assertEqual(self.keyboard.get_action(), {"b"})
        self.assertFalse(self.keyboard.is_connected)

    def test_ctrl_c_signal_is_preserved(self):
        self.assertTrue(termios.tcgetattr(self.slave)[3] & termios.ISIG)


if __name__ == "__main__":
    unittest.main()
