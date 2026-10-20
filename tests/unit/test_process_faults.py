"""Crash injection must never signal unrelated processes or hide other failures."""

from pathlib import Path
import signal
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.verification.process_faults import ExpectedCrash

START = "[INFO] [move_group-2]: process started with pid [4321]"
CRASH = "[ERROR] [move_group-2]: process has died [pid 4321, exit code -9, cmd 'move_group']."


class ProcessFaultTests(unittest.TestCase):
    def test_only_supported_core_targets(self):
        with self.assertRaises(ValueError):
            ExpectedCrash("python3")

    def inject(self, *, group=100, executable="move_group", log=START):
        crash = ExpectedCrash("move_group")
        with patch("os.pidfd_open", return_value=7) as opened, patch(
            "os.getpgid", return_value=group
        ), patch.object(
            Path, "resolve", return_value=Path("/owned/" + executable)
        ), patch(
            "signal.pidfd_send_signal"
        ) as sent, patch(
            "os.close"
        ) as closed:
            if group != 100 or executable != "move_group":
                with self.assertRaisesRegex(RuntimeError, "unowned or mismatched"):
                    crash.inject(log, 100)
                sent.assert_not_called()
                self.assertIsNone(crash.expected)
            else:
                result = crash.inject(log, 100)
                self.assertEqual(
                    result, dict(node="move_group-2", pid=4321, exit_code=-9)
                )
                opened.assert_called_once_with(4321)
                sent.assert_called_once_with(7, signal.SIGKILL)
                with self.assertRaisesRegex(RuntimeError, "Only one crash"):
                    crash.inject(log, 100)
            closed.assert_called_once_with(7)
        return crash

    def test_pidfd_signal_only_to_owned_matching_executable(self):
        crash = self.inject()
        self.assertEqual(crash.verify(CRASH, 0), [crash.expected])

    def test_other_process_group_or_executable_is_rejected(self):
        self.inject(group=200)
        self.inject(executable="unrelated")

    def test_missing_or_ambiguous_start_log_cannot_signal(self):
        for log in ("", START + "\n" + START):
            with self.subTest(log=log), patch("os.pidfd_open") as opened:
                with self.assertRaisesRegex(RuntimeError, "missing or ambiguous"):
                    ExpectedCrash("move_group").inject(log, 100)
                opened.assert_not_called()

    def test_missing_wrong_pid_extra_crash_or_nonzero_launch_fails(self):
        crash = self.inject()
        for log, code in (
            ("", 0),
            (CRASH.replace("4321", "4322"), 0),
            (CRASH + "\n" + CRASH, 0),
            (CRASH.replace("-9", "-11"), 0),
            (CRASH, 1),
        ):
            with self.subTest(log=log, code=code), self.assertRaises(RuntimeError):
                crash.verify(log, code)
        with self.assertRaises(RuntimeError):
            ExpectedCrash("move_group").verify(CRASH, 0)
