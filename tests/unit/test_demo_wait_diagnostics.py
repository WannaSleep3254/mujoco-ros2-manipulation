"""Parent wait timestamps distinguish a worker stall from forced group cleanup."""
import json
from pathlib import Path
import subprocess
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from mujoco_ros2_manipulation.operator_console.demo_jobs import DemoJobs


JOBS = "mujoco_ros2_manipulation.operator_console.demo_jobs"


class DemoWaitDiagnosticsTests(unittest.TestCase):
    def fixture(self, directory, *, timed_out=False, fail_audit=None):
        order, events = [], []
        process = Mock(pid=4242, returncode=None)
        process.poll.return_value = 130
        def wait(*, timeout):
            order.append("wait")
            if timed_out:
                raise subprocess.TimeoutExpired("owned-worker", timeout)
            process.returncode = 130
            return 130
        process.wait.side_effect = wait
        def audit(action, **details):
            order.append(action)
            if action == fail_audit:
                raise OSError("audit disk unavailable")
            events.append(dict(action=action, **details))
        app = SimpleNamespace(lock=threading.RLock(), children={"demo":process},
                              render_attempted=False, phase="demo", error=None,
                              audit=Mock(side_effect=audit))
        manager = DemoJobs(app)
        manager.process = process
        manager.folder = Path(directory)
        manager.current = dict(job_id="a" * 32, status="cancel_requested", comparison=None)
        manager.cancel_requested = True
        manager.refresh_for_cleanup = Mock(side_effect=lambda: order.append("refresh"))
        manager.cleanup_launches = Mock(side_effect=lambda: order.append("cleanup_groups") or False)
        return manager, process, order, events

    def test_completed_wait_records_elapsed_and_keeps_clean_cancellation(self):
        with tempfile.TemporaryDirectory() as directory:
            manager, process, order, events = self.fixture(directory)
            with patch(JOBS + ".time.monotonic", side_effect=[10.0, 10.25]), \
                 patch(JOBS + ".group_gone", return_value=True), \
                 patch(JOBS + ".stop_process") as stop:
                manager._stop_sync()
            process.wait.assert_called_once_with(timeout=45)
            process.send_signal.assert_not_called()
            stop.assert_not_called()
            self.assertEqual(order[:3], ["demo_worker_wait_started", "wait", "demo_worker_wait_finished"])
            self.assertEqual(events[0], dict(action="demo_worker_wait_started", job_id="a" * 32,
                worker_pid=4242, timeout_sec=45, elapsed_sec=0.0, return_code=None, timed_out=False))
            self.assertEqual(events[1], dict(action="demo_worker_wait_finished", job_id="a" * 32,
                worker_pid=4242, timeout_sec=45, elapsed_sec=.25, return_code=130, timed_out=False))
            self.assertEqual(manager.current["status"], "cancelled")
            self.assertTrue(manager.current["cleanup_clean"])
            self.assertIsNone(manager.process)

    def test_timeout_event_precedes_forced_cleanup_and_stays_unclean(self):
        with tempfile.TemporaryDirectory() as directory:
            manager, process, order, events = self.fixture(directory, timed_out=True)
            with patch(JOBS + ".time.monotonic", side_effect=[10.0, 55.125]), \
                 patch(JOBS + ".group_gone", return_value=True), \
                 patch(JOBS + ".stop_process", side_effect=lambda process: order.append("stop_worker")) as stop:
                manager._stop_sync()
            process.wait.assert_called_once_with(timeout=45)
            stop.assert_called_once_with(process)
            self.assertEqual(order[:6], ["demo_worker_wait_started", "wait", "demo_worker_wait_finished",
                                        "refresh", "cleanup_groups", "stop_worker"])
            self.assertEqual(events[1], dict(action="demo_worker_wait_finished", job_id="a" * 32,
                worker_pid=4242, timeout_sec=45, elapsed_sec=45.125, return_code=None, timed_out=True))
            self.assertEqual(manager.current["status"], "failed")
            self.assertFalse(manager.current["cleanup_clean"])
            self.assertEqual(json.loads((Path(directory) / "status.json").read_text())["cleanup_clean"], False)

    def test_wait_audit_failure_never_skips_wait_cleanup_or_changes_verdict(self):
        for timed_out in (False, True):
            for action in ("demo_worker_wait_started", "demo_worker_wait_finished"):
                with self.subTest(timed_out=timed_out, action=action), tempfile.TemporaryDirectory() as directory:
                    manager, process, order, events = self.fixture(directory, timed_out=timed_out, fail_audit=action)
                    with patch(JOBS + ".time.monotonic", side_effect=[10.0, 55.0 if timed_out else 10.1]), \
                         patch(JOBS + ".group_gone", return_value=True), \
                         patch(JOBS + ".stop_process") as stop:
                        manager._stop_sync()
                    process.wait.assert_called_once_with(timeout=45)
                    self.assertEqual(stop.call_count, int(timed_out))
                    self.assertEqual(manager.cleanup_launches.call_count, 2 if timed_out else 1)
                    self.assertEqual(manager.current["status"], "failed" if timed_out else "cancelled")
                    self.assertEqual(manager.current["cleanup_clean"], not timed_out)
                    self.assertFalse(manager.stopping)
                    self.assertIsNone(manager.process)


if __name__ == "__main__":
    unittest.main()
