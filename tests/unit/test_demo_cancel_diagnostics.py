"""Cancellation watchdog evidence is owned, bounded and never controls cleanup."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import textwrap
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from mujoco_ros2_manipulation.operator_console.demo_worker import (
    DemoCancelled, DemoJob, _CancelStackCapture,
)


WORKER = "mujoco_ros2_manipulation.operator_console.demo_worker"


class CancelStackCaptureTests(unittest.TestCase):
    def test_first_interrupt_arms_once_and_keeps_the_existing_exception(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            console = root / "runtime/operator/session"
            job = DemoJob(console / "demos" / ("a" * 32), "contact_comparison", 91,
                          console, render=False, root=root)
            with patch(WORKER + ".faulthandler.dump_traceback_later") as dump, \
                 patch(WORKER + ".faulthandler.cancel_dump_traceback_later") as cancel:
                with self.assertRaises(DemoCancelled):
                    job.interrupt()
                stream = dump.call_args.kwargs["file"]
                self.assertFalse(stream.closed)
                job.interrupt()
                dump.assert_called_once_with(10.0, file=stream, exit=False)
                job._cancel_stacks.close()
                job._cancel_stacks.close()
                cancel.assert_called_once_with()
                self.assertTrue(stream.closed)

    def test_failed_diagnostics_do_not_replace_cancellation_or_cancel_another_timer(self):
        for failure in ("open", "watchdog"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                console = root / "runtime/operator/session"
                job = DemoJob(console / "demos" / ("b" * 32), "contact_comparison", 91,
                              console, render=False, root=root)
                stream = Mock()
                open_result = dict(side_effect=OSError("disk unavailable")) if failure == "open" else dict(return_value=stream)
                with patch.object(Path, "open", **open_result), \
                     patch(WORKER + ".faulthandler.dump_traceback_later", side_effect=RuntimeError("watchdog unavailable")), \
                     patch(WORKER + ".faulthandler.cancel_dump_traceback_later") as cancel:
                    with self.assertRaises(DemoCancelled):
                        job.interrupt()
                    self.assertTrue(job.cancel_requested)
                    job._cancel_stacks.close()
                    cancel.assert_not_called()
                    if failure == "watchdog":
                        stream.close.assert_called_once_with()

    def test_diagnostics_close_error_does_not_replace_original_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            console = root / "runtime/operator/session"
            job = DemoJob(console / "demos" / ("c" * 32), "contact_comparison", 91,
                          console, render=False, root=root)
            stream = Mock()
            stream.close.side_effect = OSError("close failed")
            job._cancel_stacks._stream = stream
            with patch("mujoco_ros2_manipulation.workcells.config.load_workcell", return_value=SimpleNamespace()), \
                 patch.object(job, "_contact_pair", side_effect=RuntimeError("original failure")), \
                 patch.dict(os.environ):
                self.assertEqual(job.run(), 1)
            self.assertEqual(json.loads(job.path.read_text())["error"]["message"], "original failure")

    def test_unproven_watchdog_cancellation_keeps_its_descriptor_open(self):
        with tempfile.TemporaryDirectory() as tmp:
            capture = _CancelStackCapture(tmp)
            with patch(WORKER + ".faulthandler.dump_traceback_later"), \
                 patch(WORKER + ".faulthandler.cancel_dump_traceback_later", side_effect=RuntimeError("unavailable")):
                capture.start()
                capture.close()
                self.assertFalse(capture._stream.closed)
            with patch(WORKER + ".faulthandler.cancel_dump_traceback_later"):
                capture.close()


class CancelStackSubprocessTests(unittest.TestCase):
    SCRIPT = textwrap.dedent("""
        import json
        import os
        from pathlib import Path
        import signal
        import sys
        import threading
        import time
        from types import SimpleNamespace
        from unittest.mock import patch
        from mujoco_ros2_manipulation.operator_console.demo_worker import DemoJob, _CancelStackCapture

        root, mode = Path(sys.argv[1]), sys.argv[2]
        console = root / 'runtime/operator/session'
        job = DemoJob(console / 'demos' / ('d' * 32), 'contact_comparison', 91,
                      console, render=False, root=root)
        job._cancel_stacks = _CancelStackCapture(job.directory, delay=.05 if mode == 'blocked' else .5)
        release = threading.Event()
        def _background_wait():
            release.wait()
        threading.Thread(target=_background_wait, daemon=True).start()
        def _wait_in_cancel_cleanup(cell):
            try:
                (root / 'ready').write_text('ready')
                while True:
                    time.sleep(1)
            finally:
                (root / 'cleaning').write_text('cleaning')
                if mode == 'blocked':
                    release.wait()
        previous = signal.getsignal(signal.SIGINT)
        with patch('mujoco_ros2_manipulation.workcells.config.load_workcell', return_value=SimpleNamespace()), \\
             patch.object(job, '_contact_pair', side_effect=_wait_in_cancel_cleanup), \\
             patch.dict(os.environ):
            code = job.run()
        assert code == 130
        assert signal.getsignal(signal.SIGINT) == previous
        assert job._cancel_stacks._stream is None
        assert not (job.directory / 'comparison.json').exists()
        time.sleep(.6)  # Prove fast cancellation removed the delayed reservation.
        raise SystemExit(code)
    """)

    def wait_for(self, predicate, description):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.01)
        self.fail("Timed out waiting for " + description)

    def run_worker(self, root, mode):
        process = subprocess.Popen([sys.executable, "-c", self.SCRIPT, str(root), mode],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        self.addCleanup(self.stop_worker, process)
        self.wait_for(lambda: (root / "ready").exists() or process.poll() is not None,
                      "worker readiness")
        if process.poll() is not None:
            self.fail(process.communicate(timeout=2)[1].decode())
        process.send_signal(signal.SIGINT)
        return process

    @staticmethod
    def stop_worker(process):
        if process.poll() is None:
            process.terminate()
        process.communicate(timeout=3)

    def test_real_sigint_records_all_thread_stacks_before_parent_timeout(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            process = self.run_worker(root, "blocked")
            path = root / "runtime/operator/session/demos" / ("d" * 32) / "cancel-stacks.log"
            self.wait_for(lambda: (root / "cleaning").exists(), "cleanup wait")
            process.send_signal(signal.SIGINT)  # Repeated SIGINT must not interrupt cleanup.
            self.wait_for(lambda: path.exists() and path.stat().st_size > 0, "native watchdog dump")
            text = path.read_text()
            self.assertIn("Timeout", text)
            self.assertIn("_wait_in_cancel_cleanup", text)
            self.assertIn("_background_wait", text)
            self.assertIsNone(process.poll())  # Diagnostics must not terminate the worker.
            size = path.stat().st_size
            time.sleep(.15)
            self.assertEqual(path.stat().st_size, size)  # Only one bounded dump.

    def test_fast_cancellation_restores_handler_and_leaves_no_stall_dump(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            process = self.run_worker(root, "quick")
            _, errors = process.communicate(timeout=5)
            self.assertEqual(process.returncode, 130, errors.decode())
            directory = root / "runtime/operator/session/demos" / ("d" * 32)
            self.assertEqual((directory / "cancel-stacks.log").read_bytes(), b"")
            data = json.loads((directory / "status.json").read_text())
            self.assertEqual(data["status"], "cancelled")
            self.assertIsNone(data["comparison"])


if __name__ == "__main__":
    unittest.main()
