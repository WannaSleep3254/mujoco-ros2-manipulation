"""Exact-worker cancellation survives import, constructor and readiness races."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import textwrap
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from mujoco_ros2_manipulation.operator_console.demo_handshake import IPC_LIMIT, read_handshake
from mujoco_ros2_manipulation.operator_console.demo_jobs import DemoJobs, process_identity
from mujoco_ros2_manipulation.operator_console.demo_worker import DemoCancelled, DemoJob
from mujoco_ros2_manipulation.verification.reports import atomic_json


JOBS = "mujoco_ros2_manipulation.operator_console.demo_jobs"
WORKER = "mujoco_ros2_manipulation.operator_console.demo_worker"
JOB_ID = "a" * 32
GENERATION = "b" * 32
GRACE_NS = 45_000_000_000
VALIDATION = Path(__file__).resolve().parents[2] / "runtime/validation/comparison-startup-cancel-20261110"


class HandshakeTests(unittest.TestCase):
    SCRIPT = textwrap.dedent("""
        import os
        from pathlib import Path
        import signal
        import sys
        import time
        from types import SimpleNamespace
        from unittest.mock import patch

        root, mode, generation = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        def gate(name):
            if mode == name:
                (root / 'gated').write_text(name)
                while not (root / 'release').exists():
                    time.sleep(.005)
        gate('before_import')
        from mujoco_ros2_manipulation.operator_console import demo_worker as worker
        gate('before_constructor')
        console = root / 'runtime/operator/session'
        job = worker.DemoJob(console / 'demos' / ('a' * 32), 'contact_comparison', 91,
                             console, render=False, root=root, generation=generation)
        original_write = worker.atomic_json
        def write(path, value):
            if Path(path).name == 'worker-ready.json':
                gate('before_ready')
            return original_write(path, value)
        def scenario(_cell):
            for variant in ('first_variant', 'second_variant'):
                job.check_cancel()
                (root / variant).touch()
                while not (root / 'advance').exists():
                    job.check_cancel()
                    time.sleep(.005)
                job.check_cancel()
            return {}
        previous = signal.getsignal(signal.SIGINT)
        with patch.object(worker, 'atomic_json', side_effect=write), \
             patch('mujoco_ros2_manipulation.workcells.config.load_workcell', return_value=SimpleNamespace()) as load, \
             patch.object(job, '_contact_pair', side_effect=scenario) as pair, \
             patch.dict(os.environ):
            code = job.run()
        assert signal.getsignal(signal.SIGINT) == previous
        assert code == 130, code
        assert job.cancel_intent is not None
        assert job.cancel_deadline_ns == job.cancel_intent['deadline_monotonic_ns']
        if mode != 'ready':
            load.assert_not_called()
            pair.assert_not_called()
        assert not (root / 'second_variant').exists()
        assert not (job.directory / 'comparison.json').exists()
        raise SystemExit(code)
    """)

    def setUp(self):
        VALIDATION.mkdir(parents=True, exist_ok=True)
        self.root = Path(tempfile.mkdtemp(prefix="handshake-unit-", dir=VALIDATION))

    def manager(self, process, *, root=None):
        root = self.root if root is None else root
        folder = root / "runtime/operator/session/demos" / JOB_ID
        folder.mkdir(parents=True, exist_ok=True)
        data = dict(schema_version=1, job_id=JOB_ID, scenario="contact_comparison",
                    status="starting", stage="starting", variant=None, run_id=None,
                    active_launch=None, results=[], comparison=None, error=None)
        atomic_json(folder / "status.json", data)
        app = SimpleNamespace(lock=threading.RLock(), cleanup_lock=threading.Lock(),
                              children={"demo": process}, render_attempted=False,
                              phase="demo", error=None, audit=Mock())
        manager = DemoJobs(app)
        manager.current, manager.folder, manager.process = data, folder, process
        manager.worker_generation = GENERATION
        manager.worker_identity = process_identity(process.pid)
        return manager

    def wait_for(self, predicate, process):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if predicate():
                return
            if process.poll() is not None:
                break
            time.sleep(.005)
        self.fail("Worker did not reach its gate. Preserved fixtures in " + str(self.root))

    def spawn(self, mode, *, root=None):
        root = self.root if root is None else root
        root.mkdir(parents=True, exist_ok=True)
        stream = (root / "worker.log").open("wb")
        self.addCleanup(stream.close)
        process = subprocess.Popen([sys.executable, "-c", self.SCRIPT, str(root), mode, GENERATION],
                                   cwd=Path(__file__).resolve().parents[2],
                                   stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        self.addCleanup(self.stop_child, process)
        return process

    @staticmethod
    def stop_child(process):
        if process.poll() is None:
            process.kill()
        process.wait(timeout=3)

    @staticmethod
    def record(kind, identity, *, stamp=None):
        stamp = time.monotonic_ns() if stamp is None else stamp
        value = dict(schema_version=1, kind=kind, job_id=JOB_ID, generation=GENERATION,
                     pid=identity["pid"], start_ticks=identity["start_ticks"])
        if kind == "ready":
            value["ready_monotonic_ns"] = stamp
        else:
            value.update(requested_monotonic_ns=stamp, deadline_monotonic_ns=stamp + GRACE_NS)
        return value

    def test_stop_before_import_constructor_or_ready_is_consumed_before_scenario(self):
        for mode in ("before_import", "before_constructor", "before_ready"):
            with self.subTest(mode=mode):
                root = self.root / mode
                process = self.spawn(mode, root=root)
                manager = self.manager(process, root=root)
                self.wait_for(lambda: (root / "gated").exists(), process)
                with patch(JOBS + ".threading.Thread") as thread, \
                     patch.object(process, "send_signal", wraps=process.send_signal) as send:
                    manager.request_stop(JOB_ID)
                    intent_path = manager.folder / "cancel-intent.json"
                    original = intent_path.read_bytes()
                    intent = read_handshake(intent_path, "cancel", JOB_ID, GENERATION, manager.worker_identity)
                    self.assertEqual(intent["deadline_monotonic_ns"] - intent["requested_monotonic_ns"], GRACE_NS)
                    self.assertEqual(manager.cancel_deadline_ns, intent["deadline_monotonic_ns"])
                    manager.request_stop(JOB_ID)
                    manager._begin_cancel()
                    self.assertEqual(intent_path.read_bytes(), original)
                    self.assertFalse((manager.folder / "worker-ready.json").exists())
                    self.assertFalse((root / "first_variant").exists())
                    send.assert_not_called()
                    thread.assert_called_once()
                    time.sleep(.03)
                    self.assertIsNone(process.poll())
                    (root / "release").touch()
                    self.assertEqual(process.wait(timeout=5), 130)
                    ready = read_handshake(manager.folder / "worker-ready.json", "ready", JOB_ID,
                                           GENERATION, manager.worker_identity)
                    self.assertIsNotNone(ready)
                    manager._stop_sync()
                    send.assert_not_called()
                self.assertEqual(manager.current["status"], "cancelled")
                self.assertTrue(manager.current["cleanup_clean"])
                self.assertIsNone(manager.current["error"])
                self.assertIsNone(manager.process)
                self.assertFalse((root / "first_variant").exists())
                self.assertFalse((root / "second_variant").exists())

    def test_ready_worker_gets_one_sigint_after_intent_persistence(self):
        process = self.spawn("ready")
        manager = self.manager(process)
        self.wait_for(lambda: (self.root / "first_variant").exists(), process)
        original_send = process.send_signal
        delivered = []
        def send(sig):
            intent = read_handshake(manager.folder / "cancel-intent.json", "cancel", JOB_ID,
                                    GENERATION, manager.worker_identity)
            self.assertIsNotNone(intent)
            delivered.append(sig)
            original_send(sig)
        with patch(JOBS + ".threading.Thread"), patch.object(process, "send_signal", side_effect=send):
            manager.request_stop(JOB_ID)
            (self.root / "advance").touch()
            original = (manager.folder / "cancel-intent.json").read_bytes()
            manager.request_stop(JOB_ID)
            manager._begin_cancel()
            manager._deliver_cancel()
            manager._stop_sync()
        self.assertEqual(delivered, [signal.SIGINT])
        self.assertEqual((manager.folder / "cancel-intent.json").read_bytes(), original)
        self.assertEqual(manager.current["status"], "cancelled")
        self.assertTrue(manager.current["cleanup_clean"])
        self.assertFalse((self.root / "second_variant").exists())
        self.assertEqual(process.returncode, 130)

    def test_records_bind_schema_job_generation_pid_and_process_start(self):
        identity = dict(pid=os.getpid(), start_ticks=1)
        path = self.root / "record.json"
        now = time.monotonic_ns()
        for kind in ("ready", "cancel"):
            valid = self.record(kind, identity, stamp=now)
            atomic_json(path, valid)
            self.assertEqual(read_handshake(path, kind, JOB_ID, GENERATION, identity, now_ns=now), valid)
            changes = dict(schema_version=True, kind="other", job_id="c" * 32,
                           generation="d" * 32, pid=identity["pid"] + 1, start_ticks=2,
                           extra="unexpected")
            for field, value in changes.items():
                with self.subTest(kind=kind, field=field):
                    atomic_json(path, dict(valid, **{field: value}))
                    with self.assertRaises(ValueError):
                        read_handshake(path, kind, JOB_ID, GENERATION, identity, now_ns=now)
            with self.subTest(kind=kind, field="missing_identity"):
                atomic_json(path, valid)
                with self.assertRaises(ValueError):
                    read_handshake(path, kind, JOB_ID, GENERATION, None, now_ns=now)
            stamp_field = "ready_monotonic_ns" if kind == "ready" else "requested_monotonic_ns"
            for invalid in (0, True, now + 1):
                with self.subTest(kind=kind, stamp=invalid):
                    atomic_json(path, dict(valid, **{stamp_field: invalid}))
                    with self.assertRaises(ValueError):
                        read_handshake(path, kind, JOB_ID, GENERATION, identity, now_ns=now)
        cancel = self.record("cancel", identity, stamp=now)
        cancel["deadline_monotonic_ns"] += 1
        atomic_json(path, cancel)
        with self.assertRaises(ValueError):
            read_handshake(path, "cancel", JOB_ID, GENERATION, identity, now_ns=now)
        path.unlink()
        self.assertIsNone(read_handshake(path, "ready", JOB_ID, GENERATION, identity))

    def test_invalid_oversized_and_nonregular_records_are_bounded(self):
        identity = dict(pid=os.getpid(), start_ticks=1)
        path = self.root / "record.json"
        for raw in (b"{", b"[]", b"null", b"x" * (IPC_LIMIT + 1)):
            with self.subTest(raw=raw[:12]):
                path.write_bytes(raw)
                with self.assertRaises(ValueError):
                    read_handshake(path, "ready", JOB_ID, GENERATION, identity)
        link = self.root / "link.json"
        link.symlink_to(path)
        for candidate in (link, self.root):
            with self.subTest(path=candidate):
                with self.assertRaises((ValueError, OSError)):
                    read_handshake(candidate, "ready", JOB_ID, GENERATION, identity)
        fifo = self.root / "record.fifo"
        os.mkfifo(fifo)
        script = textwrap.dedent("""
            import sys
            from mujoco_ros2_manipulation.operator_console.demo_handshake import read_handshake
            try:
                read_handshake(sys.argv[1], 'ready', 'a' * 32, 'b' * 32, dict(pid=2, start_ticks=1))
            except (ValueError, OSError):
                raise SystemExit(0)
            raise SystemExit('FIFO accepted')
        """)
        with (self.root / "fifo-reader.log").open("wb") as stream:
            result = subprocess.run([sys.executable, "-c", script, str(fifo)],
                                    cwd=Path(__file__).resolve().parents[2], timeout=3,
                                    stdout=stream, stderr=subprocess.STDOUT)
        self.assertEqual(result.returncode, 0)

    def test_worker_accepts_only_one_intent_and_never_extends_original_deadline(self):
        for signal_first in (False, True):
            with self.subTest(signal_first=signal_first):
                root = self.root / str(signal_first)
                console = root / "runtime/operator/session"
                job = DemoJob(console / "demos" / JOB_ID, "contact_comparison", 91,
                              console, render=False, root=root, generation=GENERATION)
                job._cancel_stacks.start = Mock()
                self.addCleanup(job._cancel_stacks.close)
                requested = time.monotonic_ns() - 2_000_000_000
                intent = self.record("cancel", job.worker_identity, stamp=requested)
                if signal_first:
                    job.interrupt()
                atomic_json(job.directory / "cancel-intent.json", intent)
                with self.assertRaises(DemoCancelled):
                    job.check_cancel()
                self.assertEqual(job.cancel_deadline_ns, requested + GRACE_NS)
                job.interrupt()
                with self.assertRaises(DemoCancelled):
                    job.check_cancel()
                self.assertEqual(job.cancel_intent, intent)
                self.assertEqual(job.cancel_deadline_ns, requested + GRACE_NS)
                job._cancel_stacks.start.assert_called_once()
                replacement = dict(intent, requested_monotonic_ns=requested + 1,
                                   deadline_monotonic_ns=requested + GRACE_NS + 1)
                atomic_json(job.directory / "cancel-intent.json", replacement)
                with self.assertRaisesRegex(ValueError, "changed after acceptance"):
                    job.check_cancel()
                self.assertEqual(job.cancel_intent, intent)
                self.assertEqual(job.cancel_deadline_ns, requested + GRACE_NS)

    def test_readiness_wait_consumes_twenty_seconds_of_original_parent_budget(self):
        process = Mock(pid=4242, returncode=None)
        process.poll.side_effect = lambda: process.returncode
        manager = self.manager(process)
        manager.worker_identity = dict(pid=4242, start_ticks=1)
        now = [100_000_000_000]
        def become_ready(_seconds):
            now[0] += 20_000_000_000
            atomic_json(manager.folder / "worker-ready.json",
                        self.record("ready", manager.worker_identity, stamp=now[0]))
        def wait(*, timeout):
            process.returncode = 130
            return 130
        process.wait.side_effect = wait
        manager.refresh_for_cleanup = Mock(side_effect=lambda: manager.current.update(
            status="cancelled", error=dict(type="Cancelled")))
        with patch(JOBS + ".time.monotonic_ns", side_effect=lambda: now[0]), \
             patch(JOBS + ".time.sleep", side_effect=become_ready), \
             patch(JOBS + ".matching_process", return_value=True), \
             patch(JOBS + ".group_gone", return_value=True), \
             patch(JOBS + ".stop_process") as stop:
            manager._begin_cancel()
            original = (manager.folder / "cancel-intent.json").read_bytes()
            manager._stop_sync()
        process.wait.assert_called_once_with(timeout=25.0)
        process.send_signal.assert_called_once_with(signal.SIGINT)
        stop.assert_not_called()
        self.assertEqual(manager.cancel_deadline_ns, 145_000_000_000)
        self.assertEqual((manager.folder / "cancel-intent.json").read_bytes(), original)
        self.assertEqual(manager.current["status"], "cancelled")

    def test_matching_race_accepts_confirmed_exit_but_rejects_live_pid_reuse(self):
        for second_poll in (130, None):
            with self.subTest(second_poll=second_poll):
                process = Mock(pid=4242)
                process.poll.side_effect = [None, second_poll]
                manager = self.manager(process)
                manager.worker_identity = dict(pid=4242, start_ticks=1)
                atomic_json(manager.folder / "worker-ready.json", self.record("ready", manager.worker_identity))
                with patch(JOBS + ".matching_process", return_value=False):
                    manager._deliver_cancel()
                self.assertEqual(process.poll.call_count, 2)
                process.send_signal.assert_not_called()
                self.assertFalse(manager.cancel_signal_sent)
                if second_poll == 130:
                    self.assertIsNone(manager.monitor_error)
                else:
                    self.assertIn("identity changed", manager.monitor_error)


if __name__ == "__main__":
    unittest.main()
