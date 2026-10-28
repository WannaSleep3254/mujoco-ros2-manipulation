"""Cancel only outside ROS response conversion, without hiding real failures."""
import ast
import json
from pathlib import Path
import tempfile
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from mujoco_ros2_manipulation.operator_console.demo_worker import DemoCancelled, DemoJob


SOURCE = Path(__file__).resolve().parents[2] / "src/mujoco_ros2_manipulation/verification/ros_checks.py"


def waiter(ros, cancel_check):
    """Exercise real wait methods without importing ROS or launching processes."""
    tree = ast.parse(SOURCE.read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "Verification")
    methods = [node for node in cls.body if isinstance(node, ast.FunctionDef)
               and node.name in ("check_cancel", "spin_once", "until")]
    namespace = dict(rclpy=ros, time=time)
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(SOURCE), "exec"), namespace)
    instance = type("Waiter", (), {node.name: namespace[node.name] for node in methods})()
    instance.cancel_check = cancel_check
    return instance


class CooperativeCancelTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        console = self.root / "runtime/operator/session"
        self.job = DemoJob(console / "demos" / ("a" * 32), "contact_comparison", 91,
                           console, render=False, root=self.root)
        self.job._cancel_stacks.start = Mock()
        self.addCleanup(self.job._cancel_stacks.close)

    def test_cancel_during_response_decode_waits_for_successful_python_return(self):
        calls = []

        def decode(_node, **_options):
            calls.append("decode entered")
            # Model the native boundary which converts an asynchronous Python
            # exception to a generic conversion failure. The handler must return.
            try:
                self.job.interrupt()
            except BaseException as error:
                raise RuntimeError("Unable to convert call argument to Python object") from error
            calls.append("decode returned")

        node = waiter(SimpleNamespace(ok=lambda: True, spin_once=decode), self.job.check_cancel)
        predicate = Mock(return_value=True)
        with self.assertRaises(DemoCancelled):
            node.until(predicate, 1, "response")
        self.assertEqual(calls, ["decode entered", "decode returned"])
        predicate.assert_not_called()

    def test_pending_cancel_does_not_enter_another_ros_callback(self):
        self.job.interrupt()
        ros = SimpleNamespace(ok=lambda: True, spin_once=Mock())
        node = waiter(ros, self.job.check_cancel)
        with self.assertRaises(DemoCancelled):
            node.until(lambda: True, 1, "response")
        ros.spin_once.assert_not_called()

    def test_real_decoder_failure_wins_over_a_concurrent_cancel_flag(self):
        original = RuntimeError("genuine decoder failure")

        def decode(_node, **_options):
            self.job.interrupt()
            raise original

        node = waiter(SimpleNamespace(ok=lambda: True, spin_once=decode), self.job.check_cancel)
        with self.assertRaises(RuntimeError) as caught:
            node.until(lambda: True, 1, "response")
        self.assertIs(caught.exception, original)
        self.assertTrue(self.job.cancel_requested)

    def test_predicate_failure_wins_if_cancel_arrives_in_the_predicate(self):
        def predicate():
            self.job.interrupt()
            raise RuntimeError("invalid observed state")

        node = waiter(SimpleNamespace(ok=lambda: True, spin_once=Mock()), self.job.check_cancel)
        with self.assertRaisesRegex(RuntimeError, "invalid observed state"):
            node.until(predicate, 1, "state")

    def test_cancel_during_a_successful_predicate_cannot_complete_the_wait(self):
        def predicate():
            self.job.interrupt()
            return True

        node = waiter(SimpleNamespace(ok=lambda: True, spin_once=Mock()), self.job.check_cancel)
        with self.assertRaises(DemoCancelled):
            node.until(predicate, 1, "state")

    def test_normal_wait_and_timeout_still_work_without_a_cancel_hook(self):
        node = waiter(SimpleNamespace(ok=lambda: True, spin_once=Mock()), None)
        node.until(lambda: True, 1, "ready")
        with self.assertRaisesRegex(RuntimeError, "Timeout: missing"):
            node.until(lambda: False, 0, "missing")

    def verify(self, code, value, *, cancel_requested=True):
        ros = ModuleType("rclpy")
        ros.init = Mock(); ros.ok = Mock(return_value=False); ros.shutdown = Mock()
        signals = ModuleType("rclpy.signals")
        signals.SignalHandlerOptions = SimpleNamespace(NO=0)
        services = ModuleType("std_srvs.srv"); services.Trigger = object
        modules = {"rclpy": ros, "rclpy.signals": signals,
                   "std_srvs": ModuleType("std_srvs"), "std_srvs.srv": services}

        def run(*args, **kwargs):
            node = args[3](SimpleNamespace(), progress=Mock())
            self.assertIs(node.cancel_check.__self__, self.job)
            node.cancel_check()
            self.job.cancel_requested = cancel_requested
            if value is not None:
                path = self.root / "nested-result.json"
                path.write_text(json.dumps(value))
                self.job.active_report = SimpleNamespace(path=path)
            return code

        with patch.dict("sys.modules", modules), \
             patch("mujoco_ros2_manipulation.verification.runner.run_verification", side_effect=run):
            return self.job._run_verifier("contact", SimpleNamespace(),
                lambda *_args: SimpleNamespace(), Mock(), [], Mock(), "test_")

    @staticmethod
    def report(status, error=None, **changes):
        return dict(run_id="nested-run", status=status, error=error,
                    shutdown_clean=True, process_group_gone=True, **changes)

    def test_nested_failure_is_not_relabelled_by_concurrent_cancellation(self):
        value = self.report("failed", dict(type="RuntimeError", message="original conversion failure"))
        with self.assertRaisesRegex(RuntimeError, "original conversion failure"):
            self.verify(1, value)
        self.assertEqual(self.job.data["results"][0]["status"], "failed")
        self.assertEqual(self.job.data["results"][0]["error"], value["error"])

    def test_clean_interruption_preserves_the_nested_cancel_evidence(self):
        value = self.report("interrupted", dict(type="DemoCancelled", message="stop"))
        with self.assertRaises(DemoCancelled):
            self.verify(130, value)
        self.assertEqual(self.job.data["results"][0]["status"], "interrupted")

    def test_cancel_return_code_cannot_hide_unclean_shutdown_or_a_non_cancel_error(self):
        for field, replacement in (("shutdown_clean", False), ("process_group_gone", False),
                                   ("error", dict(type="RuntimeError", message="real failure"))):
            with self.subTest(field=field):
                self.job.cancel_requested = False
                value = self.report("interrupted", dict(type="DemoCancelled", message="stop"))
                value[field] = replacement
                with self.assertRaises(RuntimeError):
                    self.verify(130, value)

    def test_cancel_after_a_successful_run_still_prevents_the_next_variant(self):
        with self.assertRaises(DemoCancelled):
            self.verify(0, self.report("passed"))
        self.assertEqual(self.job.data["results"][0]["status"], "passed")

    def test_cancel_exit_without_a_linked_report_is_not_verified_cancellation(self):
        with self.assertRaisesRegex(RuntimeError, "Verification failed before launch"):
            self.verify(130, None)

    def test_sigint_at_popen_return_saves_owned_launch_before_cooperative_cleanup(self):
        output = self.root / "models/cell"
        output.mkdir(parents=True)
        (output / "source.json").write_text("{}")
        cell = SimpleNamespace(id="fr5_ur5e", label="test cell", fingerprint="hash", output=output)
        ros = ModuleType("rclpy")
        ros.init = Mock(); ros.ok = Mock(return_value=False); ros.shutdown = Mock()
        signals = ModuleType("rclpy.signals"); signals.SignalHandlerOptions = SimpleNamespace(NO=0)
        services = ModuleType("std_srvs.srv"); services.Trigger = object
        modules = {"rclpy": ros, "rclpy.signals": signals,
                   "std_srvs": ModuleType("std_srvs"), "std_srvs.srv": services}
        process = Mock(pid=42, returncode=0)
        process.poll.return_value = None
        node_factory = Mock()
        cleanup_snapshots = []

        def launch(*_args, **_kwargs):
            self.job.interrupt()
            return process

        def cleanup(_process):
            cleanup_snapshots.append(json.loads(self.job.path.read_text()))
            return False

        runner = "mujoco_ros2_manipulation.verification.runner."
        with patch.dict("sys.modules", modules), patch.dict("os.environ"), \
             patch(runner + "subprocess.Popen", side_effect=launch), \
             patch(runner + "stop_process", side_effect=cleanup) as stop, \
             patch(runner + "group_gone", return_value=True), \
             patch("mujoco_ros2_manipulation.operator_console.demo_worker.group_gone", return_value=True), \
             patch("mujoco_ros2_manipulation.operator_console.demo_worker.read_process_start_ticks", return_value=123), \
             patch("builtins.print"):
            with self.assertRaises(DemoCancelled):
                self.job._run_verifier("contact", cell, node_factory, Mock(), [], None, "test_")
        ros.init.assert_not_called()
        node_factory.assert_not_called()
        stop.assert_called_once_with(process)
        saved = cleanup_snapshots[0]
        self.assertEqual(saved["stage"], "launching")
        self.assertEqual(saved["active_launch"]["pid"], 42)
        self.assertEqual(saved["active_launch"]["start_ticks"], 123)
        self.assertEqual(saved["active_launch"]["report_path"], str(self.job.active_report.path))
        nested = json.loads(self.job.active_report.path.read_text())
        self.assertEqual(nested["status"], "interrupted")
        self.assertEqual(nested["error"]["type"], "DemoCancelled")
        self.assertTrue(nested["shutdown_clean"])
        self.assertTrue(nested["process_group_gone"])
        self.assertEqual(self.job.data["results"][0]["run_id"], nested["run_id"])


if __name__ == "__main__":
    unittest.main()
