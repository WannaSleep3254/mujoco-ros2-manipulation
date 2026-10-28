"""Failed cleanup must leave the worker evidence needed to diagnose it."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.operator_console.verify_demos import archive_console_evidence


class DemoEvidenceTests(unittest.TestCase):
    def fixture(self, root, events):
        session = root / "session"
        job = session / "demos" / ("a" * 32)
        job.mkdir(parents=True)
        (job / "status.json").write_text('{"status":"cancel_requested"}')
        (job / "worker.log").write_text("DemoCancelled\n")
        (job / "cancel-stacks.log").write_text("blocked in destroy_node\n")
        if events is not None:
            (session / "events.jsonl").write_text("\n".join(json.dumps(e) for e in events))
        return session, job.relative_to(session)

    def test_failed_cleanup_keeps_status_logs_and_stack(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            session, job = self.fixture(root, [dict(action="demo_finished", cleanup_clean=False)])
            target = root / "archive"
            evidence, errors = archive_console_evidence(session, target)
            self.assertTrue(evidence["copied"])
            self.assertFalse(evidence["audit_valid"])
            self.assertIn("forced or incomplete cleanup", errors[0])
            self.assertEqual((target / job / "cancel-stacks.log").read_text(), "blocked in destroy_node\n")
            self.assertEqual((target / job / "worker.log").read_text(), "DemoCancelled\n")
            self.assertEqual(json.loads((target / job / "status.json").read_text())["status"], "cancel_requested")

    def test_missing_audit_does_not_discard_worker_evidence(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            session, job = self.fixture(root, None)
            target = root / "archive"
            evidence, errors = archive_console_evidence(session, target)
            self.assertTrue(evidence["copied"])
            self.assertFalse(evidence["audit_valid"])
            self.assertTrue(errors)
            self.assertTrue((target / job / "worker.log").is_file())

    def test_live_child_remains_a_failure_after_evidence_is_copied(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            session, _ = self.fixture(root, [dict(action="process_started", pid=42)])
            target = root / "archive"
            with patch("mujoco_ros2_manipulation.operator_console.verify_demos.group_gone", return_value=False):
                evidence, errors = archive_console_evidence(session, target)
            self.assertTrue(evidence["copied"])
            self.assertFalse(evidence["audit_valid"])
            self.assertIn("child process group remains", errors[0])

    def test_copy_failure_does_not_hide_cleanup_failure(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            session, _ = self.fixture(root, [dict(action="demo_finished", cleanup_clean=False)])
            with patch("mujoco_ros2_manipulation.operator_console.verify_demos.shutil.copytree",
                       side_effect=OSError("archive unavailable")):
                evidence, errors = archive_console_evidence(session, root / "archive")
            self.assertFalse(evidence["copied"])
            self.assertEqual(len(errors), 2)
            self.assertIn("archive unavailable", errors[0])
            self.assertIn("forced or incomplete cleanup", errors[1])

    def test_successful_cleanup_retains_valid_audit(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            session, _ = self.fixture(root, [dict(action="demo_finished", cleanup_clean=True)])
            evidence, errors = archive_console_evidence(session, root / "archive")
            self.assertEqual(errors, [])
            self.assertTrue(evidence["copied"])
            self.assertTrue(evidence["audit_valid"])
            self.assertEqual(evidence["event_count"], 1)
