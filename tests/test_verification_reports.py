"""Lifecycle regressions without a ROS installation or live child processes."""

import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from verification_reports import VerificationReport, atomic_json
from verification_runner import run_verification, stop_process


class ReportTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def test_older_completion_cannot_replace_newer_latest_result(self):
        older = VerificationReport(self.root, 'fr5', 'headless', 90)
        newer = VerificationReport(self.root, 'fr5', 'headless', 91)
        older.update(status='passed')
        self.assertEqual(json.loads(newer.latest.read_text())['run_id'], newer.run_id)
        self.assertEqual(json.loads(older.path.read_text())['status'], 'passed')
        newer.update(status='failed')
        self.assertEqual(json.loads(newer.latest.read_text())['status'], 'failed')

    def test_nonfinite_serialization_cannot_replace_valid_json(self):
        path = self.root / 'result.json'
        atomic_json(path, {'status': 'failed'})
        with self.assertRaises(ValueError):
            atomic_json(path, {'value': float('nan')})
        self.assertEqual(json.loads(path.read_text()), {'status': 'failed'})
        self.assertEqual(list(self.root.iterdir()), [path])


class LifecycleTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.args = SimpleNamespace(robot='fr5', gui=False, domain=90,
                                    stop_mode='interrupt', screenshot_dir=None)
        output = self.root / 'models/fr5'
        output.mkdir(parents=True)
        (output / 'source.json').write_text('{}')
        self.profile = SimpleNamespace(id='fr5', label='FR5', fingerprint='test-hash',
                                       output=output, ros={'planning_group': 'arm'})
        self.ros = Mock()
        self.ros.ok.return_value = False
        self.node = Mock()
        self.node.run.return_value = {'joints': ['j1']}
        self.process = Mock(pid=12345678, returncode=0)
        self.process.poll.return_value = None

    def run_case(self, *, preflight=None, spawn=None):
        with patch('verification_runner.load_profile', return_value=self.profile), \
             patch('verification_runner.check_generated_model', side_effect=preflight), \
             patch('verification_runner.subprocess.Popen', side_effect=spawn, return_value=self.process), \
             patch('verification_runner.stop_process', return_value=False), \
             patch('verification_runner.group_gone', return_value=True), \
             patch.dict(os.environ), patch('builtins.print'):
            code = run_verification(self.args, self.root, self.ros,
                                    lambda profile, progress: self.node,
                                    SimpleNamespace(Request=lambda: object()))
        mode = ('gui' if self.args.gui else 'headless') + ('_window' if self.args.stop_mode == 'window' else '')
        latest = self.root / 'runtime/validation' / f'fr5_{mode}.json'
        return code, json.loads(latest.read_text())

    def test_failed_new_run_replaces_old_success_and_preserves_history(self):
        code, first = self.run_case()
        self.assertEqual(code, 0)
        self.node.run.side_effect = RuntimeError('injected motion failure')
        code, second = self.run_case()
        self.assertEqual(code, 1)
        self.assertEqual(second['status'], 'failed')
        self.assertFalse(second['motion_checks_passed'])
        self.assertEqual(second['failure_stage'], 'motion')
        self.assertEqual(second['error']['message'], 'injected motion failure')
        self.assertNotEqual(first['run_id'], second['run_id'])
        saved = self.root / 'runtime/validation/runs' / first['run_id'] / 'result.json'
        self.assertEqual(json.loads(saved.read_text())['status'], 'passed')
        alias = self.root / 'runtime/validation/fr5_headless.log'
        self.assertEqual(alias.resolve(), Path(second['log_path']))

    def test_legacy_success_is_archived_before_failed_preflight(self):
        latest = self.root / 'runtime/validation/fr5_headless.json'
        latest.parent.mkdir(parents=True)
        previous = {'motion_checks_passed': True, 'shutdown_clean': True}
        latest.write_text(json.dumps(previous))
        code, report = self.run_case(preflight=ValueError('stale generated model'))
        self.assertEqual(code, 1)
        self.assertEqual(report['failure_stage'], 'preflight')
        self.assertFalse(report['launch_started'])
        self.assertFalse(report['motion_checks_passed'])
        self.ros.init.assert_not_called()
        backup, = (latest.parent / 'runs').glob('legacy-*/result.json')
        self.assertEqual(json.loads(backup.read_text()), previous)

    def test_spawn_failure_is_reported(self):
        code, report = self.run_case(spawn=OSError('cannot spawn launch'))
        self.assertEqual(code, 1)
        self.assertEqual(report['failure_stage'], 'launch')

    def test_interruption_is_reported_with_exit_130(self):
        self.node.run.side_effect = KeyboardInterrupt()
        code, report = self.run_case()
        self.assertEqual(code, 130)
        self.assertEqual(report['status'], 'interrupted')

    def test_cleanup_does_not_hide_primary_failure(self):
        self.node.run.side_effect = RuntimeError('original failure')
        self.node.destroy_node.side_effect = OSError('cleanup failure')
        code, report = self.run_case()
        self.assertEqual(code, 1)
        self.assertEqual(report['error']['message'], 'original failure')
        self.assertIn('cleanup failure', report['cleanup_errors'][0])
        self.assertFalse(report['shutdown_clean'])

    def test_cleanup_failure_prevents_success_after_motion_passes(self):
        self.node.destroy_node.side_effect = OSError('cleanup failure')
        code, report = self.run_case()
        self.assertEqual(code, 1)
        self.assertTrue(report['motion_checks_passed'])
        self.assertEqual(report['status'], 'failed')
        self.assertEqual(report['error']['type'], 'ShutdownError')

    def test_failed_report_cannot_be_collected_even_with_old_success_flags(self):
        import capture_robot_images
        latest = self.root / 'runtime/validation/fr5_gui.json'
        latest.parent.mkdir(parents=True)
        latest.write_text(json.dumps({'status': 'failed', 'motion_checks_passed': True,
                                     'shutdown_clean': True, 'process_group_gone': True}))
        with patch.object(capture_robot_images, 'ROOT', self.root):
            with self.assertRaisesRegex(ValueError, 'latest verification did not pass'):
                capture_robot_images.collect_assets(self.profile)

    def test_unresponsive_launch_escalates_to_sigkill(self):
        process = Mock(pid=12345678)
        process.poll.return_value = None
        process.wait.side_effect = [subprocess.TimeoutExpired('launch', 12),
                                    subprocess.TimeoutExpired('launch', 5), -9]
        with patch('verification_runner.os.killpg') as kill, \
             patch('verification_runner.group_gone', return_value=True):
            self.assertTrue(stop_process(process))
        self.assertEqual([call.args[1] for call in kill.call_args_list], [signal.SIGTERM, signal.SIGKILL])

    def test_gui_window_close_path_preserves_success(self):
        self.args.gui = True
        self.args.stop_mode = 'window'
        self.node.result.return_value = SimpleNamespace(success=True, message='closed')

        def spawn(*args, **kwargs):
            kwargs['stdout'].write('Ready to take commands for planning group arm.')
            kwargs['stdout'].flush()
            return self.process

        with patch('verification_runner.subprocess.run', return_value=SimpleNamespace(
                stdout='ros2_control_node /rviz2', stderr='')):
            code, report = self.run_case(spawn=spawn)
        self.assertEqual(code, 0)
        self.assertEqual(report['status'], 'passed')
        self.assertTrue(report['mujoco_gpu_process_seen'])
        self.assertTrue((Path(report['log_path']).parent / 'nvidia_smi.txt').is_file())
        self.node.create_client.assert_called_once()

    def test_gui_failure_still_records_motion_and_cleanup(self):
        self.args.gui = True
        code, report = self.run_case()  # No RViz-ready log is supplied.
        self.assertEqual(code, 1)
        self.assertEqual(report['failure_stage'], 'gui')
        self.assertTrue(report['motion_checks_passed'])
        self.assertTrue(report['shutdown_clean'])

    def test_capture_rejects_a_different_model_build(self):
        import capture_robot_images
        latest = self.root / 'runtime/validation/fr5_gui.json'
        latest.parent.mkdir(parents=True)
        latest.write_text(json.dumps({'status': 'passed', 'model_manifest_sha256': 'old-build'}))
        with patch.object(capture_robot_images, 'ROOT', self.root), \
             patch.object(capture_robot_images, 'check_generated_model'):
            with self.assertRaisesRegex(ValueError, 'model changed since verification'):
                capture_robot_images.collect_assets(self.profile)
