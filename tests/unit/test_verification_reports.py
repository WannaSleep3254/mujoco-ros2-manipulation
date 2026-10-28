"""Lifecycle regressions without a ROS installation or live child processes."""

import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from mujoco_ros2_manipulation.verification.reports import VerificationReport, atomic_json
from mujoco_ros2_manipulation.verification.runner import run_verification, stop_process


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

    def run_case(self, *, preflight=None, spawn=None, finalize=None, expected_crash=None, stop=None):
        with patch('mujoco_ros2_manipulation.verification.runner.load_profile', return_value=self.profile), \
             patch('mujoco_ros2_manipulation.verification.runner.check_generated_model', side_effect=preflight), \
             patch('mujoco_ros2_manipulation.verification.runner.subprocess.Popen', side_effect=spawn, return_value=self.process), \
             patch('mujoco_ros2_manipulation.verification.runner.stop_process', side_effect=stop, return_value=False), \
             patch('mujoco_ros2_manipulation.verification.runner.group_gone', return_value=True), \
             patch.dict(os.environ), patch('builtins.print'):
            code = run_verification(self.args, self.root, self.ros,
                                    lambda profile, progress: self.node,
                                    SimpleNamespace(Request=lambda: object()), finalize=finalize, expected_crash=expected_crash)
        mode = ('gui' if self.args.gui else 'headless') + ('_window' if self.args.stop_mode == 'window' else '')
        scene = getattr(self.args, 'scene', 'default')
        report_id = 'fr5' if scene == 'default' else f'fr5_{scene}'
        latest = self.root / 'runtime/validation' / f'{report_id}_{mode}.json'
        return code, json.loads(latest.read_text())

    def test_only_explicit_exact_crash_can_pass(self):
        from mujoco_ros2_manipulation.verification.process_faults import ExpectedCrash
        crash = ExpectedCrash('move_group')
        crash.expected = dict(node='move_group-2', pid=4321, exit_code=-9)
        line = '[ERROR] [move_group-2]: process has died [pid 4321, exit code -9]'
        self.process.poll.return_value = 0

        def spawn(*args, **kwargs):
            kwargs['stdout'].write(line)
            kwargs['stdout'].flush()
            return self.process

        code, report = self.run_case(spawn=spawn, expected_crash=crash)
        self.assertEqual(code, 0)
        self.assertTrue(report['shutdown_clean'])
        self.assertEqual(report['expected_process_exits'], [crash.expected])
        self.assertEqual(report['shutdown_failures'], [])
        code, report = self.run_case(spawn=spawn)
        self.assertEqual(code, 1)
        self.assertFalse(report['shutdown_clean'])

        def late_crash(report, *_):
            with report.log_path.open('a') as log:
                log.write('\n[ERROR] [another-3]: process has died [pid 4322, exit code -11]')

        code, report = self.run_case(spawn=spawn, expected_crash=crash, finalize=late_crash)
        self.assertEqual(code, 1)
        self.assertFalse(report['shutdown_clean'])
        self.assertEqual(len(report['shutdown_failures']), 2)

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

    def test_scene_runs_preserve_independent_latest_results(self):
        _, original = self.run_case()
        self.args.scene = 'pedestal'
        code, raised = self.run_case()
        self.assertEqual(code, 0)
        self.assertEqual(raised['scene_id'], 'pedestal')
        path = self.root / 'runtime/validation/fr5_headless.json'
        self.assertEqual(json.loads(path.read_text())['run_id'], original['run_id'])
        self.assertNotEqual(original['run_id'], raised['run_id'])

    def test_workcell_adapter_preserves_preflight_failure_and_report_namespace(self):
        with patch('builtins.print'):
            code=run_verification(self.args,self.root,self.ros,lambda *args,**kwargs:None,None,
                profile_loader=lambda *args,**kwargs:self.profile,
                validator=Mock(side_effect=ValueError('stale workcell inputs')),
                report_prefix='workcell_')
        self.assertEqual(code,1)
        report=json.loads((self.root/'runtime/validation/workcell_fr5_headless.json').read_text())
        self.assertEqual(report['failure_stage'],'preflight')
        self.assertEqual(report['error']['message'],'stale workcell inputs')
        self.assertFalse(report['launch_started'])
        self.assertFalse((self.root/'runtime/validation/fr5_headless.json').exists())

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

    def test_cleanup_boundaries_preserve_call_order_and_completed_timings(self):
        observed = []
        self.ros.ok.return_value = True

        def observe(name):
            report = json.loads((self.root / 'runtime/validation/fr5_headless.json').read_text())
            steps = report['cleanup_diagnostics']['steps']
            self.assertEqual(steps[-1]['name'], name)
            self.assertEqual(steps[-1]['status'], 'running')
            self.assertIsNone(steps[-1]['completed_utc'])
            self.assertIsNone(steps[-1]['elapsed_sec'])
            observed.append(name)

        self.node.destroy_node.side_effect = lambda: observe('destroy_node')
        self.ros.shutdown.side_effect = lambda: observe('ros_shutdown')
        code, report = self.run_case(stop=lambda _: observe('launch_stop'),
                                     finalize=lambda *_: observe('finalize'))
        names = ['destroy_node', 'ros_shutdown', 'launch_stop', 'finalize']
        self.assertEqual(code, 0)
        self.assertEqual(observed, names)
        self.assertEqual(report['cleanup_diagnostics']['schema_version'], 1)
        self.assertEqual([step['name'] for step in report['cleanup_diagnostics']['steps']], names)
        for step in report['cleanup_diagnostics']['steps']:
            self.assertEqual(step['status'], 'completed')
            self.assertIsNone(step['error'])
            self.assertIsInstance(step['started_utc'], str)
            self.assertIsInstance(step['completed_utc'], str)
            self.assertIsInstance(step['started_monotonic_ns'], int)
            self.assertIsInstance(step['completed_monotonic_ns'], int)
            self.assertGreaterEqual(step['elapsed_sec'], 0)
            self.assertEqual(step['elapsed_sec'],
                (step['completed_monotonic_ns'] - step['started_monotonic_ns']) / 1e9)

    def test_blocked_destroy_leaves_running_boundary_and_primary_failure_on_disk(self):
        entered, release = threading.Event(), threading.Event()
        result, errors = [], []
        self.node.run.side_effect = RuntimeError('primary motion failure')

        def blocked_destroy():
            entered.set()
            if not release.wait(5):
                raise RuntimeError('Test did not release destroy_node')

        def run():
            try:
                result.append(self.run_case())
            except BaseException as error:
                errors.append(error)

        self.node.destroy_node.side_effect = blocked_destroy
        worker = threading.Thread(target=run)
        worker.start()
        try:
            self.assertTrue(entered.wait(3))
            report = json.loads((self.root / 'runtime/validation/fr5_headless.json').read_text())
            self.assertEqual(report['error']['message'], 'primary motion failure')
            self.assertEqual(report['failure_stage'], 'motion')
            self.assertIsNone(report['completed_utc'])
            steps = report['cleanup_diagnostics']['steps']
            self.assertEqual(len(steps), 1)
            self.assertEqual(steps[0]['name'], 'destroy_node')
            self.assertEqual(steps[0]['status'], 'running')
            self.assertIsNone(steps[0]['completed_monotonic_ns'])
        finally:
            release.set()
            worker.join(5)
        self.assertFalse(worker.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(result[0][0], 1)
        self.assertEqual(result[0][1]['error']['message'], 'primary motion failure')

    def test_cleanup_errors_are_separate_from_original_failure(self):
        self.node.run.side_effect = RuntimeError('original motion failure')
        self.node.destroy_node.side_effect = OSError('node cleanup failed')
        self.ros.ok.return_value = True
        self.ros.shutdown.side_effect = RuntimeError('ROS cleanup failed')
        code, report = self.run_case(stop=Mock(side_effect=OSError('launch cleanup failed')),
            finalize=Mock(side_effect=ValueError('physics unavailable')))
        self.assertEqual(code, 1)
        self.assertEqual(report['failure_stage'], 'motion')
        self.assertEqual(report['error'], dict(type='RuntimeError', message='original motion failure'))
        self.assertFalse(report['shutdown_clean'])
        self.assertEqual(len(report['cleanup_errors']), 3)
        self.assertEqual(report['diagnostics_error'], 'physics unavailable')
        steps = report['cleanup_diagnostics']['steps']
        self.assertEqual([step['status'] for step in steps], ['failed'] * 4)
        self.assertEqual([step['error']['type'] for step in steps],
                         ['OSError', 'RuntimeError', 'OSError', 'ValueError'])
        self.assertTrue(all(step['completed_utc'] is not None for step in steps))

    def test_cleanup_diagnostics_preserve_interruption_and_skipped_steps(self):
        self.node.run.side_effect = KeyboardInterrupt('user cancellation')
        code, report = self.run_case()
        self.assertEqual(code, 130)
        self.assertEqual(report['status'], 'interrupted')
        self.assertEqual(report['failure_stage'], 'motion')
        self.assertEqual(report['error']['message'], 'user cancellation')
        self.assertEqual([step['status'] for step in report['cleanup_diagnostics']['steps']],
                         ['completed', 'skipped', 'completed', 'skipped'])
        self.assertTrue(report['shutdown_clean'])
        self.ros.shutdown.assert_not_called()
        self.node.destroy_node.reset_mock()
        code, report = self.run_case(preflight=ValueError('bad model'))
        self.assertEqual(code, 1)
        self.assertEqual(report['failure_stage'], 'preflight')
        self.assertEqual([step['status'] for step in report['cleanup_diagnostics']['steps']],
                         ['skipped'] * 4)
        self.node.destroy_node.assert_not_called()

    def test_diagnostic_write_failure_does_not_skip_cleanup_or_replace_primary_error(self):
        self.node.run.side_effect = RuntimeError('original motion failure')
        original_update = VerificationReport.update
        rejected = []

        def update(report, **fields):
            if 'cleanup_diagnostics' in fields and not rejected:
                rejected.append(True)
                raise OSError('diagnostic write unavailable')
            return original_update(report, **fields)

        with patch.object(VerificationReport, 'update', new=update):
            code, report = self.run_case()
        self.assertEqual(code, 1)
        self.assertEqual(report['failure_stage'], 'motion')
        self.assertEqual(report['error']['message'], 'original motion failure')
        self.node.destroy_node.assert_called_once()
        self.assertTrue(report['shutdown_clean'])
        self.assertEqual(report['cleanup_errors'], [])
        self.assertEqual(report['cleanup_diagnostics']['write_errors'][0]['message'],
                         'diagnostic write unavailable')

    def test_missing_physics_report_prevents_success_after_motion(self):
        code, report = self.run_case(finalize=Mock(side_effect=FileNotFoundError('physics.json missing')))
        self.assertEqual(code, 1)
        self.assertEqual(report['failure_stage'], 'physics_diagnostics')
        self.assertFalse(report['physics_checks_passed'])
        self.assertTrue(report['motion_checks_passed'])
        self.assertTrue(report['shutdown_clean'])

    def test_physics_failure_cannot_hide_primary_motion_failure(self):
        self.node.run.side_effect = RuntimeError('primary motion failure')
        code, report = self.run_case(finalize=Mock(side_effect=ValueError('contact found')))
        self.assertEqual(code, 1)
        self.assertEqual(report['error']['message'], 'primary motion failure')
        self.assertEqual(report['diagnostics_error'], 'contact found')

    def test_failed_report_cannot_be_collected_even_with_old_success_flags(self):
        from mujoco_ros2_manipulation.verification import images as capture_robot_images
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
        with patch('mujoco_ros2_manipulation.verification.runner.os.killpg') as kill, \
             patch('mujoco_ros2_manipulation.verification.runner.group_gone', return_value=True):
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

        with patch('mujoco_ros2_manipulation.verification.runner.subprocess.run', return_value=SimpleNamespace(
                stdout='ros2_control_node /rviz2', stderr='')):
            code, report = self.run_case(spawn=spawn)
        self.assertEqual(code, 0)
        self.assertEqual(report['status'], 'passed')
        self.assertTrue(report['mujoco_gpu_process_seen'])
        self.assertTrue((Path(report['log_path']).parent / 'nvidia_smi.txt').is_file())
        self.node.create_client.assert_called_once()

    def test_fast_motion_waits_for_delayed_rviz_readiness(self):
        self.args.gui = True
        def become_ready(predicate, *_):
            path = self.root / 'runtime/validation/fr5_gui.log'
            with path.open('a') as stream:
                stream.write('Ready to take commands for planning group arm.')
            self.assertTrue(predicate())
        self.node.until.side_effect = become_ready
        with patch('mujoco_ros2_manipulation.verification.runner.subprocess.run',
                   return_value=SimpleNamespace(stdout='ros2_control_node /rviz2', stderr='')):
            code, report = self.run_case()
        self.assertEqual(code, 0)
        self.assertEqual(report['status'], 'passed')

    def test_gui_failure_still_records_motion_and_cleanup(self):
        self.args.gui = True
        code, report = self.run_case()  # No RViz-ready log is supplied.
        self.assertEqual(code, 1)
        self.assertEqual(report['failure_stage'], 'gui')
        self.assertTrue(report['motion_checks_passed'])
        self.assertTrue(report['shutdown_clean'])

    def test_capture_rejects_a_different_model_build(self):
        from mujoco_ros2_manipulation.verification import images as capture_robot_images
        latest = self.root / 'runtime/validation/fr5_gui.json'
        latest.parent.mkdir(parents=True)
        latest.write_text(json.dumps({'status': 'passed', 'model_manifest_sha256': 'old-build'}))
        with patch.object(capture_robot_images, 'ROOT', self.root), \
             patch.object(capture_robot_images, 'check_generated_model'):
            with self.assertRaisesRegex(ValueError, 'model changed since verification'):
                capture_robot_images.collect_assets(self.profile)
