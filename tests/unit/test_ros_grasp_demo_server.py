"""ROS viewer admission, immutable native receipt times and owned fake jobs.

These HTTP/process fixtures deliberately perform no ROS, physics or robot work.
Native geometry and actual task execution have separate model/integration checks.
"""
from copy import deepcopy
import http.client
import io
import json
from pathlib import Path
import signal
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.grasping.ros_demo_server import RosDemoApplication, serve
from mujoco_ros2_manipulation.operator_console.recording import demo_recording
from mujoco_ros2_manipulation.operator_console.scene_host import _canonical
from mujoco_ros2_manipulation.verification.reports import atomic_json


MODEL, MANIFEST, PLUGIN = 'a' * 64, 'b' * 64, 'c' * 64


class FakeDisplay:
    def __init__(self, target, run_id=None):
        self.run_id = run_id
        recording = demo_recording()
        self.manifest = recording.manifest
        self.manifest['model_sha256'] = MODEL
        self.manifest_raw = _canonical(self.manifest)
        self.static_preview_pose = deepcopy(recording.frames[0]['pose'])
        self.static_preview_pose.update(model_sha256=MODEL, scene_sha256=self.manifest['scene_sha256'])

    def pose(self, observation):
        value = observation.payload
        if value['run_id'] != self.run_id or value['model_sha256'] != MODEL:
            raise ValueError('Fake fixture identity differs')
        return dict(self.static_preview_pose, geoms=deepcopy(value['geoms']))

    def asset(self, digest):
        if digest != 'd' * 64:
            raise KeyError('Unknown asset')
        return b'fixture-mesh'


class FakeTarget:
    def __init__(self, root):
        self.root, self.output, self.manifest_sha256 = root, root / 'runtime/model', MANIFEST

    def validate(self):
        return None


class FakeProcess:
    def __init__(self, index):
        self.pid, self.returncode, self.stdout = 1_000_000_000 + index, None, None

    def poll(self):
        return self.returncode


class RosOwnedJobTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.now, self.now_ns = 0., time.monotonic_ns()
        self.calls, self.processes, self.domains = [], [], []
        self.pin = {'native/grasp_ros2_plugin/src/observation.cpp': PLUGIN, 'fixture-source': 'e' * 64}
        for name, patcher in (
            ('runtime', patch('mujoco_ros2_manipulation.grasping.ros_demo_server.check_grasp_runtime')),
            ('pin_patch', patch.object(RosDemoApplication, '_pin', lambda app: dict(self.pin))),
            ('signals', patch('mujoco_ros2_manipulation.grasping.ros_demo_server.os.kill')),
            ('groups', patch('mujoco_ros2_manipulation.grasping.ros_demo_server._group_gone', return_value=True)),
            ('native_time', patch('mujoco_ros2_manipulation.grasping.ros_demo_server.time.monotonic_ns', lambda: self.now_ns)),
        ):
            setattr(self, name, patcher.start())
            self.addCleanup(patcher.stop)
        def spawn(command, **kwargs):
            self.calls.append((command, kwargs))
            child = FakeProcess(len(self.processes))
            self.processes.append(child)
            return child
        self.app = RosDemoApplication(FakeTarget(self.root), self.root / 'runtime/viewer', domain=222,
            clock=lambda: self.now, _process_factory=spawn, _domain_check=self.domains.append, _scene_builder=FakeDisplay)
        def finish():
            for child in self.processes:
                child.returncode = 1
            self.app.close()
        self.addCleanup(finish)

    def start(self, request='1' * 32):
        return self.app.start(dict(request_id=request, model_sha256=MODEL))

    def update(self, *, event=1, native_sequence=1, clock=1., observed=None, progressed=None, captured=None, native=True):
        job = self.app.current
        stamp = self.now_ns - 10_000_000 if observed is None else observed
        progress = stamp if progressed is None else progressed
        observation = dict(run_id=job.identifier, model_sha256=MODEL, plugin_sha256=PLUGIN,
            native_version='3.12.0', sequence=native_sequence, time_sec=clock, stamp_ns=str(round(clock * 1e9)),
            capture_monotonic_ns=str(stamp if captured is None else captured), capture_clock_semantics='linux_CLOCK_MONOTONIC',
            stamp_semantics='rounded_native_mjData_time', geoms=deepcopy(job.display.static_preview_pose['geoms']),
            contacts=[], arm_positions_rad={}, tcp={'position': [0., 0., 0.]}, block={'position': [0., 0., 0.]})
        value = dict(schema_version=1, kind='observation' if native else 'task', run_id=job.identifier,
            model_sha256=MODEL, plugin_sha256=PLUGIN, sequence=event, event_sequence=event,
            observed_monotonic_ns=stamp if native else None,
            clock_progress_observed_monotonic_ns=progress if native else None,
            native_clock_has_advanced=True, native_observation=observation if native else None,
            native_guard=None, task=dict(status='running', stage='approach', physical_grasp='unconfirmed',
                planning_attachment='world', action_terminal=None, cleanup=dict(status='not_started', clean=False)))
        (job.directory / 'run').mkdir(exist_ok=True)
        atomic_json(job.directory / 'run/latest.json', value)
        return value

    def terminal(self, *, status='succeeded', clean=True, code=0):
        job = self.app.current
        result = dict(schema_version=1, kind='ros-grasp-task-v1', run_id=job.identifier, model_sha256=MODEL,
            status=status, stage='complete', physical_grasp='released', planning_attachment='world',
            action_terminal=dict(status=4, error_code=0, stage='retract'), cleanup=dict(status='complete', clean=clean))
        (job.directory / 'run').mkdir(exist_ok=True)
        atomic_json(job.directory / 'run/result.json', result)
        job.process.returncode = code

    def test_preview_is_a_distinct_model_binding_with_no_live_job_or_ros_worker(self):
        state = self.app.snapshot()
        self.assertIsNone(state['job'])
        self.assertEqual(state['view']['mode'], 'model_preview')
        pose = self.app.pose_envelope(state['view']['binding_id'])
        self.assertEqual(pose['binding']['mode'], 'model_preview')
        self.assertIsNone(pose['source_state_age_sec'])
        self.assertEqual(self.calls, [])
        self.assertEqual(self.domains, [222])

    def test_start_is_idempotent_and_fixed_child_domain_is_explicit_after_source_environment(self):
        first = self.start()
        again = self.start()
        self.assertEqual(len(self.calls), 1)
        self.assertFalse(again['request_replay']['new_job_started'])
        self.assertEqual(first['job']['job_id'], again['job']['job_id'])
        command, options = self.calls[0]
        self.assertIn('source "$1"; shift; exec /usr/bin/python3', command[4])
        self.assertEqual(options['env']['ROS_DOMAIN_ID'], '222')
        self.assertEqual(options['env']['MANIPULATION_ROS_DOMAIN_ID'], '222')
        self.assertEqual(command[command.index('--domain') + 1], '222')
        self.assertTrue(options['start_new_session'])
        self.assertEqual(self.domains, [222, 222])
        with self.assertRaises(RuntimeError): self.start('2' * 32)

    def test_live_pose_carries_original_receipt_and_unchanged_task_updates_do_not_refresh_it(self):
        self.start()
        update = self.update()
        first = self.app.snapshot()
        self.assertEqual(first['view']['mode'], 'live')
        self.assertEqual(first['job']['freshness'], 'fresh')
        self.now_ns += 2_000_000_000
        update.update(kind='task', sequence=2, event_sequence=2)
        atomic_json(self.app.current.directory / 'run/latest.json', update)
        next_state = self.app.snapshot()
        self.assertGreater(next_state['job']['source_age_sec'], 2.)
        self.assertEqual(next_state['job']['freshness'], 'stale')
        pose = self.app.pose_envelope(next_state['view']['binding_id'])
        self.assertEqual(pose['sequence'], 1)
        self.assertGreater(pose['source_state_age_sec'], 2.)

    def test_held_native_clock_cannot_become_fresh_from_new_sequence_or_file_timestamp(self):
        self.start()
        first = self.update()
        self.app.snapshot()
        self.now_ns += 1_000_000_000
        self.update(event=2, native_sequence=2, clock=1., progressed=first['clock_progress_observed_monotonic_ns'])
        state = self.app.snapshot()
        self.assertGreater(state['job']['source_age_sec'], 1.)
        self.assertEqual(state['job']['freshness'], 'stale')
        self.assertIsNone(state['job']['error'])

    def test_queued_capture_keeps_its_original_age_in_status_and_pose_after_new_task_updates(self):
        self.start()
        update = self.update(captured=self.now_ns - 400_000_000)
        first = self.app.snapshot()
        self.assertAlmostEqual(first['job']['source_age_sec'], .4)
        self.assertEqual(first['job']['freshness'], 'fresh')
        self.now_ns += 200_000_000
        update.update(kind='task', sequence=2, event_sequence=2)
        atomic_json(self.app.current.directory / 'run/latest.json', update)
        later = self.app.snapshot()
        self.assertAlmostEqual(later['job']['source_age_sec'], .6)
        self.assertEqual(later['job']['freshness'], 'stale')
        pose = self.app.pose_envelope(later['view']['binding_id'])
        self.assertAlmostEqual(pose['source_state_age_sec'], .6)
        self.assertFalse(later['job']['cancel_requested'])

    def test_capture_after_its_callback_receipt_does_not_replace_last_valid_pose(self):
        self.start()
        self.update()
        first = self.app.snapshot()
        original = self.app.pose_envelope(first['view']['binding_id'])['pose']
        self.update(event=2, native_sequence=2, clock=1.05,
                    captured=self.now_ns - 9_999_999)
        failed = self.app.snapshot()
        self.assertTrue(failed['job']['cancel_requested'])
        self.assertEqual(self.app.pose_envelope(first['view']['binding_id'])['pose'], original)
        self.signals.assert_called_once_with(self.processes[0].pid, signal.SIGINT)

    def test_bad_identity_does_not_replace_last_pose_and_requests_only_owned_cancellation(self):
        self.start()
        first = self.update()
        state = self.app.snapshot()
        binding = state['view']['binding_id']
        original = self.app.pose_envelope(binding)['pose']
        wrong = dict(first, sequence=2, run_id='f' * 32)
        atomic_json(self.app.current.directory / 'run/latest.json', wrong)
        failed = self.app.snapshot()
        self.assertTrue(failed['job']['cancel_requested'])
        self.assertEqual(failed['view']['mode'], 'live')
        self.assertEqual(self.app.pose_envelope(binding)['pose'], original)
        self.signals.assert_called_once_with(self.processes[0].pid, signal.SIGINT)

    def test_cancel_is_exact_job_generation_model_and_never_adds_open_detach_or_retry(self):
        first = self.start()
        self.update()
        self.app.snapshot()  # IPC proves the cooperative handler is installed.
        job = first['job']
        for change in ({'job_id': '2' * 32}, {'generation': True}, {'model_sha256': '0' * 64}):
            body = dict(job_id=job['job_id'], generation=job['generation'], model_sha256=MODEL)
            body.update(change)
            with self.assertRaises((ValueError, TypeError)): self.app.cancel(body)
        body = dict(job_id=job['job_id'], generation=job['generation'], model_sha256=MODEL)
        self.app.cancel(body)
        self.app.cancel(body)
        self.signals.assert_called_once_with(self.processes[0].pid, signal.SIGINT)
        self.assertEqual(len(self.calls), 1)
        self.assertFalse(self.app.snapshot()['job']['automatic_release'])
        self.assertFalse(self.app.snapshot()['job']['automatic_detach'])

    def test_immediate_cancel_is_reserved_without_signal_until_valid_worker_ipc_then_delivered_once(self):
        self.start()
        job = self.app.current
        self.app.cancel(dict(job_id=job.identifier, generation=job.generation, model_sha256=MODEL))
        self.assertTrue(job.cancel_requested)
        self.assertEqual(job.cancel_delivery, 'waiting_for_worker')
        self.signals.assert_not_called()
        self.update(native=False)  # Task IPC alone is sufficient; native launch is not required.
        self.app.snapshot()
        self.assertEqual(job.cancel_delivery, 'worker_signal_sent')
        self.signals.assert_called_once_with(job.process.pid, signal.SIGINT)
        self.app.snapshot()
        self.app.cancel(dict(job_id=job.identifier, generation=job.generation, model_sha256=MODEL))
        self.signals.assert_called_once_with(job.process.pid, signal.SIGINT)
        self.assertEqual(len(self.calls), 1)

    def test_no_ipc_immediate_cancel_keeps_original_escalation_deadline(self):
        self.start()
        job = self.app.current
        self.app.cancel(dict(job_id=job.identifier, generation=job.generation, model_sha256=MODEL))
        self.now = 49.
        self.app.snapshot()
        self.signals.assert_not_called()
        self.now = 50.
        self.app.snapshot()
        self.signals.assert_called_once_with(job.process.pid, signal.SIGTERM)
        self.now = 57.
        self.app.snapshot()
        self.assertEqual(self.signals.call_args.args, (job.process.pid, signal.SIGKILL))
        self.assertEqual(job.escalation, 2)
        self.assertFalse(job.finished)
        self.assertIsNone(job.owner_cleanup_clean)

    def test_malformed_first_ipc_aborts_queued_user_cancel_without_accepting_readiness_or_resetting_deadline(self):
        self.start()
        job = self.app.current
        self.app.cancel(dict(job_id=job.identifier, generation=job.generation, model_sha256=MODEL))
        self.now = 7.
        invalid = self.update(native=False)
        invalid['run_id'] = 'f' * 32
        atomic_json(job.directory/'run/latest.json', invalid)
        self.app.snapshot()
        self.assertIsNone(job.update)
        self.assertIsNone(job.pose)
        self.assertIsNotNone(job.display_error)
        self.assertEqual(job.cancel_delivery, 'worker_signal_sent')
        self.assertEqual(job.cancel_at, 0.)
        self.signals.assert_called_once_with(job.process.pid, signal.SIGINT)
        self.app.snapshot()
        self.signals.assert_called_once_with(job.process.pid, signal.SIGINT)
        self.now = 50.
        self.app.snapshot()
        self.assertEqual(self.signals.call_args.args, (job.process.pid, signal.SIGTERM))
        self.now = 57.
        self.app.snapshot()
        self.assertEqual(self.signals.call_args.args, (job.process.pid, signal.SIGKILL))

    def test_malformed_first_ipc_aborts_worker_immediately_and_missing_result_stays_unclean(self):
        self.start()
        job = self.app.current
        invalid = self.update(native=False)
        invalid['run_id'] = 'f' * 32
        atomic_json(job.directory/'run/latest.json', invalid)
        self.app.snapshot()
        self.assertIsNone(job.update)
        self.assertIsNone(job.observation)
        self.assertEqual(job.cancel_delivery, 'worker_signal_sent')
        self.signals.assert_called_once_with(job.process.pid, signal.SIGINT)
        job.process.returncode = -signal.SIGINT
        failed = self.app.snapshot()['job']
        self.assertEqual(failed['status'], 'failed')
        self.assertTrue(failed['finished'])
        self.assertIs(failed['owner_cleanup_clean'], False)
        self.assertFalse(failed['worker_result_observed'])

    def test_output_overflow_before_first_ipc_aborts_owned_worker(self):
        self.start()
        job = self.app.current
        job.output = SimpleNamespace(overflow=True, error=None, thread=SimpleNamespace(join=lambda **kwargs: None))
        self.app.snapshot()
        self.assertIsNone(job.update)
        self.assertTrue(job.cancel_requested)
        self.signals.assert_called_once_with(job.process.pid, signal.SIGINT)

    def test_output_error_before_first_ipc_aborts_owned_worker(self):
        self.start()
        job = self.app.current
        job.output = SimpleNamespace(overflow=False, error='fixture output write failed',
                                     thread=SimpleNamespace(join=lambda **kwargs: None))
        self.app.snapshot()
        self.assertIsNone(job.update)
        self.signals.assert_called_once_with(job.process.pid, signal.SIGINT)

    def test_artifact_overflow_before_first_ipc_aborts_owned_worker(self):
        self.start()
        job = self.app.current
        with patch('mujoco_ros2_manipulation.grasping.ros_demo_server._directory_bytes', return_value=97 * 1024 * 1024):
            self.app.snapshot()
        self.assertIsNone(job.update)
        self.signals.assert_called_once_with(job.process.pid, signal.SIGINT)

    def test_artifact_validation_failure_before_first_ipc_aborts_even_if_receipt_cannot_be_saved(self):
        self.start()
        job = self.app.current
        with patch('mujoco_ros2_manipulation.grasping.ros_demo_server._directory_bytes',
                   side_effect=ValueError('linked retention fixture')), \
             patch.object(self.app, '_save', side_effect=OSError('read-only receipt fixture')):
            self.app.snapshot()
            self.assertIsNone(job.update)
            self.signals.assert_called_once_with(job.process.pid, signal.SIGINT)
            self.now = 50.
            self.app.snapshot()
            self.assertEqual(self.signals.call_args.args, (job.process.pid, signal.SIGTERM))
            self.now = 57.
            self.app.snapshot()
            self.assertEqual(self.signals.call_args.args, (job.process.pid, signal.SIGKILL))
        self.assertTrue(any('owner receipt' in error for error in job.owner_errors))
        self.assertIsNone(job.owner_cleanup_clean)

    def test_timeout_before_first_ipc_aborts_immediately_and_retains_escalation_limits(self):
        self.start()
        job = self.app.current
        self.now = 300.
        self.app.snapshot()
        self.assertTrue(job.timed_out)
        self.assertEqual(job.cancel_at, 300.)
        self.assertIsNone(job.update)
        self.signals.assert_called_once_with(job.process.pid, signal.SIGINT)
        self.now = 350.
        self.app.snapshot()
        self.assertEqual(self.signals.call_args.args, (job.process.pid, signal.SIGTERM))
        self.now = 357.
        self.app.snapshot()
        self.assertEqual(self.signals.call_args.args, (job.process.pid, signal.SIGKILL))

    def test_owner_monitor_failure_before_first_ipc_aborts_owned_worker(self):
        self.app.closing.set()
        self.app.monitor.join(timeout=1.)
        self.app.closing.clear()
        self.start()
        job = self.app.current
        with patch.object(self.app.closing, 'wait', side_effect=(False, True)), \
             patch.object(self.app, '_update', side_effect=RuntimeError('fixture owner monitor failed')):
            self.app._watch()
        self.assertIsNone(job.update)
        self.signals.assert_called_once_with(job.process.pid, signal.SIGINT)
        self.assertTrue(any('owner monitor' in error for error in job.owner_errors))

    def test_fatal_error_preserves_worker_acknowledgement_and_never_sends_second_cancel(self):
        self.start()
        job = self.app.current
        self.app.cancel(dict(job_id=job.identifier, generation=job.generation, model_sha256=MODEL))
        value = self.update(native=False)
        value['task']['status'] = 'cancelled'
        atomic_json(job.directory/'run/latest.json', value)
        self.app.snapshot()
        self.assertEqual(job.cancel_delivery, 'worker_observed')
        with patch('mujoco_ros2_manipulation.grasping.ros_demo_server._directory_bytes', return_value=97 * 1024 * 1024):
            self.app.snapshot()
        self.assertEqual(job.cancel_delivery, 'worker_observed')
        self.signals.assert_not_called()

    def test_success_requires_worker_result_exit_and_separate_native_cleanup_evidence(self):
        self.start()
        self.terminal(clean=False)
        state = self.app.snapshot()['job']
        self.assertEqual(state['status'], 'failed')
        self.assertFalse(state['owner_cleanup_clean'])
        self.assertEqual(state['task']['physical_grasp'], 'released')
        self.assertEqual(state['task']['planning_attachment'], 'world')
        self.assertEqual(state['action_terminal']['status'], 4)
        self.assertFalse(state['cleanup']['clean'])

    def test_source_change_and_reserved_failed_spawns_cannot_create_unbounded_retries(self):
        self.pin['fixture-source'] = 'f' * 64
        with self.assertRaises(ValueError): self.start()
        self.assertFalse(self.calls)
        self.pin['fixture-source'] = 'e' * 64
        def failed(*args, **kwargs):
            raise OSError('fixture spawn failure')
        self.app.process_factory = failed
        with patch('mujoco_ros2_manipulation.grasping.ros_demo_server.MAX_JOBS', 1):
            with self.assertRaises(RuntimeError): self.start()
            self.assertTrue(self.app.current.finished)
            self.start()  # Same request observes the failed reservation; no retry.
            with self.assertRaises(RuntimeError): self.start('2' * 32)
        self.assertEqual(len(self.app.requests), 1)

    def test_cancel_grace_allows_real_worker_cleanup_before_escalation(self):
        self.start()
        self.update()
        self.app.snapshot()
        job = self.app.current
        self.app.cancel(dict(job_id=job.identifier, generation=job.generation, model_sha256=MODEL))
        self.now = 49.
        self.app.snapshot()
        self.signals.assert_called_once_with(job.process.pid, signal.SIGINT)
        self.now = 50.
        self.app.snapshot()
        self.assertEqual(self.signals.call_args.args, (job.process.pid, signal.SIGTERM))

    def test_persistent_malformed_ipc_is_quarantined_and_does_not_block_term_or_kill(self):
        self.start()
        valid = self.update()
        initial = self.app.snapshot()
        original_pose = self.app.pose_envelope(initial['view']['binding_id'])['pose']
        invalid = dict(valid, sequence=2, native_observation=[])
        atomic_json(self.app.current.directory / 'run/latest.json', invalid)
        state = self.app.snapshot()
        self.assertTrue(state['job']['cancel_requested'])
        self.assertEqual(self.app.current.update['sequence'], 1)
        self.now = 50.
        self.app.snapshot()
        self.assertEqual(self.signals.call_args.args, (self.processes[0].pid, signal.SIGTERM))
        self.now = 60.
        self.app.snapshot()
        self.assertEqual(self.signals.call_args.args, (self.processes[0].pid, signal.SIGKILL))
        self.assertEqual(self.app.current.escalation, 2)
        self.assertEqual(self.app.pose_envelope(initial['view']['binding_id'])['pose'], original_pose)

    def test_nested_malformed_task_and_guard_are_rejected_before_becoming_snapshot_state(self):
        for index, field in enumerate(('cleanup', 'lifecycle', 'planning_scene', 'native_guard'), 17):
            self.start(f'{index:032x}')
            value = self.update()
            if field == 'native_guard':
                value[field] = []
            else:
                value['task'][field] = []
            atomic_json(self.app.current.directory / 'run/latest.json', value)
            with self.subTest(field=field):
                state = self.app.snapshot()
                self.assertIsNone(self.app.current.update)
                self.assertTrue(state['job']['cancel_requested'])
                self.assertIsNotNone(state['job']['error'])
                self.terminal(status='failed', clean=True, code=1)
                self.app.snapshot()
                self.assertTrue(self.app.current.owner_cleanup_clean)

    def test_retention_and_receipt_write_failures_cannot_disable_stop_driver_or_monitor(self):
        self.start()
        self.update()
        with patch('mujoco_ros2_manipulation.grasping.ros_demo_server._directory_bytes',
                   side_effect=ValueError('linked retention fixture')), \
             patch.object(self.app, '_save', side_effect=OSError('read-only receipt fixture')):
            self.app.snapshot()
            self.now = 50.
            self.app.snapshot()
            self.assertEqual(self.signals.call_args.args, (self.processes[0].pid, signal.SIGTERM))
            self.now = 60.
            for _ in range(20):
                self.app.snapshot()
            self.assertEqual(self.signals.call_args.args, (self.processes[0].pid, signal.SIGKILL))
            self.assertEqual(self.app.current.escalation, 2)
            self.assertLessEqual(len(self.app.current.owner_errors), 8)
            self.assertTrue(self.app.monitor.is_alive())

    def test_new_request_requires_proven_cleanup_but_old_request_stays_idempotent(self):
        self.start()
        self.terminal(status='failed', clean=False, code=1)
        state = self.app.snapshot()
        self.assertTrue(state['job']['finished'])
        self.assertIs(state['job']['owner_cleanup_clean'], False)
        duplicate = self.start()
        self.assertFalse(duplicate['request_replay']['new_job_started'])
        with self.assertRaises(RuntimeError):
            self.start('2' * 32)
        self.assertEqual(len(self.calls), 1)

    def test_failed_task_can_start_again_after_its_owned_cleanup_is_confirmed(self):
        self.start()
        self.terminal(status='failed', clean=True, code=1)
        self.assertTrue(self.app.snapshot()['job']['owner_cleanup_clean'])
        self.start('2' * 32)
        self.assertEqual(len(self.calls), 2)

    def test_post_spawn_drainer_failure_keeps_live_worker_owned_and_cancelling(self):
        original_factory = self.app.process_factory
        def spawn_with_pipe(*args, **kwargs):
            child = original_factory(*args, **kwargs)
            child.stdout = io.BytesIO(b'fixture stdout')
            return child
        self.app.process_factory = spawn_with_pipe
        with patch('mujoco_ros2_manipulation.grasping.ros_demo_server._BoundedOutput',
                   side_effect=RuntimeError('fixture drainer thread failed')):
            with self.assertRaises(RuntimeError): self.start()
        job = self.app.current
        self.assertIsNone(job.process.poll())
        self.assertFalse(job.finished)
        self.assertIsNone(job.owner_cleanup_clean)
        self.assertTrue(job.cancel_requested)
        self.assertEqual(job.cancel_delivery, 'worker_signal_sent')
        self.signals.assert_called_once_with(job.process.pid, signal.SIGINT)
        with self.assertRaises(RuntimeError): self.start('2' * 32)
        self.start()  # This is an observation, never another spawn.
        self.assertEqual(len(self.calls), 1)
        self.now = 60.
        self.app.snapshot()
        self.assertEqual(job.escalation, 2)
        self.assertEqual(self.signals.call_args.args, (job.process.pid, signal.SIGKILL))

    def test_unproved_native_group_does_not_prevent_escalation_of_owned_worker(self):
        self.start()
        job = self.app.current
        self.app.cancel(dict(job_id=job.identifier, generation=job.generation, model_sha256=MODEL))
        with patch.object(self.app, '_signal_native_group', side_effect=ValueError('unproved native instance')):
            self.now = 50.
            self.app.snapshot()
            self.assertEqual(self.signals.call_args.args, (job.process.pid, signal.SIGTERM))
            self.now = 60.
            self.app.snapshot()
            self.assertEqual(self.signals.call_args.args, (job.process.pid, signal.SIGKILL))
        self.assertTrue(any('native escalation' in error for error in job.owner_errors))

    def test_native_group_binding_requires_actual_worker_parent_and_retained_instance(self):
        self.start()
        job = self.app.current
        wrong = dict(ppid=1234, group=123, session=123, start=9)
        with patch('mujoco_ros2_manipulation.grasping.ros_demo_server._proc_identity', return_value=wrong):
            self.app._bind_native_group(job, 123)
        self.assertIsNone(job.native_group)
        correct = dict(wrong, ppid=job.process.pid)
        with patch('mujoco_ros2_manipulation.grasping.ros_demo_server._proc_identity', return_value=correct), \
             patch('mujoco_ros2_manipulation.grasping.ros_demo_server._members', return_value={123: 9}):
            self.app._bind_native_group(job, 123)
        self.assertEqual(job.native_group, 123)
        with patch('mujoco_ros2_manipulation.grasping.ros_demo_server._group_gone', return_value=False), \
             patch('mujoco_ros2_manipulation.grasping.ros_demo_server._proc_identity', return_value=dict(correct, start=10)), \
             patch('mujoco_ros2_manipulation.grasping.ros_demo_server.os.killpg') as kill:
            with self.assertRaises(ValueError): self.app._signal_native_group(job, signal.SIGTERM)
            kill.assert_not_called()
        job.native_group = None

    def test_validation_binding_retains_one_actual_worker_child_instance(self):
        self.start()
        job = self.app.current
        wrong = dict(ppid=1234, group=123, session=123, start=9)
        with patch('mujoco_ros2_manipulation.grasping.ros_demo_server._proc_identity', return_value=wrong):
            self.app._bind_validation_group(job, 123)
        self.assertIsNone(job.validation_group)
        correct = dict(wrong, ppid=job.process.pid)
        with patch('mujoco_ros2_manipulation.grasping.ros_demo_server._proc_identity', return_value=correct), \
             patch('mujoco_ros2_manipulation.grasping.ros_demo_server._members', return_value={123:9}):
            self.app._bind_validation_group(job, 123)
        self.assertEqual(job.validation_group, 123)
        with self.assertRaises(ValueError):
            self.app._bind_validation_group(job, 456)
        with patch('mujoco_ros2_manipulation.grasping.ros_demo_server._group_gone', return_value=False), \
             patch('mujoco_ros2_manipulation.grasping.ros_demo_server._proc_identity', return_value=dict(correct, start=10)), \
             patch('mujoco_ros2_manipulation.grasping.ros_demo_server.os.killpg') as kill:
            with self.assertRaises(ValueError):
                self.app._signal_validation_group(job, signal.SIGTERM)
            kill.assert_not_called()
        job.validation_group = None

    def test_live_validation_group_prevents_cleanup_success_after_worker_exit(self):
        self.start()
        job = self.app.current
        job.validation_group, job.validation_leader_start = 123, 9
        self.terminal(status='cancelled', clean=True)
        with patch('mujoco_ros2_manipulation.grasping.ros_demo_server._group_gone', side_effect=lambda group: group != 123):
            state = self.app.snapshot()['job']
            self.assertFalse(state['finished'])
            self.assertIsNone(state['owner_cleanup_clean'])
            self.assertIs(state['validation_process_group_gone'], False)
        state = self.app.snapshot()['job']
        self.assertTrue(state['finished'])
        self.assertTrue(state['owner_cleanup_clean'])
        self.assertIs(state['validation_process_group_gone'], True)

    def test_unproved_reported_validation_group_cannot_be_signalled_or_declared_clean(self):
        self.start()
        job = self.app.current
        self.terminal(clean=True)
        result_path = job.directory/'run/result.json'
        result = json.loads(result_path.read_text())
        result['owned_validation_process_group'] = 123
        atomic_json(result_path, result)
        with patch('mujoco_ros2_manipulation.grasping.ros_demo_server._proc_identity', return_value=None), \
             patch('mujoco_ros2_manipulation.grasping.ros_demo_server._group_gone', side_effect=lambda group: group != 123), \
             patch('mujoco_ros2_manipulation.grasping.ros_demo_server.os.killpg') as kill:
            state = self.app.snapshot()['job']
            self.app._signal_validation_group(job, signal.SIGTERM)
            self.assertEqual(state['status'], 'failed')
            self.assertFalse(state['owner_cleanup_clean'])
            kill.assert_not_called()

    def test_unproved_validation_group_does_not_disable_owned_worker_escalation(self):
        self.start()
        job = self.app.current
        self.app.cancel(dict(job_id=job.identifier,generation=job.generation,model_sha256=MODEL))
        with patch.object(self.app, '_signal_validation_group', side_effect=ValueError('unproved validator instance')):
            self.now = 50.
            self.app.snapshot()
            self.assertEqual(self.signals.call_args.args, (job.process.pid,signal.SIGTERM))
        self.assertTrue(any('validation escalation' in error for error in job.owner_errors))

    def test_validator_ownership_is_readonly_metadata_with_strict_identity_type(self):
        self.start()
        value = self.update()
        value['task']['owned_validation_process_group'] = True
        atomic_json(self.app.current.directory/'run/latest.json',value)
        state = self.app.snapshot()['job']
        self.assertTrue(state['cancel_requested'])
        self.assertIsNone(self.app.current.validation_group)


class HandlerFixture:
    def __init__(self):
        self.calls = []
    def snapshot(self):
        return dict(schema_version=1, source=dict(id='scene:ros_fr5_grasp', model_sha256=MODEL), job=None, view=None)
    def start(self, value):
        if type(value) is not dict or set(value) != {'request_id', 'model_sha256'} or value['model_sha256'] != MODEL:
            raise ValueError('Fixed request only')
        self.calls.append(value)
        return self.snapshot()
    cancel = start
    def manifest_bytes(self, value):
        if value != 'b' * 64: raise ValueError('Wrong binding')
        return b'{}'
    pose_envelope = manifest_bytes
    def asset_bytes(self, value):
        if value != 'c' * 64: raise ValueError('Unknown asset')
        return b'fixture-asset'


class RosHTTPBoundaryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = HandlerFixture()
        cls.server = serve(cls.app, 0)
        cls.port = cls.server.server_address[1]
        cls.origin = f'http://127.0.0.1:{cls.port}'
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown(); cls.server.server_close(); cls.thread.join(timeout=2.)
    def setUp(self): self.app.calls.clear()
    def request(self, method, path, body=None, headers=None, raw=None):
        headers = dict(headers or {})
        if method == 'POST':
            headers = dict({'Origin': self.origin, 'X-Operator-Token': self.server.token, 'Content-Type': 'application/json'}, **headers)
        client = http.client.HTTPConnection('127.0.0.1', self.port, timeout=3.)
        try:
            client.request(method, path, raw if raw is not None else json.dumps(body).encode() if body is not None else None, headers)
            response = client.getresponse()
            return response.status, dict(response.getheaders()), response.read()
        finally: client.close()
    def test_origin_host_token_and_response_policy(self):
        status, headers, raw = self.request('GET', '/api/ros-demo')
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(raw)['token'], self.server.token)
        self.assertEqual(headers['Cache-Control'], 'no-store')
        self.assertIn("script-src 'self'", headers['Content-Security-Policy'])
        for extra in ({'Host': 'evil.invalid'}, {'Origin': 'https://evil.invalid'}):
            self.assertEqual(self.request('GET', '/api/ros-demo', headers=extra)[0], 403)
        for extra in ({'X-Operator-Token': 'wrong'}, {'Origin': ''}, {'Origin': 'https://evil.invalid'}):
            self.assertEqual(self.request('POST', '/api/ros-demo/start', {}, headers=extra)[0], 403)
        self.assertFalse(self.app.calls)
    def test_mutation_paths_and_payload_are_fixed_and_bounded(self):
        body = dict(request_id='a' * 32, model_sha256=MODEL)
        self.assertEqual(self.request('POST', '/api/ros-demo/start', body)[0], 200)
        for data, extra in ((b'{}', {'Transfer-Encoding': 'chunked'}), (b'{}', {'Content-Length': '1025'}),
                            (b'{"x":NaN}', {}), (b'{"x":1,"x":2}', {}), (b'{}', {'Content-Type': 'text/plain'})):
            self.assertEqual(self.request('POST', '/api/ros-demo/start', headers=extra, raw=data)[0], 400)
        for path in ('/api/ros-demo/start?target=/tmp/model', '/api/demo/start', '/api/ros/open', '/api/ros/attach'):
            self.assertEqual(self.request('POST', path, body)[0], 404)
    def test_only_allowlisted_native_view_and_local_static_assets_are_served(self):
        self.assertEqual(self.request('GET', '/api/view/manifest?binding_id=' + 'b' * 64)[0], 200)
        for path in ('/api/view/pose?binding_id=x&path=y', '/api/view/assets/../../README.md', '/run/result.json'):
            self.assertIn(self.request('GET', path)[0], (404, 409))
        for path in ('/', '/ros_demo.js', '/ros_demo.css', '/viewer.js', '/vendor/OrbitControls.js'):
            status, _, raw = self.request('GET', path)
            self.assertEqual(status, 200)
            self.assertTrue(raw)
        self.assertIn(b"from './vendor/three.module.js'", self.request('GET', '/viewer.js')[2])


if __name__ == '__main__': unittest.main()
