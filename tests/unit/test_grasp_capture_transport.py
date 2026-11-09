"""Recording-only source timing, bounded JPEG IPC, and identity regressions."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.grasping.capture_transport import (
    CapturePublisher, CaptureReceiver, MAX_JOURNAL, MAX_SIDECAR_BYTES,
    checked_registration, native_association)
from mujoco_ros2_manipulation.verification.reports import atomic_json


def owner():
    return dict(job_id='1' * 32, generation=1, binding_id='2' * 64,
        binding={'job_id': '1' * 32}, model_sha256='3' * 64,
        execution_manifest_sha256='4' * 64, domain=222, source_registration_epoch='5' * 32)


def task_value(now):
    return dict(kind='task', model_sha256='3' * 64,
        task=dict(stage='approach', status='running', cleanup={'status': 'not_started'},
                  physical_grasp='unconfirmed'),
        native_observation=dict(sequence=10, stamp_ns='9000000000', phase=1,
            capture_monotonic_ns=str(now - 20_000_000)),
        native_guard=dict(phase='ready', latched=False), native_clock_has_advanced=True,
        observed_monotonic_ns=now - 10_000_000, clock_progress_observed_monotonic_ns=now - 10_000_000)


class NativeEventAssociationTests(unittest.TestCase):
    def test_fresh_association_preserves_source_time_and_sequence(self):
        now = time.monotonic_ns()
        result = native_association(task_value(now), now, native_alive=True)
        self.assertTrue(result['available'])
        self.assertEqual(result['stamp_ns'], '9000000000')
        self.assertEqual(result['capture_monotonic_ns'], str(now - 20_000_000))
        self.assertEqual(result['sequence'], 10)

    def test_pre_native_stale_held_cleanup_and_exited_events_have_no_native_time(self):
        now = time.monotonic_ns()
        variants = []
        value = task_value(now); value['native_observation'] = None; variants.append((value, True))
        value = task_value(now); value['clock_progress_observed_monotonic_ns'] = now - 600_000_000; variants.append((value, True))
        value = task_value(now); value['native_observation']['phase'] = 8; variants.append((value, True))
        value = task_value(now); value['native_guard']['latched'] = True; variants.append((value, True))
        value = task_value(now); value['task']['cleanup']['status'] = 'running'; variants.append((value, True))
        variants.append((task_value(now), False))
        for value, alive in variants:
            with self.subTest(value=value, alive=alive):
                result = native_association(value, now, native_alive=alive)
                self.assertFalse(result['available'])
                self.assertIsNone(result['stamp_ns'])
                self.assertIsNone(result['sequence'])
                self.assertIsNone(result['capture_monotonic_ns'])


class CaptureTransportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.job = Path(self.temp.name)
        self.run = self.job / 'run'
        self.run.mkdir()
        self.owner = owner()
        self.publisher = CapturePublisher(self.run, self.owner, image_topic='/diagnostic/rgb',
                                          image_clock_domain='ros_simulation')
        # Pure deterministic exporter tests. No ROS participant or codec thread.
        self.publisher.stop.set(); self.publisher.wake.set(); self.publisher.thread.join(2)
        self.assertFalse(self.publisher.thread.is_alive())
        self.addCleanup(self.publisher.source.close)
        self.receiver = CaptureReceiver(self.job, self.owner, image_topic='/diagnostic/rgb',
                                        image_clock_domain='ros_simulation')

    def image(self, sequence=1):
        source = self.publisher.source
        source.encoder = lambda *_: b'\xff\xd8' + bytes([sequence]) + b'\xff\xd9'
        source.set_publishers(['6' * 64])
        # Bypass the rate clock wait for deterministic samples only.
        source._next_encode_at = 0
        self.assertTrue(source.receive(width=1, height=1, encoding='rgb8', step=3,
            data=b'abc', stamp_sec=sequence, stamp_nanosec=123, frame_id='camera', generation=source.generation))
        source.process_once()
        return source.snapshot()['latest']

    def publish(self):
        self.publisher.publish_once()
        self.receiver.refresh()
        self.assertIsNone(self.receiver.fault)

    def test_registration_is_fixed_to_owned_job_and_bounded(self):
        path = self.job / 'capture-registration.json'
        atomic_json(path, self.owner)
        self.assertEqual(checked_registration(path, self.run, self.owner['job_id'], 222,
                                             self.owner['execution_manifest_sha256']), self.owner)
        with self.assertRaises(ValueError):
            checked_registration(path, self.run, '7' * 32, 222, self.owner['execution_manifest_sha256'])
        with self.assertRaises(ValueError):
            checked_registration(path, self.run, self.owner['job_id'], 221, self.owner['execution_manifest_sha256'])
        path.write_bytes(b'x' * 8193)
        with self.assertRaises(ValueError):
            checked_registration(path, self.run, self.owner['job_id'], 222, self.owner['execution_manifest_sha256'])

    def test_source_registration_precedes_publisher_and_survives_first_generation(self):
        initial = self.receiver.snapshot()
        self.assertEqual(initial['status'], 'waiting_source')
        self.assertEqual(initial['source_registration_epoch'], self.owner['source_registration_epoch'])
        frame = self.image()
        self.publish()
        current = self.receiver.snapshot()
        self.assertEqual(current['generation'], frame['generation'])
        self.assertEqual(current['source_registration_epoch'], initial['source_registration_epoch'])
        self.assertNotEqual(current['generation'], initial['generation'])

    def test_rgb_metadata_and_exact_jpeg_are_copied_without_changing_clocks(self):
        frame = self.image()
        self.publish()
        metadata, raw = self.receiver.frame(frame['generation'], frame['sequence'])
        self.assertEqual(raw, b'\xff\xd8\x01\xff\xd9')
        for name in ('source_stamp_ns', 'received_monotonic_sec', 'encoded_monotonic_sec', 'jpeg_sha256'):
            self.assertEqual(metadata[name], frame[name])
        self.assertEqual(metadata['source_stamp_ns'], '1000000123')
        self.assertEqual(metadata['topic'], '/diagnostic/rgb')
        self.assertEqual(metadata['publisher_endpoint_sha256'], '6' * 64)
        self.assertEqual(self.receiver.snapshot()['age_clock_domain'], 'grasp_worker_receiver_elapsed_monotonic')

    def test_event_source_emission_and_parent_receipt_remain_distinct(self):
        now = time.monotonic_ns()
        self.publisher.event(task_value(now), emitted_ns=now, native_alive=True)
        self.publish()
        event = self.receiver.events[0]
        self.assertEqual(event['emitted_monotonic_ns'], str(now))
        self.assertGreaterEqual(int(event['parent_received_monotonic_ns']), now)
        self.receiver.refresh()
        self.assertEqual(len(self.receiver.events), 1)
        self.assertEqual(event['event_id'], self.owner['job_id'] + ':1')

    def test_same_event_sequence_cannot_change_source_content(self):
        now = time.monotonic_ns()
        self.publisher.event(task_value(now), emitted_ns=now, native_alive=True)
        self.publish()
        path = self.publisher.directory / 'latest.json'
        value = json.loads(path.read_bytes())
        value['events'][0]['stage'] = 'changed'
        atomic_json(path, value)
        self.receiver.refresh()
        self.assertIn('event changed', self.receiver.fault)

    def test_event_and_file_retention_are_bounded_and_final_journal_drains(self):
        now = time.monotonic_ns()
        for _ in range(MAX_JOURNAL + 4):
            self.publisher.event(task_value(now), emitted_ns=now, native_alive=True)
        for sequence in range(1, 8):
            self.image(sequence)
            self.publisher.publish_once()
        self.publisher.closed = True
        self.publish()
        self.assertEqual(len(self.receiver.events), MAX_JOURNAL)
        self.assertEqual(self.receiver.events_dropped, 4)
        self.assertTrue(self.receiver.closed)
        self.assertLessEqual(len(list(self.publisher.directory.glob('*.jpg'))), 3)
        self.assertLessEqual((self.publisher.directory / 'latest.json').stat().st_size, MAX_SIDECAR_BYTES)

    def test_jpeg_file_count_includes_the_publication_transition(self):
        original_unlink = Path.unlink
        observed = []
        directory = self.publisher.directory
        def unlink(path, *args, **kwargs):
            if path.parent == directory and path.suffix == '.jpg':
                observed.append(len(list(directory.glob('*.jpg'))))
            return original_unlink(path, *args, **kwargs)
        with patch.object(Path, 'unlink', unlink):
            for sequence in range(1, 8):
                self.image(sequence)
                self.publisher.publish_once()
        self.assertTrue(observed)
        self.assertLessEqual(max(observed), 3)
        self.assertLessEqual(len(list(directory.glob('*.jpg'))), 2)

    def test_new_publisher_generation_invalidates_parent_frames(self):
        frame = self.image()
        self.publish()
        self.publisher.source.set_publishers(['7' * 64])
        self.publish()
        self.assertIsNone(self.receiver.frame(frame['generation'], frame['sequence']))
        self.assertEqual(len(self.receiver.frames), 0)

    def test_wrong_owner_and_corrupt_jpeg_become_only_recording_faults(self):
        frame = self.image()
        self.publisher.publish_once()
        path = self.publisher.directory / 'latest.json'
        value = json.loads(path.read_bytes())
        value['owner']['job_id'] = '8' * 32
        atomic_json(path, value)
        self.receiver.refresh()
        self.assertIn('owner differs', self.receiver.fault)
        self.assertEqual(self.receiver.frames, {})
        self.receiver = CaptureReceiver(self.job, self.owner, image_topic='/diagnostic/rgb',
                                        image_clock_domain='ros_simulation')
        self.publisher.publish_once()
        (self.publisher.directory / (frame['jpeg_sha256'] + '.jpg')).write_bytes(b'wrong')
        self.receiver.refresh()
        self.assertIn('JPEG bytes differ', self.receiver.fault)

    def test_invalid_metadata_and_false_native_association_are_recording_faults(self):
        now = time.monotonic_ns()
        self.publisher.event(task_value(now), emitted_ns=now, native_alive=False)
        self.image()
        self.publisher.publish_once()
        path = self.publisher.directory / 'latest.json'
        original = json.loads(path.read_bytes())
        for mutation in ('receipt', 'association', 'topic'):
            value = deepcopy(original)
            if mutation == 'receipt':
                value['image']['latest']['encoded_monotonic_sec'] = 'invalid'
            elif mutation == 'association':
                value['events'][0]['native_association']['stamp_ns'] = '1'
            else:
                value['image']['topic'] = '/changed/source'
            atomic_json(path, value)
            receiver = CaptureReceiver(self.job, self.owner, image_topic='/diagnostic/rgb',
                                       image_clock_domain='ros_simulation')
            receiver.refresh()
            self.assertIsNotNone(receiver.fault)
            self.assertEqual(receiver.snapshot()['status'], 'error')

    def test_corrupt_final_image_keeps_valid_original_terminal_events(self):
        now = time.monotonic_ns()
        value = task_value(now)
        self.publisher.event(value, emitted_ns=now, native_alive=True)
        frame = self.image()
        self.publish()
        value['task'].update(stage='complete', status='cancelled')
        value['task']['cleanup']['status'] = 'complete'
        self.publisher.event(value, emitted_ns=now + 1, native_alive=False)
        self.publisher.publish_once()
        (self.publisher.directory / (frame['jpeg_sha256'] + '.jpg')).write_bytes(b'corrupt')
        # Force a new descriptor to ensure the already cached JPEG does not
        # hide this deliberately corrupt original for the fresh receiver.
        receiver = CaptureReceiver(self.job, self.owner, image_topic='/diagnostic/rgb',
                                   image_clock_domain='ros_simulation')
        receiver.refresh(finished=True)
        self.assertIn('JPEG bytes differ', receiver.fault)
        self.assertEqual([item['stage'] for item in receiver.events], ['approach', 'complete'])
        self.assertEqual(receiver.events[-1]['status'], 'cancelled')
        self.assertIsNone(receiver.events[-1]['native_association']['stamp_ns'])
        self.assertIsNone(receiver.frame(frame['generation'], frame['sequence']))

    def test_published_byte_bound_includes_actual_indentation_and_newline(self):
        self.publisher.publish_once()
        path = self.publisher.directory / 'latest.json'
        original = path.read_bytes()
        value = json.loads(original)
        compact = len(json.dumps(value).encode())
        self.assertLess(compact, len(original) - 1)
        with patch('mujoco_ros2_manipulation.grasping.capture_transport.MAX_SIDECAR_BYTES', len(original) - 1):
            with self.assertRaisesRegex(ValueError, 'exceeds its bound'):
                self.publisher.publish_once()
        self.assertEqual(path.read_bytes(), original)

    def test_missing_overwritten_blob_is_a_gap_and_does_not_fabricate_frame(self):
        frame = self.image()
        self.publisher.publish_once()
        (self.publisher.directory / (frame['jpeg_sha256'] + '.jpg')).unlink()
        self.receiver.refresh()
        self.assertIsNone(self.receiver.fault)
        self.assertEqual(self.receiver.transport_gaps, 1)
        self.assertIsNone(self.receiver.frame(frame['generation'], frame['sequence']))
        self.assertFalse(self.receiver.snapshot()['latest']['image_available'])

    def test_fifo_and_oversized_sidecar_are_bounded_recording_faults(self):
        import os
        path = self.publisher.directory / 'latest.json'
        path.unlink(missing_ok=True)
        os.mkfifo(path)
        started = time.monotonic()
        self.receiver.refresh()
        self.assertLess(time.monotonic() - started, .5)
        self.assertIn('not regular', self.receiver.fault)
        path.unlink()
        path.write_bytes(b'x' * (MAX_SIDECAR_BYTES + 1))
        receiver = CaptureReceiver(self.job, self.owner)
        receiver.refresh()
        self.assertIn('exceeds bounds', receiver.fault)

    def test_parent_directory_symlinks_and_fifos_are_never_followed(self):
        import os
        other = self.job / 'outside'
        other.mkdir()
        atomic_json(other / 'latest.json', {'kind': 'sentinel'})
        capture = self.publisher.directory
        for member in capture.iterdir():
            member.unlink()
        capture.rmdir()
        capture.symlink_to(other, target_is_directory=True)
        self.receiver.refresh()
        self.assertIsNotNone(self.receiver.fault)
        self.assertNotIn('sentinel', self.receiver.fault)
        capture.unlink()
        os.mkfifo(capture)
        started = time.monotonic()
        receiver = CaptureReceiver(self.job, self.owner)
        receiver.refresh()
        self.assertLess(time.monotonic() - started, .5)
        self.assertIsNotNone(receiver.fault)

    def test_finished_worker_requires_a_final_journal_but_disabled_source_does_not(self):
        self.publisher.publish_once()
        self.receiver.refresh()
        self.receiver.refresh(finished=True)
        self.assertIn('did not close', self.receiver.fault)
        path = self.publisher.directory / 'latest.json'
        path.unlink()
        receiver = CaptureReceiver(self.job, self.owner, image_topic='/diagnostic/rgb')
        receiver.refresh(finished=True)
        self.assertIn('no final', receiver.fault)
        receiver = CaptureReceiver(self.job, self.owner)
        receiver.refresh(finished=True)
        self.assertIsNone(receiver.fault)

    def test_source_close_keeps_frozen_metadata_and_jpeg_for_diagnostics(self):
        frame = self.image()
        self.publisher.stop_sensor()
        self.publish()
        self.assertEqual(self.receiver.snapshot()['generation'], frame['generation'])
        self.assertIsNotNone(self.receiver.frame(frame['generation'], frame['sequence']))
        self.assertEqual(self.publisher.source.snapshot()['status'], 'closed')

    def test_codec_and_event_failures_are_recording_diagnostics(self):
        self.publisher.source.encoder = lambda *_: (_ for _ in ()).throw(ValueError('codec failure'))
        # The established source contains codec failures itself.
        source = self.publisher.source
        source.set_publishers(['6' * 64])
        source.receive(width=1, height=1, encoding='rgb8', step=3, data=b'abc',
            stamp_sec=1, stamp_nanosec=0, frame_id='camera', generation=source.generation)
        source.process_once()
        self.publish()
        self.assertEqual(self.receiver.snapshot()['status'], 'error')
        now = time.monotonic_ns()
        value = task_value(now); value['model_sha256'] = '9' * 64
        self.publisher.event(value, emitted_ns=now, native_alive=True)
        self.assertIn('Recording model differs', self.publisher.fault)


class WorkerPreNativeDiagnosticsTests(unittest.TestCase):
    def test_configured_cancellation_before_native_has_only_unassociated_source_events(self):
        from mujoco_ros2_manipulation.grasping.ros_task import run_ros_grasp, RosGraspCancelled
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            job = root / 'runtime/job'
            job.mkdir(parents=True)
            registration = job / 'capture-registration.json'
            atomic_json(registration, owner())
            with patch('mujoco_ros2_manipulation.grasping.ros_task.ROOT', root), patch(
                    'mujoco_ros2_manipulation.grasping.ros_task.load_workcell',
                    side_effect=RosGraspCancelled('cancel before native')):
                result = run_ros_grasp(root / 'target', owner()['execution_manifest_sha256'], job / 'run',
                    domain=222, run_id=owner()['job_id'], image_topic='/diagnostic/rgb',
                    image_clock_domain='ros_simulation', capture_registration=registration)
            self.assertEqual(result['status'], 'cancelled')
            self.assertTrue(result['cleanup']['clean'])
            self.assertFalse(result['cleanup']['launch_started'])
            self.assertTrue(result['recording_observations']['clean'])
            receiver = CaptureReceiver(job, owner(), image_topic='/diagnostic/rgb', image_clock_domain='ros_simulation')
            receiver.refresh(finished=True)
            self.assertTrue(receiver.closed)
            self.assertGreaterEqual(len(receiver.events), 2)
            for event in receiver.events:
                self.assertFalse(event['native_association']['available'])
                self.assertIsNone(event['native_association']['stamp_ns'])

    def test_recording_export_fault_keeps_the_original_native_failure(self):
        from mujoco_ros2_manipulation.grasping.ros_task import run_ros_grasp
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            job = root / 'runtime/job'
            job.mkdir(parents=True)
            registration = job / 'capture-registration.json'
            atomic_json(registration, owner())
            with patch('mujoco_ros2_manipulation.grasping.ros_task.ROOT', root), patch(
                    'mujoco_ros2_manipulation.grasping.ros_task.load_workcell',
                    side_effect=RuntimeError('original native preparation failure')), patch.object(
                    CapturePublisher, 'publish_once', side_effect=OSError('recording disk failure')):
                result = run_ros_grasp(root / 'target', owner()['execution_manifest_sha256'], job / 'run',
                    domain=222, run_id=owner()['job_id'], image_topic='/diagnostic/rgb',
                    capture_registration=registration)
            self.assertEqual(result['status'], 'failed')
            self.assertEqual(result['error']['message'], 'original native preparation failure')
            self.assertTrue(result['cleanup']['clean'])
            self.assertIn('recording disk failure', result['recording_observations']['fault'])
            self.assertEqual(result['cleanup_errors'], [])


if __name__ == '__main__':
    unittest.main()
