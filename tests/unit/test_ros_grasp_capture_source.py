"""Pure parent-side recording sources over the existing owned-worker fixtures."""
import json
import unittest
from unittest.mock import patch

from tests.unit import test_ros_grasp_demo_server as fixtures
from mujoco_ros2_manipulation.verification.reports import atomic_json
from mujoco_ros2_manipulation.grasping.capture_transport import MAX_JOURNAL


class GraspCaptureSourceTests(unittest.TestCase):
    start = fixtures.RosOwnedJobTests.start
    update = fixtures.RosOwnedJobTests.update
    terminal = fixtures.RosOwnedJobTests.terminal

    def setUp(self):
        fixtures.RosOwnedJobTests.setUp(self)
        self.app.image_topic = '/diagnostic/rgb'
        self.app.image_clock_domain = 'ros_simulation'

    def native(self, *, event=1, sequence=1, clock=1., phase=1, advanced=True):
        job = self.app.current
        with self.app.lock:
            value = self.update(event=event, native_sequence=sequence, clock=clock)
            value['native_clock_has_advanced'] = advanced
            value['native_observation']['phase'] = phase
            value['native_observation']['arm_positions_rad'] = {
                name: float(index) / 100 for index, name in enumerate(job.display.manifest['joint_names'])}
            atomic_json(job.directory / 'run/latest.json', value)
            self.app._read_update(job)
        return value

    def test_reserved_binding_is_stable_before_native_and_registration_is_fixed(self):
        self.assertIsNone(self.app.capture_source())
        self.start()
        job = self.app.current
        source = self.app.capture_source()
        self.assertEqual(source['owner']['job_id'], job.identifier)
        self.assertEqual(source['owner']['binding_id'], job.binding_id)
        self.assertNotEqual(job.binding_id, self.app.snapshot()['view']['binding_id'])
        self.assertEqual(source['poses'], [])
        self.assertEqual(source['events'], [])
        command = self.calls[-1][0]
        registration = command[command.index('--capture-registration') + 1]
        with open(registration) as stream:
            self.assertEqual(json.load(stream), source['owner'])
        self.assertEqual(command[command.index('--image-topic') + 1], '/diagnostic/rgb')
        self.native()
        self.assertEqual(self.app.capture_source()['owner'], source['owner'])
        self.assertEqual(self.app.snapshot()['view']['binding_id'], source['owner']['binding_id'])

    def test_disabled_rgb_starts_no_exporter_and_retains_no_pose_journal(self):
        self.app.image_topic = None
        self.start()
        job = self.app.current
        self.assertNotIn('--capture-registration', self.calls[-1][0])
        self.assertNotIn('--image-topic', self.calls[-1][0])
        self.assertFalse((job.directory / 'capture-registration.json').exists())
        self.native()
        source = self.app.capture_source()
        self.assertEqual(source['poses'], [])
        self.assertFalse(source['image'].snapshot()['configured'])

    def test_pose_keeps_original_native_timestamps_and_named_arm_values(self):
        self.start()
        value = self.native()
        source = self.app.capture_source()
        self.assertIsNone(source['fault'])
        row = source['poses'][0]
        self.assertEqual(row['stamp_ns'], value['native_observation']['stamp_ns'])
        self.assertEqual(row['native']['capture_monotonic_ns'], value['native_observation']['capture_monotonic_ns'])
        self.assertEqual(row['native']['observed_monotonic_ns'], str(value['observed_monotonic_ns']))
        self.assertEqual(set(row['positions_rad']), set(source['manifest']['joint_names']))
        self.assertEqual(row['pose'], self.app.current.pose)
        source['owner']['generation'] = 999
        row['pose']['geoms'].clear()
        self.assertNotEqual(self.app.capture_source()['owner']['generation'], 999)
        self.assertTrue(self.app.capture_source()['poses'][0]['pose']['geoms'])

    def test_task_updates_and_held_or_nonprogressing_native_do_not_create_media_pose(self):
        self.start()
        self.native()
        self.native(event=2, sequence=1)
        self.native(event=3, sequence=2, clock=1.)
        self.native(event=4, sequence=3, clock=1., phase=8)
        self.assertEqual(len(self.app.capture_source()['poses']), 1)
        self.native(event=5, sequence=4, clock=1.1, advanced=False)
        self.assertEqual(len(self.app.capture_source()['poses']), 1)

    def test_recording_transport_fault_does_not_set_native_error_or_cancel(self):
        self.start()
        self.native()
        job = self.app.current
        directory = job.directory / 'run/capture'
        directory.mkdir()
        atomic_json(directory / 'latest.json', {'kind': 'wrong'})
        with patch.object(self.app, '_begin_cancel') as cancel:
            source = self.app.capture_source()
        cancel.assert_not_called()
        self.assertIn('owner differs', source['fault'])
        self.assertIsNone(job.display_error)
        self.assertIsNone(job.error)
        self.assertFalse(job.cancel_requested)

    def test_pose_ring_evictions_and_latest_ipc_source_gaps_are_distinct(self):
        self.start()
        for index in range(MAX_JOURNAL + 2):
            self.native(event=index + 1, sequence=1 + index * 2, clock=1. + index * .01)
        source = self.app.capture_source()
        self.assertEqual(len(source['poses']), MAX_JOURNAL)
        self.assertEqual(source['poses_dropped'], 2)
        self.assertEqual(source['poses_source_gaps'], MAX_JOURNAL + 1)
        self.assertEqual(source['pose_sample_semantics'], 'parent_accepted_latest_ipc')

    def test_asset_accessor_remains_bound_to_the_selected_immutable_job(self):
        self.start()
        source = self.app.capture_source()
        self.assertEqual(source['asset_bytes']('d' * 64), b'fixture-mesh')
        self.assertEqual(source['owner']['model_sha256'], fixtures.MODEL)
        self.assertEqual(source['owner']['domain'], 222)
        self.assertEqual(source['owner']['execution_manifest_sha256'], self.app.target.manifest_sha256)


if __name__ == '__main__':
    unittest.main()
