"""Startup-selected replay sources stay immutable and separate from live input."""
from copy import deepcopy
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from mujoco_ros2_manipulation.operator_console.synchronized_capture import CaptureLibrary, load_saved_recordings, _saved_size
from mujoco_ros2_manipulation.operator_console.synchronized_recording import SyncRecording
from tests.unit.test_synchronized_capture import Fixture
from tests.unit.test_synchronized_recording import fixture, recording


class SavedRecordingTests(unittest.TestCase):
    def test_startup_loader_checks_count_before_reading_and_loads_in_declared_order(self):
        value = recording()
        with patch.object(SyncRecording, 'load', return_value=value) as load:
            with self.assertRaisesRegex(ValueError, '8개'):
                load_saved_recordings(['record.json'] * 9)
            load.assert_not_called()
            self.assertEqual(load_saved_recordings(['first.json', 'second.json']), (value, value))
            self.assertEqual([call.args for call in load.call_args_list], [('first.json',), ('second.json',)])

    def test_startup_loader_stops_before_retaining_or_reading_the_rest_of_an_oversized_batch(self):
        value = recording()
        size = _saved_size(value)
        with patch.object(SyncRecording, 'load', return_value=value) as load:
            with patch('mujoco_ros2_manipulation.operator_console.synchronized_capture.MAX_CAPTURE_BYTES', size - 1):
                with self.assertRaisesRegex(ValueError, '용량'):
                    load_saved_recordings(['first.json', 'unread.json', 'also-unread.json'])
            self.assertEqual(load.call_count, 1)
        with patch.object(SyncRecording, 'load', return_value=value) as load:
            with patch('mujoco_ros2_manipulation.operator_console.synchronized_capture.MAX_LIBRARY_BYTES', size * 2 - 1):
                with self.assertRaisesRegex(ValueError, '용량'):
                    load_saved_recordings(['first.json', 'second.json', 'unread.json'])
            self.assertEqual(load.call_count, 2)

    def test_startup_loader_invalid_original_stops_without_reading_later_paths(self):
        with patch.object(SyncRecording, 'load', side_effect=ValueError('original rejected')) as load:
            with self.assertRaisesRegex(ValueError, 'original rejected'):
                load_saved_recordings(['invalid.json', 'unread.json'])
            self.assertEqual(load.call_count, 1)

    def test_explicit_startup_record_retains_original_identity_and_immutable_bytes(self):
        value = recording(asset=True)
        app = SimpleNamespace()
        library = CaptureLibrary(app)
        identifier = library.add_saved(value)
        self.assertEqual(identifier, value.sha256[:32])
        self.assertEqual(library.current['status'], 'idle')
        self.assertEqual(library.retained_count, 1)
        host = library.get(identifier)
        self.assertIsNot(host.recording, value)
        self.assertEqual(host.recording.identity, value.identity)
        self.assertEqual(host.recording.origin_ns, value.origin_ns)
        selected = host.control('seek', .1)
        self.assertEqual(selected['selection']['cursor_ns'], str(value.origin_ns + 100_000_000))
        self.assertEqual(host.recording.document, value.document)
        self.assertNotEqual(selected['binding']['operator_session_id'], value.identity['operator_session_id'])
        library.close()
        self.assertTrue(host.closed)

    def test_duplicate_seed_does_not_consume_another_slot_or_change_cursor(self):
        library = CaptureLibrary(SimpleNamespace())
        value = recording()
        identifier = library.add_saved(value)
        library.get(identifier).control('seek', .7)
        size = library.bytes
        self.assertEqual(library.add_saved(value), identifier)
        self.assertEqual(library.retained_count, 1)
        self.assertEqual(library.bytes, size)
        self.assertAlmostEqual(library.get(identifier).snapshot()['position_sec'], .7)
        library.close()

    def test_startup_import_limits_do_not_publish_a_partial_source(self):
        library = CaptureLibrary(SimpleNamespace())
        doc, assets, images = fixture()
        for index in range(8):
            changed = deepcopy(doc)
            changed['label'] = 'saved '+str(index)
            library.add_saved(SyncRecording(changed, assets, images))
        self.assertEqual(library.retained_count, 8)
        changed = deepcopy(doc)
        changed['label'] = 'overflow'
        with self.assertRaisesRegex(ValueError, '상한'):
            library.add_saved(SyncRecording(changed, assets, images))
        self.assertEqual(len(library.records), 8)
        self.assertEqual(len(library.hosts), 8)
        library.close()

    def test_unknown_path_wrong_type_oversized_and_after_collection_are_rejected(self):
        library = CaptureLibrary(SimpleNamespace())
        with self.assertRaises(ValueError):
            library.add_saved('/tmp/recording.json')
        with patch('mujoco_ros2_manipulation.operator_console.synchronized_capture.MAX_CAPTURE_BYTES', 1):
            with self.assertRaisesRegex(ValueError, '용량'):
                library.add_saved(recording())
        self.assertEqual(library.records, {})
        library.current['status'] = 'recording'
        with self.assertRaisesRegex(ValueError, '시작 설정'):
            library.add_saved(recording())
        library.current['status'] = 'idle'
        library.request_close()
        with self.assertRaisesRegex(ValueError, '시작 설정'):
            library.add_saved(recording())

    def test_non_motion_task_cannot_capture_old_normal_pose_and_sensor_state(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Fixture(directory)
            source.app.require_motion_task = Mock(side_effect=RuntimeError('현재 집기 작업을 선택했습니다.'))
            state = source.library.snapshot()
            self.assertFalse(state['capture']['can_start'])
            self.assertIn('집기', state['capture']['availability_reason'])
            with self.assertRaisesRegex(ValueError, '집기'):
                source.library.start()
            self.assertEqual(source.library.records, {})
            self.assertIsNone(source.library.thread)


if __name__ == '__main__':
    unittest.main()
