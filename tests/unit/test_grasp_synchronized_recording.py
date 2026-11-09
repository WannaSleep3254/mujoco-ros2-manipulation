"""Additive grasp recording identity, source clocks and read-only diagnostics."""
from contextlib import contextmanager
from copy import deepcopy
import hashlib
import os
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.operator_console.scene_host import _canonical
from mujoco_ros2_manipulation.operator_console.synchronized_recording import SyncRecording, SynchronizedSceneHost
from mujoco_ros2_manipulation.operator_console.synchronized_capture import CaptureLibrary
from mujoco_ros2_manipulation.operator_console.view_binding import make_binding, binding_id
from tests.unit.test_synchronized_recording import fixture, JPEG, JPEG_SHA
from tests.unit.test_scene_host import Clock
from tests.unit.test_view_http import handler


JOB = 'b' * 32


def grasp_fixture(*, media=True, asset=False):
    doc, assets, images = fixture(origin=10_000_000_000, asset=asset)
    binding = make_binding('e' * 32, 7, 'workcell:fixture', 'live', job_id=JOB, run_id=JOB)
    identity = dict(operator_session_id='a'*32, selection_id='f'*32, job_id=JOB, job_generation=7,
        binding=binding, binding_id=binding_id(binding), model_sha256=doc['manifest']['model_sha256'],
        execution_manifest_sha256='1'*64, domain=221, source_registration_epoch='2'*32,
        native_clock_domain='mujoco-native:'+JOB, native_stamp_semantics='rounded_native_mjData_time',
        image_clock_domain='ros_simulation', clock_association='declared_same_run',
        image_generation='c'*32 if media else None, publisher_endpoint_sha256='d'*64 if media else None,
        image_source_id='grasp_rgb', topic='/diagnostic/image')
    for index, item in enumerate(doc['poses']):
        capture = 100_000_000_000 + index * 100_000_000
        item['native'] = dict(time_sec=int(item['stamp_ns'])/1e9, capture_monotonic_ns=str(capture),
            observed_monotonic_ns=str(capture+1_000_000), progress_monotonic_ns=str(capture+1_000_000),
            stamp_semantics='rounded_native_mjData_time')
    for item in doc['images']:
        item['source'] = dict(generation='c'*32, source_id='grasp_rgb', topic='/diagnostic/image', frame_id='camera',
            publisher_endpoint_sha256='d'*64, clock_domain='ros_simulation',
            received_monotonic_sec=100.1, encoded_monotonic_sec=100.11)
    association = dict(available=True, reason='fresh_progressing_native_observation',
        stamp_ns=doc['poses'][1]['stamp_ns'], sequence=doc['poses'][1]['sequence'],
        capture_monotonic_ns=doc['poses'][1]['native']['capture_monotonic_ns'])
    if not media:
        doc['poses'], doc['images'], images = [], [], {}
        association = dict(available=False, reason='native_observation_unavailable', stamp_ns=None,
                           sequence=None, capture_monotonic_ns=None)
    doc.update(schema_version=2, kind='synchronized-grasp-recording-v2', identity=identity,
        origin_ns=doc['origin_ns'] if media else None, media_state='ready' if media else 'unavailable',
        media_reason=None if media else 'no_native_or_sensor_observations', task_status='cancelled' if not media else 'succeeded',
        pose_sample_semantics='parent_accepted_latest_ipc',
        observation_counters=dict(poses_dropped=0, events_dropped=0, poses_source_gaps=3,
                                  image_transport_gaps=1, image_counters={'raw_overwrites':2}),
        events=[dict(event_id=JOB+':1', event_sequence=1, emitted_monotonic_ns='100110000000',
            parent_received_monotonic_ns='100120000000', stage='approach', status='running', details={},
            native_association=association)])
    return doc, assets, images


class GraspRecordingTests(unittest.TestCase):
    def test_same_class_dispatch_preserves_v1_document_bytes_and_identity(self):
        old, assets, images = fixture()
        value = SyncRecording(old, assets, images)
        self.assertEqual(_canonical(old), _canonical(value.document))
        self.assertEqual(value.sha256, hashlib.sha256(_canonical(old)).hexdigest())
        self.assertEqual(value.version, 1)
        self.assertEqual(value.events, [])
        self.assertEqual(value.media_state, 'ready')

    def test_exact_native_image_and_source_event_bytes_survive_bundle_roundtrip(self):
        doc, assets, images = grasp_fixture(asset=True)
        value = SyncRecording(doc, assets, images)
        self.assertEqual(value.document, doc)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'recording.json'
            value.save(path)
            loaded = SyncRecording.load(path)
            self.assertEqual(loaded.sha256, value.sha256)
            self.assertEqual(loaded.document, doc)
            self.assertEqual(loaded.images[JPEG_SHA], JPEG)
            self.assertEqual(path.read_bytes(), _canonical(doc))

    def test_v2_loader_rejects_track_counts_before_opening_any_assets_or_images(self):
        for field, count in (('poses', 2401), ('images', 601), ('events', 129)):
            with self.subTest(field=field), tempfile.TemporaryDirectory() as directory:
                doc, _, _ = grasp_fixture(asset=True)
                doc[field] = [deepcopy(doc[field][0]) for _ in range(count)]
                path = Path(directory)/'recording.json'
                path.write_bytes(_canonical(doc))
                original_open = os.open
                with patch('mujoco_ros2_manipulation.operator_console.synchronized_recording.os.open',
                           wraps=original_open) as opened:
                    with self.assertRaisesRegex(ValueError, 'count'):
                        SyncRecording.load(path)
                self.assertTrue(all(call.args[0] not in ('assets', 'images') for call in opened.call_args_list))

    def test_event_only_cancel_diagnostic_has_null_timing_and_disabled_media(self):
        value = SyncRecording(*grasp_fixture(media=False))
        self.assertIsNone(value.origin_ns)
        self.assertIsNone(value.duration)
        host = SynchronizedSceneHost(value, clock=Clock())
        view = host.snapshot()
        self.assertEqual(view['recording_version'], 2)
        self.assertEqual(view['media_state'], 'unavailable')
        for key in ('position_sec', 'duration_sec', 'origin_ns'):
            self.assertIsNone(view[key])
        self.assertIsNone(view['selection']['cursor_ns'])
        self.assertIsNone(view['selection']['pose'])
        self.assertEqual(view['selection']['event_ids'], [])
        self.assertIsNone(view['events'][0]['time_sec'])
        for action in ('play', 'pause', 'previous', 'next', 'seek', 'speed'):
            with self.subTest(action=action), self.assertRaises(ValueError):
                host.control(action, position=0 if action == 'seek' else None,
                             speed=1 if action == 'speed' else None)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'recording.json'
            value.save(path)
            self.assertEqual(SyncRecording.load(path).document, value.document)

    def test_event_highlight_changes_atomically_with_both_tracks_and_retains_raw_source(self):
        doc, assets, images = grasp_fixture()
        original = deepcopy(doc['events'])
        host = SynchronizedSceneHost(SyncRecording(doc, assets, images), clock=Clock())
        before = host.control('seek', .05)
        after = host.control('seek', .1)
        self.assertEqual(before['selection']['event_ids'], [])
        self.assertEqual(after['selection']['event_ids'], [JOB+':1'])
        self.assertEqual(after['events'][0]['time_sec'], .1)
        self.assertEqual(host.recording.events, original)
        self.assertNotEqual(before['binding_id'], after['binding_id'])

    def test_late_arm_preserves_association_outside_retained_media_without_fake_seek(self):
        doc, assets, images = grasp_fixture()
        doc['events'][0]['native_association']['stamp_ns'] = '9000000000'
        original = deepcopy(doc['events'][0]['native_association'])
        view = SynchronizedSceneHost(SyncRecording(doc, assets, images), clock=Clock()).snapshot()
        self.assertEqual(view['events'][0]['native_association'], original)
        self.assertIsNone(view['events'][0]['time_sec'])
        self.assertEqual(view['events'][0]['media_seek_reason'], 'outside_retained_media')
        self.assertEqual(view['selection']['event_ids'], [])

    def test_cross_job_epoch_clock_and_native_fabrication_are_rejected(self):
        mutations = (
            lambda d: d['identity'].update(job_id='3'*32),
            lambda d: d['identity'].update(binding_id='0'*64),
            lambda d: d['identity'].update(model_sha256='0'*64),
            lambda d: d['identity'].update(image_clock_domain='unknown'),
            lambda d: d['identity'].update(clock_association='inferred_from_clock'),
            lambda d: d['identity'].update(image_generation=None),
            lambda d: d['poses'][0]['native'].update(time_sec=10.01),
            lambda d: d['poses'][0]['native'].update(capture_monotonic_ns=100_000_000_000),
            lambda d: d['images'][0]['source'].update(generation='4'*32),
            lambda d: d['images'][0]['source'].update(publisher_endpoint_sha256='4'*64),
            lambda d: d['images'][0]['source'].update(encoded_monotonic_sec=99.),
            lambda d: d['events'][0].update(event_id='other:1'),
            lambda d: d['events'][0].update(parent_received_monotonic_ns='1'),
            lambda d: d['events'][0]['native_association'].update(capture_monotonic_ns='1'),
            lambda d: d['events'][0]['native_association'].update(available=False),
            lambda d: d.update(events=d['events']*2),
            lambda d: d.update(task_status=0),
            lambda d: d.update(origin_ns='0'),
            lambda d: d.update(pose_sample_semantics='every_native_physics_step'),
            lambda d: d['observation_counters'].update(poses_dropped=True),
            lambda d: d['observation_counters'].update(image_transport_gaps=-1),
            lambda d: d['observation_counters']['image_counters'].update(encoded=1.5),
            lambda d: d['observation_counters'].update(extra=0),
        )
        for mutate in mutations:
            with self.subTest(mutation=mutate):
                doc, assets, images = grasp_fixture()
                mutate(doc)
                with self.assertRaises(ValueError):
                    SyncRecording(doc, assets, images)

    def test_native_sequence_gaps_and_image_gaps_remain_source_facts(self):
        doc, assets, images = grasp_fixture()
        doc['poses'][1]['sequence'] = 11
        value = SyncRecording(doc, assets, images)
        view = SynchronizedSceneHost(value, clock=Clock()).control('seek', .8)
        self.assertEqual(view['selection']['pose_status'], 'missing')
        self.assertEqual(view['selection']['image_status'], 'missing')
        self.assertEqual(value.document, doc)


class Image:
    def __init__(self, document):
        self.document = document
        identity = document['identity']
        self.info = dict(configured=True, clock_domain='ros_simulation', clock_invalid=False,
            publisher_count=1, status='ready', generation='c'*32, source_id='grasp_rgb',
            topic='/diagnostic/image', publisher_endpoint_sha256='d'*64, history=[],
            transport_gaps=1, counters={'raw_overwrites':2})
        self.frames = {}
        for item in document['images']:
            descriptor = dict(item['source'], sequence=item['sequence'], source_stamp_ns=item['stamp_ns'],
                jpeg_sha256=JPEG_SHA, byte_length=len(JPEG), width=2, height=2, image_available=True)
            self.info['history'].append(descriptor)
            self.frames[descriptor['sequence']] = descriptor
    def snapshot(self):
        return deepcopy(self.info)
    def frame(self, generation, sequence):
        if generation != self.info['generation'] or sequence not in self.frames:
            return None
        return deepcopy(self.frames[sequence]), JPEG


class Tasks:
    selected = 'grasp'
    selection_id = 'f'*32
    def __init__(self, source): self.source = source
    def require_capture_selection(self): pass
    def capture_context(self):
        result = dict(self.source)
        result.update(owner=deepcopy(self.source['owner']), poses=deepcopy(self.source['poses']),
                      events=deepcopy(self.source['events']), selection_id=self.selection_id)
        return result
    @contextmanager
    def capture_admission(self, owner):
        if owner['native_owner'] != self.source['owner'] or self.selection_id != owner['selection_id']:
            raise ValueError('source changed')
        yield


class DeferredThread:
    def __init__(self, *, target, args, **kwargs): self.target, self.args = target, args
    def start(self): pass
    def is_alive(self): return False
    def run(self): self.target(*self.args)


class GraspCaptureTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.document, self.assets, _ = grasp_fixture()
        identity = self.document['identity']
        owner = dict(job_id=JOB, generation=7, binding=identity['binding'], binding_id=identity['binding_id'],
            model_sha256=identity['model_sha256'], execution_manifest_sha256='1'*64, domain=221,
            source_registration_epoch='2'*32)
        self.image = Image(self.document)
        self.source = dict(owner=owner, manifest=self.document['manifest'], image=self.image,
            asset_bytes=lambda digest:self.assets[digest], poses=self.document['poses'], events=self.document['events'],
            poses_dropped=0, events_dropped=0, poses_source_gaps=3, finished=False, fault=None, job_status='running')
        self.app = SimpleNamespace(session_id='a'*32, directory=Path(self.temp.name), lock=threading.RLock(),
            tasks=Tasks(self.source), bridge=SimpleNamespace(image_source=None), audit=lambda *args,**kwargs:None)
        self.library = CaptureLibrary(self.app, clock=lambda:100.)
    def arm(self):
        token = self.library.snapshot()['capture']['expected_owner_token']
        poses = self.source['poses']
        self.source['poses'] = []
        try:
            with patch('mujoco_ros2_manipulation.operator_console.synchronized_capture.threading.Thread', DeferredThread):
                return self.library.start(token)
        finally:
            self.source['poses'] = poses

    def test_arm_before_first_native_or_rgb_and_exact_token_stop(self):
        self.source['poses'], self.source['events'] = [], []
        self.image.info.update(publisher_count=0, publisher_endpoint_sha256=None, history=[])
        capture = self.arm()
        self.assertEqual(capture['status'], 'armed')
        self.assertTrue(capture['awaiting_sources'])
        self.assertEqual(capture['owner']['job_id'], JOB)
        with self.assertRaises(ValueError): self.library.stop(capture['id'], '0'*64)
        self.assertFalse(self.library.stop_event.is_set())
        self.library.stop(capture['id'], capture['owner_token'])
        self.assertTrue(self.library.stop_event.is_set())

    def test_terminal_cancel_without_native_is_saved_as_valid_event_only_diagnostic(self):
        doc, _, _ = grasp_fixture(media=False)
        self.source.update(poses=[], events=doc['events'])
        self.image.info.update(publisher_count=0, publisher_endpoint_sha256=None, history=[])
        capture = self.arm()
        self.source.update(finished=True, job_status='cancelled')
        self.library.thread.run()
        self.assertEqual(self.library.current['status'], 'completed')
        value = self.library.records[capture['id']]
        self.assertIsNone(value.origin_ns)
        self.assertEqual(value.document['task_status'], 'cancelled')
        self.assertEqual(value.events, doc['events'])
        self.assertEqual(SyncRecording.load(self.app.directory/'synchronized'/capture['id']/'recording.json').sha256,
                         value.sha256)

    def test_original_native_and_rgb_rows_auto_finish_and_gap_is_not_source_change(self):
        self.image.info['status'] = 'stale'
        capture = self.arm()
        self.source.update(finished=True, job_status='succeeded')
        self.library.thread.run()
        self.assertEqual(self.library.current['status'], 'completed')
        value = self.library.records[capture['id']]
        self.assertEqual(value.poses, self.document['poses'])
        self.assertEqual(value.image_frames, self.document['images'])
        self.assertEqual(value.events, self.document['events'])
        self.assertEqual(value.document['observation_counters'], self.document['observation_counters'])
        self.assertEqual(value.document['pose_sample_semantics'], 'parent_accepted_latest_ipc')
        self.assertEqual(self.library.get(capture['id']).snapshot()['observation_counters'],
                         self.document['observation_counters'])
        self.assertEqual(value.ended_reason, 'task_finished')

    def test_completed_capture_exposes_task_status_separately_from_save_status(self):
        capture = self.arm()
        self.source.update(finished=True, job_status='succeeded')
        self.library.thread.run()
        self.assertEqual(self.library.records[capture['id']].ended_reason, 'task_finished')
        catalog = self.library.snapshot()
        self.assertEqual(catalog['capture']['task_status'], 'succeeded')
        self.assertEqual(catalog['recordings'][0]['task_status'], 'succeeded')
        self.assertEqual(catalog['capture']['status'], 'completed')

    def test_ring_boundary_uses_accepted_indices_and_retains_next_unconsumed_row(self):
        # Admission observed 128 earlier accepted rows, whose native sequences
        # are allowed to jump. Evicting exactly those consumed rows loses none.
        self.source['poses'] = [deepcopy(self.document['poses'][0]) for _ in range(128)]
        token = self.library.snapshot()['capture']['expected_owner_token']
        with patch('mujoco_ros2_manipulation.operator_console.synchronized_capture.threading.Thread', DeferredThread):
            capture = self.library.start(token)
        self.source.update(poses_dropped=128, poses=deepcopy(self.document['poses']),
                           finished=True, job_status='succeeded')
        self.library.thread.run()
        self.assertEqual(self.library.records[capture['id']].ended_reason, 'task_finished')
        self.assertEqual(self.library.records[capture['id']].poses, self.document['poses'])

    def test_actual_unconsumed_ring_eviction_ends_without_filling_missing_rows(self):
        capture = self.arm()
        self.source.update(poses_dropped=1, finished=True)
        self.library.thread.run()
        value = self.library.records[capture['id']]
        self.assertEqual(value.ended_reason, 'source_journal_limit')
        self.assertEqual(value.poses, [])
        self.assertEqual(value.events, self.document['events'])

    def test_final_source_events_drain_before_terminal_image_fault(self):
        self.source['events'] = []
        capture = self.arm()
        final = deepcopy(self.document['events'][0])
        final.update(stage='cancelled', status='cancelled')
        self.source.update(finished=True, job_status='cancelled', events=[final], fault='JPEG encoding failed')
        self.image.info['status'] = 'error'
        self.library.thread.run()
        value = self.library.records[capture['id']]
        self.assertEqual(value.events, [final])
        self.assertEqual(value.document['task_status'], 'cancelled')
        self.assertEqual(value.ended_reason, 'recording_source_fault')
        self.assertEqual(value.media_state, 'unavailable')

    def test_clock_reversal_precedes_its_automatically_changed_image_generation(self):
        capture = self.arm()
        self.image.info.update(clock_invalid=True, generation='4'*32)
        self.library.thread.run()
        self.assertEqual(self.library.records[capture['id']].ended_reason, 'clock_reversed')

    def test_duration_jump_is_excluded_before_exceeding_retained_track_budget(self):
        capture = self.arm()
        last = self.source['poses'][-1]
        last['stamp_ns'] = '131000000000'
        last['native']['time_sec'] = 131.
        self.source.update(finished=True, job_status='succeeded')
        self.library.thread.run()
        value = self.library.records[capture['id']]
        self.assertEqual(value.ended_reason, 'duration_limit')
        self.assertEqual(len(value.poses), 3)
        self.assertTrue(all(int(item['stamp_ns']) <= 120_000_000_000 for item in value.poses))

    def test_stale_owner_token_cannot_arm_or_fetch_even_if_image_generation_is_reused(self):
        token = self.library.snapshot()['capture']['expected_owner_token']
        self.source['owner']['generation'] += 1
        with self.assertRaises(ValueError): self.library.start(token)
        with self.assertRaises(ValueError): self.library.image_source(token)
        self.assertEqual(self.library.current['status'], 'idle')

    def test_owner_rechecked_after_slow_assets_before_admission(self):
        doc, assets, _ = grasp_fixture(asset=True)
        self.source['manifest'], self.source['asset_bytes'] = doc['manifest'], lambda digest:assets[digest]
        original = self.source['asset_bytes']
        def changed(digest):
            result = original(digest)
            self.app.tasks.selection_id = '3'*32
            return result
        self.source['asset_bytes'] = changed
        token = self.library.snapshot()['capture']['expected_owner_token']
        with self.assertRaises(ValueError): self.library.start(token)
        self.assertEqual(self.library.current['status'], 'failed')
        self.assertIsNone(self.library.thread)
        self.assertEqual(self.library.retained_count, 0)

    def test_publisher_epoch_replacement_ends_capture_without_adopting_new_source(self):
        capture = self.arm()
        self.image.info.update(generation='4'*32, publisher_endpoint_sha256='5'*64)
        self.library.thread.run()
        value = self.library.records[capture['id']]
        self.assertEqual(value.ended_reason, 'source_changed')
        self.assertEqual(value.identity['image_generation'], 'c'*32)
        self.assertEqual(value.image_frames, [])

    def test_capture_failure_retains_raw_partial_and_cannot_call_control(self):
        capture = self.arm()
        self.source.update(finished=True, job_status='succeeded')
        self.source['poses'][0]['native']['time_sec'] += .01
        self.library.thread.run()
        self.assertEqual(self.library.current['status'], 'failed')
        folder = self.app.directory/'synchronized'/capture['id']
        self.assertTrue((folder/'partial.json').is_file())
        self.assertIn(b'events', (folder/'partial.json').read_bytes())
        self.assertEqual((folder/'images'/str(JPEG_SHA+'.jpg')).read_bytes(), JPEG)
        self.assertNotIn(capture['id'], self.library.records)


class GraspCaptureHTTPTests(unittest.TestCase):
    setUp = GraspCaptureTests.setUp
    def request(self, path, *, payload=None, selection=None):
        value, response, app = handler(path, method='GET' if payload is None else 'POST', payload=payload)
        app.sync_recordings = self.library
        app.tasks = SimpleNamespace(selected='grasp', capture_request=self.capture_request)
        if selection is not None:
            value.headers['X-Workspace-Selection'] = selection
        return value, response

    @contextmanager
    def capture_request(self, headers):
        if headers != [self.app.tasks.selection_id]:
            raise ValueError('selection changed')
        yield

    def test_grasp_start_and_stop_require_header_and_exact_retained_owner(self):
        token = self.library.snapshot()['capture']['expected_owner_token']
        value, response = self.request('/api/sync-recordings/start', payload={'expected_owner_token':token})
        value.do_POST()
        self.assertEqual(response.call_args.args[1], 400)
        self.assertEqual(self.library.current['status'], 'idle')
        value, response = self.request('/api/sync-recordings/start', payload={'expected_owner_token':token},
                                       selection=self.app.tasks.selection_id)
        with patch('mujoco_ros2_manipulation.operator_console.synchronized_capture.threading.Thread', DeferredThread):
            value.do_POST()
        self.assertEqual(response.call_args.args[1], 202)
        current = self.library.current
        value, response = self.request('/api/sync-recordings/stop',
            payload={'id':current['id'], 'expected_owner_token':token}, selection=self.app.tasks.selection_id)
        value.do_POST()
        self.assertEqual(response.call_args.args[1], 202)
        self.assertTrue(self.library.stop_event.is_set())

    def test_grasp_image_http_never_accepts_missing_duplicate_or_stale_owner(self):
        token = self.library.snapshot()['capture']['expected_owner_token']
        for query in ('', '?owner_token='+'0'*64, '?owner_token='+token+'&owner_token='+token,
                      '?owner_token='+token+'&topic=/private'):
            value, response = self.request('/api/image-source'+query)
            value.do_GET()
            self.assertEqual(response.call_args.args[1], 400)
        value, response = self.request('/api/image-source?owner_token='+token)
        value.do_GET()
        self.assertEqual(response.call_args.args[0]['owner_token'], token)
        seq = self.document['images'][0]['sequence']
        path = '/api/image-source/frame?generation='+'c'*32+'&sequence='+str(seq)+'&owner_token='+token
        value, response = self.request(path)
        value.do_GET()
        self.assertEqual(response.call_args.args[0], JPEG)
        self.source['owner']['generation'] += 1
        value, response = self.request(path)
        value.do_GET()
        self.assertEqual(response.call_args.args[1], 400)

    def test_image_owner_is_rechecked_after_frame_io(self):
        token = self.library.snapshot()['capture']['expected_owner_token']
        original = self.image.frame
        def changed(generation, sequence):
            result = original(generation, sequence)
            self.source['owner']['generation'] += 1
            return result
        self.image.frame = changed
        seq = self.document['images'][0]['sequence']
        value, response = self.request('/api/image-source/frame?generation='+'c'*32+'&sequence='+str(seq)+'&owner_token='+token)
        value.do_GET()
        self.assertEqual(response.call_args.args[1], 400)
        self.assertNotEqual(response.call_args.args[0], JPEG)
