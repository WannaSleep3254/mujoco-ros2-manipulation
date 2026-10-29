"""Acquisition epochs, gaps, preservation and read-only HTTP boundaries."""
from copy import deepcopy
import hashlib
import tempfile
import threading
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from mujoco_ros2_manipulation.operator_console.image_source import ImageSource
from mujoco_ros2_manipulation.operator_console.recording import demo_recording
from mujoco_ros2_manipulation.operator_console.scene_host import _canonical
from mujoco_ros2_manipulation.operator_console.synchronized_capture import CaptureLibrary
from mujoco_ros2_manipulation.operator_console.synchronized_recording import SyncRecording
from mujoco_ros2_manipulation.operator_console.view_binding import make_binding, binding_id
from tests.unit.test_view_http import handler


class Fixture:
    def __init__(self, root):
        self.now = 10.
        self.step = 0
        self.model = demo_recording()
        self.binding = make_binding('a'*32, 1, 'workcell:test', 'live', motion_session_id='b'*32)
        self.identifier = binding_id(self.binding)
        self.image = ImageSource(topic='/diagnostic/image', clock=lambda:self.now, clock_domain='ros_simulation')
        self.image.set_publishers(['c'*64])
        self.publish()
        self.host = NS(snapshot=lambda:dict(status='ready',pose_ready=True,binding=deepcopy(self.binding),binding_id=self.identifier),
            capture_sample=lambda identifier:self.sample(), manifest_bytes=lambda identifier:_canonical(self.model.manifest),
            asset_bytes=lambda digest:self.model.assets[digest])
        self.app = NS(lock=threading.RLock(),state=NS(lock=threading.RLock(),identity=True,clock_backwards=False,clock_advanced_at=self.now,
            graph={'clock':1,'joints':1},server={'session_id':'b'*32}),closing=threading.Event(),phase='running',
            demos=NS(busy=False),display_mode='web3d',views=NS(host=self.host,mode='live'),bridge=NS(image_source=self.image),
            session_id='a'*32,generation=1,directory=Path(root),audit=Mock())
        self.library = CaptureLibrary(self.app,clock=lambda:self.now)

    def publish(self):
        stamp = round(self.now*1e9)
        self.image.observe_clock_stamp(stamp//10**9,stamp%10**9)
        self.image.receive(width=2,height=2,encoding='rgb8',step=6,data=bytes([self.step%256,0,0])*4,
            stamp_sec=stamp//10**9,stamp_nanosec=stamp%10**9,frame_id='diagnostic',generation=self.image.generation)
        self.image.process_once()

    def sample(self):
        frame = self.model.frames[self.step%len(self.model.frames)]
        return dict(sequence=self.step+1,source_stamp_ns=str(round(self.now*1e9)),
            source_received_monotonic=self.now,positions_rad=deepcopy(frame['positions_rad']),pose=deepcopy(frame['pose']))

    def collect(self, *, gap=False, change=False, count=18):
        with patch('mujoco_ros2_manipulation.operator_console.synchronized_capture.threading.Thread'):
            started = self.library.start()
        identifier = started['id']
        original = self.library._context
        def context(**kwargs):
            self.step += 1
            self.now = 10.+self.step*.2
            if not gap or self.step < 5 or self.step > 12:
                self.publish()
            if change and self.step == 7:
                self.image.new_epoch()
                self.publish()
            value = original(**kwargs)
            if self.step >= count:
                self.library.stop_event.set()
            return value
        self.library._context = context
        self.library.stop_event.wait = lambda timeout:False
        args=(identifier,self.host,self.image,original()[-1],self.model.manifest,{})
        self.library._run(*args)
        return identifier


class CaptureTests(unittest.TestCase):
    def test_capture_preserves_exact_tracks_and_publishes_checked_immutable_bundle(self):
        with tempfile.TemporaryDirectory() as directory:
            f=Fixture(directory);identifier=f.collect()
            self.assertEqual(f.library.snapshot()['capture']['status'],'completed')
            recording=SyncRecording.load(Path(directory)/'synchronized'/identifier/'recording.json')
            self.assertEqual(recording.identity['motion_session_id'],'b'*32)
            self.assertGreater(len(recording.poses),2)
            self.assertGreater(len(recording.image_frames),1)
            self.assertTrue(all(isinstance(item['stamp_ns'],str) for item in recording.poses))
            self.assertEqual(f.library.get(identifier).recording.sha256,recording.sha256)
            self.assertFalse(f.app.audit.called)

    def test_image_gap_is_retained_as_missing_instead_of_recent_frame_replacement(self):
        with tempfile.TemporaryDirectory() as directory:
            f=Fixture(directory);identifier=f.collect(gap=True)
            self.assertEqual(f.library.current['status'],'completed')
            host=f.library.get(identifier)
            # Cursor absolute 12s lies inside the intentional 11..12.4s image gap.
            cursor=12.-host.recording.origin_ns/1e9
            view=host.control('seek',cursor)
            self.assertEqual(view['selection']['pose_status'],'ready')
            self.assertEqual(view['selection']['image_status'],'missing')
            self.assertIsNone(view['selection']['image'])

    def test_new_sensor_generation_ends_previous_record_instead_of_combining_epochs(self):
        with tempfile.TemporaryDirectory() as directory:
            f=Fixture(directory);identifier=f.collect(change=True)
            self.assertEqual(f.library.current['status'],'completed')
            self.assertEqual(f.library.get(identifier).recording.ended_reason,'source_changed')
            self.assertNotEqual(f.library.get(identifier).recording.identity['image_generation'],f.image.generation)

    def test_admission_rejects_unknown_clock_preview_and_non_unique_sources(self):
        for mutate in (lambda f:setattr(f.image,'clock_domain','unknown'),
                       lambda f:setattr(f.app.views,'mode','plan_preview'),
                       lambda f:f.app.state.graph.update(joints=2),
                       lambda f:setattr(f.app.state,'clock_advanced_at',f.now-1.)):
            with tempfile.TemporaryDirectory() as directory:
                f=Fixture(directory);mutate(f)
                with self.assertRaises(ValueError):f.library.start()
                self.assertEqual(f.library.current['status'],'failed')
                self.assertEqual(f.library.records,{})

    def test_insufficient_capture_preserves_diagnostic_but_has_no_playable_record(self):
        with tempfile.TemporaryDirectory() as directory:
            f=Fixture(directory);identifier=f.collect(count=1)
            self.assertEqual(f.library.current['status'],'failed')
            self.assertTrue((Path(directory)/'synchronized'/identifier/'partial.json').is_file())
            self.assertFalse((Path(directory)/'synchronized'/identifier/'recording.json').exists())
            with self.assertRaises(ValueError):f.library.get(identifier)

    def test_stop_never_dispatches_and_rejects_wrong_record_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            f=Fixture(directory)
            with patch('mujoco_ros2_manipulation.operator_console.synchronized_capture.threading.Thread'):
                capture=f.library.start()
            with self.assertRaises(ValueError):f.library.stop('d'*32)
            f.library.stop(capture['id'])
            self.assertTrue(f.library.stop_event.is_set())
            self.assertEqual(f.library.records,{})


class CaptureHTTPTests(unittest.TestCase):
    def test_controls_require_exact_origin_and_token_and_do_not_accept_paths(self):
        for token,origin in [('wrong','http://127.0.0.1:8890'),('secret','https://example.test')]:
            value,respond,app=handler('/api/sync-recordings/start',method='POST',payload={},token=token,origin=origin)
            app.sync_recordings=Mock();value.do_POST()
            self.assertEqual(respond.call_args.args[1],403);app.sync_recordings.start.assert_not_called()
        value,respond,app=handler('/api/sync-recordings/start',method='POST',payload={'path':'/tmp/private'})
        app.sync_recordings=Mock();value.do_POST()
        self.assertEqual(respond.call_args.args[1],404);app.sync_recordings.start.assert_not_called()

    def test_recording_selected_by_registered_id_and_snapshot_contains_both_tracks(self):
        value,respond,app=handler('/api/sync-recordings/'+'a'*32)
        selected={'selection':{'pose':{'sequence':3},'image':{'sequence':7}}}
        host=Mock();host.snapshot.return_value=selected
        app.sync_recordings=Mock();app.sync_recordings.get.return_value=host
        value.do_GET();app.sync_recordings.get.assert_called_once_with('a'*32)
        self.assertEqual(respond.call_args.args[0]['view'],selected)

    def test_manifest_keeps_binding_guard_and_control_only_accepts_playback_fields(self):
        value,respond,app=handler('/api/sync-recordings/'+'a'*32+'/manifest?binding_id='+'b'*64)
        host=Mock();host.manifest_bytes.return_value=b'{}'
        app.sync_recordings=Mock();app.sync_recordings.get.return_value=host
        value.do_GET();host.manifest_bytes.assert_called_once_with('b'*64)
        for payload in ({'action':'seek','time_sec':1.,'speed':2}, {'action':'play','command':'run'}):
            value,respond,app=handler('/api/sync-recordings/'+'a'*32+'/control',method='POST',payload=payload)
            app.sync_recordings=Mock();value.do_POST();self.assertEqual(respond.call_args.args[1],400)
            app.sync_recordings.get.assert_not_called()


if __name__=='__main__':unittest.main()
