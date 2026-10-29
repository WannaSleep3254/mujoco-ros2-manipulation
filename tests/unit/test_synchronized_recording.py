"""Immutable two-track replay: exact epochs, missing observations and safe bundles.

No ROS, simulator, image codec, GL, sockets or physical execution is involved.
"""
import base64
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.operator_console.display_contract import scene_digest
from mujoco_ros2_manipulation.operator_console.recording import demo_recording
from mujoco_ros2_manipulation.operator_console.scene_host import _canonical
from mujoco_ros2_manipulation.operator_console.synchronized_recording import (
    MAX_STAMP_NS, SyncRecording, SynchronizedSceneHost)
from mujoco_ros2_manipulation.operator_console.view_binding import make_binding, binding_id
from tests.unit.test_scene_host import Clock


JPEG = base64.b64decode(
    '/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAgGBgcGBQgHBwcJCQgKDBQNDAsLDBkSEw8UHRofHh0aHBwgJC4nICIsIxwcKDcpLDAx'
    'NDQ0Hyc5PTgyPC4zNDL/2wBDAQkJCQwLDBgNDRgyIRwhMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIy'
    'MjIyMjIyMjL/wAARCAACAAIDASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUF'
    'BAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVW'
    'V1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi'
    '4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREAAgECBAQDBAcFBAQAAQJ3AAEC'
    'AxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVm'
    'Z2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq'
    '8vP09fb3+Pn6/9oADAMBAAIRAxEAPwDAooor5g/TT//Z')
JPEG_SHA = hashlib.sha256(JPEG).hexdigest()
ORIGIN = 1_793_234_567_123_456_789


def fixture(*, pose_offsets=(0, 100_000_000, 900_000_000, 1_200_000_000),
            image_offsets=(0, 200_000_000, 1_000_000_000), origin=ORIGIN, asset=False):
    source = demo_recording()
    manifest = source.manifest
    assets = {}
    if asset:
        raw = struct.pack('<9f3I', 0., 0., 0., 1., 0., 0., 0., 1., 0., 0, 1, 2)
        digest = hashlib.sha256(raw).hexdigest()
        manifest['assets'] = [dict(sha256=digest, byte_length=len(raw), media_type='application/octet-stream',
            layout=dict(positions=dict(byte_offset=0, byte_length=36, count=3, item_size=3, component_type='float32'),
                        indices=dict(byte_offset=36, byte_length=12, count=3, item_size=1, component_type='uint32')))]
        manifest['geoms'][0].update(type='mesh', asset_sha256=digest)
        manifest['scene_sha256'] = scene_digest(manifest)
        assets[digest] = raw
    binding = make_binding('a'*32, 7, 'workcell:fixture', 'live', motion_session_id='b'*32)
    identity = dict(binding=binding, operator_session_id='a'*32, motion_session_id='b'*32,
                    app_generation=2, image_generation='c'*32, image_source_id='camera0',
                    topic='/camera/image', publisher_endpoint_sha256='d'*64, clock_domain='ros_simulation')
    poses = []
    for index, offset in enumerate(pose_offsets):
        frame = deepcopy(source.frames[index])
        frame['pose']['scene_sha256'] = manifest['scene_sha256']
        poses.append(dict(stamp_ns=str(origin+offset), sequence=10+index*3,
                          positions_rad=frame['positions_rad'], pose=frame['pose']))
    images = [dict(stamp_ns=str(origin+offset), sequence=20+index*4, jpeg_sha256=JPEG_SHA,
                   byte_length=len(JPEG), width=2, height=2) for index, offset in enumerate(image_offsets)]
    doc = dict(schema_version=1, kind='synchronized-recording-v1', label='Synthetic acquisition fixture',
               manifest=manifest, identity=identity, origin_ns=str(origin), poses=poses,
               images=images, ended_reason='user_stop')
    return doc, assets, {JPEG_SHA: JPEG}


def recording(**kwargs):
    return SyncRecording(*fixture(**kwargs))


class SynchronizedRecordingTests(unittest.TestCase):
    def test_epoch_nanoseconds_are_preserved_without_float_roundtrip(self):
        value = recording(pose_offsets=(0, 1, 2), image_offsets=(0, 1, 2))
        self.assertEqual(ORIGIN/1e9, (ORIGIN+1)/1e9)
        self.assertEqual(value.origin_ns, ORIGIN)
        self.assertEqual(value.duration_ns, 2)
        host = SynchronizedSceneHost(value, clock=Clock())
        selected = host.control('seek', 1e-9)['selection']
        self.assertEqual(selected['cursor_ns'], str(ORIGIN+1))
        self.assertEqual(selected['pose']['source']['stamp_ns'], str(ORIGIN+1))
        self.assertEqual(selected['image']['stamp_ns'], str(ORIGIN+1))
        self.assertEqual(selected['pose_time_sec'], 1e-9)

    def test_recording_input_and_outputs_are_defensive_and_bytes_are_read_only(self):
        doc, assets, images = fixture(asset=True)
        value = SyncRecording(doc, assets, images)
        original_sha = value.sha256
        doc['poses'][0]['pose']['geoms'][0]['position'][0] = 99
        doc['identity']['binding']['generation'] = 300
        assets.clear(); images.clear()
        for copy in (value.document, value.manifest, value.identity, value.poses, value.image_frames):
            if isinstance(copy, dict): copy.clear()
            else: copy.clear()
        self.assertEqual(value.sha256, original_sha)
        self.assertEqual(value.identity['binding']['generation'], 7)
        self.assertEqual(value.poses[0]['pose']['geoms'][0]['position'][0], 0)
        self.assertEqual(value.images[JPEG_SHA], JPEG)
        with self.assertRaises(TypeError): value.images[JPEG_SHA] = b'changed'
        self.assertEqual(len(value.assets), 1)

    def test_clock_binding_and_session_declarations_are_required(self):
        mutations = (
            lambda d: d['identity'].update(clock_domain='unknown'),
            lambda d: d['identity'].update(operator_session_id='e'*32),
            lambda d: d['identity'].update(motion_session_id='e'*32),
            lambda d: d['identity'].update(image_generation='../other'),
            lambda d: d['identity'].update(app_generation=True),
            lambda d: d['identity'].update(topic='/camera//image'),
            lambda d: d['identity'].update(topic='https://external/image'),
            lambda d: d['identity'].update(publisher_endpoint_sha256='unknown'),
            lambda d: d['identity']['binding'].update(mode='model_preview'),
            lambda d: d['identity'].update(extra='unreviewed'),
        )
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                doc, assets, images = fixture(); mutate(doc)
                with self.assertRaises(ValueError): SyncRecording(doc, assets, images)

    def test_bad_exact_timestamps_sequences_and_epoch_ranges_are_rejected(self):
        mutations = (
            lambda d: d.update(origin_ns=ORIGIN),
            lambda d: d.update(origin_ns='0'+str(ORIGIN)),
            lambda d: d.update(origin_ns=str(ORIGIN-1)),
            lambda d: d['poses'][1].update(stamp_ns=d['poses'][0]['stamp_ns']),
            lambda d: d['images'][1].update(stamp_ns=str(ORIGIN-1)),
            lambda d: d['poses'][1].update(sequence=d['poses'][0]['sequence']),
            lambda d: d['images'][1].update(sequence=True),
            lambda d: d['poses'][-1].update(stamp_ns=str(ORIGIN+3_600_000_000_001)),
            lambda d: d['images'][-1].update(stamp_ns=str(MAX_STAMP_NS+1)),
            lambda d: d['images'][0].update(stamp_ns='1.25'),
            lambda d: d['poses'][0].update(stamp_ns='-1'),
            lambda d: d.update(poses=d['poses'][:1]),
            lambda d: d.update(images=[]),
        )
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                doc, assets, images = fixture(); mutate(doc)
                with self.assertRaises(ValueError): SyncRecording(doc, assets, images)

    def test_wrong_model_joint_or_geometry_cannot_enter_the_track(self):
        mutations = (
            lambda d: d['poses'][1]['pose'].update(model_sha256='f'*64),
            lambda d: d['poses'][1]['pose'].update(scene_sha256='f'*64),
            lambda d: d['poses'][1]['pose']['geoms'].pop(),
            lambda d: d['poses'][1]['positions_rad'].pop('joint1'),
            lambda d: d['poses'][1]['positions_rad'].update(joint1=float('nan')),
            lambda d: d['poses'][1]['positions_rad'].update(joint1=10**400),
            lambda d: d['poses'][1]['pose']['geoms'][0].update(rotation=[0.]*9),
        )
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                doc, assets, images = fixture(); mutate(doc)
                with self.assertRaises(ValueError): SyncRecording(doc, assets, images)

    def test_content_hash_length_dimensions_and_member_sets_are_checked(self):
        doc, assets, images = fixture(asset=True)
        for changed_assets, changed_images in (
                ({}, images), (dict(assets, **{'f'*64: b'extra'}), images),
                ({next(iter(assets)): b'bad'}, images), (assets, {}),
                (assets, {JPEG_SHA: JPEG[:-2]+b'XX'}),
                (assets, dict(images, **{'f'*64: b'extra'}))):
            with self.subTest(assets=changed_assets.keys(), images=changed_images.keys()):
                with self.assertRaises(ValueError): SyncRecording(doc, changed_assets, changed_images)
        for mutate in (
                lambda d: d['images'][0].update(byte_length=len(JPEG)-1),
                lambda d: d['images'][0].update(width=3),
                lambda d: d['images'][0].update(width=641),
                lambda d: d['images'][0].update(jpeg_sha256='../private'),
                lambda d: d['images'][0].update(extra='x')):
            changed = deepcopy(doc); mutate(changed)
            with self.assertRaises(ValueError): SyncRecording(changed, assets, images)

    def test_budgets_fail_before_a_bundle_is_published(self):
        for constant, limit in (('MAX_RECORDING_BYTES', 10), ('MAX_POSES', 2), ('MAX_IMAGES', 2),
                                ('MAX_TOTAL_IMAGE_BYTES', 10), ('MAX_TOTAL_ASSET_BYTES', 10)):
            with self.subTest(budget=constant), patch(
                    'mujoco_ros2_manipulation.operator_console.synchronized_recording.'+constant, limit):
                with self.assertRaises(ValueError): recording(asset=True)

    def test_save_load_roundtrip_and_existing_bundle_is_never_replaced(self):
        with tempfile.TemporaryDirectory() as folder:
            value = recording(asset=True)
            path = Path(folder)/'nested'/'run'/'recording.json'
            self.assertEqual(value.save(path), path)
            loaded = SyncRecording.load(path)
            self.assertEqual(loaded.sha256, value.sha256)
            self.assertEqual(loaded.document, value.document)
            self.assertEqual(dict(loaded.images), dict(value.images))
            before = path.read_bytes()
            with self.assertRaises(FileExistsError): value.save(path)
            self.assertEqual(path.read_bytes(), before)
            digest = next(iter(value.assets))
            self.assertEqual((path.parent/'assets'/(digest+'.bin')).read_bytes(), value.assets[digest])

    def test_loader_rejects_parent_final_and_member_symlinks(self):
        for location in ('parent', 'document', 'images_directory', 'image', 'asset'):
            with self.subTest(location=location), tempfile.TemporaryDirectory() as folder:
                root = Path(folder); path = root/'original'/'recording.json'
                value = recording(asset=True); value.save(path)
                candidate = path
                if location == 'parent':
                    link = root/'link'; link.symlink_to(path.parent, target_is_directory=True)
                    candidate = link/path.name
                    with self.assertRaises(OSError): value.save(link/'new.json')
                elif location == 'document':
                    candidate = path.parent/'linked.json'; candidate.symlink_to(path)
                elif location == 'images_directory':
                    original = path.parent/'images'; moved = root/'moved-images'; original.rename(moved)
                    original.symlink_to(moved, target_is_directory=True)
                else:
                    target = (path.parent/'images'/(JPEG_SHA+'.jpg') if location == 'image'
                              else path.parent/'assets'/(next(iter(value.assets))+'.bin'))
                    moved = root/'moved-member'; target.rename(moved); target.symlink_to(moved)
                with self.assertRaises(OSError): SyncRecording.load(candidate)

    def test_loader_rejects_fifo_duplicate_json_and_corrupted_member(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); fifo = root/'fifo'; os.mkfifo(fifo)
            with self.assertRaises(ValueError): SyncRecording.load(fifo)
            value = recording(); path = root/'recording.json'; value.save(path)
            original = path.read_bytes()
            path.write_bytes(original.replace(b'"schema_version":1', b'"schema_version":1,"schema_version":1', 1))
            with self.assertRaises(ValueError): SyncRecording.load(path)
            path.write_bytes(original)
            (root/'images'/(JPEG_SHA+'.jpg')).write_bytes(b'x'*len(JPEG))
            with self.assertRaises(ValueError): SyncRecording.load(path)

    def test_save_rejects_conflicting_existing_members_without_publishing_json(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); (root/'images').mkdir()
            (root/'images'/(JPEG_SHA+'.jpg')).write_bytes(b'old')
            with self.assertRaises(ValueError): recording().save(root/'recording.json')
            self.assertFalse((root/'recording.json').exists())
            self.assertEqual((root/'images'/(JPEG_SHA+'.jpg')).read_bytes(), b'old')

    def test_parent_traversal_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            (Path(folder)/'child').mkdir()
            candidate = Path(folder)/'child'/'..'/'recording.json'
            with self.assertRaises(ValueError): recording().save(candidate)


class SynchronizedPlaybackTests(unittest.TestCase):
    def test_single_cursor_selects_past_observations_without_interpolation(self):
        host = SynchronizedSceneHost(recording(), clock=Clock())
        selected = host.control('seek', .15)
        self.assertEqual(selected['selection']['cursor_ns'], str(ORIGIN+150_000_000))
        self.assertEqual(selected['selection']['pose']['source']['sequence'], 13)
        self.assertEqual(selected['selection']['image']['sequence'], 20)
        self.assertAlmostEqual(selected['selection']['pose_delta_sec'], .05)
        self.assertAlmostEqual(selected['selection']['image_delta_sec'], .15)
        self.assertEqual(selected['sequence'], 13)
        self.assertEqual(selected['selection']['pose_status'], 'ready')
        self.assertEqual(selected['selection']['image_status'], 'ready')
        self.assertEqual(selected['selection']['pose']['pose'], recording().poses[1]['pose'])

    def test_snapshot_identifies_the_acquisition_epoch_without_exposing_mutable_identity(self):
        host = SynchronizedSceneHost(recording(), clock=Clock())
        snapshot = host.snapshot()
        self.assertEqual(snapshot['origin_ns'], str(ORIGIN))
        self.assertEqual(snapshot['acquisition_clock_domain'], 'ros_simulation')
        self.assertEqual(snapshot['acquisition_identity'], recording().identity)
        snapshot['acquisition_identity']['binding']['motion_session_id'] = 'f'*32
        self.assertEqual(host.snapshot()['acquisition_identity']['binding']['motion_session_id'], 'b'*32)

    def test_future_frame_is_never_substituted_and_missing_gaps_are_explicit(self):
        host = SynchronizedSceneHost(recording(image_offsets=(200_000_000, 1_000_000_000)), clock=Clock())
        first = host.snapshot()['selection']
        self.assertIsNone(first['image']); self.assertIsNone(first['image_time_sec'])
        self.assertEqual(first['image_status'], 'missing')
        boundary = host.control('seek', .7)['selection']
        self.assertIsNone(boundary['pose'])
        self.assertIsNotNone(boundary['image'])
        missing = host.control('seek', .700000001)['selection']
        self.assertIsNone(missing['pose']); self.assertIsNone(missing['image'])
        self.assertEqual(missing['image_status'], 'missing')
        self.assertAlmostEqual(missing['image_delta_sec'], .500000001)
        self.assertEqual(missing['image_time_sec'], .2)

    def test_snapshot_uses_one_clock_read_and_later_calls_do_not_mutate_old_selection(self):
        class AdvancingClock:
            def __init__(self): self.value = 0.; self.reads = 0
            def __call__(self):
                value = self.value; self.value += .1; self.reads += 1; return value
        clock = AdvancingClock(); host = SynchronizedSceneHost(recording(), clock=clock)
        host.control('play')
        before = clock.reads; first = host.snapshot()
        self.assertEqual(clock.reads-before, 1)
        frozen = deepcopy(first)
        self.assertEqual(first['position_sec'], .1)
        self.assertEqual(first['selection']['image']['sequence'], 20)
        second = host.snapshot()
        self.assertEqual(second['selection']['image']['sequence'], 24)
        self.assertEqual(first, frozen)

    def test_seek_and_each_frame_step_make_old_bindings_invalid(self):
        host = SynchronizedSceneHost(recording(), clock=Clock())
        old = host.identifier
        selected = host.control('seek', .15)
        self.assertNotEqual(old, host.identifier)
        self.assertEqual(selected['binding_id'], binding_id(selected['binding']))
        with self.assertRaises(ValueError): host.manifest_bytes(old)
        before = host.identifier
        following = host.control('next')
        self.assertAlmostEqual(following['position_sec'], .2)
        self.assertNotEqual(before, host.identifier)
        self.assertFalse(following['playing'])
        previous = host.control('previous')
        self.assertAlmostEqual(previous['position_sec'], .1)
        self.assertEqual(host.control('seek', 0)['position_sec'], 0)
        self.assertEqual(host.control('previous')['position_sec'], 0)
        host.control('seek', 1.2)
        self.assertEqual(host.control('next')['position_sec'], 1.2)

    def test_speed_change_preserves_position_pause_freezes_and_eof_replay_rebinds(self):
        clock = Clock(); host = SynchronizedSceneHost(recording(), clock=clock)
        host.control('play'); clock.advance(.2)
        changed = host.control('speed', speed=2.)
        self.assertAlmostEqual(changed['position_sec'], .2)
        clock.advance(.1)
        self.assertAlmostEqual(host.snapshot()['position_sec'], .4)
        stopped = host.control('pause'); clock.advance(4)
        self.assertEqual(host.snapshot()['position_sec'], stopped['position_sec'])
        host.control('speed', speed=.25); host.control('play'); clock.advance(.4)
        self.assertAlmostEqual(host.snapshot()['position_sec'], .5)
        clock.advance(10)
        eof = host.snapshot(); self.assertFalse(eof['playing'])
        self.assertEqual(eof['position_sec'], 1.2)
        old = host.identifier; restarted = host.control('play')
        self.assertEqual(restarted['position_sec'], 0.)
        self.assertTrue(restarted['playing']); self.assertNotEqual(host.identifier, old)

    def test_invalid_controls_and_playback_clock_reversal_fail_closed(self):
        clock = Clock(); host = SynchronizedSceneHost(recording(), clock=clock)
        for action, position, speed in (
                ('execute', None, None), ('seek', -1, None), ('seek', float('nan'), None),
                ('seek', 10**400, None), ('seek', True, None), ('seek', 2., None),
                ('play', 0., None), ('pause', None, 1.), ('speed', None, True),
                ('speed', None, float('inf')), ('speed', None, 3.)):
            with self.subTest(action=action, position=position, speed=speed):
                with self.assertRaises(ValueError): host.control(action, position, speed)
        host.control('play'); clock.advance(.3); host.snapshot(); clock.advance(-.1)
        with self.assertRaisesRegex(ValueError, 'reversed'): host.snapshot()

    def test_closed_source_and_unknown_members_are_not_readable(self):
        host = SynchronizedSceneHost(recording(asset=True), clock=Clock())
        identifier = host.identifier
        self.assertEqual(host.image_bytes(JPEG_SHA), JPEG)
        digest = next(iter(host.recording.assets))
        self.assertEqual(host.asset_bytes(digest), host.recording.assets[digest])
        with self.assertRaises(ValueError): host.image_bytes('../private')
        with self.assertRaises(ValueError): host.asset_bytes('f'*64)
        host.close()
        self.assertEqual(host.snapshot()['status'], 'closed')
        self.assertIsNone(host.snapshot()['selection']['pose'])
        self.assertIsNone(host.snapshot()['selection']['image'])
        for read in (lambda: host.manifest_bytes(identifier), lambda: host.image_bytes(JPEG_SHA),
                     lambda: host.asset_bytes(digest), lambda: host.control('play')):
            with self.assertRaises(ValueError): read()

    def test_replay_has_no_engine_codec_or_ros_imports(self):
        script = """
import builtins
original=builtins.__import__
def guarded(name,*args,**kwargs):
    if name.split('.')[0] in ('rclpy','mujoco','numpy','PIL'):
        raise RuntimeError('Runtime dependency import: '+name)
    return original(name,*args,**kwargs)
builtins.__import__=guarded
from tests.unit.test_synchronized_recording import recording
from mujoco_ros2_manipulation.operator_console.synchronized_recording import SynchronizedSceneHost
host=SynchronizedSceneHost(recording())
assert host.control('seek',.15)['selection']['image_status']=='ready'
"""
        result = subprocess.run([sys.executable, '-c', script], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
