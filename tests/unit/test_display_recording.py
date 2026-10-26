"""Common-format producer replacement without a simulator, ROS or a socket."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from mujoco_ros2_manipulation.operator_console.display_contract import (
    from_compiled_manifest, scene_digest, validate_manifest, DisplayHostAdapter)
from mujoco_ros2_manipulation.operator_console.recording import Recording, RecordedSceneHost, demo_recording
from mujoco_ros2_manipulation.operator_console.scene_host import _canonical
from tests.unit.test_scene_host import make_manifest, Clock, Factory


def document():
    recording = demo_recording()
    return dict(schema_version=1, kind="display-recording-v1", label=recording.label,
                manifest=deepcopy(recording.manifest), frames=deepcopy(recording.frames))


class RecordingTests(unittest.TestCase):
    def test_fixture_has_complete_names_world_transforms_and_no_engine_dependency(self):
        value = demo_recording()
        self.assertIsNone(value.manifest['provenance']['engine'])
        self.assertNotEqual(value.frames[0]['pose'], value.frames[20]['pose'])
        self.assertEqual(value.duration, 8.)
        self.assertEqual(value.frames[0]['positions_rad'], value.manifest['home_rad'])
        script = """
import builtins
original=builtins.__import__
def guarded(name,*args,**kwargs):
    if name.split('.')[0] in ('rclpy','mujoco','numpy','yaml'):
        raise RuntimeError('Engine import attempted: '+name)
    return original(name,*args,**kwargs)
builtins.__import__=guarded
from mujoco_ros2_manipulation.operator_console.recording import demo_recording,RecordedSceneHost
host=RecordedSceneHost(demo_recording())
assert host.pose_envelope(host.identifier)['source']['clock_domain'].startswith('recording:')
"""
        result = subprocess.run([sys.executable, '-c', script], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_recording_rejects_cross_model_partial_joints_bad_transform_or_time(self):
        for mutate in (
            lambda d: d['frames'][1]['pose'].update(model_sha256='a'*64),
            lambda d: d['frames'][1]['positions_rad'].pop('joint2'),
            lambda d: d['frames'][1].update(time_sec=0),
            lambda d: d['frames'][1].update(time_sec=float('nan')),
            lambda d: d['frames'][1]['pose']['geoms'][0].update(rotation=[0.]*9),
            lambda d: d['frames'][1]['pose']['geoms'].pop(),
        ):
            with self.subTest(mutate=mutate):
                value = document(); mutate(value)
                with self.assertRaises(ValueError): Recording(value)

    def test_source_freeze_does_not_rejuvenate_age_and_seek_restarts_binding(self):
        clock = Clock(); host = RecordedSceneHost(demo_recording(), clock=clock)
        original = host.identifier
        host.control('play'); clock.advance(.3)
        first = host.pose_envelope(original)
        self.assertGreater(first['source']['sequence'], 1)
        host.control('pause'); clock.advance(2)
        paused = host.pose_envelope(original)
        self.assertEqual(first['sequence'], paused['sequence'])
        self.assertGreaterEqual(paused['source_state_age_sec'], 2)
        host.control('seek', 0)
        self.assertNotEqual(original, host.identifier)
        with self.assertRaises(ValueError): host.pose_envelope(original)
        self.assertEqual(host.pose_envelope(host.identifier)['source']['stamp_sec'], 0)

    def test_end_of_recording_is_stale_and_replacement_rebinds(self):
        clock = Clock(); host = RecordedSceneHost(demo_recording(), clock=clock)
        host.control('play'); clock.advance(9)
        old = host.identifier
        self.assertFalse(host.snapshot()['playing'])
        first = host.pose_envelope(old); clock.advance(1)
        self.assertEqual(first['sequence'], host.pose_envelope(old)['sequence'])
        self.assertGreaterEqual(host.pose_envelope(old)['source_state_age_sec'], 1)
        host.select(demo_recording())
        self.assertNotEqual(host.identifier, old)
        host.close()
        with self.assertRaises(ValueError): host.pose_envelope(host.identifier)

    def test_loader_rejects_unknown_fields_duplicate_fields_and_symlinks(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'recording.json'
            path.write_bytes(_canonical(document()))
            self.assertEqual(Recording.load(path).duration, 8)
            linked=Path(folder)/'link.json'; linked.symlink_to(path)
            with self.assertRaises(OSError): Recording.load(linked)
            path.write_bytes(b'{"schema_version":1,"schema_version":2}')
            with self.assertRaises(ValueError): Recording.load(path)

    def test_common_format_has_no_mandatory_engine_and_hashes_conventions(self):
        manifest = demo_recording().manifest
        for key in ('mujoco_version','compiled_model_sha256'):
            self.assertNotIn(key, manifest)
        value = deepcopy(manifest); value['world_frame']['handedness']='left'
        value['scene_sha256']=scene_digest(value)
        with self.assertRaises(ValueError): validate_manifest(value)
        value=deepcopy(manifest); value['geoms'][0]['size'][0]=3
        with self.assertRaises(ValueError): validate_manifest(value)

    def test_degenerate_primitives_are_rejected_before_browser_load(self):
        for kind, size in [('box',[0.,1.,1.]), ('box',[1.,0.,1.]),
                           ('sphere',[0.,0.,0.]), ('cylinder',[1.,0.,0.])]:
            value = deepcopy(demo_recording().manifest)
            value['geoms'][0].update(type=kind,size=size)
            value['scene_sha256']=scene_digest(value)
            with self.assertRaises(ValueError): validate_manifest(value)

    def test_mujoco_adapter_preserves_shapes_and_world_pose_without_double_transform(self):
        from mujoco_ros2_manipulation.operator_console.scene_host import SceneHost, SceneSource
        from mujoco_ros2_manipulation.operator_console.view_binding import make_binding
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder); xml=root/'model.xml'; xml.write_text('<mujoco/>')
            legacy=make_manifest(xml,['j1'],{'j1':0.})
            common=from_compiled_manifest(legacy)
            self.assertEqual(legacy['geoms'], common['geoms'])
            self.assertEqual(common['provenance']['engine']['compiled_sha256'],legacy['compiled_model_sha256'])
            clock=Clock(); factory=Factory()
            raw=SceneHost(root/'worker',sys.executable,clock=clock,process_factory=factory,start_thread=False)
            host=DisplayHostAdapter(raw)
            try:
                source=SceneSource('workcell:test','Test',xml,('j1',),{'j1':0.},lambda:None)
                binding=make_binding('a'*32,1,source.id,'live')
                identifier=host.activate(binding,source); host._step()
                host.submit({'j1':0.},source_received_monotonic=clock());host._step()
                factory.processes[-1].reply(1,processed=clock());host._step()
                converted=json.loads(host.manifest_bytes(identifier))
                pose=host.pose_envelope(identifier)
                self.assertEqual(converted['schema_version'],2)
                self.assertEqual(converted['scene_sha256'],pose['pose']['scene_sha256'])
                self.assertEqual(pose['pose']['geoms'][0]['position'],[1.,2.,3.])
                self.assertEqual(hashlib.sha256(host.manifest_bytes(identifier)).hexdigest(),host.snapshot()['manifest_descriptor']['sha256'])
            finally: host.close()
