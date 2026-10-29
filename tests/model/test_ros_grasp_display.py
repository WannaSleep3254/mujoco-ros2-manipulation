"""Actual native3.12 geometry export and independent full-qpos display inputs.

The probe only performs static FK on isolated test inputs. These synthetic
observation envelopes test display transport; they are not physical-grasp proof.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.operator_console.display_contract import validate_manifest, validate_pose
from mujoco_ros2_manipulation.grasping.ros_model import build_model
from mujoco_ros2_manipulation.grasping.ros_target import load_ros_grasp_target
from mujoco_ros2_manipulation.grasping.ros_display import build_scene, frame_from_observation
from mujoco_ros2_manipulation.grasping.ros_observation import GraspObservationStore, GraspObservation
from mujoco_ros2_manipulation.workcells.config import load_workcell


PROBE = r'''
#include <mujoco/mujoco.h>
#include <fstream>
#include <iomanip>
#include <iostream>
int main(int argc,char**argv) {
  if(argc<2)return 2;
  char error[2048]{};mjModel*m=mj_loadXML(argv[1],nullptr,error,2048);
  if(!m){std::cerr<<error;return 3;}
  mjData*d=mj_makeData(m);if(m->nkey)mj_resetDataKeyframe(m,d,0);
  if(argc>2){std::ifstream input(argv[2]);for(int i=0;i<m->nq;i++)if(!(input>>d->qpos[i]))return 4;}
  mj_kinematics(m,d);mj_comPos(m,d);mj_comVel(m,d);
  std::cout<<std::setprecision(17)<<"{\"qpos\":[";
  for(int i=0;i<m->nq;i++){if(i)std::cout<<',';std::cout<<d->qpos[i];}
  std::cout<<"],\"geoms\":[";
  for(int g=0;g<m->ngeom;g++){
    if(g){std::cout<<',';}
    std::cout<<"{\"id\":"<<g<<",\"position\":[";
    for(int i=0;i<3;i++){if(i)std::cout<<',';std::cout<<d->geom_xpos[3*g+i];}
    std::cout<<"],\"rotation\":[";
    for(int i=0;i<9;i++){if(i)std::cout<<',';std::cout<<d->geom_xmat[9*g+i];}
    std::cout<<"]}";
  }
  std::cout<<"],\"tcp\":{\"position\":[";
  const int tcp=mj_name2id(m,mjOBJ_SITE,"fr5_grasp_tcp");
  for(int i=0;i<3;i++){if(i)std::cout<<',';std::cout<<d->site_xpos[3*tcp+i];}
  mjtNum quat[4]{};mju_mat2Quat(quat,d->site_xmat+9*tcp);
  std::cout<<"],\"quaternion_wxyz\":[";
  for(int i=0;i<4;i++){if(i)std::cout<<',';std::cout<<quat[i];}
  const int block=mj_name2id(m,mjOBJ_BODY,"fr5_grasp_block");
  std::cout<<"]},\"block\":{\"position\":[";
  for(int i=0;i<3;i++){if(i)std::cout<<',';std::cout<<d->xpos[3*block+i];}
  std::cout<<"],\"quaternion_wxyz\":[";
  for(int i=0;i<4;i++){if(i)std::cout<<',';std::cout<<d->xquat[4*block+i];}
  std::cout<<"]},\"mesh_first_world_vertices\":[";
  bool comma=false;
  for(int g=0;g<m->ngeom;g++)if(m->geom_type[g]==mjGEOM_MESH){
    if(comma){std::cout<<',';}
    comma=true;
    const int mesh=m->geom_dataid[g];
    const float*v=m->mesh_vert+3*m->mesh_vertadr[mesh];
    std::cout<<"{\"geom_id\":"<<g<<",\"world_point\":[";
    for(int row=0;row<3;row++){
      if(row)std::cout<<',';
      double value=d->geom_xpos[3*g+row];
      for(int col=0;col<3;col++)value+=d->geom_xmat[9*g+3*row+col]*v[col];
      std::cout<<value;
    }
    std::cout<<"]}";
  }
  std::cout<<"]}";mj_deleteData(d);mj_deleteModel(m);
}
'''


class _NativeRoot:
    def __init__(self, target, root):
        self.original, self.root = target, root

    def __getattr__(self, name):
        return getattr(self.original, name)


class RosGraspDisplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vendor = ROOT / 'runtime/root/opt/ros/humble/opt/mujoco_vendor'
        library = cls.vendor / 'lib/libmujoco.so.3.12.0'
        if not library.is_file():
            raise unittest.SkipTest('Native MuJoCo3.12 runtime required')
        cls.temporary = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.folder = Path(cls.temporary.name)
        cls.model = cls.folder / 'model'
        base = load_workcell('fr5_ur5e')
        build_model(base, cls.model)
        target = load_ros_grasp_target(base, cls.model, file_sha256(cls.model / 'grasp.json'))
        cls.native_root = cls.folder / 'native-build'
        for relative in ('native/grasp_scene_export.cpp', 'native/grasp_ros2_plugin/src/observation.cpp'):
            destination = cls.native_root / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / relative, destination)
        binary = cls.native_root / 'runtime/native/grasp_scene_export'
        binary.parent.mkdir(parents=True)
        common = ['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', '-I' + str(cls.vendor / 'include')]
        link = [str(library), '-Wl,-rpath,' + str(cls.vendor / 'lib')]
        subprocess.run([*common,
            '-DMANIPULATION_GRASP_SCENE_EXPORT_SHA256="' + file_sha256(ROOT / 'native/grasp_scene_export.cpp') + '"',
            str(ROOT / 'native/grasp_scene_export.cpp'), *link, '-o', str(binary)], check=True, capture_output=True)
        probe_source = cls.folder / 'probe.cpp'
        probe_source.write_text(PROBE)
        cls.probe = cls.folder / 'native-static-fk'
        subprocess.run([*common, str(probe_source), *link, '-o', str(cls.probe)], check=True, capture_output=True)
        cls.target = _NativeRoot(target, cls.native_root)
        cls.initial = cls.native_frame()
        cls.run_id = 'a' * 32
        cls.scene = build_scene(cls.target, run_id=cls.run_id)

    @classmethod
    def native_frame(cls, qpos=None):
        command = [str(cls.probe), str(cls.model / 'cell.xml')]
        if qpos is not None:
            path = cls.folder / 'static-input-qpos.txt'
            path.write_text(' '.join(map(str, qpos)))
            command.append(str(path))
        return json.loads(subprocess.run(command, check=True, capture_output=True, timeout=15.).stdout)

    def observation(self, scene=None, *, frame=None, sequence=1, clock=.5, run=None):
        scene = self.scene if scene is None else scene
        frame = self.initial if frame is None else frame
        run = self.run_id if run is None else run
        qpos = frame['qpos']
        layout = self.target.manifest['qpos_state_contract']['layout']
        addresses = {row['name']: row['qpos_address'] for row in layout}
        payload = dict(schema_version=1, kind='ros-grasp-observation-v1', run_id=run,
            model_sha256=file_sha256(self.model / 'cell.xml'),
            plugin_sha256=file_sha256(self.native_root / 'native/grasp_ros2_plugin/src/observation.cpp'),
            native_version='3.12.0', sequence=sequence, time_sec=clock, stamp_ns=str(round(clock * 1e9)),
            stamp_semantics='rounded_native_mjData_time', phase=0, armed=False, heartbeat_fresh=False,
            pose_sample_sec=clock, force_evaluation_sec=max(0., clock - .002), force_timing_verified=False,
            force_time_semantics='nominal last mj_step evaluation; invalid when native physics is held',
            local_overwrites_or_skips=0, qpos=qpos, qvel=[0.] * 20,
            arm_positions_rad={name: qpos[addresses[name]] for name in self.target.joints},
            finger_positions_m={name: qpos[addresses[name]] for name in self.target.finger_joints},
            geoms=frame['geoms'], tcp=frame['tcp'],
            block=dict(frame['block'], linear_velocity_m_sec=[0., 0., 0.], angular_velocity_rad_sec=[0., 0., 0.]),
            contacts=[])
        store = GraspObservationStore(run_id=run, model_sha256=payload['model_sha256'], plugin_sha256=payload['plugin_sha256'],
                                      manifest=self.target.description(), geom_names=scene.geom_names)
        self.assertTrue(store.ingest(payload, receipt_sec=10. + clock).accepted)
        return store.snapshot(now_sec=10. + clock), payload

    def fresh_scene(self, run=None):
        return build_scene(self.target, run_id=self.run_id if run is None else run)

    def test_native_materials_complete_catalog_and_common_buffers_use_engine312(self):
        scene = self.scene
        manifest = validate_manifest(scene.manifest)
        self.assertEqual(len(manifest['geoms']), 49)
        self.assertEqual(len(scene.geom_names), 49)
        self.assertEqual(manifest['provenance']['engine']['version'], '3.12.0')
        self.assertEqual(manifest['joint_names'], self.target.joints)
        self.assertFalse(set(self.target.finger_joints) & set(manifest['joint_names']))
        self.assertGreater(len({tuple(geom['rgba']) for geom in manifest['geoms']}), 5)
        self.assertTrue(any(not geom['default_visible'] for geom in manifest['geoms']))
        metadata = scene.metadata()
        self.assertEqual(metadata['physics_steps_during_export'], 0)
        self.assertEqual(metadata['meshes'], 34)
        self.assertEqual(len(metadata['compiled_sha256']), 64)
        assets = scene.assets
        self.assertEqual(len(manifest['assets']), len(assets))
        for descriptor in manifest['assets']:
            blob = scene.asset(descriptor['sha256'])
            self.assertEqual(hashlib.sha256(blob).hexdigest(), descriptor['sha256'])
            self.assertEqual(len(blob), descriptor['byte_length'])
            self.assertEqual(descriptor['layout']['indices']['byte_offset'], descriptor['layout']['positions']['byte_length'])
            count = descriptor['layout']['positions']['count']
            start = descriptor['layout']['indices']['byte_offset']
            for (index,) in struct.iter_unpack('<I', blob[start:]):
                self.assertLess(index, count)
        self.assertEqual(scene.static_preview_pose['geoms'], self.initial['geoms'])
        self.assertEqual(len(scene.body_hierarchy), len({body['id'] for body in scene.body_hierarchy}))

    def test_native_compiled_vertices_pair_with_geom_world_transform_without_second_mesh_offset(self):
        manifest = self.scene.manifest
        pose = self.scene.static_preview_pose
        for witness in self.initial['mesh_first_world_vertices']:
            index = witness['geom_id']
            descriptor = manifest['geoms'][index]
            self.assertNotIn('mesh_pos', descriptor)
            self.assertNotIn('mesh_quat', descriptor)
            first = struct.unpack_from('<fff', self.scene.asset(descriptor['asset_sha256']))
            transform = pose['geoms'][index]
            actual = [transform['position'][row] + sum(transform['rotation'][3 * row + col] * first[col] for col in range(3))
                      for row in range(3)]
            for got, expected in zip(actual, witness['world_point']):
                self.assertAlmostEqual(got, expected, places=12)

    def test_dynamic_fingers_and_free_block_copy_all_native_transforms_without_python_engine(self):
        scene = self.fresh_scene()
        first, _ = self.observation(scene)
        scene.pose(first)
        qpos = list(self.initial['qpos'])
        qpos[6], qpos[7] = .01, .02
        qpos[14] += .04; qpos[15] -= .03; qpos[16] += .08
        qpos[17:21] = [1., 0., 0., 0.]
        frame = self.native_frame(qpos)
        observation, payload = self.observation(scene, frame=frame, sequence=2, clock=.6)
        preserved = deepcopy(payload)
        original_import = __import__
        def no_mujoco(name, *args, **kwargs):
            if name == 'mujoco':
                raise AssertionError('Dynamic ROS display must not use Python MuJoCo')
            return original_import(name, *args, **kwargs)
        with patch('builtins.__import__', side_effect=no_mujoco):
            pose = frame_from_observation(scene, observation)
        self.assertEqual(pose['geoms'], frame['geoms'])
        validate_pose(pose, scene.manifest)
        for name in (*self.target.finger_joints,):
            body = 'fr5_left_finger_geom' if 'left' in name else 'fr5_right_finger_geom'
            index = scene.geom_names.index(body)
            self.assertNotEqual(pose['geoms'][index], self.initial['geoms'][index])
        block = scene.geom_names.index('fr5_block_geom')
        self.assertEqual(pose['geoms'][block]['position'], qpos[14:17])
        self.assertNotEqual(pose['geoms'][block]['rotation'], self.initial['geoms'][block]['rotation'])
        self.assertEqual(observation.payload, preserved)

    def test_scene_build_itself_does_not_import_python_mujoco(self):
        original_import = __import__
        def no_mujoco(name, *args, **kwargs):
            if name == 'mujoco':
                raise AssertionError('Native geometry export must not use Python MuJoCo')
            return original_import(name, *args, **kwargs)
        with patch('builtins.__import__', side_effect=no_mujoco):
            scene = self.fresh_scene()
        self.assertEqual(scene.manifest['provenance']['engine']['version'], '3.12.0')

    def test_owner_run_is_explicit_and_foreign_model_plugin_or_engine_is_rejected(self):
        unbound = build_scene(self.target)
        observation, payload = self.observation(unbound)
        with self.assertRaises(ValueError): unbound.pose(observation)
        unbound.pose(observation, expected_run_id=self.run_id)
        with self.assertRaises(ValueError): unbound.pose(observation, expected_run_id='b' * 32)
        for field, wrong in (('run_id', 'b' * 32), ('model_sha256', 'b' * 64),
                             ('plugin_sha256', 'b' * 64), ('native_version', '3.14.0')):
            value = dict(payload, **{field: wrong})
            forged = GraspObservation(10.5, 10.5, False, json.dumps(value).encode())
            with self.subTest(field=field), self.assertRaises(ValueError):
                unbound.pose(forged)

    def test_repeated_reads_are_detached_but_reordered_or_changed_sequence_is_rejected(self):
        scene = self.fresh_scene()
        first, payload = self.observation(scene)
        pose = scene.pose(first)
        pose['geoms'][0]['position'][0] = 999.
        self.assertNotEqual(scene.pose(first), pose)
        self.assertEqual(scene.pose(first)['geoms'], self.initial['geoms'])
        changed = deepcopy(payload); changed['geoms'][0]['position'][0] += .1
        with self.assertRaises(ValueError):
            scene.pose(GraspObservation(10.5, 10.5, False, json.dumps(changed).encode()))
        second, _ = self.observation(scene, sequence=2, clock=.6)
        scene.pose(second)
        with self.assertRaises(ValueError): scene.pose(first)

    def test_missing_geometry_and_clock_reversal_cannot_turn_into_static_or_fresh_pose(self):
        scene = self.fresh_scene()
        first, payload = self.observation(scene)
        scene.pose(first)
        missing = deepcopy(payload); missing.update(sequence=2, time_sec=.6, pose_sample_sec=.6,
            force_evaluation_sec=.598, stamp_ns='600000000'); missing['geoms'].pop()
        with self.assertRaises(ValueError):
            scene.pose(GraspObservation(10.6, 10.6, True, json.dumps(missing).encode()))
        reversed_clock, rollback_payload = self.observation(scene, sequence=3, clock=.4)
        # Source time regresses while the independent receiver clock advances.
        reversed_clock = GraspObservation(10.7, 10.5, False, json.dumps(rollback_payload).encode())
        with self.assertRaises(ValueError): scene.pose(reversed_clock)
        later, _ = self.observation(scene, sequence=4, clock=.7)
        with self.assertRaises(ValueError): scene.pose(later)

    def test_exporter_source_identity_and_immutable_public_assets_are_enforced(self):
        source = self.native_root / 'native/grasp_scene_export.cpp'
        original = source.read_bytes()
        try:
            source.write_bytes(original + b'\n// stale binary test\n')
            with self.assertRaisesRegex(ValueError, 'stale'):
                self.fresh_scene()
        finally:
            source.write_bytes(original)
        manifest = self.scene.manifest
        manifest['geoms'][0]['rgba'][0] = 0.
        self.assertNotEqual(self.scene.manifest, manifest)
        assets = self.scene.assets
        assets.clear()
        self.assertTrue(self.scene.assets)
        with self.assertRaises(ValueError): self.scene.asset('../../private')
        with self.assertRaises(KeyError): self.scene.asset('0' * 64)


if __name__ == '__main__':
    unittest.main()
