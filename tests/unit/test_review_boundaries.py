"""Independent pure boundary regressions; synthetic bytes and mocked owned children.

No CAD engine, MuJoCo, ROS, network, existing outputs or real child process runs.
"""
from copy import deepcopy
import importlib
import itertools
import math
from pathlib import Path
import signal
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import mujoco_ros2_manipulation.workcells.environment_assets as assets
import mujoco_ros2_manipulation.workcells.environment_overlay as overlays
step_import=importlib.import_module('mujoco_ros2_manipulation.workcells.import_step')


def sample():
    spec=dict(id='owned_box',declared_length_unit='mm',source_frame='cad',asset_frame='asset',
        source_up_axis='Y',source_to_asset_rotation=[1,0,0,0,0,-1,0,1,0],
        expected_bbox_size_m=[.1,.3,.2],bbox_tolerance_m=1e-6,deflection_m=.001,padding_m=.005)
    low=[-70.,-40.,20.];high=[30.,160.,320.]
    tess=dict(valid=True,solid_count=1,bbox_min_mm=low,bbox_max_mm=high,
        vertices_mm=[list(p) for p in itertools.product(*zip(low,high))],
        triangles=[[0,2,3],[0,3,1],[4,5,7],[4,7,6],[0,1,5],[0,5,4],
                   [2,6,7],[2,7,3],[0,4,6],[0,6,2],[1,3,7],[1,7,5]])
    converter=dict(name='FreeCAD',version='0.0-test',occt_version='0.0-test',
        script_sha256='a'*64,executable_sha256='b'*64)
    return spec,tess,converter


class IndependentEnvironmentBoundaryTests(unittest.TestCase):
    def test_asymmetric_origin_rotates_once_then_installs_same_visual_and_box(self):
        spec,tess,converter=sample();manifest,mesh=assets.normalize(b'owned-synthetic-step',tess,spec,converter)
        # Proper source +Y -> asset +Z changes axes, retaining nonzero origin.
        self.assertEqual(manifest['normalization']['cad_bbox_min_m'],[-.07,-.32,-.04])
        self.assertEqual(manifest['normalization']['cad_bbox_max_m'],[.03,-.02,.16])
        placed=dict(name='box',parent_frame='world',xyz_m=[1.,2.,3.],rpy_rad=[.2,-.3,.4],rgba=[.5,.6,.7,1.])
        mj=ET.fromstring('<mujoco><worldbody/></mujoco>');urdf=ET.fromstring('<robot><link name="world"/></robot>')
        overlays.append_environment(mj,urdf,manifest,placed,'assets/visual.stl',Path('/tmp/owned-only/visual.stl'))
        body=mj.find("worldbody/body[@name='environment_box']")
        self.assertEqual([float(v) for v in body.get('pos').split()],placed['xyz_m'])
        self.assertIsNone(body.find("geom[@name='environment_box_visual']").get('pos'),
            'The original CAD origin is already in mesh coordinates')
        joint=urdf.find("joint[@name='environment_box_fixed']")
        self.assertEqual([float(v) for v in joint.find('origin').get('xyz').split()],placed['xyz_m'])
        self.assertEqual([float(v) for v in joint.find('origin').get('rpy').split()],placed['rpy_rad'])
        mj_center=[float(v) for v in body.find("geom[@name='environment_box_collision']").get('pos').split()]
        urdf_center=[float(v) for v in urdf.find("link[@name='environment_box']/collision/origin").get('xyz').split()]
        self.assertEqual(mj_center,manifest['collision']['center_m']);self.assertEqual(urdf_center,mj_center)
        size=manifest['collision']['size_m'];center=manifest['collision']['center_m']
        rotation=assets.rpy_matrix(placed['rpy_rad'])
        # Every normalized CAD corner is inside the separately padded local box;
        # one common installation transform must keep this world-space relation.
        for point in assets.corners(manifest['normalization']['cad_bbox_min_m'],manifest['normalization']['cad_bbox_max_m']):
            self.assertTrue(all(center[k]-size[k]/2<=point[k]<=center[k]+size[k]/2 for k in range(3)))
            world=assets.transformed(point,rotation,placed['xyz_m'])
            again=assets.transformed(world,rotation,placed['xyz_m'])
            self.assertGreater(max(abs(a-b) for a,b in zip(world,again)),.1,
                'This fixture is asymmetric enough to expose accidental double installation')

    def test_bundle_rejects_mesh_traversal_even_with_a_recomputed_manifest_hash(self):
        spec,tess,converter=sample();manifest,mesh=assets.normalize(b'owned-step',tess,spec,converter)
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);bundle=root/'bundle';bundle.mkdir();(root/'outside.stl').write_bytes(mesh)
            (bundle/'visual.stl').write_bytes(mesh)
            for path in ('../outside.stl','/tmp/outside.stl','file:///tmp/outside.stl'):
                candidate=deepcopy(manifest);candidate['visual']['path']=path
                raw=assets.canonical(candidate);(bundle/'asset.json').write_bytes(raw)
                with self.subTest(path=path),self.assertRaises(ValueError):
                    assets.validate_bundle(bundle,expected_sha256=assets.sha(raw))

    def test_overlay_destination_cannot_enter_active_base_through_a_parent_symlink(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);base=root/'base';base.mkdir();alias=root/'alias';alias.symlink_to(base,target_is_directory=True)
            cell=SimpleNamespace(id='base',output=base)
            def must_not_read_or_build(_):self.fail('Active-base nesting passed the destination guard')
            with self.assertRaisesRegex(ValueError,'active base|nest|output'):
                overlays.build_overlay(cell,[],alias/'overlay',overlay_id='new_overlay',base_validator=must_not_read_or_build)
            self.assertEqual(list(base.iterdir()),[])

    def test_owned_converter_is_cleaned_when_the_caller_is_interrupted(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);binary=root/'owned-freecad';binary.write_bytes(b'old executable')
            calls=[]
            class Child:
                pid=123456;returncode=None
                def wait(self,timeout=None):
                    calls.append(('wait',timeout))
                    if len([v for v in calls if v[0]=='wait'])==1:raise KeyboardInterrupt('synthetic caller cancellation')
                    self.returncode=-signal.SIGTERM;return self.returncode
                def poll(self):return self.returncode
            with patch.object(step_import.subprocess,'Popen',return_value=Child()),patch.object(step_import.os,'killpg',side_effect=lambda *args:calls.append(('kill',args))):
                with self.assertRaises(KeyboardInterrupt):step_import.run_freecad(dict(operation='convert'),root,executable=str(binary))
            self.assertTrue(any(row[0]=='kill' and row[1]==(123456,signal.SIGTERM) for row in calls),
                'Caller cancellation must terminate and reap its newly owned converter')

    def test_converter_replacement_during_execution_is_not_attested_as_the_executed_binary(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);binary=root/'owned-freecad';binary.write_bytes(b'old executed bytes')
            class Child:
                pid=123456;returncode=0
                def wait(self,timeout=None):binary.write_bytes(b'new nonexecuted bytes');return 0
                def poll(self):return 0
            with patch.object(step_import.subprocess,'Popen',return_value=Child()),self.assertRaisesRegex(ValueError,'changed|converter|executable|binary'):
                step_import.run_freecad(dict(operation='convert'),root,executable=str(binary))

    def test_converter_timeout_signals_only_its_new_owned_process_group(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);binary=root/'owned-freecad';binary.write_bytes(b'executable')
            calls=[]
            class Child:
                pid=123456;returncode=None
                def wait(self,timeout=None):
                    calls.append(('wait',timeout))
                    if len([v for v in calls if v[0]=='wait'])==1:raise subprocess.TimeoutExpired('owned-only',timeout)
                    self.returncode=-signal.SIGTERM;return self.returncode
                def poll(self):return self.returncode
            with patch.object(step_import.subprocess,'Popen',return_value=Child()),patch.object(step_import.os,'killpg',side_effect=lambda *args:calls.append(('kill',args))):
                with self.assertRaises((ValueError,subprocess.TimeoutExpired)):
                    step_import.run_freecad(dict(operation='convert'),root,executable=str(binary),timeout=.01)
            self.assertEqual([v[1][0] for v in calls if v[0]=='kill'],[123456])


if __name__=='__main__':unittest.main()
