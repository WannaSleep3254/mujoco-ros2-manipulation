from copy import deepcopy
import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import yaml

from mujoco_ros2_manipulation.workcells.environment_assets import canonical,normalize,sha
from mujoco_ros2_manipulation.workcells.environment_overlay import OUTPUTS,build_overlay,check_overlay,check_overlay_identity
from tests.unit.test_environment_assets import CONVERTER,spec,tessellation


@unittest.skipUnless(importlib.util.find_spec('mujoco'),'CPU MuJoCo is optional for fixture compilation')
class EnvironmentOverlayTests(unittest.TestCase):
    def setUp(self):
        temporary=TemporaryDirectory();self.addCleanup(temporary.cleanup);self.root=Path(temporary.name)
        output=self.root/'existing_base';output.mkdir()
        self.cell=SimpleNamespace(id='base',root=self.root,output=output,fingerprint='f'*64,joints=['q'],home=[0.])
        mj='<mujoco model="base"><worldbody><geom name="floor" type="plane" size="5 5 .1"/><body pos="0 0 1"><joint name="q"/><geom type="sphere" size=".1"/></body></worldbody></mujoco>'
        urdf='<robot name="base"><link name="world"/><link name="tool"/><joint name="q" type="revolute"><parent link="world"/><child link="tool"/><origin xyz="0 0 1"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint><ros2_control><hardware><param name="mujoco_model">'+str(output/'cell.xml')+'</param></hardware><joint name="q"/></ros2_control></robot>'
        srdf='<robot name="base"><group name="arm"><joint name="q"/></group></robot>'
        (output/'cell.xml').write_text(mj);(output/'cell.urdf').write_text(urdf);(output/'cell.srdf').write_text(srdf)
        (output/'controllers.yaml').write_text('joints: [q]\n');(output/'moveit.rviz').write_text('fixture\n')
        (output/'moveit.yaml').write_text(yaml.safe_dump(dict(robot_description=urdf,robot_description_semantic=srdf)))
        self.metadata=dict(schema_version=1,workcell_sha256=self.cell.fingerprint,joints=['q'],home_rad=[0.],
            joint_limits={'q':{'lower':-3.,'upper':3.}},outputs={name:sha((output/name).read_bytes()) for name in OUTPUTS})
        (output/'source.json').write_bytes(canonical(self.metadata)+b'\n')
        self.before={p.name:p.read_bytes() for p in output.iterdir()}
        manifest,mesh=normalize(b'synthetic fixture STEP',tessellation(),spec(),CONVERTER)
        self.bundle=self.root/'new_bundle';self.bundle.mkdir()
        (self.bundle/'asset.json').write_bytes(canonical(manifest)+b'\n');(self.bundle/'visual.stl').write_bytes(mesh)
        self.asset=dict(bundle=self.bundle,manifest_sha256=sha((self.bundle/'asset.json').read_bytes()),
            installation=dict(name='fixture',parent_frame='world',xyz_m=[0.,.65,.2],rpy_rad=[0.,0.,1.5707963267948966],rgba=[.4,.6,.8,1.]))

    def validate_base(self,cell):
        self.assertIs(cell,self.cell)
        self.assertEqual({p.name:p.read_bytes() for p in cell.output.iterdir()},self.before)
        return deepcopy(self.metadata)

    def build(self,name='installed_overlay'):
        directory=self.root/name
        return directory,build_overlay(self.cell,[self.asset],directory,overlay_id=name,base_validator=self.validate_base)

    def test_real_compilation_preserves_base_and_matches_rotated_urdf_mjcf_boxes(self):
        directory,report=self.build()
        self.assertEqual(report['artifact_kind'],'environment-overlay-v1')
        self.assertFalse((directory/'source.json').exists(),'Overlay never impersonates legacy workcell provenance')
        self.assertLess(report['geometry_check']['visual_world_bbox_error_m'],1e-6)
        self.assertLess(report['geometry_check']['urdf_mjcf_corner_error_m'],1e-12)
        self.assertEqual(report['geometry_check']['home_contacts'],0)
        self.assertEqual(report['geometry_check']['physics_steps'],0)
        self.assertEqual(report['joint_names'],self.cell.joints)
        self.assertEqual(check_overlay(directory,self.cell,base_validator=self.validate_base),report)
        self.assertEqual({p.name:p.read_bytes() for p in self.cell.output.iterdir()},self.before)

    def test_existing_output_is_never_overwritten(self):
        directory,report=self.build();before=(directory/'overlay.json').read_bytes()
        with self.assertRaisesRegex(ValueError,'new ID and output'):
            build_overlay(self.cell,[self.asset],directory,overlay_id='another',base_validator=self.validate_base)
        self.assertEqual((directory/'overlay.json').read_bytes(),before)

    def test_tampered_geometry_is_rejected_even_after_output_hash_is_recomputed(self):
        directory,report=self.build()
        path=directory/'cell.xml';xml=ET.parse(path)
        xml.find(".//geom[@name='environment_fixture_collision']").set('pos','0.01 0 0')
        xml.write(path)
        report['outputs']['cell.xml']=sha(path.read_bytes())
        (directory/'overlay.json').write_bytes(canonical(report)+b'\n')
        with self.assertRaisesRegex(ValueError,'geometry differs|XML contract changed'):
            check_overlay(directory,self.cell,base_validator=self.validate_base)

    def test_changed_base_during_composition_aborts_without_publishing(self):
        calls=[0]
        def unstable(cell):
            calls[0]+=1
            result=self.validate_base(cell)
            if calls[0]>1:result['changed']=True
            return result
        destination=self.root/'not_published'
        with self.assertRaisesRegex(ValueError,'Base changed'):
            build_overlay(self.cell,[self.asset],destination,overlay_id='not_published',base_validator=unstable)
        self.assertFalse(destination.exists())
        self.assertEqual({p.name:p.read_bytes() for p in self.cell.output.iterdir()},self.before)

class EnvironmentIdentityTests(unittest.TestCase):
    setUp=EnvironmentOverlayTests.setUp
    validate_base=EnvironmentOverlayTests.validate_base

    def build(self):
        directory=self.root/'identity_overlay'
        measured=dict(engine='MuJoCo fixture',physics_steps=0,home_contacts=0,
            visual_world_bbox_error_m=0.,collision_center_error_m=0.,urdf_mjcf_corner_error_m=0.)
        with patch('mujoco_ros2_manipulation.workcells.environment_overlay.geometry_check',return_value=measured):
            report=build_overlay(self.cell,[self.asset],directory,overlay_id='identity_overlay',base_validator=self.validate_base)
        return directory,report

    def test_identity_never_imports_engine_or_yaml_and_matches_exact_report(self):
        directory,report=self.build()
        import builtins
        original=builtins.__import__
        def blocked(name,*args,**kwargs):
            if name.split('.')[0] in ('mujoco','yaml','rclpy'):raise AssertionError('Parent identity imported '+name)
            return original(name,*args,**kwargs)
        with patch('builtins.__import__',side_effect=blocked):
            self.assertEqual(check_overlay_identity(directory,self.cell,expected_sha256=sha((directory/'overlay.json').read_bytes()),base_validator=self.validate_base),report)

    def test_missing_or_rehashed_changed_robot_contract_is_not_an_identity_pass(self):
        directory,report=self.build()
        path=directory/'cell.xml';xml=ET.parse(path);xml.find('.//joint').set('axis','1 0 0');xml.write(path)
        report['outputs']['cell.xml']=sha(path.read_bytes());(directory/'overlay.json').write_bytes(canonical(report)+b'\n')
        with self.assertRaisesRegex(ValueError,'XML contract changed'):
            check_overlay_identity(directory,self.cell,base_validator=self.validate_base)

    def test_identity_rejects_changed_producer_and_unknown_metadata_fields(self):
        directory,report=self.build()
        for change in ('producer','unknown'):
            changed=deepcopy(report)
            if change=='producer':
                changed['inputs']['producer_sha256']['mujoco_ros2_manipulation.workcells.environment_assets.py']='0'*64
                changed['fingerprint']=sha(canonical(changed['inputs']))
            else:changed['unreviewed_path']='/private/model'
            (directory/'overlay.json').write_bytes(canonical(changed)+b'\n')
            with self.subTest(change=change),self.assertRaises(ValueError):
                check_overlay_identity(directory,self.cell,base_validator=self.validate_base)

    def test_external_asset_hash_is_pinned_even_when_base_validator_returns_same_metadata(self):
        # Extend this independent base with an actual external mesh dependency.
        external=self.root/'external.stl';external.write_bytes((self.bundle/'visual.stl').read_bytes())
        path=self.cell.output/'cell.xml';xml=ET.parse(path);asset=ET.SubElement(xml.getroot(),'asset')
        ET.SubElement(asset,'mesh',name='external',file=str(external));xml.write(path)
        self.metadata['outputs']['cell.xml']=sha(path.read_bytes())
        (self.cell.output/'source.json').write_bytes(canonical(self.metadata)+b'\n')
        self.before={p.name:p.read_bytes() for p in self.cell.output.iterdir()}
        directory,report=self.build()
        self.assertIn(str(external),report['dependencies'])
        external.write_bytes(external.read_bytes()+b'changed')
        with self.assertRaisesRegex(ValueError,'external dependencies changed'):
            check_overlay_identity(directory,self.cell,base_validator=self.validate_base)

    def test_full_checker_never_skips_cpu_geometry(self):
        directory,_=self.build()
        with patch('mujoco_ros2_manipulation.workcells.environment_overlay.geometry_check',side_effect=RuntimeError('CPU preflight required')):
            with self.assertRaisesRegex(RuntimeError,'CPU preflight required'):
                check_overlay(directory,self.cell,base_validator=self.validate_base)

if __name__=='__main__':unittest.main()
