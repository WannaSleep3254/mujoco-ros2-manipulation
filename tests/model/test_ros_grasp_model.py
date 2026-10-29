"""Static generated two-arm/tool/free-body model coverage; never launches ROS."""
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.integrity import OUTPUTS,check_workcell
from mujoco_ros2_manipulation.grasping.ros_model import build_model,check_model,FINGER_JOINTS,FREE_JOINT,POLICY
from mujoco_ros2_manipulation.grasping.ros_target import load_ros_grasp_target


@unittest.skipUnless(importlib.util.find_spec('mujoco') and importlib.util.find_spec('numpy'), 'MuJoCo/NumPy required')
class RosGraspModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import mujoco
        cls.mj=mujoco;cls.base=load_workcell('fr5_ur5e')
        try:check_workcell(cls.base)
        except ValueError as error:raise unittest.SkipTest('Checked generated base assets required: '+str(error))
        cls.folder=tempfile.TemporaryDirectory()
        cls.output=Path(cls.folder.name)/'grasp'
        cls.original={name:(cls.base.output/name).read_bytes() for name in (*OUTPUTS,'source.json')}
        cls.manifest=build_model(cls.base,cls.output)
        cls.raw=(cls.output/'grasp.json').read_bytes()
        cls.sha=hashlib.sha256(cls.raw).hexdigest()
        cls.model=mujoco.MjModel.from_xml_path(str(cls.output/'cell.xml'))

    @classmethod
    def tearDownClass(cls):cls.folder.cleanup()

    def test_whole_world_has_both_arms_and_metres_fingers_plus_complete_free_block(self):
        m=self.model
        self.assertEqual((m.nu,m.njnt,m.nq,m.nv),(14,15,21,20))
        self.assertEqual(m.neq,0);self.assertEqual(m.nmocap,0);self.assertEqual(m.nuserdata,2)
        for name in self.base.joints:self.assertEqual(m.joint(name).type[0],self.mj.mjtJoint.mjJNT_HINGE)
        for name in FINGER_JOINTS:self.assertEqual(m.joint(name).type[0],self.mj.mjtJoint.mjJNT_SLIDE)
        self.assertEqual(m.joint(FREE_JOINT).type[0],self.mj.mjtJoint.mjJNT_FREE)
        units=self.manifest['scalar_joint_units']
        self.assertEqual({units[name] for name in self.base.joints},{'rad'})
        self.assertEqual({units[name] for name in FINGER_JOINTS},{'m'})

    def test_original_arm_dynamics_actuators_and_source_files_are_preserved(self):
        source=ET.fromstring(self.original['cell.xml']);result=ET.parse(self.output/'cell.xml').getroot()
        for name in self.base.joints:
            self.assertEqual(source.find(".//joint[@name='"+name+"']").attrib,result.find(".//joint[@name='"+name+"']").attrib)
            self.assertEqual(source.find("actuator/position[@joint='"+name+"']").attrib,result.find("actuator/position[@joint='"+name+"']").attrib)
        for original in source.findall('.//inertial'):
            self.assertIn(original.attrib,[node.attrib for node in result.findall('.//inertial')])
        self.assertIsNotNone(result.find("worldbody/body[@name='right_world']"))
        self.assertEqual([row.attrib for row in source.findall('contact/exclude')],[row.attrib for row in result.findall('contact/exclude')])
        for name,raw in self.original.items():self.assertEqual((self.base.output/name).read_bytes(),raw)

    def test_initial_keyframe_uses_joint_addresses_not_assumed_arm_order(self):
        data=self.mj.MjData(self.model);self.mj.mj_resetDataKeyframe(self.model,data,0);self.mj.mj_forward(self.model,data)
        for name,value in zip(self.base.joints,self.manifest['home_rad']):
            self.assertAlmostEqual(data.qpos[int(self.model.joint(name).qposadr[0])],value)
        for name in FINGER_JOINTS:self.assertEqual(data.qpos[int(self.model.joint(name).qposadr[0])],0.)
        self.assertEqual(self.manifest['qpos_state_contract']['layout'][6]['name'],FINGER_JOINTS[0])
        self.assertGreater(int(self.model.joint(self.base.joints[6]).qposadr[0]),6)
        self.assertEqual(data.time,0.)
        site=int(self.model.site('fr5_grasp_tcp').id)
        self.assertLess(max(abs(float(a)-b) for a,b in zip(data.site_xpos[site],self.manifest['geometry']['initial_tcp_position_m'])),1e-10)

    def test_ros_geometry_and_controller_metadata_separate_arm_and_hand(self):
        robot=ET.parse(self.output/'cell.urdf').getroot();semantic=ET.parse(self.output/'cell.srdf').getroot()
        self.assertEqual([node.get('name') for node in robot.findall('ros2_control/joint')],list(self.base.joints)+list(FINGER_JOINTS))
        self.assertIsNone(robot.find("link[@name='fr5_grasp_block']"))
        self.assertEqual(robot.find("joint[@name='fr5_synthetic_gripper_fixed']/origin").get('rpy'),'3.1415926535897931 0 0')
        self.assertIsNotNone(semantic.find("group[@name='both_arms']"))
        pairs=[row.attrib for row in semantic.findall('disable_collisions')][-3:]
        self.assertEqual(pairs,self.manifest['tool']['adjacent_collision_exclusions'])
        config=yaml.safe_load((self.output/'controllers.yaml').read_text())
        self.assertEqual(config['gripper_controller']['ros__parameters']['joints'],list(FINGER_JOINTS))
        original=yaml.safe_load(self.original['controllers.yaml'])
        for name in ('left_controller','right_controller'):self.assertEqual(config[name],original[name])
        custom=ET.parse(self.output/'cell.xml').find("custom/text[@name='manipulation_grasp_policy']")
        self.assertEqual(custom.get('data'),POLICY)
        plugins=yaml.safe_load((self.output/'grasp_plugins.yaml').read_text())
        self.assertEqual(plugins,{'/**':{'ros__parameters':{'mujoco_plugins':{
            'manipulation_grasp':{'type':'manipulation_grasp/ObservationPlugin'}}}}})
        self.assertEqual(self.manifest['outputs']['grasp_plugins.yaml'],
                         hashlib.sha256((self.output/'grasp_plugins.yaml').read_bytes()).hexdigest())

    def test_sha_pair_and_repeated_build_validate_without_claiming_native_execution(self):
        target=load_ros_grasp_target(self.base,self.output,self.sha)
        self.assertEqual(len(target.joints),12);self.assertEqual(len(target.allowed_scalar_joints),14)
        self.assertFalse(target.validate()['native_execution_verified'])
        with tempfile.TemporaryDirectory() as folder:
            other=Path(folder)/'model';second=build_model(self.base,other)
            # Output hardware paths differ explicitly, but physical model and provenance do not.
            self.assertEqual(second['outputs']['cell.xml'],self.manifest['outputs']['cell.xml'])
            self.assertEqual(second['inputs'],self.manifest['inputs'])
        with self.assertRaises(ValueError):load_ros_grasp_target(self.base,self.output,'0'*64)
        with self.assertRaises(ValueError):build_model(self.base,self.output)

    def test_modified_output_units_qpos_or_policy_is_rejected(self):
        for change in ('output','units','qpos','policy','native_claim'):
            with self.subTest(change=change),tempfile.TemporaryDirectory() as folder:
                import shutil
                copied=Path(folder)/'grasp';shutil.copytree(self.output,copied)
                # Hardware paths stay bound to original publication; use final_directory only to test rehash semantics.
                value=json.loads(self.raw)
                if change=='output':(copied/'cell.xml').write_bytes((copied/'cell.xml').read_bytes()+b' ')
                elif change=='units':value['scalar_joint_units'][FINGER_JOINTS[0]]='rad'
                elif change=='qpos':value['qpos_state_contract']['layout'][6]['qpos_width']=7
                elif change=='policy':value['contact_policy']['finger_geoms']=['unrelated']
                else:value['validation']['native_execution_verified']=True
                (copied/'grasp.json').write_text(json.dumps(value))
                with self.assertRaises(ValueError):check_model(copied,self.base,final_directory=self.output)


if __name__=='__main__':unittest.main()
