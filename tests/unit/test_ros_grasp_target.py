"""Explicit target selection remains transport-free and preserves arm units."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.grasping.ros_target import load_ros_grasp_target
from mujoco_ros2_manipulation.grasping.ros_model import FINGER_JOINTS


def report():
    return dict(id="fr5_parallel_block_v1",fingerprint="a"*64,home_rad=[0.]*12,
        scalar_joint_units={**{f"arm_{n}":"rad" for n in range(12)},**dict.fromkeys(FINGER_JOINTS,"m")},
        qpos_state_contract=dict(nq=21,nv=20,njnt=15,nu=14),tool={},block={},table={},contact_policy={},outputs={})


class RosGraspTargetTests(unittest.TestCase):
    def test_absent_pair_preserves_default_base_and_partial_or_malformed_pair_is_rejected(self):
        base=NS()
        self.assertIsNone(load_ros_grasp_target(base))
        for path,sha in (("/tmp/model",None),(None,"a"*64),("/tmp/model","not-a-sha"),("/tmp/model","A"*64),("/tmp/model",True)):
            with self.subTest(path=path,sha=sha),self.assertRaises(ValueError):load_ros_grasp_target(base,path,sha)

    def test_loader_freezes_checked_target_and_arm_finger_units_are_separate(self):
        base=NS(root=Path('/tmp/base'),id='fr5_ur5e',joints=[f'arm_{n}' for n in range(12)],instances=('left','right'))
        document=report()
        with patch('mujoco_ros2_manipulation.grasping.ros_target.check_model',return_value=document) as check:
            target=load_ros_grasp_target(base,'/tmp/grasp','b'*64)
            self.assertEqual(check.call_count,2)
            self.assertEqual(check.call_args.kwargs,{'expected_sha256':'b'*64})
            self.assertEqual(target.joints,base.joints)
            self.assertEqual(target.allowed_scalar_joints,list(base.joints)+list(FINGER_JOINTS))
            self.assertEqual(target.finger_joints,list(FINGER_JOINTS))
            self.assertEqual(target.model_path,Path('/tmp/grasp/cell.xml'))
            self.assertEqual(target.instances,('left','right'))
            self.assertEqual(target.root,base.root)
            self.assertFalse(target.description()['native_execution_verified'])
            self.assertEqual(target.description()['scalar_joint_units'][FINGER_JOINTS[0]],'m')
            document['home_rad'][0]=1.
            self.assertEqual(target.home[0],0.)
            with self.assertRaises(ValueError):target.validate()

    def test_target_description_and_manifest_are_detached(self):
        base=NS(root=Path('/tmp/base'),id='fr5_ur5e',joints=[f'arm_{n}' for n in range(12)],instances=('left','right'))
        with patch('mujoco_ros2_manipulation.grasping.ros_target.check_model',return_value=report()):
            target=load_ros_grasp_target(base,'/tmp/grasp','b'*64)
            first=target.description();first['scalar_joint_units'][FINGER_JOINTS[0]]='rad'
            second=target.manifest;second['home_rad'][0]=2.
            self.assertEqual(target.description()['scalar_joint_units'][FINGER_JOINTS[0]],'m')
            self.assertEqual(target.home[0],0.)


if __name__=='__main__':unittest.main()
