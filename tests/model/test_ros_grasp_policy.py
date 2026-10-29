"""Actual native3.12 policy, retaining the old whole-world default interlock."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.grasping.ros_model import build_model


class RosGraspNativePolicyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vendor=ROOT/'runtime/root/opt/ros/humble/opt/mujoco_vendor'
        if not (cls.vendor/'lib/libmujoco.so.3.12.0').is_file():raise unittest.SkipTest('Native MuJoCo runtime required')
        cls.temp=tempfile.TemporaryDirectory();cls.addClassCleanup(cls.temp.cleanup)
        cls.folder=Path(cls.temp.name);cls.model=cls.folder/'model'
        build_model(load_workcell('fr5_ur5e'),cls.model)
        cls.monitor=hashlib.sha256((ROOT/'native/step_monitor.hpp').read_bytes()).hexdigest()
        cls.policy=hashlib.sha256((ROOT/'native/grasp_policy.hpp').read_bytes()).hexdigest()
        for suffix,definitions in (('checked',['-DMANIPULATION_GRASP_POLICY_SHA256="'+cls.policy+'"']),('unverified',[])):
            subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-Wconversion',
                '-DMANIPULATION_MONITOR_SHA256="'+cls.monitor+'"',*definitions,
                '-I'+str(ROOT/'native'),'-I'+str(cls.vendor/'include'),str(ROOT/'native/grasp_policy_probe.cpp'),
                str(cls.vendor/'lib/libmujoco.so.3.12.0'),'-Wl,-rpath,'+str(cls.vendor/'lib'),
                '-o',str(cls.folder/suffix)],check=True,capture_output=True)

    def run_case(self,mode,binary='checked'):
        output=self.folder/(mode+'.json')
        subprocess.run([str(self.folder/binary),str(self.model/'cell.xml'),mode],check=True,timeout=15,
            env=dict(os.environ,MANIPULATION_CONTACT_STOP='1',MANIPULATION_DIAGNOSTICS_PATH=str(output),
                     MANIPULATION_DIAGNOSTICS_RUN_ID='native-grasp-'+mode),capture_output=True)
        return json.loads(output.read_text()) if output.exists() else None

    def test_phase_matrix_allows_only_exact_tool_object_and_support_pairs(self):
        self.run_case('matrix')

    def test_resting_free_block_does_not_stop_idle_common_physics(self):
        value=self.run_case('idle')
        self.assertFalse(value['stop_latched']);self.assertGreater(value['contacts'],0)
        self.assertEqual(value['steps'],150)

    def test_transport_table_and_unexpected_arm_contact_latch_the_whole_world(self):
        for mode in ('transport_table','unexpected'):
            with self.subTest(mode=mode):
                value=self.run_case(mode);self.assertTrue(value['stop_latched'])
                self.assertEqual(value['stop_reason'],'contact')
                self.assertGreater(value['hold_ticks'],0)

    def test_cancel_and_heartbeat_loss_latch_before_another_physics_integration(self):
        for mode,reason in (('cancel','grasp_cancel'),('expired','grasp_heartbeat')):
            with self.subTest(mode=mode):
                value=self.run_case(mode);self.assertEqual(value['engine_steps'],0)
                self.assertEqual(value['stop_reason'],reason)

    def test_missing_compiled_policy_identity_fails_closed(self):
        value=self.run_case('unverified','unverified')
        self.assertEqual(value['stop_reason'],'grasp_policy_invalid')
        self.assertEqual(value['engine_steps'],0)


if __name__=='__main__':unittest.main()
