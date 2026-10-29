"""Independent native offline hold diagnostics, never a live ROS state edit.

This small gravity/contact fixture checks the probe's physical controls and
identity. CI133 state reconstruction is a separate preserved runtime artifact;
these tests do not claim that a simple box fixture reproduces the complete arm.
"""
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

from mujoco_ros2_manipulation.paths import ROOT


MODEL = '''<mujoco>
  <option timestep=".002" integrator="implicitfast"/>
  <default><geom condim="4" friction=".8 .01 .001" solref=".008 1" solimp=".95 .99 .001"/></default>
  <worldbody>
    <site name="fr5_grasp_tcp" pos="0 0 .2"/>
    <geom name="fr5_table_geom" type="box" size=".15 .15 .05"/>
    <body name="left" pos="-.04 0 .2">
      <joint name="left" type="slide" axis="1 0 0" range="0 .034" damping="1" armature=".001"/>
      <geom name="fr5_left_finger_geom" type="box" size=".005 .02 .04" mass=".03"/>
    </body>
    <body name="right" pos=".04 0 .2">
      <joint name="right" type="slide" axis="-1 0 0" range="0 .034" damping="1" armature=".001"/>
      <geom name="fr5_right_finger_geom" type="box" size=".005 .02 .04" mass=".03"/>
    </body>
    <body name="fr5_grasp_block" pos="0 0 .2">
      <freejoint/>
      <geom name="fr5_block_geom" type="box" size=".02 .015 .025" mass=".08"/>
    </body>
  </worldbody>
  <actuator>
    <position name="fr5_left_finger_servo" joint="left" kp="2000" kv="20" forcelimited="true" forcerange="-20 20"/>
    <position name="fr5_right_finger_servo" joint="right" kp="2000" kv="20" forcelimited="true" forcerange="-20 20"/>
  </actuator>
</mujoco>'''


class RosGraspHoldProbeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        vendor = ROOT / 'runtime/root/opt/ros/humble/opt/mujoco_vendor'
        library = vendor / 'lib/libmujoco.so.3.12.0'
        if not library.is_file():
            raise unittest.SkipTest('Native MuJoCo 3.12 runtime required')
        cls.temporary = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.folder = Path(cls.temporary.name)
        cls.model = cls.folder / 'fixture.xml'
        cls.model.write_text(MODEL)
        cls.state = cls.folder / 'state.txt'
        cls.state.write_text('9 8 2 0\n.015 .015 0 0 .2 1 0 0 0\n'
                             '0 0 0 0 0 0 0 0\n.026 .026\n0 0 0\n1 0 0 0\n')
        source = ROOT / 'native/grasp_hold_probe.cpp'
        cls.source_sha256 = hashlib.sha256(source.read_bytes()).hexdigest()
        cls.probe = cls.folder / 'probe'
        subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
            '-I'+str(vendor/'include'),
            '-DMANIPULATION_GRASP_HOLD_PROBE_SHA256="'+cls.source_sha256+'"',
            str(source), str(library), '-Wl,-rpath,'+str(vendor/'lib'), '-o', str(cls.probe)],
            check=True, capture_output=True, timeout=30)

    def run_probe(self, cone='elliptic10', friction='normal', fingers='closed', seconds='2'):
        value = subprocess.run([str(self.probe), str(self.model), str(self.state), seconds,
                                cone, friction, fingers], capture_output=True, check=True, timeout=15)
        return json.loads(value.stdout)

    def test_native_identity_original_options_and_preserved_reference(self):
        original = self.run_probe(cone='original')
        self.assertEqual(original['native_version'], '3.12.0')
        self.assertEqual(original['source_sha256'], self.source_sha256)
        self.assertFalse(original['live_state_writes'])
        self.assertEqual((original['cone'], original['impratio']), (0, 1))
        self.assertIn('not re-anchored', original['reference'])
        self.assertEqual(original['steps'], 1000)
        self.assertAlmostEqual(original['elapsed_sec'], 2)

    def test_candidate_holds_real_free_body_against_gravity(self):
        value = self.run_probe()
        self.assertEqual((value['cone'], value['impratio']), (1, 10))
        self.assertLess(value['max_relative_position_error_m'], .005)
        self.assertTrue(all(min(s['finger_force_n']) > 10 for s in value['samples'][1:]))
        self.assertTrue(all(s['table_force_n'] == 0 for s in value['samples']))
        self.assertAlmostEqual(value['samples'][-1]['block_z_m'], .2, delta=.005)

    def test_low_friction_and_open_fingers_do_not_report_success(self):
        for mode in ('low-friction', 'open'):
            with self.subTest(mode=mode):
                value = self.run_probe(friction='low-friction' if mode=='low-friction' else 'normal',
                                       fingers='open' if mode=='open' else 'closed')
                self.assertGreater(value['max_relative_position_error_m'], .05)
                self.assertGreaterEqual(value['first_contact_gate_failure_sec'], 0)
                self.assertLessEqual(value['first_contact_gate_failure_sec'], .251)
                self.assertLess(value['samples'][-1]['block_z_m'], .1)

    def test_original_close_reference_is_not_reset_to_input_pose(self):
        original = self.state.read_text()
        try:
            self.state.write_text(original.replace('\n0 0 0\n1 0 0 0\n', '\n0 0 .006\n1 0 0 0\n'))
            value = self.run_probe()
        finally:
            self.state.write_text(original)
        self.assertAlmostEqual(value['samples'][0]['relative_position_error_m'], .006)
        self.assertGreater(value['max_relative_position_error_m'], .005)

    def test_bad_state_shape_is_rejected_before_simulation(self):
        original = self.state.read_text()
        try:
            self.state.write_text(original.replace('9 8 2 0', '21 20 14 0'))
            value = subprocess.run([str(self.probe), str(self.model), str(self.state), '2',
                                    'elliptic10', 'normal', 'closed'], capture_output=True, timeout=15)
        finally:
            self.state.write_text(original)
        self.assertEqual(value.returncode, 2)
        self.assertIn(b'dimensions differ', value.stderr)
        self.assertFalse(value.stdout)


if __name__ == '__main__':
    unittest.main()
