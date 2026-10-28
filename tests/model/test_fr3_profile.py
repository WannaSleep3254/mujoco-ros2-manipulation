"""Detached FR3 conversion checks; no ROS runtime or published models are written."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from mujoco_ros2_manipulation.profiles import ROOT, check_generated_model, load_profile
from tests.helpers import urdf_fk


@unittest.skipUnless(importlib.util.find_spec("mujoco") and importlib.util.find_spec("numpy"),
                     "MuJoCo/NumPy are required for isolated model checks")
class FR3ModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import mujoco
        import numpy as np
        from mujoco_ros2_manipulation.model_build.prepare import StagedProfile, _prepare
        cls.mj, cls.np = mujoco, np
        profile = load_profile("fr3")
        if not profile.source_path("urdf").is_file():
            raise unittest.SkipTest("Locked FR3 manufacturer assets are required")
        cls.temporary = tempfile.TemporaryDirectory(prefix="fr3-model-check-")
        cls.profile = StagedProfile(profile.root, profile.data, Path(cls.temporary.name), scene_data=profile.scene_data)
        cls.report = _prepare(cls.profile, cls.profile)
        check_generated_model(cls.profile)
        cls.model = mujoco.MjModel.from_xml_path(str(cls.profile.scene_path))

    @classmethod
    def tearDownClass(cls):
        if hasattr(cls, "temporary"):
            cls.temporary.cleanup()

    def test_generated_model_has_six_arm_joints_and_limited_real_servos(self):
        model = self.model
        self.assertEqual((model.nq, model.nv, model.nu, model.neq), (6, 6, 6, 0))
        self.assertEqual(self.report["gripper"], "not included")
        for index, name in enumerate(self.profile.joints):
            joint, actuator = model.joint(name), model.actuator(name)
            limits = self.report["joint_limits"][name]
            self.assertEqual(joint.type[0], self.mj.mjtJoint.mjJNT_HINGE)
            self.np.testing.assert_allclose(model.jnt_range[joint.id], [limits["lower"], limits["upper"]], atol=1e-12)
            self.np.testing.assert_allclose(model.actuator_forcerange[actuator.id], [-limits["effort"], limits["effort"]])
            self.assertEqual(model.actuator_gainprm[actuator.id, 0], self.profile.data["servo"]["kp"][index])
            self.assertEqual(model.actuator_biasprm[actuator.id, 2], -self.profile.data["servo"]["kv"][index])
            self.assertEqual(int(model.actuator_trnid[actuator.id, 0]), joint.id)

    def test_home_keyframe_and_static_settling_evidence_are_finite_without_contacts(self):
        data = self.mj.MjData(self.model)
        self.mj.mj_resetDataKeyframe(self.model, data, 0)
        self.np.testing.assert_allclose(data.ctrl, self.profile.home, atol=0, rtol=0)
        for name, value in zip(self.profile.joints, self.profile.home):
            self.assertEqual(data.joint(name).qpos[0], value)
        self.assertEqual(self.report["contacts_at_settled_home"], 0)
        self.assertTrue(self.np.isfinite(self.report["initial_pose_error_rad"]).all())
        self.assertLess(max(abs(value) for value in self.report["initial_pose_error_rad"]), .005)

    def test_independent_urdf_fk_matches_generated_mjcf_at_all_three_declared_poses(self):
        robot = ET.parse(self.profile.urdf_path).getroot()
        data = self.mj.MjData(self.model)
        for pose in (self.profile.home, self.profile.data["verification"]["direct_target_rad"],
                     self.profile.data["verification"]["moveit_target_rad"]):
            coordinates = dict(zip(self.profile.joints, pose))
            for name, value in coordinates.items():
                data.joint(name).qpos[0] = value
            self.mj.mj_forward(self.model, data)
            expected = urdf_fk(robot, coordinates)[self.profile.ros["tip_frame"]]
            actual = data.body(self.profile.ros["tip_frame"])
            self.np.testing.assert_allclose(actual.xpos, expected[:3, 3], atol=1e-6, rtol=0)
            self.np.testing.assert_allclose(actual.xmat.reshape(3, 3), expected[:3, :3], atol=1e-6, rtol=0)

    def test_only_manufacturer_collision_exclusions_are_carried_into_the_model(self):
        original = ET.parse(self.profile.source_path("srdf")).getroot()
        expected = [(row.attrib["link1"], row.attrib["link2"]) for row in original.findall("disable_collisions")]
        generated = ET.parse(self.profile.scene_path).getroot()
        actual = [(row.attrib["body1"], row.attrib["body2"]) for row in generated.findall("contact/exclude")]
        self.assertEqual(actual, expected)
        self.assertEqual(len(actual), 9)

    def test_provenance_and_hardware_plugin_are_bound_to_this_isolated_output(self):
        self.assertEqual(self.report["source_commit"], "fcf0c7f0d60d949d8a9a4238f929a44d07f60379")
        self.assertEqual(self.report["robot_profile_sha256"], self.profile.fingerprint)
        self.assertEqual(self.report["mujoco_python_version"], self.mj.__version__)
        robot = ET.parse(self.profile.urdf_path).getroot()
        hardware = robot.find("ros2_control/hardware")
        self.assertEqual(hardware.find("plugin").text, "mujoco_ros2_control/MujocoSystemInterface")
        self.assertEqual(hardware.find("param[@name='mujoco_model']").text, str(self.profile.scene_path))
        self.assertEqual(len(robot.findall("ros2_control/joint")), 6)


if __name__ == "__main__":
    unittest.main()
