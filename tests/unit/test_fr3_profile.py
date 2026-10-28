"""FAIRINO FR3 V6 profile matches the existing locked manufacturer assets."""
from copy import deepcopy
import hashlib
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from mujoco_ros2_manipulation.build import load_manifest, build_plan
from mujoco_ros2_manipulation.profiles import ROOT, controller_config, load_profile, moveit_controller_config


class FR3ProfileTests(unittest.TestCase):
    def setUp(self):
        self.profile = load_profile("fr3")

    def test_profile_is_explicitly_experimental_and_uses_six_named_axes(self):
        self.assertTrue(self.profile.data["enabled"])
        self.assertEqual(self.profile.data["status"], "experimental")
        self.assertEqual(self.profile.joints, ["j"+str(index) for index in range(1, 7)])
        self.assertEqual(self.profile.ros["robot_name"], "fairino3_v6_robot")
        self.assertEqual(self.profile.ros["planning_group"], "fairino3_v6_group")
        self.assertEqual(self.profile.ros["tip_frame"], "wrist3_link")

    def test_moveit_package_is_registered_under_the_existing_source_lock(self):
        config = load_manifest(ROOT)
        self.assertEqual(config["repositories"]["fairino"]["commit"], "fcf0c7f0d60d949d8a9a4238f929a44d07f60379")
        self.assertEqual(config["packages"]["fairino3_v6_moveit2_config"],
                         dict(repository="fairino", path="fairino3_v6_moveit2_config"))
        self.assertEqual(self.profile.data["source"]["commit"], config["repositories"]["fairino"]["commit"])
        plan = build_plan(["fr3"])
        self.assertEqual(plan["robots"], ["fr3"])
        self.assertEqual(plan["repositories"], ["fairino"])
        self.assertEqual({package["name"] for package in plan["packages"]},
                         {"fairino3_v6_moveit2_config", "fairino_description"})

    def test_controller_and_moveit_configs_use_fr3_not_fr5_names(self):
        controllers = controller_config(self.profile)
        manager = controllers["controller_manager"]["ros__parameters"]
        self.assertIn("fairino3_controller", manager)
        self.assertNotIn("fairino5_controller", manager)
        configured = controllers["fairino3_controller"]["ros__parameters"]
        self.assertEqual(configured["joints"], self.profile.joints)
        self.assertEqual(configured["state_interfaces"], ["position", "velocity"])
        moveit = moveit_controller_config(self.profile)["moveit_simple_controller_manager"]
        self.assertEqual(moveit["controller_names"], ["fairino3_controller"])
        self.assertEqual(moveit["fairino3_controller"]["joints"], self.profile.joints)

    def test_initial_gains_are_simulation_values_with_the_existing_force_limit_path(self):
        self.assertEqual(self.profile.data["servo"], load_profile("fr5").data["servo"])
        source = (ROOT/"config/robots/fr3.yaml").read_text()
        self.assertIn("not manufacturer controller gains", source)
        self.assertEqual(self.profile.data["physics"], dict(timestep=.002, integrator="implicitfast", sim_speed_factor=1.))

    def test_locked_urdf_srdf_meshes_and_named_goals_are_consistent(self):
        urdf = self.profile.source_path("urdf")
        srdf = self.profile.source_path("srdf")
        if not urdf.is_file() or not srdf.is_file():
            self.skipTest("Locked manufacturer checkout is required")
        self.assertEqual(hashlib.sha256(urdf.read_bytes()).hexdigest(),
                         "ba9266ba6d32391b8c68020bea2533f6070dc3be654607abe7ed1b7f25e13021")
        robot, semantics = ET.parse(urdf).getroot(), ET.parse(srdf).getroot()
        self.assertEqual(robot.attrib["name"], self.profile.ros["robot_name"])
        self.assertEqual(semantics.attrib["name"], robot.attrib["name"])
        joints = {joint.attrib["name"]: joint for joint in robot.findall("joint") if joint.attrib["type"] != "fixed"}
        self.assertEqual(set(joints), set(self.profile.joints))
        group = semantics.find("group[@name='fairino3_v6_group']")
        self.assertEqual([joint.attrib["name"] for joint in group.findall("joint")], self.profile.joints)
        self.assertEqual(group.find("chain").attrib, dict(base_link="base_link", tip_link="wrist3_link"))
        for state in (self.profile.home, self.profile.data["verification"]["direct_target_rad"],
                      self.profile.data["verification"]["moveit_target_rad"]):
            for name, value in zip(self.profile.joints, state):
                limits = joints[name].find("limit").attrib
                self.assertLessEqual(float(limits["lower"]), value)
                self.assertLessEqual(value, float(limits["upper"]))
        meshes = {mesh.attrib["filename"] for mesh in robot.findall(".//mesh")}
        self.assertEqual(len(meshes), 7)
        for uri in meshes:
            self.assertTrue(uri.startswith("package://fairino_description/meshes/fairino3_v6/"))
            path = self.profile.path("external/frcobot_ros2/fairino_description/"+uri.split("fairino_description/", 1)[1])
            self.assertTrue(path.is_file())


if __name__ == "__main__":
    unittest.main()
