"""Experimental FR16 V6 registration uses the existing manufacturer source lock."""
import hashlib
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from mujoco_ros2_manipulation.build import build_plan, load_manifest
from mujoco_ros2_manipulation.profiles import (
    ROOT, controller_config, load_profile, moveit_controller_config,
)


class FR16ProfileTests(unittest.TestCase):
    def setUp(self):
        self.profile = load_profile("fr16")

    def test_profile_declares_experimental_status_and_six_named_axes(self):
        self.assertTrue(self.profile.data["enabled"])
        self.assertEqual(self.profile.data["status"], "experimental")
        self.assertEqual(self.profile.label, "FAIRINO FR16 V6")
        self.assertEqual(self.profile.joints, ["j" + str(index) for index in range(1, 7)])
        expected = {
            "robot_name": "fairino16_v6_robot",
            "system_name": "FR16MuJoCo",
            "moveit_package": "fairino16_v6_moveit2_config",
            "planning_group": "fairino16_v6_group",
            "trajectory_controller": "fairino16_controller",
            "state_controller": "joint_state_broadcaster",
            "world_frame": "world",
            "base_frame": "base_link",
            "tip_frame": "wrist3_link",
        }
        for key, value in expected.items():
            with self.subTest(key=key):
                self.assertEqual(self.profile.ros[key], value)
        self.assertEqual(self.profile.ros["tf_child_frames"], [
            "shoulder_link", "upperarm_link", "forearm_link",
            "wrist1_link", "wrist2_link", "wrist3_link",
        ])

    def test_build_plan_selects_existing_locked_description_and_moveit_packages(self):
        manifest = load_manifest(ROOT)
        pin = "fcf0c7f0d60d949d8a9a4238f929a44d07f60379"
        self.assertEqual(manifest["repositories"]["fairino"]["commit"], pin)
        self.assertEqual(self.profile.data["source"]["commit"], pin)
        self.assertEqual(manifest["packages"]["fairino16_v6_moveit2_config"], {
            "repository": "fairino", "path": "fairino16_v6_moveit2_config",
        })
        plan = build_plan(["fr16"])
        self.assertEqual(plan["robots"], ["fr16"])
        self.assertEqual(plan["repositories"], ["fairino"])
        self.assertEqual({package["name"] for package in plan["packages"]}, {
            "fairino_description", "fairino16_v6_moveit2_config",
        })
        self.assertTrue(all(Path(package["path"]).is_relative_to(ROOT) for package in plan["packages"]))

    def test_generated_controller_configuration_uses_fr16_identity_and_joint_order(self):
        controllers = controller_config(self.profile)
        manager = controllers["controller_manager"]["ros__parameters"]
        self.assertIn("fairino16_controller", manager)
        self.assertNotIn("fairino10_controller", manager)
        self.assertNotIn("fairino5_controller", manager)
        configured = controllers["fairino16_controller"]["ros__parameters"]
        self.assertEqual(configured["joints"], self.profile.joints)
        self.assertEqual(configured["state_interfaces"], ["position", "velocity"])
        moveit = moveit_controller_config(self.profile)["moveit_simple_controller_manager"]
        self.assertEqual(moveit["controller_names"], ["fairino16_controller"])
        self.assertEqual(moveit["fairino16_controller"]["joints"], self.profile.joints)

    def test_initial_gains_are_explicit_simulation_candidates(self):
        self.assertEqual(self.profile.data["servo"], load_profile("fr10").data["servo"])
        profile_source = (ROOT / "config/robots/fr16.yaml").read_text()
        self.assertIn("not manufacturer controller gains", profile_source)
        self.assertIn("FR16 tracking remains unverified", profile_source)
        self.assertEqual(self.profile.data["physics"], {
            "timestep": .002, "integrator": "implicitfast", "sim_speed_factor": 1.,
        })

    def test_locked_assets_match_frames_limits_and_declared_test_poses(self):
        urdf = self.profile.source_path("urdf")
        srdf = self.profile.source_path("srdf")
        if not urdf.is_file() or not srdf.is_file():
            self.skipTest("Locked FR16 manufacturer assets are required")
        self.assertEqual(hashlib.sha256(urdf.read_bytes()).hexdigest(),
                         "0cc434b71338c960f5b00a187e17fc09ff26358599ce16525e51c2b5c777eb98")
        self.assertEqual(hashlib.sha256(srdf.read_bytes()).hexdigest(),
                         "7a046a9a807ba75b7dbbf5666cc93b3575f6a70caf9ddea58f91f356562d8cee")
        robot = ET.parse(urdf).getroot()
        semantics = ET.parse(srdf).getroot()
        self.assertEqual(robot.get("name"), self.profile.ros["robot_name"])
        self.assertEqual(semantics.get("name"), robot.get("name"))
        joints = {joint.get("name"): joint for joint in robot.findall("joint")
                  if joint.get("type") != "fixed"}
        self.assertEqual(set(joints), set(self.profile.joints))
        self.assertEqual([float(joints[name].find("limit").get("effort"))
                          for name in self.profile.joints], [150., 150., 150., 28., 28., 28.])
        group = semantics.find("group[@name='fairino16_v6_group']")
        self.assertIsNotNone(group)
        self.assertEqual([joint.get("name") for joint in group.findall("joint")], self.profile.joints)
        self.assertEqual(group.find("chain").attrib, {
            "base_link": "base_link", "tip_link": "wrist3_link",
        })
        for pose in (self.profile.home, self.profile.data["verification"]["direct_target_rad"],
                     self.profile.data["verification"]["moveit_target_rad"]):
            for name, value in zip(self.profile.joints, pose):
                with self.subTest(joint=name, pose=pose):
                    limits = joints[name].find("limit")
                    self.assertLessEqual(float(limits.get("lower")), value)
                    self.assertLessEqual(value, float(limits.get("upper")))
        meshes = {mesh.get("filename") for mesh in robot.findall(".//mesh")}
        prefix = "package://fairino_description/"
        self.assertEqual(len(meshes), 7)
        for uri in meshes:
            self.assertTrue(uri.startswith(prefix + "meshes/fairino16_v6/"))
            mesh = self.profile.path("external/frcobot_ros2/fairino_description/" + uri[len(prefix):])
            self.assertTrue(mesh.is_file())
        self.assertEqual(len(semantics.findall("disable_collisions")), 11)


if __name__ == "__main__":
    unittest.main()
