"""Detached manufacturer-arm prototype checks, independent of ROS services."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from mujoco_ros2_manipulation.grasping.fr5_model import ARM_JOINTS, TCP_SITE, build_model, write_model
from mujoco_ros2_manipulation.grasping.fr5_run import run_prototype, solve_ik
from mujoco_ros2_manipulation.paths import ROOT


@unittest.skipUnless(importlib.util.find_spec("mujoco") and importlib.util.find_spec("numpy"),
                     "MuJoCo/NumPy are required for detached FR5 prototype checks")
class FR5PrototypeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import mujoco
        import numpy as np
        cls.mj, cls.np = mujoco, np
        cls.source = ROOT / "runtime/generated/workcells/fr5_ur5e"
        if not (cls.source/"cell.xml").is_file() or not (ROOT/"models/fr5/source.json").is_file():
            raise unittest.SkipTest("Checked generated FR5 workcell assets are not present")
        cls.before_cell = (cls.source/"cell.xml").read_bytes()
        cls.before_metadata = (cls.source/"source.json").read_bytes()
        cls.raw, cls.manifest = build_model(cls.source)
        cls.model = mujoco.MjModel.from_xml_string(cls.raw.decode())

    def setUp(self):
        self.data = self.mj.MjData(self.model)
        self.mj.mj_resetDataKeyframe(self.model, self.data, 0)
        self.mj.mj_forward(self.model, self.data)

    def test_source_and_mesh_provenance_is_bound_without_source_edits(self):
        manifest = self.manifest
        self.assertEqual(manifest["source_cell_sha256"], hashlib.sha256(self.before_cell).hexdigest())
        self.assertEqual(manifest["source_manifest_sha256"], hashlib.sha256(self.before_metadata).hexdigest())
        self.assertEqual(manifest["mjcf_sha256"], hashlib.sha256(self.raw).hexdigest())
        self.assertEqual(len(manifest["mesh_inputs_sha256"]), 7)
        for name, digest in manifest["mesh_inputs_sha256"].items():
            self.assertEqual(hashlib.sha256(Path(name).read_bytes()).hexdigest(), digest)
        self.assertEqual((self.source/"cell.xml").read_bytes(), self.before_cell)
        self.assertEqual((self.source/"source.json").read_bytes(), self.before_metadata)

    def test_original_arm_inertia_joints_and_servos_are_retained(self):
        original = ET.fromstring(self.before_cell)
        result = ET.fromstring(self.raw)
        for name in ARM_JOINTS:
            self.assertEqual(original.find(".//joint[@name='"+name+"']").attrib,
                             result.find(".//joint[@name='"+name+"']").attrib)
            self.assertEqual(original.find("actuator/position[@joint='"+name+"']").attrib,
                             result.find("actuator/position[@joint='"+name+"']").attrib)
        for original_body in original.find("worldbody/body[@name='left_world']").iter("body"):
            inertia = original_body.find("inertial")
            if inertia is not None:
                copied = result.find(".//body[@name='"+original_body.attrib["name"]+"']/inertial")
                self.assertEqual(inertia.attrib, copied.attrib)
        self.assertIsNone(result.find(".//body[@name='right_world']"))
        self.assertEqual(self.manifest["added_collision_exclusions"], [])
        self.assertEqual([dict(row.attrib) for row in result.findall("contact/exclude")],
                         self.manifest["inherited_source_collision_exclusions"])

    def test_block_is_free_and_tool_is_a_real_two_finger_mechanism(self):
        self.assertEqual(self.model.nu, 8)
        self.assertEqual(self.model.neq, 0)
        self.assertEqual(self.model.joint("fr5_block_free").type[0], self.mj.mjtJoint.mjJNT_FREE)
        self.assertEqual(int(self.model.body("fr5_grasp_block").mocapid[0]), -1)
        for side in ("left", "right"):
            self.assertEqual(self.model.joint("fr5_"+side+"_finger_slide").type[0], self.mj.mjtJoint.mjJNT_SLIDE)
        self.assertEqual(self.manifest["tool"]["tcp_site"], TCP_SITE)

    def test_local_ik_reaches_a_waypoint_without_moving_the_physical_data(self):
        before = self.data.qpos.copy()
        answer = solve_ik(self.model, self.data.qpos, self.manifest["pickup_tcp_position_m"],
                          self.manifest["tcp_rotation_matrix"])
        self.assertTrue(answer["success"])
        self.assertLessEqual(answer["position_error_m"], .0002)
        self.assertLessEqual(answer["orientation_error_rad"], .002)
        self.np.testing.assert_array_equal(self.data.qpos, before)
        self.assertEqual(self.data.time, 0.)

    def test_unreachable_ik_failure_reports_the_returned_evaluated_state(self):
        goal = self.np.array([10., 10., 10.])
        answer = solve_ik(self.model, self.data.qpos, goal, self.manifest["tcp_rotation_matrix"],
                          maximum_iterations=20)
        self.assertFalse(answer["success"])
        self.assertEqual(answer["iterations"], 20)
        scratch = self.mj.MjData(self.model)
        scratch.qpos[:] = self.data.qpos
        for name, value in zip(ARM_JOINTS, answer["target_rad"]):
            scratch.qpos[self.model.joint(name).qposadr[0]] = value
        self.mj.mj_forward(self.model, scratch)
        observed_error = float(self.np.linalg.norm(goal-scratch.site_xpos[self.model.site(TCP_SITE).id]))
        self.assertAlmostEqual(answer["position_error_m"], observed_error, places=10)

    def test_invalid_ik_inputs_fail_before_iteration(self):
        valid = dict(model=self.model, observed_qpos=self.data.qpos,
            position_m=self.manifest["pickup_tcp_position_m"], rotation=self.manifest["tcp_rotation_matrix"])
        for change in ({"maximum_iterations": True}, {"maximum_iterations": 201},
                       {"maximum_iterations": 1.5}, {"rotation": self.np.zeros((3, 3))},
                       {"rotation": self.np.diag([1., 1., -1.])}, {"observed_qpos": [0.]},
                       {"position_m": [0., float("nan"), 0.]}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                solve_ik(**dict(valid, **change))

    def test_mismatched_source_sha_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/"cell.xml").write_bytes(self.before_cell)
            metadata = json.loads(self.before_metadata)
            metadata["outputs"]["cell.xml"] = "0"*64
            (root/"source.json").write_text(json.dumps(metadata))
            with self.assertRaisesRegex(ValueError, "does not match"):
                build_model(root)

    def test_existing_output_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path/"keep.txt").write_text("preserve")
            with self.assertRaises(FileExistsError):
                write_model(self.source, path)
            self.assertEqual((path/"keep.txt").read_text(), "preserve")
            self.assertEqual(sorted(p.name for p in path.iterdir()), ["keep.txt"])

    def test_representative_arm_grasp_uses_physics_and_preserves_scope(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/"new-prototype"
            receipt = run_prototype(self.source, output)
            result = json.loads((output/"result.json").read_text())
            self.assertEqual(receipt["outcome"], "succeeded")
            self.assertIsNone(result["error"])
            self.assertGreaterEqual(result["lift_evidence"]["lift_m"], .06)
            self.assertLessEqual(result["lift_evidence"]["relative_position_error_m"], .005)
            self.assertEqual(result["physics"]["equality_constraints"], 0)
            self.assertEqual(result["physics"]["object_pose_overwrites_after_initialization"], 0)
            self.assertEqual(result["physics"]["physical_joint_pose_overwrites_after_initialization"], 0)
            self.assertFalse(result["physics"]["extra_applied_forces"])
            self.assertFalse(result["w07_complete"])
            self.assertIn("not ROS/MoveIt", result["scope"])
            self.assertEqual(result["task"]["planning_attachment"], "world")
            self.assertEqual(result["task"]["physical_grasp"], "released")
            self.assertTrue(all(not row["unexpected_contacts"] for row in result["samples"]))
            self.assertTrue(all(path["complete"] for path in result["ik_paths"]))
            self.assertTrue(all(path["physical_data_pose_assignments"] == 0 for path in result["ik_paths"]))
            contract = result["qpos_state_contract"]
            self.assertEqual(contract["nq"], self.model.nq)
            self.assertEqual(len(contract["scalar_joint_names"]), 8)
            self.assertEqual(contract["scalar_joint_units"]["left_j1"], "rad")
            self.assertEqual(contract["scalar_joint_units"]["fr5_left_finger_slide"], "m")
            for frame in (result["samples"][0], result["samples"][len(result["samples"])//2], result["final"]):
                self.assertEqual(len(frame["qpos"]), self.model.nq)
                self.assertTrue(self.np.isfinite(frame["qpos"]).all())
                self.assertEqual(set(frame["scalar_joint_positions"]), set(contract["scalar_joint_names"]))
                replay = self.mj.MjData(self.model)
                replay.qpos[:] = frame["qpos"]
                self.mj.mj_forward(self.model, replay)
                for name in contract["scalar_joint_names"]:
                    self.assertEqual(frame["scalar_joint_positions"][name],
                                     frame["qpos"][self.model.joint(name).qposadr[0]])
                self.np.testing.assert_allclose(replay.site_xpos[self.model.site(TCP_SITE).id],
                                                frame["tcp_position_m"], atol=1e-10, rtol=0)
                self.np.testing.assert_allclose(replay.xpos[self.model.body("fr5_grasp_block").id],
                                                frame["block_position_m"], atol=1e-10, rtol=0)
            closed = next(row for row in result["segments"] if row["phase"] == "close")["final"]
            # A compressed finger's actual position is not its .025m command.
            self.assertGreater(abs(closed["scalar_joint_positions"]["fr5_left_finger_slide"]-.025), .001)


if __name__ == "__main__":
    unittest.main()
