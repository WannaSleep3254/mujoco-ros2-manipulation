"""Portable descriptions preserve workcell identity without ROS or engine imports."""
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.contracts import describe_workcell


def fixture():
    cell = load_workcell()
    manifest = dict(
        schema_version=1, workcell_sha256=cell.fingerprint,
        joints=cell.joints, home_rad=cell.home,
        joint_limits={name: dict(lower=-10.0, upper=10.0, velocity=1.0, effort=2.0)
                      for name in cell.joints},
        inputs={"config/workcells/fr5_ur5e.yaml": "1" * 64},
        outputs={name: str(index) * 64 for index, name in enumerate(
            ("cell.xml", "cell.urdf", "cell.srdf", "controllers.yaml", "moveit.yaml", "moveit.rviz"), 1)},
    )
    return cell, manifest


class WorkcellContractTests(unittest.TestCase):
    def test_identity_units_frames_and_named_ownership_are_portable(self):
        cell, manifest = fixture()
        result = describe_workcell(cell, manifest)
        self.assertEqual(json.loads(json.dumps(result, allow_nan=False)), result)
        self.assertEqual(result["schema_version"], 1)
        self.assertEqual(result["kind"], "workcell_description")
        self.assertEqual(result["identity"]["workcell_sha256"], cell.fingerprint)
        self.assertEqual(result["identity"]["inputs_sha256"], manifest["inputs"])
        self.assertEqual(result["identity"]["outputs_sha256"], manifest["outputs"])
        self.assertEqual(result["units"]["length"], "m")
        self.assertEqual(result["units"]["joint_position"], "rad")
        self.assertEqual(result["units"]["joint_velocity"], "rad/s")
        self.assertEqual(result["world_frame"], "world")
        self.assertEqual(result["home_rad"], dict(zip(cell.joints, cell.home)))
        self.assertEqual(result["joint_limits"], manifest["joint_limits"])
        for instance, robot in zip(cell.instances, result["robots"]):
            self.assertEqual(robot["joint_map"], dict(zip(instance.profile.joints, instance.joints)))
            self.assertEqual(robot["installation_pose"]["frame"], "world")
            self.assertEqual(robot["installation_pose"]["xyz_m"], instance.data["xyz"])
            self.assertEqual(robot["installation_pose"]["rpy_rad"], instance.data["rpy"])
            self.assertEqual(robot["base_frame"], instance.prefix + instance.profile.ros["base_frame"])
            self.assertEqual(robot["tip_frame"], instance.prefix + instance.profile.ros["tip_frame"])
            self.assertEqual(result["identity"]["profile_sha256"][instance.id], instance.profile.fingerprint)
        for asset in result["assets"]["outputs"]:
            path = Path(asset["path"])
            self.assertFalse(path.is_absolute())
            self.assertEqual(asset["sha256"], manifest["outputs"][path.name])
        self.assertNotIn(str(ROOT), json.dumps(result))
        self.assertFalse(result["scope"]["runtime_plugins"])

    def test_description_does_not_mutate_or_alias_inputs(self):
        cell, manifest = fixture()
        before_cell, before_manifest = deepcopy(cell.data), deepcopy(manifest)
        result = describe_workcell(cell, manifest)
        result["environment"]["objects"][0]["xyz"][0] += 100
        result["joint_limits"][cell.joints[0]]["lower"] = -999
        result["robots"][0]["installation_pose"]["xyz_m"][0] += 100
        result["identity"]["inputs_sha256"].clear()
        self.assertEqual(cell.data, before_cell)
        self.assertEqual(manifest, before_manifest)

    def test_mismatched_manifest_invalid_schema_hashes_and_joint_limits_rejected(self):
        cell, original = fixture()
        mutations = (
            lambda m: m.update(schema_version=True),
            lambda m: m.update(workcell_sha256="0" * 64),
            lambda m: m["joints"].reverse(),
            lambda m: m["home_rad"].__setitem__(0, 0.5),
            lambda m: m["joint_limits"].pop(cell.joints[0]),
            lambda m: m["joint_limits"][cell.joints[0]].update(velocity=float("nan")),
            lambda m: m["joint_limits"][cell.joints[0]].update(lower=1.0),
            lambda m: m["inputs"].update({"../outside": "1" * 64}),
            lambda m: m["inputs"].update({"/absolute/file": "1" * 64}),
            lambda m: m["outputs"].update({"cell.xml": "not-a-sha256"}),
            lambda m: m["outputs"].pop("cell.urdf"),
        )
        for mutate in mutations:
            manifest = deepcopy(original)
            mutate(manifest)
            with self.subTest(manifest=manifest), self.assertRaises(ValueError):
                describe_workcell(cell, manifest)

    def test_import_and_description_work_when_engine_transport_and_files_are_unavailable(self):
        # A clean interpreter blocks accidental imports even on a ROS-equipped host.
        code = """
import importlib.abc
import json
from types import SimpleNamespace
import sys

class BlockBackend(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split(".")[0] in {"rclpy", "mujoco", "numpy", "yaml", "sensor_msgs", "moveit_msgs"}:
            raise RuntimeError("Unexpected backend import: " + fullname)

sys.meta_path.insert(0, BlockBackend())
from mujoco_ros2_manipulation.workcells.contracts import describe_workcell

profile = SimpleNamespace(id="robot", fingerprint="a"*64, joints=["j"],
                          ros=dict(base_frame="base", tip_frame="tip"))
instances = [SimpleNamespace(id=side, prefix=side+"_", profile=profile,
    joints=[side+"_j"], group=side+"_arm", controller=side+"_controller",
    data=dict(xyz=[0, 0, 0], rpy=[0, 0, 0])) for side in ["left", "right"]]
cell = SimpleNamespace(id="test", label="Test", fingerprint="b"*64,
    joints=["left_j", "right_j"], home=[0, 0], instances=instances, data=dict(objects=[]))
manifest = dict(schema_version=1, workcell_sha256=cell.fingerprint,
    joints=cell.joints, home_rad=cell.home,
    joint_limits={n:dict(lower=-1, upper=1, effort=1, velocity=1) for n in cell.joints},
    inputs={"config/test.yaml": "c"*64},
    outputs={name:"d"*64 for name in ["cell.xml", "cell.urdf", "cell.srdf",
                                     "controllers.yaml", "moveit.yaml", "moveit.rviz"]})
result = describe_workcell(cell, manifest)
assert result["home_rad"] == {"left_j": 0, "right_j": 0}
print(json.dumps(result, allow_nan=False))
"""
        result = subprocess.run([sys.executable, "-c", code], cwd="/tmp",
                                env=dict(os.environ, PYTHONPATH=str(ROOT / "src")),
                                text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["workcell_id"], "test")
