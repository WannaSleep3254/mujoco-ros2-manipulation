"""Name isolation, shared controls, configuration and trajectory gating regressions."""

from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET
import yaml
from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.workcells.config import load_workcell, controllers
from mujoco_ros2_manipulation.workcells.prepare import prefixed_urdf, prefixed_mjcf
from mujoco_ros2_manipulation.workcells.trajectory import sampled_states, partitions


class WorkcellConfigTests(unittest.TestCase):
    def write_fixture(self, tmp, data):
        root = Path(tmp)
        for name in (
            "config/build.yaml",
            "config/scenes/default.yaml",
            "config/robots/fr5.yaml",
            "config/robots/ur5e.yaml",
            "config/controller_defaults.yaml",
        ):
            path = root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes((ROOT / name).read_bytes())
        path = root / "config/workcells/fr5_ur5e.yaml"
        path.parent.mkdir(parents=True)
        path.write_text(yaml.safe_dump(data))
        return root

    def test_same_model_instances_and_controller_ownership(self):
        original = load_workcell()
        data = deepcopy(original.data)
        data["robots"][1]["model"] = "fr5"
        with tempfile.TemporaryDirectory() as tmp:
            cell = load_workcell(root=self.write_fixture(tmp, data))
            self.assertEqual(len(set(cell.joints)), 12)
            configs = controllers(cell)
            for inst in cell.instances:
                parameters = configs[inst.controller]["ros__parameters"]
                self.assertEqual(parameters["joints"], inst.joints)
                self.assertTrue(
                    all(name in parameters["constraints"] for name in inst.joints)
                )
            self.assertFalse(
                set(cell.instances[0].joints) & set(cell.instances[1].joints)
            )
            defaults_path = cell.root / "config/controller_defaults.yaml"
            defaults = yaml.safe_load(defaults_path.read_text())
            defaults["update_rate"] = 37
            defaults_path.write_text(yaml.safe_dump(defaults))
            self.assertEqual(
                controllers(cell)["controller_manager"]["ros__parameters"][
                    "update_rate"
                ],
                37,
            )

    def test_malformed_and_ambiguous_instance_configuration_rejected(self):
        original = load_workcell().data
        for case in (
            "duplicate",
            "unknown",
            "nan",
            "bad_size",
            "extra",
            "ambiguous_prefix",
        ):
            data = deepcopy(original)
            if case == "duplicate":
                data["robots"][1]["instance"] = "left"
            elif case == "unknown":
                data["robots"][1]["model"] = "unknown"
            elif case == "nan":
                data["robots"][1]["xyz"][0] = float("nan")
            elif case == "bad_size":
                data["objects"][0]["size"][0] = 0
            elif case == "ambiguous_prefix":
                data["robots"][1]["instance"] = "left_extra"
            else:
                data["robots"][0]["namespace"] = "unexpected"
            with self.subTest(case=case), tempfile.TemporaryDirectory() as tmp:
                root = self.write_fixture(tmp, data)
                with self.assertRaises(ValueError):
                    load_workcell(root=root)

    def test_urdf_prefixes_materials_links_joints_and_removes_hardware(self):
        source = ET.fromstring(
            '<robot name="arm"><link name="world"/><link name="base"><visual><material name="paint"/></visual></link><link name="workcell_floor"/><joint name="workcell_floor_fixed" type="fixed"><parent link="world"/><child link="workcell_floor"/></joint><joint name="mount" type="fixed"><parent link="world"/><child link="base"/></joint><ros2_control name="real_driver"/></robot>'
        )
        cloned = prefixed_urdf(source, "left_")
        self.assertIsNone(cloned.find("ros2_control"))
        self.assertEqual(
            {e.get("name") for e in cloned.findall("link")}, {"left_world", "left_base"}
        )
        self.assertEqual(cloned.find("joint/child").get("link"), "left_base")
        self.assertEqual(cloned.find(".//material").get("name"), "left_paint")
        self.assertIsNotNone(source.find("ros2_control"))

    def test_mjcf_symbol_references_and_mesh_paths_remain_resolved(self):
        source = ET.fromstring(
            '<mujoco><asset><mesh name="base" file="meshes/base.stl"/></asset><worldbody><body name="base"><joint name="j1"/><geom mesh="base"/></body></worldbody><contact><exclude body1="base" body2="tip"/></contact><actuator><position name="j1" joint="j1"/></actuator></mujoco>'
        )
        cloned = prefixed_mjcf(source, "right_", ROOT)
        self.assertEqual(cloned.find(".//geom").get("mesh"), "right_base")
        self.assertEqual(cloned.find(".//position").get("joint"), "right_j1")
        self.assertEqual(cloned.find(".//exclude").get("body2"), "right_tip")
        self.assertEqual(
            cloned.find(".//mesh").get("file"), str(ROOT / "meshes/base.stl")
        )


class SharedTrajectoryTests(unittest.TestCase):
    def test_interpolation_holds_uncommanded_arm_and_bounds_each_step(self):
        states = sampled_states(
            ["left_j1"], [(0, [0]), (1, [0.2])], {"left_j1": 0, "right_j1": 0.5}
        )
        self.assertTrue(all(state["right_j1"] == 0.5 for state in states))
        self.assertEqual(states[-1]["left_j1"], 0.2)
        self.assertLessEqual(
            max(abs(b["left_j1"] - a["left_j1"]) for a, b in zip(states, states[1:])),
            0.025,
        )

    def test_unknown_joints_nonfinite_stale_start_and_invalid_time_rejected(self):
        for names, points in [
            (["unknown"], [(0, [0]), (1, [1])]),
            (["left_j1"], [(0, [float("nan")]), (1, [1])]),
            (["left_j1"], [(0, [0.1]), (1, [1])]),
            (["left_j1"], [(0, [0]), (0, [1])]),
            (["left_j1", "left_j1"], [(0, [0, 0]), (1, [1, 1])]),
        ]:
            with self.subTest(names=names, points=points), self.assertRaises(
                ValueError
            ):
                sampled_states(names, points, {"left_j1": 0})

    def test_reordered_joint_partition_is_complete_and_cannot_overlap(self):
        instances = [
            SimpleNamespace(id="left", joints=["left_j1", "left_j2"]),
            SimpleNamespace(id="right", joints=["right_j1", "right_j2"]),
        ]
        names = ["right_j2", "left_j2", "right_j1", "left_j1"]
        self.assertEqual(
            partitions(names, instances), {"left": [3, 1], "right": [2, 0]}
        )
        with self.assertRaises(ValueError):
            partitions(names[:-1], instances)
        instances[1].joints = ["left_j1", "right_j2"]
        with self.assertRaises(ValueError):
            partitions(names, instances)
