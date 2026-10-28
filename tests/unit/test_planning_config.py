"""Opt-in planner/cache configuration, without ROS or generated model writes."""
from copy import deepcopy
from types import SimpleNamespace
import unittest

from mujoco_ros2_manipulation.workcells.planning_config import (
    planning_configuration, trajectory_cache_identity, PLANNER_TYPES)
from mujoco_ros2_manipulation.workcells.prepare import moveit_parameters


class PlanningConfigurationTests(unittest.TestCase):
    def test_default_and_explicit_configuration_are_strict(self):
        default = planning_configuration()
        self.assertEqual(default["planner_id"], "RRTConnectkConfigDefault")
        self.assertFalse(default["trajectory_cache"])
        self.assertEqual(default["cache_start_tolerance_rad"], 0.)
        for planner in PLANNER_TYPES:
            explicit = planning_configuration(planner, True, .0001)
            self.assertEqual(explicit["planner_type"], PLANNER_TYPES[planner])
        for args in (("RRTstar", False, 0), (1, False, 0),
                     ("RRTkConfigDefault", 1, 0), ("RRTkConfigDefault", True, True),
                     ("RRTkConfigDefault", True, float("nan")),
                     ("RRTkConfigDefault", True, -.001), ("RRTkConfigDefault", True, .00101),
                     ("RRTkConfigDefault", False, .0001)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                planning_configuration(*args)

    def test_both_planners_are_registered_without_changing_connect_default_order(self):
        cell = SimpleNamespace(instances=[SimpleNamespace(group="arm", controller="arm_controller", joints=["j1"])])
        parameters = moveit_parameters(cell, "urdf", "srdf", {"j1": dict(velocity=1)})
        ompl = parameters["ompl"]
        self.assertEqual({key: row["type"] for key, row in ompl["planner_configs"].items()}, PLANNER_TYPES)
        for group in ("arm", "both_arms"):
            self.assertEqual(ompl[group]["planner_configs"], ["RRTConnectkConfigDefault", "RRTkConfigDefault"])

    def test_every_runtime_scene_field_and_policy_configuration_bind_cache_identity(self):
        runtime = dict(build=dict(outputs_sha256={"cell.xml": "a"*64}),
                       planning_scene_sha256="b"*64, simulator_description_sha256="c"*64)
        kwargs = dict(source_sha256="d"*64, policy=dict(clearance=.03, start_tolerance_rad=0.),
                      planner=planning_configuration())
        original = trajectory_cache_identity(runtime, **kwargs)
        self.assertEqual(len(original), 6)
        self.assertIsNone(original["tool_sha256"])
        for field in ("build", "planning_scene_sha256", "simulator_description_sha256"):
            changed = deepcopy(runtime)
            changed[field] = dict(outputs_sha256={"cell.xml": "e"*64}) if field == "build" else "f"*64
            self.assertNotEqual(trajectory_cache_identity(changed, **kwargs), original)
        for field, value in (("source_sha256", "e"*64), ("policy", dict(clearance=.03,start_tolerance_rad=.0001)),
                             ("planner", planning_configuration("RRTkConfigDefault"))):
            self.assertNotEqual(trajectory_cache_identity(runtime, **dict(kwargs, **{field:value})), original)
        with self.assertRaises(ValueError): trajectory_cache_identity({}, **kwargs)


if __name__ == "__main__":
    unittest.main()
