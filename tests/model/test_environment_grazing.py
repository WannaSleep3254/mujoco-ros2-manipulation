"""Preserved public CI curve: inter-arm clearance alone allowed a grazing post."""
import gzip
import hashlib
import json
import unittest

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.distance import DistanceEvaluator
from mujoco_ros2_manipulation.workcells.environment_clearance import load_environment_policy, required_environment_distance
from mujoco_ros2_manipulation.workcells.interference import Spline
from mujoco_ros2_manipulation.workcells.path_quality import evaluate_path


class PreservedEnvironmentPathTests(unittest.TestCase):
    def test_original_clear_nominal_curve_is_rejected_by_separate_environment_reserve(self):
        cell = load_workcell("fr5_ur5e_eval_post")
        if not (ROOT / "runtime/native/path_distance.so").is_file() or not (cell.output / "cell.xml").is_file():
            self.skipTest("Pinned native runtime and fixed-post model are required")
        raw = (ROOT / "tests/fixtures/environment-post-grazing.json.gz").read_bytes()
        self.assertEqual(hashlib.sha256(raw).hexdigest(),
                         "9a6df18143960d1874e013a3e0ffde70f06c58e59ca32cd06e2f6bcd0301baeb")
        fixture = json.loads(gzip.decompress(raw))
        curve = Spline(**fixture["trajectory"])
        self.assertEqual(len(curve.points), 817)
        self.assertEqual(curve.duration, 16.316851699)
        _, policy = load_environment_policy(cell)
        with DistanceEvaluator(cell, environment=True) as evaluator:
            result = evaluate_path([curve], fixture["initial_rad"], cell, evaluator)
            rejected = evaluate_path([curve], fixture["initial_rad"], cell, evaluator,
                reject_below=.03, environment_reject_below=required_environment_distance(policy))
        self.assertTrue(result["complete"])
        self.assertGreater(result["minimum"]["distance_m"], .03)
        self.assertGreater(result["environment_minimum"]["distance_m"], 0.)
        self.assertLess(result["environment_minimum"]["distance_m"], .003)
        self.assertEqual(result["environment_minimum"]["bodies"],
                         ["left_wrist3_link", "workcell_evaluation_post"])
        self.assertFalse(rejected["complete"])
        self.assertEqual(rejected["rejection_kind"], "environment_clearance")
        self.assertLess(rejected["samples"], result["samples"])
        self.assertLess(rejected["environment_minimum"]["distance_m"], .015)
        self.assertGreater(rejected["minimum"]["distance_m"], .03)
