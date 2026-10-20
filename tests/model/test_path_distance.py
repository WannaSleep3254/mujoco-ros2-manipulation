"""Pinned native engine distances against analytic geometry, independent of robot meshes."""
from pathlib import Path
from types import SimpleNamespace as NS
import tempfile
import unittest

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.workcells.distance import DistanceEvaluator


class NativeDistanceTests(unittest.TestCase):
    def test_signed_distance_nearest_points_frame_fk_and_visual_geometry_filter(self):
        if not (ROOT / "runtime/native/path_distance.so").is_file():
            self.skipTest("Native distance evaluator not built")
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path / "cell.xml").write_text(
                """<mujoco><worldbody>
              <body name="left_base"><joint name="l" type="slide" axis="1 0 0"/>
                <geom type="sphere" size=".1"/><body name="left_tip" pos=".1 0 0"/>
                <geom type="sphere" size="2" contype="0" conaffinity="0" mass="0"/></body>
              <body name="right_base" pos="1 0 0"><joint name="r" type="slide" axis="1 0 0"/>
                <geom type="sphere" size=".1"/><body name="right_tip" pos="-.1 0 0"/></body>
              </worldbody></mujoco>"""
            )
            instances = [
                NS(prefix=p, profile=NS(ros=dict(base_frame="base", tip_frame="tip")))
                for p in ("left_", "right_")
            ]
            cell = NS(root=ROOT, output=path, joints=["l", "r"], instances=instances)
            with DistanceEvaluator(cell) as evaluator:
                initial = evaluator.at(dict(l=0.0, r=0.0))
                self.assertAlmostEqual(initial["distance_m"], 0.8)
                self.assertAlmostEqual(initial["nearest_points_m"][0][0], 0.1)
                self.assertAlmostEqual(initial["nearest_points_m"][1][0], 0.9)
                self.assertEqual(initial["bodies"], ["left_base", "right_base"])
                self.assertAlmostEqual(
                    evaluator.at(dict(l=0.25, r=0.0))["tips_m"][0][0], 0.35
                )
                self.assertAlmostEqual(
                    evaluator.at(dict(l=0.9, r=0.0))["distance_m"], -0.1
                )
                with self.assertRaises(ValueError):
                    evaluator.at(dict(l=float("nan"), r=0.0))
                with self.assertRaises(ValueError):
                    evaluator.at(dict(l=0.0))
                self.assertEqual(evaluator.version, "3.12.0")
            with self.assertRaises(ValueError):
                evaluator.at(dict(l=0.0, r=0.0))

    def test_saved_ci_grazing_spline_is_rejected_before_execution(self):
        import gzip
        import json
        from mujoco_ros2_manipulation.workcells.config import load_workcell
        from mujoco_ros2_manipulation.workcells.interference import Spline
        from mujoco_ros2_manipulation.workcells.path_quality import evaluate_path
        from mujoco_ros2_manipulation.workcells.timing import load_policy

        cell = load_workcell()
        if (
            not (ROOT / "runtime/native/path_distance.so").is_file()
            or not (cell.output / "cell.xml").is_file()
        ):
            self.skipTest("Native runtime and shared robot model not built")
        evidence = json.loads(
            gzip.decompress(
                (
                    ROOT / "docs/timing_coordination/ci-grazing-trajectories.json.gz"
                ).read_bytes()
            )
        )
        curve = Spline(**evidence["coordinated"])
        _, policy = load_policy(cell)
        threshold = policy["min_inter_arm_distance_m"] + policy["tracking_reserve_m"]
        with DistanceEvaluator(cell) as evaluator:
            full = evaluate_path([curve], evidence["initial_rad"], cell, evaluator)
            gate = evaluate_path(
                [curve],
                evidence["initial_rad"],
                cell,
                evaluator,
                reject_below=threshold,
            )
        self.assertGreater(full["minimum"]["distance_m"], 0.0)
        self.assertLess(full["minimum"]["distance_m"], 0.001)
        self.assertEqual(
            full["minimum"]["bodies"], ["left_wrist2_link", "right_wrist_3_link"]
        )
        self.assertFalse(gate["complete"])
        self.assertLess(gate["minimum"]["distance_m"], threshold)
        self.assertLess(gate["samples"], full["samples"])
