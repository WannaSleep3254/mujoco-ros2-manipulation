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
