"""Spline overshoot, physical retiming, shared ownership and fixed case validation."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS
import tempfile
import unittest
import yaml

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.interference import (
    Spline,
    stretch,
    sampled_parallel,
    load_case,
)


def curve(name="left", p0=0.0, p1=0.2, duration=1.0, v=None, a=None):
    v = ([], []) if v is None else ([v[0]], [v[1]])
    a = ([], []) if a is None else ([a[0]], [a[1]])
    return Spline(
        [name],
        [dict(t=0.0, p=[p0], v=v[0], a=a[0]), dict(t=duration, p=[p1], v=v[1], a=a[1])],
    )


class SplineTests(unittest.TestCase):
    def test_linear_cubic_and_quintic_match_known_polynomials(self):
        self.assertAlmostEqual(curve(p1=1).position(0.25)[0], 0.25)
        self.assertAlmostEqual(curve(p1=1, v=(0, 3)).position(0.25)[0], 0.25 ** 3)
        self.assertAlmostEqual(
            curve(p1=1, v=(0, 5), a=(0, 20)).position(0.25)[0], 0.25 ** 5
        )
        self.assertEqual(curve().position(0), [0])
        self.assertEqual(curve().position(1), [0.2])

    def test_derivatives_can_create_collision_relevant_overshoot_between_equal_positions(
        self,
    ):
        self.assertEqual(curve(p1=0).position(0.5), [0])
        self.assertAlmostEqual(curve(p1=0, v=(4, -4)).position(0.5)[0], 1)
        self.assertAlmostEqual(curve(p1=0, v=(4, -4), a=(0, 0)).position(0.5)[0], 1.25)

    def test_stretch_preserves_shape_and_scales_derivatives_without_mutating_source(
        self,
    ):
        trajectory = NS(
            joint_names=["left"],
            points=[
                NS(
                    positions=[0.0],
                    velocities=[0.0],
                    accelerations=[0.0],
                    time_from_start=NS(sec=0, nanosec=0),
                ),
                NS(
                    positions=[1.0],
                    velocities=[5.0],
                    accelerations=[20.0],
                    time_from_start=NS(sec=1, nanosec=0),
                ),
            ],
        )
        result = stretch(trajectory, 2)
        self.assertEqual(result.points[-1].velocities, [2.5])
        self.assertEqual(result.points[-1].accelerations, [5])
        self.assertEqual(trajectory.points[-1].velocities, [5])
        before, after = Spline.from_ros(trajectory), Spline.from_ros(result)
        for t in [0, 0.1, 0.25, 0.5, 0.7, 1]:
            self.assertAlmostEqual(before.position(t)[0], after.position(t * 2)[0])
        for duration in [0.5, float("nan"), 36]:
            with self.assertRaises(ValueError):
                stretch(trajectory, duration)

    def test_malformed_or_ambiguous_splines_fail_closed(self):
        original = curve().points
        variants = []
        for field, value in [
            ("t", -1),
            ("t", 0),
            ("t", 36),
            ("p", [float("nan")]),
            ("p", []),
            ("v", [float("inf")]),
            ("a", [0.0]),
        ]:
            points = deepcopy(original)
            points[-1][field] = value
            variants.append(points)
        for points in variants:
            with self.subTest(points=points), self.assertRaises(ValueError):
                Spline(["left"], points)
        with self.assertRaises(ValueError):
            Spline(["left", "left"], original)
        with self.assertRaises(ValueError):
            curve().position(-0.1)
        with self.assertRaises(ValueError):
            curve().position(float("nan"))

    def test_shared_time_axis_and_ownership_hold_uncommanded_joints(self):
        initial = dict(left=0.0, right=0.5)
        samples = sampled_parallel([curve()], initial, complete=False)
        self.assertTrue(all(state["right"] == 0.5 for _, state in samples))
        self.assertLessEqual(
            max(b[0] - a[0] for a, b in zip(samples, samples[1:])), 0.02000001
        )
        right = curve("right", p0=0.5, p1=0.7)
        combined = sampled_parallel([curve(), right], initial)
        self.assertEqual(combined[-1][1], dict(left=0.2, right=0.7))
        for paths, state, kw in [
            ([curve()], initial, {}),
            ([curve(), curve()], initial, {}),
            ([curve("unknown")], initial, {}),
            ([curve(), curve("right", duration=2)], initial, {}),
            ([curve()], dict(left=1), {}),
            ([curve()], dict(left=0), dict(dt=0)),
            ([curve()], dict(left=0), dict(dt=0.000001)),
        ]:
            with self.subTest(paths=paths, kw=kw), self.assertRaises(ValueError):
                sampled_parallel(paths, state, **kw)


class CrossingCaseTests(unittest.TestCase):
    def test_case_defines_both_instances_and_rejects_bad_inputs(self):
        cell = load_workcell()
        path, data = load_case(cell)
        self.assertEqual(set(data["start_rad"]), set(cell.joints))
        original = yaml.safe_load(path.read_text())
        for kind in ["dt", "planning", "workcell", "missing", "extra", "nan", "shape"]:
            value = deepcopy(original)
            if kind == "dt":
                value["sample_dt_sec"] = 0.1
            elif kind == "planning":
                value["planning"]["path_tolerance"] = 0.1
            elif kind == "workcell":
                value["workcell"] = "other"
            elif kind == "missing":
                del value["start_rad"]["right"]
            elif kind == "extra":
                value["goal_rad"]["other"] = [0] * 6
            elif kind == "nan":
                value["goal_rad"]["left"][0] = float("nan")
            else:
                value["goal_rad"]["left"] = [0]
            with tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                folder = root / "config/scenarios"
                folder.mkdir(parents=True)
                (folder / "dual_arm_crossing.yaml").write_text(yaml.safe_dump(value))
                with self.subTest(kind=kind), self.assertRaises(ValueError):
                    load_case(NS(root=root, id=cell.id, instances=cell.instances))
        with self.assertRaises(ValueError):
            load_case(cell, "../escape")
