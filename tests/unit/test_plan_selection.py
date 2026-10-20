"""Reject grazing RRT candidates before dispatch without hiding execution failures."""
from types import SimpleNamespace as NS
import unittest

from mujoco_ros2_manipulation.workcells.plan_selection import (
    MAX_PLAN_CANDIDATES,
    select_clearance_plan,
)


def trajectory(positions):
    return NS(
        joint_names=["l", "r"],
        points=[
            NS(
                positions=p,
                velocities=[],
                accelerations=[],
                time_from_start=NS(sec=t, nanosec=0),
            )
            for t, p in enumerate(positions)
        ],
    )


class SelectionTests(unittest.TestCase):
    def setUp(self):
        self.cell = NS(
            joints=["l", "r"],
            instances=[NS(id="left", joints=["l"]), NS(id="right", joints=["r"])],
        )
        self.initial = dict(l=0.0, r=0.0)
        self.policy = dict(
            min_inter_arm_distance_m=0.02, tracking_reserve_m=0.01, sample_dt_sec=0.01
        )
        self.evaluator = NS(
            at=lambda s: dict(
                distance_m=((s["l"] - 0.5) ** 2 + (s["r"] - 0.5) ** 2) ** 0.5 - 0.2,
                tips_m=[[s["l"], 0, 0], [0, s["r"], 0]],
                bodies=["left", "right"],
            )
        )
        self.bad = trajectory([[0.0, 0.0], [1.0, 1.0]])
        self.safe = trajectory([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])

    def test_grazing_candidate_saved_then_safe_spline_selected(self):
        proposals = iter([(1, self.bad), (1, self.safe)])
        saved, checked = [], []

        def save(candidates):
            saved.append(list(candidates))

        def moveit(curve):
            checked.append(curve)
            return dict(invalid_samples=0)

        result = select_clearance_plan(
            lambda: next(proposals),
            self.initial,
            self.cell,
            self.evaluator,
            self.policy,
            moveit,
            save,
        )
        self.assertIs(result, self.safe)
        self.assertEqual(len(checked), 1)  # Native rejection precedes MoveIt/dispatch.
        self.assertEqual([len(s) for s in saved], [1, 2])
        self.assertFalse(saved[-1][0]["accepted"])
        self.assertLess(saved[-1][0]["native"]["minimum"]["distance_m"], 0.03)
        self.assertTrue(saved[-1][1]["accepted"])
        self.assertEqual(saved[-1][0]["curve"]["points"][1]["p"], [1.0, 1.0])

    def test_moveit_failure_or_collision_not_accepted_even_with_native_gap(self):
        proposals = iter([(-1, None), (1, self.safe), (1, self.safe)])
        checks = iter([dict(invalid_samples=1), dict(invalid_samples=0)])
        saved = []
        result = select_clearance_plan(
            lambda: next(proposals),
            self.initial,
            self.cell,
            self.evaluator,
            self.policy,
            lambda c: next(checks),
            lambda c: saved.append(list(c)),
        )
        self.assertIs(result, self.safe)
        self.assertEqual([r["accepted"] for r in saved[-1]], [False, False, True])
        self.assertEqual(saved[-1][0]["moveit_error_code"], -1)

    def test_no_valid_candidate_fails_at_bound_and_preserves_all_rejections(self):
        saved = []

        def must_not_check(curve):
            self.fail("Rejected native candidate reached MoveIt gate")

        with self.assertRaisesRegex(RuntimeError, "no candidate dispatched"):
            select_clearance_plan(
                lambda: (1, self.bad),
                self.initial,
                self.cell,
                self.evaluator,
                self.policy,
                must_not_check,
                lambda c: saved.append(list(c)),
            )
        self.assertEqual(len(saved), MAX_PLAN_CANDIDATES)
        self.assertEqual(len(saved[-1]), MAX_PLAN_CANDIDATES)
        self.assertTrue(all(not r["accepted"] for r in saved[-1]))
