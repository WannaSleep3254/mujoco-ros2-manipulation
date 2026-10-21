"""Random workload reproducibility, unfiltered goals and fail-closed time budgets."""
from copy import deepcopy
from types import SimpleNamespace as NS
import unittest

from mujoco_ros2_manipulation.workcells.random_workload import make_workload, check_workload, workload_hash
from mujoco_ros2_manipulation.workcells.planning_budget import PlanningBudget, PlanningBudgetExceeded


class WorkloadTests(unittest.TestCase):
    def setUp(self):
        self.cell = NS(id="pair", fingerprint="model", joints=["l", "r"], home=[0.9, 0.0])
        self.limits = {n: dict(lower=-1.0, upper=1.0) for n in self.cell.joints}

    def test_seed_case_pairing_and_unclipped_limit_intersection(self):
        value = make_workload(self.cell, self.limits, seed=7, count=4, radius=0.25)
        self.assertEqual(value, make_workload(self.cell, self.limits, seed=7, count=4, radius=0.25))
        self.assertNotEqual(workload_hash(value), workload_hash(make_workload(self.cell, self.limits, seed=8, count=4)))
        for i, case in enumerate(value["cases"]):
            self.assertEqual(case["strategy_order"], ["coordinated", "departure_timing"] if i % 2 == 0 else ["departure_timing", "coordinated"])
            self.assertTrue(0.65 <= case["target_rad"]["l"] < 1.0)
            self.assertTrue(-0.25 <= case["target_rad"]["r"] <= 0.25)
        check_workload(value, self.cell, self.limits)

    def test_changed_targets_limits_model_or_order_cannot_replay_as_same_input(self):
        value = make_workload(self.cell, self.limits)
        for field in ("target", "model", "order", "extra"):
            changed = deepcopy(value)
            if field == "target": changed["cases"][0]["target_rad"]["l"] += 0.001
            if field == "model": changed["workcell_sha256"] = "other"
            if field == "order": changed["cases"][0]["strategy_order"].reverse()
            if field == "extra": changed["selected_successes_only"] = True
            with self.subTest(field=field), self.assertRaises(ValueError):
                check_workload(changed, self.cell, self.limits)
        for keyword, bad in (("seed", True), ("count", 0), ("radius", float("nan")), ("count", 201)):
            with self.assertRaises(ValueError):
                make_workload(self.cell, self.limits, **{keyword: bad})


class BudgetTests(unittest.TestCase):
    def test_boundary_failure_and_execution_time_excluded_after_finish(self):
        now = [0]
        budget = PlanningBudget(1, clock=lambda: now[0])
        now[0] = 400_000_000
        self.assertAlmostEqual(budget.remaining(), 0.6)
        saved = budget.finish()
        now[0] = 9_000_000_000
        self.assertEqual(saved, budget.finish())
        budget = PlanningBudget(0.1, clock=lambda: now[0])
        now[0] += 100_000_000
        with self.assertRaises(PlanningBudgetExceeded): budget.check()
        with self.assertRaises(PlanningBudgetExceeded): budget.remaining()
        now[0] += 300_000_000
        self.assertAlmostEqual(budget.finish()["elapsed_wall_sec"], 0.4)
        self.assertTrue(budget.finish()["exceeded"])

    def test_invalid_budget_is_not_an_unlimited_search(self):
        for value in (0, True, -1, float("nan"), float("inf"), 61):
            with self.subTest(value=value), self.assertRaises(ValueError): PlanningBudget(value)
