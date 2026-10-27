"""Run the production coordinated proposal/selector boundary without ROS.

The invalid trajectories below are deterministic contract fixtures. The CI
failure did not retain its rejected spline, so they do not reconstruct its data.
"""
import ast
from contextlib import nullcontext
import json
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from mujoco_ros2_manipulation.workcells.interference import Spline
from mujoco_ros2_manipulation.workcells.motion_diagnostics import update_candidate_path_diagnostic
from mujoco_ros2_manipulation.workcells.plan_selection import (
    PlanCandidateRejected, PlanCandidatesExhausted, select_clearance_plan,
)


def require(value, message):
    if not value:
        raise RuntimeError(message)


def trajectory(duration=1., first=0.):
    def point(stamp, position):
        return NS(time_from_start=NS(sec=stamp, nanosec=0),
                  positions=[position, position], velocities=[0., 0.], accelerations=[0., 0.])
    return NS(joint_names=["left", "right"], points=[point(first, 0.), point(duration, .1)])


def coordinated_method():
    source = Path(__file__).resolve().parents[2] / "src/mujoco_ros2_manipulation/workcells/motion_server.py"
    tree = ast.parse(source.read_text(), filename=str(source))
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "MotionCommandServer")
    method = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == "coordinated_path")
    namespace = dict(Spline=Spline, select_clearance_plan=select_clearance_plan, require=require,
        PlanCandidateRejected=PlanCandidateRejected, MoveItErrorCodes=NS(SUCCESS=1),
        MotionPlanningFailed=type("MotionPlanningFailed", (RuntimeError,), {}),
        CommandCancelled=type("CommandCancelled", (RuntimeError,), {}),
        PlanningBudgetExceeded=type("PlanningBudgetExceeded", (RuntimeError,), {}),
        update_candidate_path_diagnostic=update_candidate_path_diagnostic)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])), str(source), "exec"), namespace)
    return namespace["coordinated_path"]


class SpatialProposalContractTests(unittest.TestCase):
    method = staticmethod(coordinated_method())

    def setUp(self):
        self.saved = []
        self.trace = NS(data=dict(clearance_endpoints=dict(endpoint_violations=[])),
                        span=lambda name: nullcontext())
        self.initial = dict(left=0., right=0.)
        self.target = dict(left=.1, right=.1)
        self.evaluator = NS(at=Mock(return_value=dict(distance_m=1., bodies=["left", "right"],
            nearest_points_m=[[0., 0., 0.], [1., 0., 0.]], tips_m=[[0., 0., 0.], [1., 0., 0.]])))
        self.node = NS(goal_tolerance_rad=.003, max_plan_candidates=2,
            policy=dict(min_inter_arm_distance_m=.02, tracking_reserve_m=.01, sample_dt_sec=.02),
            cell=NS(joints=["left", "right"], instances=[NS(id="left", joints=["left"]), NS(id="right", joints=["right"])]),
            evaluator=self.evaluator, event=Mock(), compute_checkpoint=Mock(),
            assess_spatial_candidate=Mock(return_value=dict(invalid_samples=0)),
            persist=self.persist)

    def persist(self, trace):
        # Exercise the same finite-JSON constraint as persisted request evidence.
        self.saved.append(json.loads(json.dumps(trace.data, allow_nan=False)))

    def run_path(self):
        return self.method(self.node, self.initial, self.target, self.trace)

    def test_invalid_time_candidates_are_recorded_then_safe_candidate_is_checked(self):
        cases = [
            (trajectory(36.), "end_exceeds_35s"),
            (trajectory(first=.1), "nonzero_start"),
            (trajectory(0.), "nonincreasing_time"),
            (trajectory(float("inf")), "nonfinite_time"),
        ]
        for invalid, reason in cases:
            with self.subTest(reason=reason):
                self.setUp()
                self.node.plan = Mock(side_effect=[invalid, trajectory()])
                curve = self.run_path()
                self.assertEqual(curve.duration, 1.)
                self.assertEqual(self.node.plan.call_count, 2)
                first, second = self.trace.data["plan_selection"]["candidates"]
                self.assertEqual(first["status"], "rejected")
                self.assertEqual(first["stage"], "trajectory")
                self.assertEqual(first["rejection_kind"], "invalid_trajectory")
                self.assertIn(reason, first["trajectory_diagnostic"]["timing_issues"])
                self.assertNotIn("curve", first)
                self.assertNotIn("native", first)
                self.assertTrue(second["accepted"])
                self.node.assess_spatial_candidate.assert_called_once()
                self.assertEqual(self.saved[-1]["plan_selection"]["selected_attempt"], 2)

    def test_all_invalid_candidates_exhaust_the_original_bound_without_native_checks(self):
        self.node.plan = Mock(side_effect=[trajectory(36.), trajectory(first=.1)])
        with self.assertRaisesRegex(PlanCandidatesExhausted, "no candidate dispatched"):
            self.run_path()
        self.assertEqual(self.node.plan.call_count, 2)
        self.evaluator.at.assert_not_called()
        self.node.assess_spatial_candidate.assert_not_called()
        selection = self.saved[-1]["plan_selection"]
        self.assertEqual(selection["status"], "exhausted")
        self.assertIsNone(selection["selected_attempt"])
        self.assertTrue(all(candidate["rejection_kind"] == "invalid_trajectory" for candidate in selection["candidates"]))

    def test_wrong_joint_contract_is_not_retried(self):
        wrong = trajectory()
        wrong.joint_names = ["left", "unexpected"]
        self.node.plan = Mock(side_effect=[wrong, trajectory()])
        with self.assertRaisesRegex(RuntimeError, "cover the whole workcell"):
            self.run_path()
        self.assert_failed_contract()

    def test_wrong_endpoint_contract_is_not_retried(self):
        wrong = trajectory()
        wrong.points[-1].positions[0] = .2
        self.node.plan = Mock(side_effect=[wrong, trajectory()])
        with self.assertRaisesRegex(RuntimeError, "declared goal tolerance"):
            self.run_path()
        self.assert_failed_contract()

    def assert_failed_contract(self):
        self.node.plan.assert_called_once()
        self.evaluator.at.assert_not_called()
        self.node.assess_spatial_candidate.assert_not_called()
        candidate = self.saved[-1]["plan_selection"]["candidates"][0]
        self.assertEqual(candidate["status"], "failed")
        self.assertEqual(candidate["stage"], "trajectory")
        self.assertNotIn("rejection_kind", candidate)

    def test_proposal_failure_is_preserved_without_retry(self):
        error = ValueError("Planner adapter state mismatch")
        self.node.plan = Mock(side_effect=error)
        with self.assertRaises(ValueError) as caught:
            self.run_path()
        self.assertIs(caught.exception, error)
        self.node.plan.assert_called_once()
        self.evaluator.at.assert_not_called()
        self.assertEqual(self.saved[-1]["plan_selection"]["candidates"][0]["stage"], "proposal")

    def test_evaluator_failure_is_preserved_without_retry(self):
        error = ValueError("Native evaluator state mismatch")
        self.node.plan = Mock(return_value=trajectory())
        self.evaluator.at.side_effect = error
        with self.assertRaises(ValueError) as caught:
            self.run_path()
        self.assertIs(caught.exception, error)
        self.node.plan.assert_called_once()
        self.assertEqual(self.saved[-1]["plan_selection"]["candidates"][0]["stage"], "native")

    def test_checkpoint_failure_is_preserved_before_proposal(self):
        error = RuntimeError("Request budget expired")
        self.node.plan = Mock(return_value=trajectory())
        self.node.compute_checkpoint.side_effect = error
        with self.assertRaises(RuntimeError) as caught:
            self.run_path()
        self.assertIs(caught.exception, error)
        self.node.plan.assert_not_called()
        self.evaluator.at.assert_not_called()
        self.assertEqual(self.saved[-1]["plan_selection"]["status"], "interrupted")
