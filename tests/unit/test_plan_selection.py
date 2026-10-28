"""Reject grazing RRT candidates before dispatch without hiding execution failures."""
from copy import deepcopy
import json
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from mujoco_ros2_manipulation.workcells.plan_selection import (
    MAX_PLAN_CANDIDATES,
    PlanCandidateRejected,
    PlanCandidatesExhausted,
    invalid_trajectory_diagnostic,
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


    def select(self, propose, *, moveit=None, save=None, **options):
        return select_clearance_plan(
            propose, self.initial, self.cell, self.evaluator, self.policy,
            moveit if moveit is not None else lambda curve: dict(invalid_samples=0),
            save if save is not None else lambda candidates: None,
            **options,
        )

    def test_one_allowed_candidate_can_succeed_and_is_saved_before_return(self):
        diagnostics, persisted = {}, []
        propose = Mock(return_value=(1, self.safe))
        def save(candidates):
            persisted.append(deepcopy(diagnostics))
        with patch("mujoco_ros2_manipulation.workcells.plan_selection.time.monotonic", side_effect=[2.0, 2.5, 3.0, 3.5]):
            result = self.select(propose, save=save, diagnostics=diagnostics, max_candidates=1)
        self.assertIs(result, self.safe)
        propose.assert_called_once_with()
        self.assertEqual(persisted[-1]["status"], "selected")
        self.assertEqual(diagnostics["selected_attempt"], 1)
        self.assertEqual(diagnostics["attempted"], 1)
        self.assertEqual(diagnostics["max_candidates"], 1)
        self.assertEqual(diagnostics["candidates"][0]["stage"], "selection")
        self.assertEqual(diagnostics["candidates"][0]["elapsed_wall_sec"], 0.5)
        self.assertEqual(diagnostics["elapsed_wall_sec"], 1.5)

    def test_single_candidate_exhaustion_has_typed_failure_and_saved_status(self):
        diagnostics, persisted = {}, []
        propose = Mock(return_value=(1, self.bad))
        with self.assertRaisesRegex(PlanCandidatesExhausted, "within 1 candidates; no candidate dispatched"):
            self.select(propose, max_candidates=1, diagnostics=diagnostics,
                        save=lambda candidates: persisted.append(deepcopy(diagnostics)))
        propose.assert_called_once_with()
        self.assertEqual(persisted[-1]["status"], "exhausted")
        self.assertEqual(diagnostics["attempted"], 1)
        self.assertIsNone(diagnostics["selected_attempt"])
        rejected = diagnostics["candidates"][0]
        self.assertEqual(rejected["rejection_kind"], "native_clearance")
        self.assertEqual(rejected["status"], "rejected")
        self.assertFalse(rejected["native"]["complete"])

    def test_rejection_kinds_distinguish_planner_and_moveit_then_accept(self):
        diagnostics = {}
        propose = Mock(side_effect=[(-1, None), (1, self.safe), (1, self.safe)])
        moveit = Mock(side_effect=[dict(invalid_samples=1), dict(invalid_samples=0)])
        self.assertIs(self.select(propose, moveit=moveit, diagnostics=diagnostics), self.safe)
        self.assertEqual(propose.call_count, 3)
        self.assertEqual(moveit.call_count, 2)
        self.assertEqual([item.get("rejection_kind") for item in diagnostics["candidates"]],
                         ["planner_failed", "moveit_invalid", None])
        self.assertEqual(diagnostics["selected_attempt"], 3)

    def test_malformed_spline_is_rejected_before_distance_or_moveit(self):
        diagnostics = {}
        propose = Mock(side_effect=[(1, trajectory([[0.0, 0.0]])), (1, self.safe)])
        moveit = Mock(return_value=dict(invalid_samples=0))
        self.assertIs(self.select(propose, moveit=moveit, diagnostics=diagnostics), self.safe)
        first = diagnostics["candidates"][0]
        self.assertEqual(first["rejection_kind"], "invalid_trajectory")
        self.assertNotIn("native", first)
        self.assertEqual(moveit.call_count, 1)

    def test_proposal_exception_is_saved_and_propagated_without_another_attempt(self):
        diagnostics, saved = {}, []
        error = RuntimeError("RPC transport failed")
        propose = Mock(side_effect=error)
        with self.assertRaises(RuntimeError) as caught:
            self.select(propose, diagnostics=diagnostics,
                        save=lambda candidates: saved.append(deepcopy(candidates)))
        self.assertIs(caught.exception, error)
        propose.assert_called_once_with()
        self.assertEqual(len(saved[-1]), 1)
        self.assertEqual(diagnostics["status"], "failed")
        self.assertEqual(saved[-1][0]["stage"], "proposal")
        self.assertEqual(saved[-1][0]["error"], dict(type="RuntimeError", message=str(error)))

    def test_checkpoint_before_proposal_preserves_cancel_and_makes_no_rpc(self):
        class Cancelled(RuntimeError):
            pass
        diagnostics, saved = {}, []
        error = Cancelled("operator requested cancellation")
        propose = Mock(return_value=(1, self.safe))
        with self.assertRaises(Cancelled) as caught:
            self.select(propose, checkpoint=Mock(side_effect=error), diagnostics=diagnostics,
                        save=lambda candidates: saved.append(deepcopy(candidates)))
        self.assertIs(caught.exception, error)
        propose.assert_not_called()
        self.assertEqual(diagnostics["status"], "interrupted")
        self.assertEqual(saved[-1][0]["error"]["type"], "Cancelled")
        self.assertFalse(saved[-1][0]["accepted"])

    def test_checkpoint_after_proposal_keeps_error_code_without_evaluating(self):
        diagnostics, saved = {}, []
        error = RuntimeError("budget ended during RPC")
        checkpoint = Mock(side_effect=[None, error])
        propose, moveit = Mock(return_value=(1, self.safe)), Mock()
        with self.assertRaises(RuntimeError) as caught:
            self.select(propose, checkpoint=checkpoint, moveit=moveit, diagnostics=diagnostics,
                        save=lambda candidates: saved.append(deepcopy(candidates)))
        self.assertIs(caught.exception, error)
        propose.assert_called_once_with()
        moveit.assert_not_called()
        self.assertEqual(saved[-1][0]["moveit_error_code"], 1)
        self.assertEqual(diagnostics["status"], "interrupted")
        self.assertNotIn("native", saved[-1][0])

    def test_budget_during_native_sampling_keeps_partial_queries_and_stops(self):
        from mujoco_ros2_manipulation.workcells.planning_budget import PlanningBudget, PlanningBudgetExceeded
        now, original = [0], self.evaluator
        def slow_distance(state):
            now[0] += 1_000_000
            return original.at(state)
        self.evaluator = NS(at=slow_distance)
        budget = PlanningBudget(0.02, clock=lambda: now[0])
        diagnostics, saved = {}, []
        propose, moveit = Mock(return_value=(1, self.safe)), Mock()
        with self.assertRaises(PlanningBudgetExceeded):
            self.select(propose, checkpoint=budget.check, moveit=moveit, diagnostics=diagnostics,
                        save=lambda candidates: saved.append(deepcopy(candidates)))
        propose.assert_called_once_with()
        moveit.assert_not_called()
        partial = saved[-1][0]
        self.assertEqual(partial["native_progress"]["samples"], 20)
        self.assertIsNotNone(partial["native_progress"]["minimum"])
        self.assertEqual(partial["status"], "interrupted")
        self.assertEqual(partial["error"]["type"], "PlanningBudgetExceeded")
        self.assertEqual(partial["stage"], "native")
        self.assertNotIn("native", partial)

    def test_evaluator_exception_stops_search_with_completed_prefix_evidence(self):
        original, calls = self.evaluator, [0]
        error = ValueError("invalid native geometry")
        def broken_distance(state):
            calls[0] += 1
            if calls[0] == 2:
                raise error
            return original.at(state)
        self.evaluator = NS(at=broken_distance)
        diagnostics, saved = {}, []
        propose = Mock(return_value=(1, self.safe))
        with self.assertRaises(ValueError) as caught:
            self.select(propose, diagnostics=diagnostics,
                        save=lambda candidates: saved.append(deepcopy(candidates)))
        self.assertIs(caught.exception, error)
        propose.assert_called_once_with()
        self.assertEqual(diagnostics["status"], "failed")
        self.assertEqual(saved[-1][0]["stage"], "native")
        self.assertEqual(saved[-1][0]["native_progress"]["samples"], 1)
        self.assertIn("curve", saved[-1][0])
        self.assertFalse(saved[-1][0]["accepted"])

    def test_moveit_exception_stops_after_valid_native_candidate(self):
        diagnostics, saved = {}, []
        error = RuntimeError("validity service unavailable")
        propose = Mock(return_value=(1, self.safe))
        with self.assertRaises(RuntimeError) as caught:
            self.select(propose, moveit=Mock(side_effect=error), diagnostics=diagnostics,
                        save=lambda candidates: saved.append(deepcopy(candidates)))
        self.assertIs(caught.exception, error)
        propose.assert_called_once_with()
        self.assertEqual(diagnostics["status"], "failed")
        self.assertTrue(saved[-1][0]["native"]["complete"])
        self.assertEqual(saved[-1][0]["stage"], "moveit")

    def test_cancel_at_selection_never_returns_or_accepts_candidate(self):
        class Cancelled(RuntimeError):
            pass
        diagnostics, saved = {}, []
        error = Cancelled("cancel before selection")
        def checkpoint():
            if diagnostics["stage"] == "selection":
                raise error
        propose = Mock(return_value=(1, self.safe))
        moveit = Mock(return_value=dict(invalid_samples=0))
        with self.assertRaises(Cancelled) as caught:
            self.select(propose, checkpoint=checkpoint, moveit=moveit, diagnostics=diagnostics,
                        save=lambda candidates: saved.append(deepcopy(candidates)))
        self.assertIs(caught.exception, error)
        propose.assert_called_once_with()
        moveit.assert_called_once()
        self.assertEqual(diagnostics["status"], "interrupted")
        self.assertIsNone(diagnostics["selected_attempt"])
        self.assertFalse(saved[-1][0]["accepted"])
        self.assertEqual(saved[-1][0]["stage"], "selection")

    def test_keyboard_interrupt_during_proposal_is_saved_and_not_retried(self):
        diagnostics, saved = {}, []
        error = KeyboardInterrupt("shutdown")
        propose = Mock(side_effect=error)
        with self.assertRaises(KeyboardInterrupt) as caught:
            self.select(propose, diagnostics=diagnostics,
                        save=lambda candidates: saved.append(deepcopy(candidates)))
        self.assertIs(caught.exception, error)
        propose.assert_called_once_with()
        self.assertEqual(diagnostics["status"], "interrupted")
        self.assertEqual(saved[-1][0]["error"]["type"], "KeyboardInterrupt")

    def test_save_failure_does_not_replace_original_checkpoint_exception(self):
        diagnostics = {}
        error = RuntimeError("request cancelled")
        with self.assertRaises(RuntimeError) as caught:
            self.select(Mock(), checkpoint=Mock(side_effect=error), diagnostics=diagnostics,
                        save=Mock(side_effect=OSError("disk unavailable")))
        self.assertIs(caught.exception, error)
        self.assertEqual(diagnostics["error"]["message"], "request cancelled")
        self.assertEqual(diagnostics["save_error"]["type"], "OSError")

    def test_save_failure_does_not_return_a_selected_trajectory(self):
        diagnostics = {}
        error = OSError("disk unavailable")
        propose = Mock(return_value=(1, self.safe))
        with self.assertRaises(OSError) as caught:
            self.select(propose, diagnostics=diagnostics, save=Mock(side_effect=error))
        self.assertIs(caught.exception, error)
        propose.assert_called_once_with()
        self.assertEqual(diagnostics["status"], "failed")
        self.assertFalse(diagnostics["candidates"][0]["accepted"])
        self.assertIsNone(diagnostics["selected_attempt"])

    def test_candidate_limit_requires_an_integer_within_fixed_hard_bound(self):
        for limit in [0, -1, MAX_PLAN_CANDIDATES + 1, 1.0, True, False, None, "2"]:
            with self.subTest(limit=limit):
                propose, save = Mock(), Mock()
                with self.assertRaisesRegex(ValueError, "max_candidates"):
                    self.select(propose, save=save, max_candidates=limit)
                propose.assert_not_called()
                save.assert_not_called()


    def test_explicit_proposal_rejection_advances_without_fabricating_moveit_failure(self):
        diagnostics, saved = {}, []
        rejected = PlanCandidateRejected("Serial spline exceeds the 35s bound")
        propose = Mock(side_effect=[rejected, (1, self.safe)])
        moveit = Mock(return_value=dict(invalid_samples=0))
        result = self.select(propose, moveit=moveit, diagnostics=diagnostics,
                             save=lambda candidates: saved.append(deepcopy(candidates)))
        self.assertIs(result, self.safe)
        self.assertEqual(propose.call_count, 2)
        moveit.assert_called_once()
        first, second = saved[-1]
        self.assertEqual(first["status"], "rejected")
        self.assertEqual(first["rejection_kind"], "invalid_trajectory")
        self.assertEqual(first["rejection"], str(rejected))
        self.assertNotIn("moveit_error_code", first)
        self.assertNotIn("error", first)
        self.assertTrue(second["native"]["complete"])
        self.assertTrue(second["accepted"])
        self.assertEqual(diagnostics["selected_attempt"], 2)

    def test_explicit_proposal_rejection_at_single_bound_exhausts_with_saved_diagnostics(self):
        diagnostics, saved = {}, []
        propose = Mock(side_effect=PlanCandidateRejected("Unsupported assembled spline layout"))
        with self.assertRaisesRegex(PlanCandidatesExhausted, "no candidate dispatched"):
            self.select(propose, max_candidates=1, diagnostics=diagnostics,
                        save=lambda candidates: saved.append(deepcopy(diagnostics)))
        propose.assert_called_once_with()
        self.assertEqual(saved[-1]["status"], "exhausted")
        self.assertEqual(saved[-1]["attempted"], 1)
        self.assertIsNone(saved[-1]["selected_attempt"])
        self.assertEqual(saved[-1]["candidates"][0]["rejection_kind"], "invalid_trajectory")
        self.assertNotIn("moveit_error_code", saved[-1]["candidates"][0])

    def test_ordinary_proposal_value_error_is_not_a_retryable_rejection(self):
        diagnostics, saved = {}, []
        error = ValueError("Unexpected adapter state mismatch")
        propose = Mock(side_effect=[error, (1, self.safe)])
        with self.assertRaises(ValueError) as caught:
            self.select(propose, diagnostics=diagnostics,
                        save=lambda candidates: saved.append(deepcopy(candidates)))
        self.assertIs(caught.exception, error)
        propose.assert_called_once_with()
        self.assertEqual(diagnostics["status"], "failed")
        self.assertEqual(saved[-1][0]["error"]["type"], "ValueError")
        self.assertNotIn("rejection_kind", saved[-1][0])

    def test_explicit_rejection_type_from_evaluator_is_not_silently_retried(self):
        diagnostics, saved = {}, []
        error = PlanCandidateRejected("Wrong component raised an adapter rejection")
        self.evaluator = NS(at=Mock(side_effect=error))
        propose = Mock(return_value=(1, self.safe))
        with self.assertRaises(PlanCandidateRejected) as caught:
            self.select(propose, diagnostics=diagnostics,
                        save=lambda candidates: saved.append(deepcopy(candidates)))
        self.assertIs(caught.exception, error)
        propose.assert_called_once_with()
        self.assertEqual(diagnostics["status"], "failed")
        self.assertEqual(saved[-1][0]["stage"], "native")
        self.assertNotIn("rejection_kind", saved[-1][0])

    def test_checkpoint_after_explicit_rejection_still_interrupts_before_next_proposal(self):
        diagnostics, saved = {}, []
        error = RuntimeError("budget exhausted during assembly")
        propose = Mock(side_effect=[PlanCandidateRejected("Serial spline exceeds 35s"), (1, self.safe)])
        with self.assertRaises(RuntimeError) as caught:
            self.select(propose, diagnostics=diagnostics, checkpoint=Mock(side_effect=[None, error]),
                        save=lambda candidates: saved.append(deepcopy(candidates)))
        self.assertIs(caught.exception, error)
        propose.assert_called_once_with()
        self.assertEqual(diagnostics["status"], "interrupted")
        self.assertEqual(saved[-1][0]["error"]["message"], str(error))
        self.assertFalse(saved[-1][0]["accepted"])

    def test_explicit_rejection_kind_cannot_inject_unrecognized_classification(self):
        with self.assertRaisesRegex(ValueError, "kind must be invalid_trajectory"):
            PlanCandidateRejected("Rejected", kind="planner_failed")

    def test_curve_contract_failure_is_not_a_retryable_trajectory_rejection(self):
        error = ValueError("Unexpected joint/endpoint contract")
        diagnostics = {}
        propose = Mock(return_value=(1, self.safe))
        with self.assertRaises(ValueError) as caught:
            self.select(propose, diagnostics=diagnostics, validate_curve=Mock(side_effect=error))
        self.assertIs(caught.exception, error)
        propose.assert_called_once()
        candidate = diagnostics["candidates"][0]
        self.assertEqual(candidate["status"], "failed")
        self.assertEqual(candidate["stage"], "trajectory")
        self.assertNotIn("rejection_kind", candidate)

    def test_invalid_diagnostic_preserves_exact_time_parts_without_nonfinite_json(self):
        malformed = trajectory([[0., 0.], [1., 1.]])
        malformed.points[0].time_from_start.nanosec = 1
        malformed.points[1].time_from_start = NS(sec=36, nanosec=17)
        diagnostic = invalid_trajectory_diagnostic(malformed)
        self.assertEqual(diagnostic["first_time"], dict(sec=0, nanosec=1, seconds=1e-9))
        self.assertEqual(diagnostic["last_time"]["sec"], 36)
        self.assertEqual(diagnostic["last_time"]["nanosec"], 17)
        self.assertEqual(diagnostic["timing_issues"], ["nonzero_start", "end_exceeds_35s"])
        malformed.points[1].time_from_start.sec = float("nan")
        malformed.points[1].positions = [float("inf")] * 10000
        diagnostic = invalid_trajectory_diagnostic(malformed)
        restored = json.loads(json.dumps(diagnostic, allow_nan=False))
        self.assertIn("nonfinite_time", restored["timing_issues"])
        self.assertEqual(restored["last_time"]["sec"], dict(nonfinite="nan"))
        self.assertEqual(restored["points"][1]["positions_count"], 10000)
        self.assertNotIn("positions", restored["points"][1])

    def test_invalid_diagnostic_caps_captured_times_and_identifies_truncation(self):
        malformed = trajectory([[0., 0.]] * 2100)
        diagnostic = invalid_trajectory_diagnostic(malformed)
        self.assertEqual(diagnostic["point_count"], 2100)
        self.assertEqual(diagnostic["captured_point_count"], 2000)
        self.assertEqual(len(diagnostic["points"]), 2000)
        self.assertTrue(diagnostic["truncated"])
        self.assertEqual(diagnostic["last_time"]["sec"], 2099)
        self.assertIn("point_count_outside_2_2000", diagnostic["timing_issues"])

    def test_diagnostic_failure_does_not_hide_invalid_candidate_or_block_next(self):
        diagnostics = {}
        malformed = trajectory([[0., 0.]])
        propose = Mock(side_effect=[(1, malformed), (1, self.safe)])
        with patch("mujoco_ros2_manipulation.workcells.plan_selection.invalid_trajectory_diagnostic",
                   side_effect=OSError("Diagnostic unavailable")):
            self.assertIs(self.select(propose, diagnostics=diagnostics), self.safe)
        candidate = diagnostics["candidates"][0]
        self.assertEqual(candidate["status"], "rejected")
        self.assertEqual(candidate["rejection_kind"], "invalid_trajectory")
        self.assertEqual(candidate["trajectory_diagnostic_error"]["type"], "OSError")
        self.assertNotIn("error", candidate)
